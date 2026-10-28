//! E3, conservation across junctions: what the upstream approaches send
//! toward a link should arrive in its camera zone. Kept as a running balance
//! of PCU on the link between the upstream stop line and the zone, held
//! inside what physics allows; how far it has to be pushed back is the
//! evidence. Thresholds: docs/decisions.md D25.

use std::collections::VecDeque;

use super::EvidenceParams;

/// One interval on a link, PCU.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkInput {
    /// Sent toward the link by every upstream approach and movement feeding it.
    pub sent: f64,
    /// Arrived in the downstream camera zone.
    pub arrived: f64,
    /// Stopped in the downstream camera zone.
    pub queue: f64,
    /// Downstream camera zone length, m.
    pub zone_m: f64,
    pub interval: f64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LinkReading {
    /// E3 score. None until the balance can be trusted (start, after a gap).
    pub score: Option<f64>,
    /// Balance as a share of what the link can store. None with `score`.
    pub fill: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct LinkBalance {
    length_m: f64,
    lanes: f64,
    last_t: Option<f64>,
    /// PCU believed to be on the link outside the camera zone.
    balance: f64,
    /// PCU a recent queue may still hold on the link beyond free-flow transit.
    allowance: f64,
    /// When `allowance` first ran out since the last reset.
    settled_at: Option<f64>,
    /// (time, PCU sent), as long as a vehicle may take to arrive.
    sent: VecDeque<(f64, f64)>,
    /// (time, PCU sent, PCU the balance had to be corrected by), last `link_window_s`.
    window: VecDeque<(f64, f64, f64)>,
}

/// PCU sent less than `within` seconds before `t`.
fn transit(sent: &VecDeque<(f64, f64)>, t: f64, within: f64) -> f64 {
    let mut total = 0.0;
    for &(s, pcu) in sent {
        if t - s < within {
            total += pcu;
        }
    }
    total
}

impl LinkBalance {
    pub fn new(length_m: f64, lanes: u32) -> Self {
        LinkBalance {
            length_m,
            lanes: f64::from(lanes),
            last_t: None,
            balance: 0.0,
            allowance: 0.0,
            settled_at: None,
            sent: VecDeque::new(),
            window: VecDeque::new(),
        }
    }

    fn reset(&mut self) {
        self.last_t = None;
        self.settled_at = None;
        self.sent.clear();
        self.window.clear();
    }

    /// `input` None: the link or one of its feeders sent no measurement, so
    /// the balance is lost and starts again.
    pub fn update(&mut self, t: f64, input: Option<LinkInput>, p: &EvidenceParams) -> LinkReading {
        let Some(x) = input else {
            self.reset();
            return LinkReading::default();
        };
        if self.last_t.is_some_and(|l| (t - l - x.interval).abs() >= 1e-6) {
            self.reset();
        }
        // a camera that sees the whole link leaves nothing to balance
        let stretch = self.length_m - x.zone_m;
        if stretch <= 0.0 {
            self.reset();
            return LinkReading::default();
        }
        let storage = stretch * self.lanes / p.min_spacing_m;
        if self.last_t.is_none() {
            // what is on the link already is unknown: allow anything up to full
            self.balance = 0.0;
            self.allowance = storage;
        }
        self.last_t = Some(t);

        let earliest = stretch / (p.free_flow_mps * p.fast_factor);
        let latest = stretch / p.free_flow_mps + p.slack_s;
        self.sent.push_back((t, x.sent));
        while self.sent.front().is_some_and(|&(s, _)| t - s >= latest) {
            self.sent.pop_front();
        }
        self.balance += x.sent - x.arrived;

        // a queue filling the zone may extend onto the link; once it clears,
        // what it held drains at most at saturation flow
        let zone_storage = x.zone_m * self.lanes / p.min_spacing_m;
        self.allowance = if x.queue >= p.congested_fraction * zone_storage {
            storage
        } else {
            (self.allowance - self.lanes * p.saturation_flow * x.interval).max(0.0)
        };
        if self.allowance <= 0.0 && self.settled_at.is_none() {
            self.settled_at = Some(t);
        }

        // sent too recently to have arrived: still on the link. Sent longer
        // ago than the slowest trip, with no queue to hold it: arrived.
        let lower = transit(&self.sent, t, earliest);
        let upper = (transit(&self.sent, t, latest) + self.allowance).min(storage).max(lower);
        // while a queue holds the link the balance is only known to lie
        // between the bounds, so pulling it back is a silent re-sync (an
        // error from a fault must not surface when the queue clears much
        // later); only more than the link can store counts then
        let correction = if self.allowance > 0.0 {
            (self.balance - storage).max(0.0)
        } else if self.balance < lower {
            lower - self.balance
        } else if self.balance > upper {
            self.balance - upper
        } else {
            0.0
        };
        self.balance = self.balance.clamp(lower, upper);

        // judge only once anything on the link before the start has had time to arrive
        if self.settled_at.is_none_or(|s| t - s < latest) {
            return LinkReading::default();
        }
        self.window.push_back((t, x.sent, correction));
        while self.window.front().is_some_and(|&(s, _, _)| t - s >= p.link_window_s) {
            self.window.pop_front();
        }
        let (mut sent, mut wrong) = (0.0, 0.0);
        for &(_, s, c) in &self.window {
            sent += s;
            wrong += c;
        }
        let share = wrong / sent.max(p.min_volume_pcu);
        LinkReading {
            score: Some(((share - p.link_tolerance) / p.link_tolerance).clamp(0.0, 1.0)),
            fill: Some(self.balance / storage),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 285 m link, 2 lanes, 60 m zone: 225 m outside the zone, storage 128.6
    // PCU, trips take 13.5 s at the earliest and 46.2 s at the latest, and a
    // fresh balance settles after 129 s plus 46.2 s.

    /// (sent, arrived, queue) per second from t = 0; readings per second.
    fn run(rows: &[(f64, f64, f64)]) -> Vec<LinkReading> {
        let p = EvidenceParams::default();
        let mut link = LinkBalance::new(285.0, 2);
        rows.iter()
            .enumerate()
            .map(|(t, &(sent, arrived, queue))| {
                let x = LinkInput { sent, arrived, queue, zone_m: 60.0, interval: 1.0 };
                link.update(t as f64, Some(x), &p)
            })
            .collect()
    }

    /// A platoon of 10 PCU released upstream every 60 s (signals send traffic
    /// in platoons), arriving `delay` s later, scaled by `seen`.
    fn platoon(t: usize) -> f64 {
        if t.is_multiple_of(60) { 10.0 } else { 0.0 }
    }

    fn flow(n: usize, delay: usize, seen: f64) -> Vec<(f64, f64, f64)> {
        (0..n).map(|t| (platoon(t), if t >= delay { platoon(t - delay) * seen } else { 0.0 }, 0.0)).collect()
    }

    fn last_score(r: &[LinkReading]) -> Option<f64> {
        r.last().unwrap().score
    }

    #[test]
    fn conserved_flow_is_no_evidence() {
        let r = run(&flow(400, 20, 1.0));
        assert_eq!(last_score(&r), Some(0.0));
        assert!(r.last().unwrap().fill.unwrap() < 0.1);
    }

    #[test]
    fn waits_until_the_balance_has_settled() {
        let r = run(&flow(400, 20, 1.0));
        assert_eq!(r[174].score, None);
        assert!(r[175].score.is_some());
    }

    #[test]
    fn slow_but_valid_arrivals_are_no_evidence() {
        assert_eq!(last_score(&run(&flow(400, 44, 1.0))), Some(0.0));
    }

    #[test]
    fn travel_time_window_boundaries() {
        // just inside both ends of 13.5 - 46.2 s
        assert_eq!(last_score(&run(&flow(400, 14, 1.0))), Some(0.0));
        assert_eq!(last_score(&run(&flow(400, 46, 1.0))), Some(0.0));
        // earlier than the fastest trip: vehicles appear from nowhere
        assert_eq!(last_score(&run(&flow(400, 13, 1.0))), Some(1.0));
        // later than the slowest trip with no queue: vehicles go missing
        assert_eq!(last_score(&run(&flow(400, 50, 1.0))), Some(1.0));
    }

    #[test]
    fn undercount_downstream_is_evidence() {
        assert_eq!(last_score(&run(&flow(400, 20, 0.4))), Some(1.0));
    }

    #[test]
    fn small_miscounts_stay_below_tolerance() {
        assert_eq!(last_score(&run(&flow(400, 20, 0.9))), Some(0.0));
    }

    #[test]
    fn double_counting_downstream_is_evidence() {
        assert_eq!(last_score(&run(&flow(400, 20, 2.0))), Some(1.0));
    }

    #[test]
    fn no_upstream_traffic_is_no_evidence() {
        let mut rows = vec![(0.0, 0.0, 0.0); 400];
        assert_eq!(last_score(&run(&rows)), Some(0.0));
        // a stray arrival is tiny next to the minimum volume
        rows[390].1 = 1.0;
        assert_eq!(last_score(&run(&rows)), Some(0.0));
    }

    #[test]
    fn missing_data_restarts_without_evidence() {
        let p = EvidenceParams::default();
        let mut link = LinkBalance::new(285.0, 2);
        let rows = flow(700, 20, 1.0);
        let mut last = LinkReading::default();
        for (t, &(sent, arrived, queue)) in rows.iter().enumerate() {
            let x = (!(300..310).contains(&t)).then_some(LinkInput { sent, arrived, queue, zone_m: 60.0, interval: 1.0 });
            last = link.update(t as f64, x, &p);
            if (300..485).contains(&t) {
                // vehicles sent during the gap arrive after it: never judged
                assert_eq!(last.score, None, "t={t}");
            }
        }
        assert_eq!(last.score, Some(0.0));
    }

    #[test]
    fn queue_backing_onto_the_link_is_no_evidence() {
        // free flow, then the downstream zone fills and nothing arrives for
        // 100 s; the 20 PCU held then drain at 1 PCU/s and platoons resume
        let mut rows = flow(250, 20, 1.0);
        rows.extend((250..350).map(|t| (platoon(t), 0.0, 20.0)));
        rows.extend((350..520).map(|t| {
            let arrived = if t < 370 { 1.0 } else if t >= 380 { platoon(t - 20) } else { 0.0 };
            (platoon(t), arrived, 0.0)
        }));
        let r = run(&rows);
        assert!(r[300..].iter().all(|x| x.score == Some(0.0)));
        assert!(r[349].fill.unwrap() > 0.1);
    }

    #[test]
    fn error_held_by_a_queue_does_not_surface_when_it_clears() {
        // while the zone is jammed an upstream camera overcounts 0.2 PCU/s
        // (20 PCU that never existed); the real 20 PCU then drain, platoons
        // resume, and the phantom 20 must not be blamed on the link later
        let mut rows = flow(250, 20, 1.0);
        rows.extend((250..350).map(|t| (platoon(t) + 0.2, 0.0, 20.0)));
        rows.extend((350..700).map(|t| {
            let arrived = if t < 370 { 1.0 } else if t >= 380 { platoon(t - 20) } else { 0.0 };
            (platoon(t), arrived, 0.0)
        }));
        assert!(run(&rows)[350..].iter().all(|x| x.score == Some(0.0)));
    }

    #[test]
    fn more_on_the_link_than_it_can_hold_is_evidence() {
        // the zone looks jammed, but 2 PCU/s keep going in and nothing arrives for 200 s
        let mut rows = flow(250, 20, 1.0);
        rows.extend((0..200).map(|_| (2.0, 0.0, 20.0)));
        assert_eq!(last_score(&run(&rows)), Some(1.0));
    }

    #[test]
    fn camera_seeing_the_whole_link_has_no_e3() {
        let p = EvidenceParams::default();
        let mut link = LinkBalance::new(50.0, 2);
        for t in 0..300 {
            let x = LinkInput { sent: 1.0, arrived: 1.0, queue: 0.0, zone_m: 60.0, interval: 1.0 };
            assert_eq!(link.update(f64::from(t), Some(x), &p), LinkReading::default());
        }
    }
}
