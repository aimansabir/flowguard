//! G3 (docs/specs/guard.md): a queue estimate per approach that does not
//! need the approach's own camera. A point queue in PCU,
//! `Q(t+1) = max(0, Q(t) + A(t) - D(t))`, run every second for every
//! approach so it is ready when a camera turns FAULTY. Rules: D26.

use std::collections::VecDeque;

/// Where the arrivals A(t) come from this interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Supply {
    /// Entry approach: no camera upstream, use the trusted arrival rate.
    Entry,
    /// PCU the upstream approaches sent toward this one this interval.
    Sent(f64),
    /// An upstream camera is missing or FAULTY: what it sent is unknown.
    Unknown,
}

/// The camera reading the estimate re-anchors to, given only on seconds the
/// approach is TRUSTED with no suspicious evidence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Anchor {
    /// Measured queue, PCU.
    pub queue: f64,
    /// Measured arrivals into the zone this interval, PCU.
    pub arrivals: f64,
    /// The zone's queue is long enough that it may continue upstream of it.
    pub zone_full: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QueueInput {
    pub interval: f64,
    /// The envelope showed this approach green during the interval.
    pub green: bool,
    pub supply: Supply,
    pub anchor: Option<Anchor>,
}

/// Fixed per-approach physics, from the corridor map and `evidence.link`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QueueParams {
    /// Free-flow time from the upstream stop line to this stop line, s.
    pub travel_s: f64,
    /// PCU per second the approach discharges under green (saturation flow x lanes).
    pub discharge_pcu_per_s: f64,
    /// Most PCU the approach can hold (length x lanes / min spacing).
    pub storage_pcu: f64,
    /// Trusted seconds the arrival rate is averaged over.
    pub arrival_window_s: f64,
    /// Margin growth per second without a trusted reading, PCU/s.
    pub margin_growth: f64,
}

#[derive(Debug, Clone)]
pub struct VirtualQueue {
    p: QueueParams,
    q: f64,
    /// Seconds since the last anchor (trusted reading).
    untrusted_s: f64,
    /// (time sent, interval, PCU or None if unknown) on the way here.
    in_transit: VecDeque<(f64, f64, Option<f64>)>,
    /// (interval, PCU) arrivals seen on trusted seconds, newest last, at most
    /// `arrival_window_s` seconds of them however old.
    trusted_arrivals: VecDeque<(f64, f64)>,
}

impl VirtualQueue {
    pub fn new(p: QueueParams) -> Self {
        VirtualQueue { p, q: 0.0, untrusted_s: 0.0, in_transit: VecDeque::new(), trusted_arrivals: VecDeque::new() }
    }

    /// Queue estimate, PCU.
    pub fn queue(&self) -> f64 {
        self.q
    }

    /// Uncertainty of `queue()`, PCU: 0 at a trusted reading, then growing
    /// linearly with the time since.
    pub fn margin(&self) -> f64 {
        self.p.margin_growth * self.untrusted_s
    }

    /// Mean arrivals per second over the trusted window; 0 before any
    /// trusted second (nothing to go on: the estimate can only drain).
    pub fn arrival_rate(&self) -> f64 {
        let (mut s, mut pcu) = (0.0, 0.0);
        for &(dt, a) in &self.trusted_arrivals {
            s += dt;
            pcu += a;
        }
        if s > 0.0 { pcu / s } else { 0.0 }
    }

    /// One interval ending at `t`.
    pub fn update(&mut self, t: f64, x: &QueueInput) {
        // A: what reaches the stop line this interval
        let arrived = match x.supply {
            Supply::Entry => self.arrival_rate() * x.interval,
            Supply::Sent(pcu) => self.arrive(t, x.interval, Some(pcu)),
            Supply::Unknown => self.arrive(t, x.interval, None),
        };
        // D: saturation flow while green, never more than is there
        let capacity = if x.green { self.p.discharge_pcu_per_s * x.interval } else { 0.0 };
        self.q = (self.q + arrived - capacity).clamp(0.0, self.p.storage_pcu);

        match x.anchor {
            Some(a) => {
                // a full zone only shows the front of the queue: the camera is
                // then a lower bound, the estimate may continue past it
                self.q = if a.zone_full { self.q.max(a.queue) } else { a.queue };
                self.untrusted_s = 0.0;
                self.trusted_arrivals.push_back((x.interval, a.arrivals));
                let mut kept: f64 = self.trusted_arrivals.iter().map(|&(dt, _)| dt).sum();
                while kept > self.p.arrival_window_s {
                    let (dt, _) = self.trusted_arrivals.pop_front().expect("kept > 0");
                    kept -= dt;
                }
            }
            None => self.untrusted_s += x.interval,
        }
    }

    /// Queues what was sent at `t` and returns what has completed the trip:
    /// sent at least `travel_s` ago. Unknown amounts arrive at the trusted rate.
    fn arrive(&mut self, t: f64, interval: f64, sent: Option<f64>) -> f64 {
        self.in_transit.push_back((t, interval, sent));
        let mut arrived = 0.0;
        while self.in_transit.front().is_some_and(|&(s, _, _)| t - s >= self.p.travel_s) {
            let (_, dt, pcu) = self.in_transit.pop_front().expect("front checked");
            arrived += pcu.unwrap_or_else(|| self.arrival_rate() * dt);
        }
        arrived
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> QueueParams {
        // 285 m, 2 lanes: 20.5 s at 13.89 m/s, 1 PCU/s discharge, 162.9 PCU storage
        QueueParams {
            travel_s: 285.0 / 13.89,
            discharge_pcu_per_s: 1.0,
            storage_pcu: 285.0 * 2.0 / 3.5,
            arrival_window_s: 300.0,
            margin_growth: 0.05,
        }
    }

    fn step(v: &mut VirtualQueue, t: usize, green: bool, supply: Supply, anchor: Option<Anchor>) {
        v.update(t as f64, &QueueInput { interval: 1.0, green, supply, anchor });
    }

    fn anchor(queue: f64, arrivals: f64) -> Option<Anchor> {
        Some(Anchor { queue, arrivals, zone_full: false })
    }

    #[test]
    fn trusted_reading_anchors_the_queue() {
        let mut v = VirtualQueue::new(params());
        step(&mut v, 0, false, Supply::Entry, anchor(7.0, 0.0));
        assert_eq!((v.queue(), v.margin()), (7.0, 0.0));
    }

    #[test]
    fn upstream_departures_arrive_after_the_travel_time() {
        let mut v = VirtualQueue::new(params());
        step(&mut v, 0, false, Supply::Sent(5.0), anchor(0.0, 0.0));
        for t in 1..=20 {
            step(&mut v, t, false, Supply::Sent(0.0), None);
            assert_eq!(v.queue(), 0.0, "t={t}");
        }
        step(&mut v, 21, false, Supply::Sent(0.0), None);
        assert_eq!(v.queue(), 5.0);
    }

    #[test]
    fn green_discharges_and_never_below_zero() {
        let mut v = VirtualQueue::new(params());
        step(&mut v, 0, false, Supply::Entry, anchor(3.0, 0.0));
        step(&mut v, 1, true, Supply::Entry, None);
        assert_eq!(v.queue(), 2.0);
        for t in 2..10 {
            step(&mut v, t, true, Supply::Entry, None);
        }
        assert_eq!(v.queue(), 0.0);
    }

    #[test]
    fn red_yellow_and_all_red_do_not_discharge() {
        // `green` is false for every non-green stage
        let mut v = VirtualQueue::new(params());
        step(&mut v, 0, false, Supply::Entry, anchor(3.0, 0.0));
        for t in 1..60 {
            step(&mut v, t, false, Supply::Entry, None);
        }
        assert_eq!(v.queue(), 3.0);
    }

    #[test]
    fn entry_uses_the_trusted_arrival_rate() {
        let mut v = VirtualQueue::new(params());
        // 100 trusted seconds with 0.5 PCU/s arriving (one PCU every 2 s)
        for t in 0..100 {
            step(&mut v, t, false, Supply::Entry, anchor(0.0, if t.is_multiple_of(2) { 1.0 } else { 0.0 }));
        }
        assert_eq!(v.arrival_rate(), 0.5);
        // untrusted under red: the queue grows at the trusted rate
        for t in 100..140 {
            step(&mut v, t, false, Supply::Entry, None);
        }
        assert_eq!(v.queue(), 20.0);
    }

    #[test]
    fn arrival_rate_keeps_the_last_trusted_window_however_old() {
        let p = QueueParams { arrival_window_s: 10.0, ..params() };
        let mut v = VirtualQueue::new(p);
        for t in 0..10 {
            step(&mut v, t, false, Supply::Entry, anchor(0.0, 9.0));
        }
        for t in 10..20 {
            step(&mut v, t, false, Supply::Entry, anchor(0.0, 1.0));
        }
        assert_eq!(v.arrival_rate(), 1.0);
        // a long untrusted spell does not age the history away
        for t in 20..1000 {
            step(&mut v, t, true, Supply::Entry, None);
        }
        assert_eq!(v.arrival_rate(), 1.0);
    }

    #[test]
    fn no_trusted_history_means_no_arrivals() {
        let mut v = VirtualQueue::new(params());
        for t in 0..50 {
            step(&mut v, t, false, Supply::Entry, None);
        }
        assert_eq!((v.arrival_rate(), v.queue()), (0.0, 0.0));
    }

    #[test]
    fn unknown_upstream_arrives_at_the_trusted_rate() {
        let mut v = VirtualQueue::new(params());
        for t in 0..100 {
            step(&mut v, t, false, Supply::Sent(0.0), anchor(0.0, 0.25));
        }
        // the upstream camera fails: 40 s sent unknown, 30 s known as 0
        for t in 100..140 {
            step(&mut v, t, false, Supply::Unknown, None);
        }
        for t in 140..170 {
            step(&mut v, t, false, Supply::Sent(0.0), None);
        }
        assert_eq!(v.queue(), 40.0 * 0.25);
    }

    #[test]
    fn queue_can_exceed_what_the_camera_sees() {
        let mut v = VirtualQueue::new(params());
        step(&mut v, 0, false, Supply::Entry, Some(Anchor { queue: 30.0, arrivals: 0.0, zone_full: true }));
        for t in 1..=40 {
            step(&mut v, t, false, Supply::Sent(1.0), None);
        }
        // what was sent at t = 1..19 has made the 20.5 s trip: 19 PCU on top
        // of the 30 the camera last saw
        assert_eq!(v.queue(), 49.0);
        // a trusted but full zone is a lower bound, not a reset (one more arrives)
        step(&mut v, 41, false, Supply::Sent(0.0), Some(Anchor { queue: 30.0, arrivals: 0.0, zone_full: true }));
        assert_eq!(v.queue(), 50.0);
        // never more than the approach holds
        for t in 42..1000 {
            step(&mut v, t, false, Supply::Sent(5.0), None);
        }
        assert_eq!(v.queue(), params().storage_pcu);
    }

    #[test]
    fn margin_grows_while_untrusted_and_resets_on_anchor() {
        let mut v = VirtualQueue::new(params());
        step(&mut v, 0, false, Supply::Entry, anchor(5.0, 0.0));
        for t in 1..=100 {
            step(&mut v, t, false, Supply::Entry, None);
        }
        assert!((v.margin() - 5.0).abs() < 1e-9);
        step(&mut v, 101, false, Supply::Entry, anchor(2.0, 0.0));
        assert_eq!((v.queue(), v.margin()), (2.0, 0.0));
    }

    #[test]
    fn reanchors_cleanly_after_recovery() {
        // a wrong estimate built while untrusted is dropped at the next trusted reading
        let mut v = VirtualQueue::new(params());
        for t in 0..100 {
            step(&mut v, t, false, Supply::Sent(1.0), None);
        }
        assert!(v.queue() > 50.0);
        step(&mut v, 100, false, Supply::Sent(0.0), anchor(4.0, 0.0));
        assert_eq!(v.queue(), 4.0);
    }

    #[test]
    fn approaches_are_independent() {
        let (mut a, mut b) = (VirtualQueue::new(params()), VirtualQueue::new(params()));
        for t in 0..60 {
            step(&mut a, t, false, Supply::Sent(1.0), None);
            step(&mut b, t, true, Supply::Sent(0.0), anchor(0.0, 0.0));
        }
        assert!(a.queue() > 0.0);
        assert_eq!(b.queue(), 0.0);
    }
}
