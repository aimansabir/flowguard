//! G1 evidence (docs/specs/guard.md): signs, per approach and second, that a
//! measurement should not be taken at face value. Each check gives a score in
//! [0, 1] (0 = nothing suspicious) or None when it has nothing to judge.
//! Nothing here changes trust; combining the scores is G2. Thresholds are
//! placeholders, chosen in docs/decisions.md D24.
//!
//! E3 (conservation across junctions) and E4 (command response) are not here
//! yet.

use std::collections::VecDeque;

use crate::contracts::{ClassCounts, Command, Measurement, Movement, MovementCounts, Quality, Stage};
use crate::corridor::CorridorIndex;
use crate::error::{Error, Result};
use crate::guard::GuardInput;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EvidenceParams {
    /// E1: brightness at or below this counts as black, at or above 1 - this as saturated.
    pub brightness_edge: f64,
    /// E1: blur above this (0 sharp, 1 unusable).
    pub blur_max: f64,
    /// E1: mean detector confidence below this.
    pub confidence_min: f64,
    /// E1: track breaks per minute above this.
    pub track_breaks_max: f64,
    /// E1: scene shift above this, pixels.
    pub displacement_max_px: f64,
    /// E1 score when any flag is bad. Below 1 because a bad image is not
    /// proof of bad counts (scope.md).
    pub quality_weight: f64,
    /// E2: seconds of suspicious repetition for a score of 1.
    pub frozen_s: f64,
    /// E2: an upstream departure toward the approach counts as supply for this long, s.
    pub supply_s: f64,
    /// E2: score cap while the repeated measurement is a standing queue, which
    /// a blocked exit produces too.
    pub standing_weight: f64,
    /// E5: residuals kept, intervals (seconds in the simulator).
    pub window: usize,
    /// E5: residuals are summed over blocks this long, so a vehicle reported a
    /// few intervals late cancels out.
    pub block: usize,
    /// E5: mean |block residual| tolerated, vehicles. Score 1 at twice this.
    pub tolerance: f64,
}

impl Default for EvidenceParams {
    fn default() -> Self {
        EvidenceParams {
            brightness_edge: 0.05,
            blur_max: 0.7,
            confidence_min: 0.3,
            track_breaks_max: 30.0,
            displacement_max_px: 20.0,
            quality_weight: 0.5,
            frozen_s: 20.0,
            supply_s: 60.0,
            standing_weight: 0.5,
            window: 60,
            block: 5,
            tolerance: 1.0,
        }
    }
}

impl EvidenceParams {
    pub fn validate(&self) -> Result<()> {
        let fraction = |x: f64| (0.0..=1.0).contains(&x);
        let ok = fraction(self.brightness_edge)
            && fraction(self.quality_weight)
            && fraction(self.standing_weight)
            && self.frozen_s > 0.0
            && self.supply_s >= 0.0
            && self.tolerance > 0.0
            && self.block >= 1
            && self.window >= self.block;
        if !ok {
            return Err(Error::Config("invalid evidence parameters".into()));
        }
        Ok(())
    }
}

/// Evidence for one approach this second.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Evidence {
    /// E1. None: the producer reported no quality information.
    pub quality: Option<f64>,
    /// E2. None: no measurement this second.
    pub frozen: Option<f64>,
    /// E5. None: no measurement, or too few consecutive ones since the last gap.
    pub conservation: Option<f64>,
}

/// E1: the producer's own quality report. Any bad flag gives `quality_weight`;
/// fields the producer left out are not held against it.
pub fn quality(q: &Quality, p: &EvidenceParams) -> Option<f64> {
    let mut flags = Vec::new();
    if let Some(h) = &q.image_health {
        flags.push(h.frozen);
        flags.push(h.brightness <= p.brightness_edge || h.brightness >= 1.0 - p.brightness_edge);
        flags.push(h.blur > p.blur_max);
        flags.push(h.displacement_px > p.displacement_max_px);
    }
    if let Some(c) = &q.detector_confidence {
        flags.push(c.mean < p.confidence_min);
    }
    if let Some(b) = q.track_breaks_per_min {
        flags.push(b > p.track_breaks_max);
    }
    if flags.is_empty() {
        return None;
    }
    Some(if flags.contains(&true) { p.quality_weight } else { 0.0 })
}

/// Whether the approach had green during the last interval, from the command
/// the envelope issued for its junction. Yellow and all-red are not green.
pub fn is_green(ix: &CorridorIndex, approach: usize, last_commands: &[Option<Command>]) -> bool {
    let j = ix.junction_of(approach);
    last_commands[j].as_ref().is_some_and(|c| {
        c.stage == Stage::Green && ix.phases[j].get(c.phase_index).is_some_and(|p| p.contains(&approach))
    })
}

/// The counts of a measurement, which is what E2 compares and E5 balances.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Counts {
    time: f64,
    zone: ClassCounts,
    queue: ClassCounts,
    arrivals: ClassCounts,
    departures: MovementCounts,
}

impl Counts {
    fn of(m: &Measurement) -> Self {
        Counts { time: m.time, zone: m.zone_counts, queue: m.queue_counts, arrivals: m.arrivals, departures: m.departures }
    }

    fn same_as(&self, o: &Counts) -> bool {
        self.zone == o.zone && self.queue == o.queue && self.arrivals == o.arrivals && self.departures == o.departures
    }

    /// Every vehicle in the zone stopped and nothing moving in or out.
    fn standing(&self) -> bool {
        let z = self.zone.total();
        z > 0 && self.queue.total() == z && self.arrivals.total() == 0 && self.departures.total().total() == 0
    }
}

/// Vehicle conservation over one interval:
/// zone(t) - zone(t-1) - arrivals(t) + departures(t). 0 for a consistent producer.
fn residual(prev_zone: &ClassCounts, now: &Counts) -> i64 {
    i64::from(now.zone.total()) - i64::from(prev_zone.total()) - i64::from(now.arrivals.total())
        + i64::from(now.departures.total().total())
}

/// E5 score from consecutive residuals: mean |sum over each block| against
/// the tolerance. None until half a window has been seen.
fn conservation(residuals: &VecDeque<i64>, p: &EvidenceParams) -> Option<f64> {
    if residuals.len() < (p.window / 2).max(p.block) {
        return None;
    }
    let r: Vec<i64> = residuals.iter().copied().collect();
    let blocks: Vec<f64> = r.windows(p.block).map(|w| w.iter().sum::<i64>().abs() as f64).collect();
    let mean = blocks.iter().sum::<f64>() / blocks.len() as f64;
    Some(((mean - p.tolerance) / p.tolerance).clamp(0.0, 1.0))
}

/// What the checks remember about one approach.
#[derive(Debug, Clone, Default)]
struct Track {
    /// Last measurement received, however long ago.
    last: Option<Counts>,
    /// E2: seconds the measurement has repeated while it should have changed.
    frozen_s: f64,
    /// E2: last time an upstream approach sent vehicles toward this one.
    supplied_at: Option<f64>,
    /// E5: residuals of consecutive intervals since the last gap, oldest first.
    residuals: VecDeque<i64>,
}

/// Runs E1, E2 and E5 for every approach, once per second.
pub struct EvidenceTracker {
    params: EvidenceParams,
    /// Approach -> (movement, approach that movement feeds) inside the corridor.
    feeds: Vec<Vec<(Movement, usize)>>,
    tracks: Vec<Track>,
}

impl EvidenceTracker {
    pub fn new(ix: &CorridorIndex, params: EvidenceParams) -> Result<Self> {
        params.validate()?;
        let feeds = ix
            .corridor
            .approaches
            .iter()
            .map(|a| {
                Movement::ALL
                    .into_iter()
                    .filter_map(|m| a.movements.get(m).and_then(|e| ix.approach(e)).map(|d| (m, d)))
                    .collect()
            })
            .collect();
        Ok(EvidenceTracker { params, feeds, tracks: vec![Track::default(); ix.corridor.approaches.len()] })
    }

    /// Evidence per approach, indexed like `input.measurements`.
    pub fn update(&mut self, input: &GuardInput<'_>) -> Vec<Evidence> {
        // supply first: E2 on an approach looks at what its upstream sent this second
        for (u, m) in input.measurements.iter().enumerate() {
            let Some(m) = m else { continue };
            for &(mv, d) in &self.feeds[u] {
                if m.departures.get(mv).total() > 0 {
                    self.tracks[d].supplied_at = Some(input.t);
                }
            }
        }
        (0..self.tracks.len()).map(|a| self.check(input, a)).collect()
    }

    fn check(&mut self, input: &GuardInput<'_>, a: usize) -> Evidence {
        let p = &self.params;
        let track = &mut self.tracks[a];
        // a missing measurement holds E2 (no change, no growth) and breaks E5's chain
        let Some(m) = &input.measurements[a] else {
            track.residuals.clear();
            return Evidence::default();
        };
        let now = Counts::of(m);

        // E2: a repeat only counts when the measurement should have changed,
        // under green with vehicles in the zone or sent from upstream
        if track.last.is_some_and(|l| l.same_as(&now)) {
            let supplied = track.supplied_at.is_some_and(|s| input.t - s <= p.supply_s);
            if is_green(input.corridor, a, input.last_commands) && (now.zone.total() > 0 || supplied) {
                track.frozen_s += m.interval_s;
            }
        } else {
            track.frozen_s = 0.0;
        }
        let cap = if now.standing() { p.standing_weight } else { 1.0 };
        let frozen = (track.frozen_s / p.frozen_s).min(cap);

        // E5: only across consecutive intervals
        match track.last {
            Some(l) if (now.time - l.time - m.interval_s).abs() < 1e-6 => {
                track.residuals.push_back(residual(&l.zone, &now));
                if track.residuals.len() > p.window {
                    track.residuals.pop_front();
                }
            }
            _ => track.residuals.clear(),
        }
        track.last = Some(now);

        Evidence {
            quality: quality(&m.quality, p),
            frozen: Some(frozen),
            conservation: conservation(&track.residuals, p),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contracts::{DetectorConfidence, ImageHealth, Source};
    use crate::corridor::tests::two_junctions;

    // two_junctions: A has phases [a_w], [a_n]; B has [b_w], [b_n].
    // a_w goes straight into b_w, so b_w has an upstream; a_n is an entry.
    const A_W: usize = 0;
    const A_N: usize = 1;
    const B_W: usize = 2;

    fn cars(n: u32) -> ClassCounts {
        ClassCounts { car: n, ..Default::default() }
    }

    /// zone, queue, arrivals, straight departures, all cars.
    fn meas(approach: &str, t: f64, zone: u32, queue: u32, arr: u32, dep: u32) -> Measurement {
        let junction = if approach.starts_with('a') { "A" } else { "B" };
        Measurement {
            schema_version: "2.0".into(),
            time: t,
            interval_s: 1.0,
            junction_id: junction.into(),
            approach_id: approach.into(),
            source: Source::Sim,
            zone_length_m: 60.0,
            zone_counts: cars(zone),
            queue_counts: cars(queue),
            arrivals: cars(arr),
            departures: MovementCounts { straight: cars(dep), ..Default::default() },
            quality: Quality::default(),
        }
    }

    fn cmd(junction: &str, phase: usize, stage: Stage) -> Option<Command> {
        Some(Command {
            schema_version: "2.0".into(),
            time: 0.0,
            junction_id: junction.into(),
            phase_index: phase,
            phase_started_at: 0.0,
            controller_name: "test".into(),
            stage,
            target_phase: phase,
            requested_phase: None,
            intervention: None,
        })
    }

    /// A tracker on two_junctions; `step` feeds one second, `ms` being the
    /// measurements that arrived.
    struct Rig {
        ix: CorridorIndex,
        ev: EvidenceTracker,
    }

    impl Rig {
        fn new() -> Self {
            let ix = CorridorIndex::new(two_junctions()).unwrap();
            let ev = EvidenceTracker::new(&ix, EvidenceParams::default()).unwrap();
            Rig { ix, ev }
        }

        fn step(&mut self, t: f64, ms: Vec<Measurement>, commands: &[Option<Command>]) -> Vec<Evidence> {
            let mut slots = vec![None; 4];
            for m in ms {
                let a = self.ix.approach(&m.approach_id).unwrap();
                slots[a] = Some(m);
            }
            let input = GuardInput { t, corridor: &self.ix, measurements: &slots, last_commands: commands };
            self.ev.update(&input)
        }
    }

    fn green_a_w() -> [Option<Command>; 2] {
        [cmd("A", 0, Stage::Green), cmd("B", 0, Stage::Green)]
    }

    fn red_a_w() -> [Option<Command>; 2] {
        [cmd("A", 1, Stage::Green), cmd("B", 1, Stage::Green)]
    }

    // ------------------------------------------------------------ E1

    fn health(frozen: bool, blur: f64, brightness: f64, displacement_px: f64) -> Option<ImageHealth> {
        Some(ImageHealth { frozen, blur, brightness, displacement_px })
    }

    #[test]
    fn e1_missing_quality_is_no_evidence() {
        assert_eq!(quality(&Quality::default(), &EvidenceParams::default()), None);
    }

    #[test]
    fn e1_healthy_camera_scores_zero() {
        let q = Quality {
            detector_confidence: Some(DetectorConfidence { mean: 0.8, std: 0.1 }),
            track_breaks_per_min: Some(2.0),
            image_health: health(false, 0.1, 0.5, 1.0),
            count_error: None,
        };
        assert_eq!(quality(&q, &EvidenceParams::default()), Some(0.0));
    }

    #[test]
    fn e1_each_bad_flag_is_weak_evidence() {
        let p = EvidenceParams::default();
        let ok = health(false, 0.1, 0.5, 1.0);
        let cases = [
            Quality { image_health: health(true, 0.1, 0.5, 1.0), ..Default::default() },
            Quality { image_health: health(false, 0.9, 0.5, 1.0), ..Default::default() },
            Quality { image_health: health(false, 0.1, 0.01, 1.0), ..Default::default() },
            Quality { image_health: health(false, 0.1, 0.99, 1.0), ..Default::default() },
            Quality { image_health: health(false, 0.1, 0.5, 50.0), ..Default::default() },
            Quality { detector_confidence: Some(DetectorConfidence { mean: 0.1, std: 0.1 }), image_health: ok, ..Default::default() },
            Quality { track_breaks_per_min: Some(60.0), image_health: ok, ..Default::default() },
        ];
        for q in cases {
            assert_eq!(quality(&q, &p), Some(p.quality_weight), "{q:?}");
        }
    }

    #[test]
    fn e1_never_reaches_full_evidence_alone() {
        let p = EvidenceParams::default();
        let worst = Quality {
            detector_confidence: Some(DetectorConfidence { mean: 0.0, std: 0.0 }),
            track_breaks_per_min: Some(1000.0),
            image_health: health(true, 1.0, 0.0, 500.0),
            count_error: Some(50.0),
        };
        assert!(quality(&worst, &p).unwrap() < 1.0);
    }

    #[test]
    fn e1_judges_only_what_was_reported() {
        let p = EvidenceParams::default();
        let only_conf = |mean| Quality { detector_confidence: Some(DetectorConfidence { mean, std: 0.0 }), ..Default::default() };
        assert_eq!(quality(&only_conf(0.9), &p), Some(0.0));
        assert_eq!(quality(&only_conf(0.1), &p), Some(p.quality_weight));
    }

    // ------------------------------------------------------------ E2

    #[test]
    fn green_follows_the_command() {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        assert!(is_green(&ix, A_W, &green_a_w()));
        assert!(!is_green(&ix, A_N, &green_a_w()));
        assert!(!is_green(&ix, A_W, &red_a_w()));
        assert!(!is_green(&ix, A_W, &[cmd("A", 0, Stage::Yellow), None]));
        assert!(!is_green(&ix, A_W, &[cmd("A", 0, Stage::AllRed), None]));
        assert!(!is_green(&ix, A_W, &[None, None]));
    }

    fn frozen(e: &[Evidence], a: usize) -> f64 {
        e[a].frozen.unwrap()
    }

    #[test]
    fn e2_repeat_under_green_builds_to_full_evidence() {
        let mut rig = Rig::new();
        let mut e = vec![];
        for t in 0..=20 {
            e = rig.step(f64::from(t), vec![meas("a_w", f64::from(t), 5, 2, 1, 1)], &green_a_w());
            if t == 10 {
                assert_eq!(frozen(&e, A_W), 0.5);
            }
        }
        assert_eq!(frozen(&e, A_W), 1.0);
    }

    #[test]
    fn e2_changing_counts_are_healthy() {
        let mut rig = Rig::new();
        for t in 0..60 {
            let z = 5 + (t % 3);
            let e = rig.step(f64::from(t), vec![meas("a_w", f64::from(t), z, 2, 1, 1)], &green_a_w());
            assert!(frozen(&e, A_W) < 0.2);
        }
    }

    #[test]
    fn e2_change_resets_the_count() {
        let mut rig = Rig::new();
        for t in 0..15 {
            rig.step(f64::from(t), vec![meas("a_w", f64::from(t), 5, 2, 1, 1)], &green_a_w());
        }
        let e = rig.step(15.0, vec![meas("a_w", 15.0, 6, 2, 1, 0)], &green_a_w());
        assert_eq!(frozen(&e, A_W), 0.0);
    }

    #[test]
    fn e2_repeat_under_red_is_not_evidence() {
        let mut rig = Rig::new();
        for t in 0..100 {
            let e = rig.step(f64::from(t), vec![meas("a_w", f64::from(t), 8, 8, 0, 0)], &red_a_w());
            assert_eq!(frozen(&e, A_W), 0.0);
        }
    }

    #[test]
    fn e2_red_holds_instead_of_resetting() {
        // a camera frozen through a whole cycle keeps the green seconds it built up
        let mut rig = Rig::new();
        let m = |t: i32| vec![meas("a_w", f64::from(t), 5, 2, 1, 1)];
        for t in 0..=10 {
            rig.step(f64::from(t), m(t), &green_a_w());
        }
        for t in 11..40 {
            rig.step(f64::from(t), m(t), &red_a_w());
        }
        let e = rig.step(40.0, m(40), &green_a_w());
        assert_eq!(frozen(&e, A_W), 0.55);
    }

    #[test]
    fn e2_empty_entry_road_is_legitimate() {
        // a_n has no upstream junction: an empty zone under green proves nothing
        let mut rig = Rig::new();
        let green_a_n = [cmd("A", 1, Stage::Green), None];
        for t in 0..100 {
            let e = rig.step(f64::from(t), vec![meas("a_n", f64::from(t), 0, 0, 0, 0)], &green_a_n);
            assert_eq!(frozen(&e, A_N), 0.0);
        }
    }

    #[test]
    fn e2_empty_zone_with_upstream_supply_is_evidence() {
        // a_w keeps sending vehicles straight into b_w, yet b_w reports nothing
        // under green: what a blackout looks like
        let mut rig = Rig::new();
        let mut e = vec![];
        for t in 0..=20 {
            let dep = u32::from(t % 4 == 0);
            let ms = vec![meas("a_w", f64::from(t), 3, 0, dep, dep), meas("b_w", f64::from(t), 0, 0, 0, 0)];
            e = rig.step(f64::from(t), ms, &green_a_w());
        }
        assert_eq!(frozen(&e, B_W), 1.0);
    }

    #[test]
    fn e2_supply_expires() {
        // one upstream departure at t=0 makes b_w's empty zone suspicious for
        // supply_s seconds, after that an empty zone is legitimate again
        let mut rig = Rig::new();
        rig.step(0.0, vec![meas("a_w", 0.0, 3, 0, 1, 1), meas("b_w", 0.0, 0, 0, 0, 0)], &green_a_w());
        for t in 1..=100 {
            rig.step(f64::from(t), vec![meas("b_w", f64::from(t), 0, 0, 0, 0)], &green_a_w());
        }
        assert_eq!(rig.ev.tracks[B_W].frozen_s, EvidenceParams::default().supply_s);
    }

    #[test]
    fn e2_standing_queue_is_capped() {
        // all stopped, nothing in or out, under green: a blocked exit looks like this too
        let mut rig = Rig::new();
        let mut e = vec![];
        for t in 0..100 {
            e = rig.step(f64::from(t), vec![meas("a_w", f64::from(t), 11, 11, 0, 0)], &green_a_w());
        }
        assert_eq!(frozen(&e, A_W), EvidenceParams::default().standing_weight);
    }

    #[test]
    fn e2_missing_measurement_holds() {
        let mut rig = Rig::new();
        let m = |t: i32| vec![meas("a_w", f64::from(t), 5, 2, 1, 1)];
        for t in 0..=10 {
            rig.step(f64::from(t), m(t), &green_a_w());
        }
        let e = rig.step(11.0, vec![], &green_a_w());
        assert_eq!(e[A_W], Evidence::default());
        let e = rig.step(12.0, m(12), &green_a_w());
        assert_eq!(frozen(&e, A_W), 0.55);
    }

    // ------------------------------------------------------------ E5

    /// A queue that fills and empties, consistent second by second.
    /// Returns (zone, arrivals, departures) per second.
    fn consistent_flow(n: usize) -> Vec<(u32, u32, u32)> {
        let mut zone = 0u32;
        (0..n)
            .map(|t| {
                let arr = u32::from(t % 3 == 0);
                let dep = u32::from(zone > 0 && t % 5 < 2);
                zone = zone + arr - dep;
                (zone, arr, dep)
            })
            .collect()
    }

    fn conservation_after(rows: &[(u32, u32, u32)]) -> Option<f64> {
        let mut rig = Rig::new();
        let mut e = vec![];
        for (t, &(z, a, d)) in rows.iter().enumerate() {
            let t = t as f64;
            e = rig.step(t, vec![meas("a_w", t, z, 0, a, d)], &green_a_w());
        }
        e[A_W].conservation
    }

    #[test]
    fn e5_residual_is_zero_for_a_consistent_producer() {
        let prev = Counts::of(&meas("a_w", 0.0, 4, 0, 0, 0));
        let now = Counts::of(&meas("a_w", 1.0, 5, 0, 2, 1));
        assert_eq!(residual(&prev.zone, &now), 0);
        let now = Counts::of(&meas("a_w", 1.0, 9, 0, 2, 1));
        assert_eq!(residual(&prev.zone, &now), 4);
    }

    #[test]
    fn e5_consistent_counts_score_zero() {
        assert_eq!(conservation_after(&consistent_flow(200)), Some(0.0));
    }

    #[test]
    fn e5_needs_half_a_window() {
        assert_eq!(conservation_after(&consistent_flow(20)), None);
        assert!(conservation_after(&consistent_flow(40)).is_some());
    }

    #[test]
    fn e5_counts_that_do_not_add_up_are_evidence() {
        // zone jumps around while flows say nothing happened (undercount-like)
        let rows: Vec<_> = (0..120).map(|t| ([8, 3, 6, 2, 7, 1, 5][t % 7], 0, 0)).collect();
        assert_eq!(conservation_after(&rows), Some(1.0));
    }

    #[test]
    fn e5_vehicles_vanishing_without_departing_are_evidence() {
        // zone drains steadily but no departure is ever reported (displaced-like)
        let rows: Vec<_> = (0..120).map(|t| ([6, 5, 4, 3, 2, 1][t % 6], u32::from(t % 6 == 0) * 6, 0)).collect();
        assert_eq!(conservation_after(&rows), Some(1.0));
    }

    #[test]
    fn e5_tolerates_late_departures_and_flicker() {
        // realistic camera noise: every departure reported 2 s late, and the
        // zone count off by one for a single second now and then
        let clean = consistent_flow(300);
        let rows: Vec<_> = (0..clean.len())
            .map(|t| {
                let (z, a, _) = clean[t];
                let late = if t >= 2 { clean[t - 2].2 } else { 0 };
                let flicker = u32::from(t % 37 == 0);
                (z + flicker, a, late)
            })
            .collect();
        assert_eq!(conservation_after(&rows), Some(0.0));
    }

    #[test]
    fn e5_gap_breaks_the_chain_without_false_evidence() {
        let rows = consistent_flow(100);
        let mut rig = Rig::new();
        let mut e = vec![];
        for (t, &(z, a, d)) in rows.iter().enumerate() {
            if (50..55).contains(&t) {
                // dropout: nothing arrives, and the zone changes unseen
                e = rig.step(t as f64, vec![], &green_a_w());
                assert_eq!(e[A_W].conservation, None);
                continue;
            }
            e = rig.step(t as f64, vec![meas("a_w", t as f64, z, 0, a, d)], &green_a_w());
            if (55..85).contains(&t) {
                assert_eq!(e[A_W].conservation, None, "t={t}");
            }
        }
        assert_eq!(e[A_W].conservation, Some(0.0));
    }

    #[test]
    fn rejects_bad_params() {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        let bad = EvidenceParams { window: 2, block: 5, ..Default::default() };
        assert!(matches!(EvidenceTracker::new(&ix, bad), Err(Error::Config(_))));
        let bad = EvidenceParams { tolerance: 0.0, ..Default::default() };
        assert!(EvidenceTracker::new(&ix, bad).is_err());
    }
}
