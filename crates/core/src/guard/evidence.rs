//! G1 evidence (docs/specs/guard.md): signs, per approach and second, that a
//! measurement should not be taken at face value. Each check gives a score in
//! [0, 1] (0 = nothing suspicious) or None when it has nothing to judge.
//! Nothing here changes trust; combining the scores is G2. Thresholds are
//! placeholders, chosen in docs/decisions.md D24 (E1, E2, E5) and D25 (E3,
//! E4). flowguard_sim.evidence_replay is a Python copy for offline
//! calibration, held to the same scores by tests/fixtures/evidence_replay.json.

mod link;
mod response;

use std::collections::VecDeque;

pub use link::{LinkBalance, LinkInput, LinkReading};
pub use response::{Response, ResponseInput};

use crate::contracts::{
    ClassCounts, Command, EvidenceConfig, Measurement, Movement, MovementCounts, PcuWeights, Quality, Stage,
};
use crate::corridor::CorridorIndex;
use crate::error::{Error, Result};
use crate::guard::GuardInput;

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(test, derive(serde::Deserialize))]
#[cfg_attr(test, serde(deny_unknown_fields))]
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
    /// E3: free-flow speed on links, m/s. Not in the corridor map (D25).
    pub free_flow_mps: f64,
    /// E3: the fastest drivers go this much over free-flow speed.
    pub fast_factor: f64,
    /// E3: extra seconds the slowest trip may take (start-up, slow classes, turns).
    pub slack_s: f64,
    /// E3: metres of lane one PCU takes in the densest standing queue.
    pub min_spacing_m: f64,
    /// E3: the downstream zone counts as congested, and a link as full, from
    /// this share of what it can store.
    pub congested_fraction: f64,
    /// E3: PCU per second per lane a released queue drains at, at most.
    pub saturation_flow: f64,
    /// E3: seconds of corrections and sent PCU compared.
    pub link_window_s: f64,
    /// E3: PCU sent below which a window counts as this much, so a few
    /// miscounts in light traffic stay small.
    pub min_volume_pcu: f64,
    /// E3: share of the sent PCU the balance may be corrected by. Score 1 at twice this.
    pub link_tolerance: f64,
    /// E4: seconds into green before departures are expected.
    pub startup_s: f64,
    /// E4: seconds of stalled green for the full E4 stall score.
    pub stall_s: f64,
    /// E4: cap on the stall score; a queue that does not discharge has
    /// innocent causes too (gridlock, an incident, a blocked exit).
    pub stall_weight: f64,
    /// E4: seconds of history for red-light departures.
    pub red_window_s: f64,
    /// E4: red seconds below this count as this many, so one red runner in a
    /// short red is not a high rate.
    pub red_floor_s: f64,
    /// E4: red-light departures per minute of red tolerated: left, straight, right.
    pub red_tolerance_per_min: [f64; 3],
}

/// The contract's nested `EvidenceConfig` as the flat set the checks read.
impl From<&EvidenceConfig> for EvidenceParams {
    fn from(c: &EvidenceConfig) -> Self {
        let (q, f, e5, l, r) = (&c.quality, &c.frozen, &c.conservation, &c.link, &c.response);
        let red = &r.red_tolerance_per_min;
        EvidenceParams {
            brightness_edge: q.brightness_edge,
            blur_max: q.blur_max,
            confidence_min: q.confidence_min,
            track_breaks_max: q.track_breaks_max,
            displacement_max_px: q.displacement_max_px,
            quality_weight: q.weight,
            frozen_s: f.frozen_s,
            supply_s: f.supply_s,
            standing_weight: f.standing_weight,
            window: e5.window,
            block: e5.block,
            tolerance: e5.tolerance,
            free_flow_mps: l.free_flow_mps,
            fast_factor: l.fast_factor,
            slack_s: l.slack_s,
            min_spacing_m: l.min_spacing_m,
            congested_fraction: l.congested_fraction,
            saturation_flow: l.saturation_flow,
            link_window_s: l.window_s,
            min_volume_pcu: l.min_volume_pcu,
            link_tolerance: l.tolerance,
            startup_s: r.startup_s,
            stall_s: r.stall_s,
            stall_weight: r.stall_weight,
            red_window_s: r.red_window_s,
            red_floor_s: r.red_floor_s,
            red_tolerance_per_min: [red.left, red.straight, red.right],
        }
    }
}

/// The defaults are the contract's: `{"kind": "v1"}` and these agree.
impl Default for EvidenceParams {
    fn default() -> Self {
        EvidenceParams::from(&EvidenceConfig::default())
    }
}

impl EvidenceParams {
    pub fn validate(&self) -> Result<()> {
        let fraction = |x: f64| (0.0..=1.0).contains(&x);
        let ok = fraction(self.brightness_edge)
            && fraction(self.quality_weight)
            && fraction(self.standing_weight)
            && fraction(self.stall_weight)
            && self.frozen_s > 0.0
            && self.supply_s >= 0.0
            && self.tolerance > 0.0
            && self.block >= 1
            && self.window >= self.block
            && self.free_flow_mps > 0.0
            && self.fast_factor >= 1.0
            && self.slack_s >= 0.0
            && self.min_spacing_m > 0.0
            && self.congested_fraction > 0.0
            && self.congested_fraction <= 1.0
            && self.saturation_flow > 0.0
            && self.link_window_s > 0.0
            && self.min_volume_pcu > 0.0
            && self.link_tolerance > 0.0
            && self.startup_s >= 0.0
            && self.stall_s > 0.0
            && self.red_window_s > 0.0
            && self.red_floor_s > 0.0
            && self.red_tolerance_per_min.iter().all(|&x| x > 0.0);
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
    /// E3. None: an entry approach (nothing upstream), or the balance is not
    /// settled since the start or the last missing measurement here or upstream.
    pub cross_junction: Option<f64>,
    /// E4, queue not discharging under green. None: no measurement or command.
    pub stalled_green: Option<f64>,
    /// E4, departures under red. None: no measurement or command.
    pub red_departures: Option<f64>,
    /// E6, improbable silence. Needs the trust history, so Guard v1 fills it
    /// in (guard/v1.rs); always None from this tracker.
    pub silence: Option<f64>,
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

/// What an approach was shown during the last interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Signal {
    /// Green, for `elapsed` seconds at the end of the interval.
    Green { elapsed: f64 },
    /// Yellow or all-red clearing the approach's own phase.
    Clearing,
    /// Another phase has the junction. The corridor map lights every movement
    /// of an approach with its phase, so no movement may go (no free turns).
    Red,
}

/// The signal from the command the envelope issued for the approach's
/// junction; None when there is none. `t` is the end of the interval.
pub fn signal(ix: &CorridorIndex, approach: usize, last_commands: &[Option<Command>], t: f64) -> Option<Signal> {
    let j = ix.junction_of(approach);
    let c = last_commands[j].as_ref()?;
    let lit = ix.phases[j].get(c.phase_index).is_some_and(|p| p.contains(&approach));
    Some(match (lit, c.stage) {
        (true, Stage::Green) => Signal::Green { elapsed: t - c.phase_started_at },
        (true, _) => Signal::Clearing,
        (false, _) => Signal::Red,
    })
}

/// Whether the approach had green during the last interval. Yellow and
/// all-red are not green.
pub fn is_green(ix: &CorridorIndex, approach: usize, last_commands: &[Option<Command>]) -> bool {
    matches!(signal(ix, approach, last_commands, 0.0), Some(Signal::Green { .. }))
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

    /// Whether `self` is the interval right after `prev`.
    fn follows(&self, prev: &Counts, interval: f64) -> bool {
        (self.time - prev.time - interval).abs() < 1e-6
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
    /// E4 state.
    response: Response,
}

/// Runs E1-E5 for every approach, once per second.
pub struct EvidenceTracker {
    params: EvidenceParams,
    pcu: PcuWeights,
    /// Approach -> (movement, approach that movement feeds) inside the corridor.
    feeds: Vec<Vec<(Movement, usize)>>,
    /// Approach -> (upstream approach, its movement) feeding it: through
    /// traffic and turns from the side streets. Empty for entry approaches.
    feeders: Vec<Vec<(usize, Movement)>>,
    /// E3 per approach with feeders.
    links: Vec<Option<LinkBalance>>,
    tracks: Vec<Track>,
}

/// PCU the feeders of an approach sent toward it this interval; None if
/// any feeder sent no measurement. Shared by E3 and the virtual sensor.
pub fn sent_toward(measurements: &[Option<Measurement>], feeders: &[(usize, Movement)], pcu: &PcuWeights) -> Option<f64> {
    let mut sent = 0.0;
    for &(u, mv) in feeders {
        sent += measurements[u].as_ref()?.departures.get(mv).pcu(pcu);
    }
    Some(sent)
}

/// E3 input for link `d`; None if it or any feeder sent no measurement.
fn link_input(input: &GuardInput<'_>, d: usize, feeders: &[(usize, Movement)], pcu: &PcuWeights) -> Option<LinkInput> {
    let down = input.measurements[d].as_ref()?;
    Some(LinkInput {
        sent: sent_toward(input.measurements, feeders, pcu)?,
        arrived: down.arrivals.pcu(pcu),
        queue: down.queue_counts.pcu(pcu),
        zone_m: down.zone_length_m,
        interval: down.interval_s,
    })
}

/// Whether a link the approach feeds is full: Some(true) if any is, None if
/// any cannot be judged, Some(false) otherwise (also when it feeds only exits).
fn blocked(targets: &[(Movement, usize)], links: &[LinkReading], full: f64) -> Option<bool> {
    let fills: Vec<Option<f64>> = targets.iter().map(|&(_, d)| links[d].fill).collect();
    if fills.iter().any(|f| f.is_some_and(|f| f >= full)) {
        return Some(true);
    }
    if fills.iter().any(Option::is_none) {
        return None;
    }
    Some(false)
}

impl EvidenceTracker {
    pub fn new(ix: &CorridorIndex, pcu: PcuWeights, params: EvidenceParams) -> Result<Self> {
        params.validate()?;
        pcu.validate()?;
        let approaches = &ix.corridor.approaches;
        let feeds: Vec<Vec<(Movement, usize)>> = approaches
            .iter()
            .map(|a| {
                Movement::ALL
                    .into_iter()
                    .filter_map(|m| a.movements.get(m).and_then(|e| ix.approach(e)).map(|d| (m, d)))
                    .collect()
            })
            .collect();
        let mut feeders = vec![Vec::new(); approaches.len()];
        for (u, targets) in feeds.iter().enumerate() {
            for &(mv, d) in targets {
                feeders[d].push((u, mv));
            }
        }
        let links = approaches
            .iter()
            .zip(&feeders)
            .map(|(a, f)| (!f.is_empty()).then(|| LinkBalance::new(a.length_m, a.lanes)))
            .collect();
        Ok(EvidenceTracker {
            params,
            pcu,
            feeds,
            feeders,
            links,
            tracks: vec![Track::default(); approaches.len()],
        })
    }

    /// Upstream approaches and movements feeding approach `a`.
    pub fn feeders(&self, a: usize) -> &[(usize, Movement)] {
        &self.feeders[a]
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
        // links next: E4 on an approach needs to know whether the links it feeds are full
        let mut readings = vec![LinkReading::default(); self.links.len()];
        for (d, link) in self.links.iter_mut().enumerate() {
            if let Some(link) = link {
                let x = link_input(input, d, &self.feeders[d], &self.pcu);
                readings[d] = link.update(input.t, x, &self.params);
            }
        }
        (0..self.tracks.len()).map(|a| self.check(input, a, &readings)).collect()
    }

    fn check(&mut self, input: &GuardInput<'_>, a: usize, links: &[LinkReading]) -> Evidence {
        let p = &self.params;
        let track = &mut self.tracks[a];
        // a missing measurement holds E2 and E4 (no change, no growth) and breaks E5's chain
        let Some(m) = &input.measurements[a] else {
            track.residuals.clear();
            return Evidence::default();
        };
        let now = Counts::of(m);
        let follows = track.last.is_some_and(|l| now.follows(&l, m.interval_s));

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

        // E4: against the envelope's command
        let response = signal(input.corridor, a, input.last_commands, input.t).map(|signal| {
            let d = &now.departures;
            let x = ResponseInput {
                signal,
                interval: m.interval_s,
                queue_before: track.last.filter(|_| follows).map(|l| l.queue.total()),
                departures: [d.left.total(), d.straight.total(), d.right.total()],
                blocked: blocked(&self.feeds[a], links, p.congested_fraction),
            };
            track.response.update(input.t, &x, p)
        });

        // E5: only across consecutive intervals
        match track.last {
            Some(l) if follows => {
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
            cross_junction: links[a].score,
            stalled_green: response.map(|r| r.0),
            red_departures: response.map(|r| r.1),
            silence: None,
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

    const PCU: PcuWeights = PcuWeights { car: 1.0, motorcycle: 0.5, rickshaw: 1.0, qingqi: 1.0, bus: 3.0, truck: 3.0 };

    impl Rig {
        fn new() -> Self {
            let ix = CorridorIndex::new(two_junctions()).unwrap();
            let ev = EvidenceTracker::new(&ix, PCU, EvidenceParams::default()).unwrap();
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
        assert!(matches!(EvidenceTracker::new(&ix, PCU, bad), Err(Error::Config(_))));
        for bad in [
            EvidenceParams { tolerance: 0.0, ..Default::default() },
            EvidenceParams { min_spacing_m: 0.0, ..Default::default() },
            EvidenceParams { fast_factor: 0.9, ..Default::default() },
            EvidenceParams { red_tolerance_per_min: [2.0, 0.0, 2.0], ..Default::default() },
        ] {
            assert!(EvidenceTracker::new(&ix, PCU, bad).is_err());
        }
    }

    // ------------------------------------------------------------ E3, E4 in the tracker

    #[test]
    fn signal_follows_the_envelope_command() {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        assert_eq!(signal(&ix, A_W, &green_a_w(), 7.0), Some(Signal::Green { elapsed: 7.0 }));
        assert_eq!(signal(&ix, A_N, &green_a_w(), 7.0), Some(Signal::Red));
        assert_eq!(signal(&ix, A_W, &[cmd("A", 0, Stage::Yellow), None], 7.0), Some(Signal::Clearing));
        assert_eq!(signal(&ix, A_W, &[cmd("A", 0, Stage::AllRed), None], 7.0), Some(Signal::Clearing));
        assert_eq!(signal(&ix, A_N, &[cmd("A", 0, Stage::AllRed), None], 7.0), Some(Signal::Red));
        assert_eq!(signal(&ix, A_W, &[None, None], 7.0), None);
    }

    /// a_w releases 10 cars straight into b_w every 60 s; b_w sees `seen` of
    /// each platoon arrive 20 s later. All four approaches report.
    fn corridor_flow(rig: &mut Rig, from: i32, to: i32, seen: u32) -> Vec<Evidence> {
        let mut e = vec![];
        for t in from..to {
            let platoon = |t: i32| if t % 60 == 0 { 10 } else { 0 };
            let ms = vec![
                meas("a_w", f64::from(t), 2, 0, platoon(t), platoon(t)),
                meas("a_n", f64::from(t), 0, 0, 0, 0),
                meas("b_w", f64::from(t), 2, 0, if t >= 20 { platoon(t - 20) * seen / 10 } else { 0 }, 0),
                meas("b_n", f64::from(t), 0, 0, 0, 0),
            ];
            e = rig.step(f64::from(t), ms, &green_a_w());
        }
        e
    }

    #[test]
    fn e3_only_on_approaches_with_an_upstream() {
        let mut rig = Rig::new();
        let e = corridor_flow(&mut rig, 0, 300, 10);
        assert_eq!(e[A_W].cross_junction, None);
        assert_eq!(e[A_N].cross_junction, None);
        assert_eq!(e[B_W].cross_junction, Some(0.0));
    }

    #[test]
    fn e3_undercount_downstream_through_the_tracker() {
        let mut rig = Rig::new();
        assert_eq!(corridor_flow(&mut rig, 0, 400, 4)[B_W].cross_junction, Some(1.0));
    }

    #[test]
    fn e3_missing_upstream_measurement_restarts_the_link() {
        let mut rig = Rig::new();
        corridor_flow(&mut rig, 0, 300, 10);
        // a_w drops out for a second: b_w's balance is lost
        rig.step(300.0, vec![meas("b_w", 300.0, 2, 0, 0, 0)], &green_a_w());
        assert_eq!(corridor_flow(&mut rig, 301, 330, 10)[B_W].cross_junction, None);
    }

    #[test]
    fn e4_stall_waits_until_the_exit_is_known() {
        // a_w holds a queue under green with no departures; whether b_w (its
        // exit) is full is unknown until b_w's balance settles
        let mut rig = Rig::new();
        let mut e = vec![];
        for t in 0..230 {
            let ms = vec![meas("a_w", f64::from(t), 5, 5, 0, 0), meas("b_w", f64::from(t), 0, 0, 0, 0)];
            e = rig.step(f64::from(t), ms, &green_a_w());
            if t == 150 {
                assert_eq!(e[A_W].stalled_green, Some(0.0));
            }
        }
        assert_eq!(e[A_W].stalled_green, Some(EvidenceParams::default().stall_weight));
    }

    #[test]
    fn e4_red_departures_through_the_tracker() {
        // a_n is red while A shows phase 0, yet a vehicle leaves every 10 s
        let mut rig = Rig::new();
        let mut e = vec![];
        for t in 0..130 {
            let ms = vec![meas("a_n", f64::from(t), 3, 3, 0, u32::from(t % 10 == 0))];
            e = rig.step(f64::from(t), ms, &green_a_w());
        }
        assert_eq!(e[A_N].red_departures, Some(1.0));
        assert_eq!(e[A_N].stalled_green, Some(0.0));
        assert_eq!(e[A_W], Evidence::default());
    }

    // ------------------------------------------------------------ parity with the Python copy

    /// tests/fixtures/evidence_replay.json: inputs from a real SUMO fault run
    /// and the scores flowguard_sim.evidence_replay gives. Both must agree.
    #[test]
    fn matches_the_python_replay_fixture() {
        use serde_json::Value;
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../");
        let read = |p: &str| std::fs::read_to_string(format!("{root}{p}")).unwrap();
        let fixture: Value = serde_json::from_str(&read("tests/fixtures/evidence_replay.json")).unwrap();
        let defaults: EvidenceParams = serde_json::from_value(fixture["default_params"].clone()).unwrap();
        assert_eq!(defaults, EvidenceParams::default(), "Python and Rust default parameters differ");
        let params: EvidenceParams = serde_json::from_value(fixture["params"].clone()).unwrap();
        let pcu: PcuWeights = serde_json::from_value(fixture["pcu"].clone()).unwrap();
        let ix = CorridorIndex::new(serde_json::from_str(&read("data/net/corridor.json")).unwrap()).unwrap();
        let mut ev = EvidenceTracker::new(&ix, pcu, params).unwrap();

        // a count vector is six numbers, or 0 when all are zero
        let counts = |v: &Value| {
            let n = |i: usize| v.get(i).map_or(0, |x| x.as_u64().unwrap() as u32);
            ClassCounts { car: n(0), motorcycle: n(1), rickshaw: n(2), qingqi: n(3), bus: n(4), truck: n(5) }
        };
        let steps = fixture["steps"].as_array().unwrap();
        let expected = fixture["expected"].as_array().unwrap();
        assert_eq!(steps.len(), expected.len());
        let mut bad = Vec::new();
        for (step, want) in steps.iter().zip(expected) {
            let t = step["t"].as_f64().unwrap();
            let mut ms = vec![None; ix.corridor.approaches.len()];
            for row in step["m"].as_array().unwrap() {
                let a = row[0].as_u64().unwrap() as usize;
                let approach = &ix.corridor.approaches[a];
                ms[a] = Some(Measurement {
                    schema_version: "2.0".into(),
                    time: t,
                    interval_s: row[8].as_f64().unwrap(),
                    junction_id: approach.junction.clone(),
                    approach_id: approach.id.clone(),
                    source: Source::Sim,
                    zone_length_m: row[7].as_f64().unwrap(),
                    zone_counts: counts(&row[1]),
                    queue_counts: counts(&row[2]),
                    arrivals: counts(&row[3]),
                    departures: MovementCounts { left: counts(&row[4]), straight: counts(&row[5]), right: counts(&row[6]) },
                    quality: if row[9].is_null() { Quality::default() } else { serde_json::from_value(row[9].clone()).unwrap() },
                });
            }
            let commands: Vec<Option<Command>> = step["c"]
                .as_array()
                .unwrap()
                .iter()
                .zip(&ix.corridor.junctions)
                .map(|(c, j)| {
                    (!c.is_null()).then(|| Command {
                        schema_version: "2.0".into(),
                        time: t - 1.0,
                        junction_id: j.id.clone(),
                        phase_index: c[0].as_u64().unwrap() as usize,
                        phase_started_at: c[2].as_f64().unwrap(),
                        controller_name: "fixture".into(),
                        stage: serde_json::from_value(c[1].clone()).unwrap(),
                        target_phase: c[0].as_u64().unwrap() as usize,
                        requested_phase: None,
                        intervention: None,
                    })
                })
                .collect();
            let input = GuardInput { t, corridor: &ix, measurements: &ms, last_commands: &commands };
            for (a, (got, want)) in ev.update(&input).iter().zip(want.as_array().unwrap()).enumerate() {
                let got = [got.quality, got.frozen, got.conservation, got.cross_junction, got.stalled_green, got.red_departures];
                for (k, (g, w)) in got.iter().zip(want.as_array().unwrap()).enumerate() {
                    let same = match (g, w.as_f64()) {
                        (None, None) => w.is_null(),
                        (Some(g), Some(w)) => (g - w).abs() <= 1e-9,
                        _ => false,
                    };
                    if !same {
                        bad.push(format!("t={t} {} score {k}: rust {g:?}, fixture {w}", ix.corridor.approaches[a].id));
                    }
                }
            }
        }
        assert!(bad.is_empty(), "{} mismatches with the Python replay, first: {:#?}", bad.len(), &bad[..bad.len().min(10)]);
    }
}
