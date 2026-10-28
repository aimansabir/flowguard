//! Guard v1: evidence (G1) -> one score (G2 combine) -> trust state (G2)
//! -> virtual queue (G3) -> the ApproachState controllers read. What each
//! state means for the output (docs/decisions.md D26):
//!
//! | trust     | queue_estimate      | margin                              | override_flag |
//! |-----------|---------------------|-------------------------------------|---------------|
//! | TRUSTED   | measured            | 0                                   | false         |
//! | SUSPECT   | measured            | |measured - virtual| + virtual margin | false         |
//! | FAULTY    | virtual             | virtual margin                      | true          |
//!
//! `margin` is a +/- PCU range around `queue_estimate` that a conservative
//! controller can use (control spec C4). With `extend_queue_beyond_zone`, a
//! TRUSTED approach whose zone is full reports the virtual queue when it is
//! longer than the camera can see, with `override_flag` set.

use crate::contracts::{
    ApproachState, CheckWeights, CombineConfig, EvidenceConfig, PcuWeights, SilenceEvidence, TrustConfig,
    TrustState, VirtualSensorConfig,
};
use crate::corridor::CorridorIndex;
use crate::error::{Error, Result};
use crate::guard::evidence::{self, EvidenceParams, EvidenceTracker, Signal};
use crate::guard::silence::{self, Silence};
use crate::guard::trust::{self, Trust};
use crate::guard::virtual_sensor::{Anchor, QueueInput, QueueParams, Supply, VirtualQueue};
use crate::guard::{Guard, GuardInput};

pub struct GuardV1 {
    evidence: EvidenceTracker,
    weights: CheckWeights,
    trust_cfg: TrustConfig,
    vs: VirtualSensorConfig,
    pcu: PcuWeights,
    /// For "zone full": what share of the zone's storage a queue fills.
    congested_fraction: f64,
    min_spacing_m: f64,
    lanes: Vec<f64>,
    silence_cfg: SilenceEvidence,
    silence: Vec<Silence>,
    trust: Vec<Trust>,
    queues: Vec<VirtualQueue>,
    /// Last interval seen per approach, to advance the estimate without a measurement.
    interval: Vec<f64>,
    /// Combined score per approach last second, for tests and diagnostics.
    scores: Vec<Option<f64>>,
}

impl GuardV1 {
    pub fn new(
        ix: &CorridorIndex,
        pcu: PcuWeights,
        evidence: &EvidenceConfig,
        combine: &CombineConfig,
        trust: &TrustConfig,
        vs: &VirtualSensorConfig,
    ) -> Result<Self> {
        let params = EvidenceParams::from(evidence);
        trust::validate(trust, &combine.weights)?;
        silence::validate(&evidence.silence)?;
        if !(vs.arrival_window_s > 0.0 && vs.margin_growth_pcu_per_s >= 0.0) {
            return Err(Error::Config("invalid virtual sensor parameters".into()));
        }
        let tracker = EvidenceTracker::new(ix, pcu, params)?;
        let approaches = &ix.corridor.approaches;
        let queues = approaches
            .iter()
            .map(|a| {
                let lanes = f64::from(a.lanes);
                VirtualQueue::new(QueueParams {
                    travel_s: a.length_m / params.free_flow_mps,
                    discharge_pcu_per_s: params.saturation_flow * lanes,
                    storage_pcu: a.length_m * lanes / params.min_spacing_m,
                    arrival_window_s: vs.arrival_window_s,
                    margin_growth: vs.margin_growth_pcu_per_s,
                })
            })
            .collect();
        let n = approaches.len();
        Ok(GuardV1 {
            evidence: tracker,
            weights: combine.weights,
            trust_cfg: *trust,
            vs: *vs,
            pcu,
            congested_fraction: params.congested_fraction,
            min_spacing_m: params.min_spacing_m,
            lanes: approaches.iter().map(|a| f64::from(a.lanes)).collect(),
            silence_cfg: evidence.silence,
            silence: vec![Silence::default(); n],
            trust: vec![Trust::default(); n],
            queues,
            interval: vec![1.0; n],
            scores: vec![None; n],
        })
    }

    /// Trust state of an approach (also while it has no measurement).
    pub fn trust_state(&self, approach: usize) -> TrustState {
        self.trust[approach].state
    }

    /// Virtual queue estimate of an approach, PCU.
    pub fn virtual_queue(&self, approach: usize) -> f64 {
        self.queues[approach].queue()
    }

    /// Combined evidence score of an approach last second.
    pub fn score(&self, approach: usize) -> Option<f64> {
        self.scores[approach]
    }
}

impl Guard for GuardV1 {
    fn name(&self) -> &str {
        "v1"
    }

    fn rate(&mut self, input: &GuardInput<'_>, states: &mut [Option<ApproachState>]) {
        // G1 and G2: evidence, one score, trust (held without a measurement).
        // E6 needs each approach's trusted arrival rate, which the virtual
        // sensor keeps (from earlier seconds).
        let mut rated = self.evidence.update(input);
        for (a, e) in rated.iter_mut().enumerate() {
            if let Some(m) = &input.measurements[a] {
                let rate = self.queues[a].settled_arrival_rate();
                e.silence = self.silence[a].update(silence::is_empty(m), m.interval_s, rate, &self.silence_cfg);
            }
        }
        for (a, e) in rated.iter().enumerate() {
            self.scores[a] = trust::combine(e, &self.weights);
            if let Some(m) = &input.measurements[a] {
                self.interval[a] = m.interval_s;
                self.trust[a].update(self.scores[a], trust::can_confirm_clean(e), m.interval_s, &self.trust_cfg);
            }
        }

        // G3: every approach, every second, measured or not
        for (a, state) in states.iter().enumerate() {
            let feeders = self.evidence.feeders(a);
            let supply = if feeders.is_empty() {
                Supply::Entry
            } else if feeders.iter().any(|&(u, _)| self.trust[u].state == TrustState::Faulty) {
                Supply::Unknown
            } else {
                evidence::sent_toward(input.measurements, feeders, &self.pcu).map_or(Supply::Unknown, Supply::Sent)
            };
            let clean = self.scores[a].is_some_and(|s| s < self.trust_cfg.suspect_score);
            let anchor = match (&input.measurements[a], state) {
                (Some(m), Some(s)) if self.trust[a].state == TrustState::Trusted && clean => Some(Anchor {
                    queue: s.measured_queue_pcu,
                    arrivals: s.arrivals_pcu,
                    zone_full: s.measured_queue_pcu
                        >= self.congested_fraction * m.zone_length_m * self.lanes[a] / self.min_spacing_m,
                }),
                _ => None,
            };
            let green = matches!(
                evidence::signal(input.corridor, a, input.last_commands, input.t),
                Some(Signal::Green { .. })
            );
            let x = QueueInput { interval: self.interval[a], green, supply, anchor };
            self.queues[a].update(input.t, &x);
        }

        // what the controllers see
        for (a, state) in states.iter_mut().enumerate() {
            let Some(s) = state else { continue };
            let (virtual_q, virtual_margin) = (self.queues[a].queue(), self.queues[a].margin());
            s.trust_state = self.trust[a].state;
            match s.trust_state {
                TrustState::Trusted => {
                    if self.vs.extend_queue_beyond_zone && virtual_q > s.measured_queue_pcu {
                        s.queue_estimate = virtual_q;
                        s.override_flag = true;
                    }
                }
                TrustState::Suspect => {
                    s.margin = (s.measured_queue_pcu - virtual_q).abs() + virtual_margin;
                }
                TrustState::Faulty => {
                    s.queue_estimate = virtual_q;
                    s.margin = virtual_margin;
                    s.override_flag = true;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contracts::{ClassCounts, Command, Measurement, MovementCounts, Quality, Source, Stage};
    use crate::corridor::tests::two_junctions;
    use crate::state;

    // two_junctions: a_w (A) feeds b_w (B) straight; a_n, b_n are entries.
    const A_W: usize = 0;
    const A_N: usize = 1;
    const B_W: usize = 2;
    const PCU: PcuWeights = PcuWeights { car: 1.0, motorcycle: 0.5, rickshaw: 1.0, qingqi: 1.0, bus: 3.0, truck: 3.0 };

    fn cars(n: u32) -> ClassCounts {
        ClassCounts { car: n, ..Default::default() }
    }

    fn meas(approach: &str, t: f64, zone: u32, queue: u32, arr: u32, dep: u32) -> Measurement {
        Measurement {
            schema_version: "2.0".into(),
            time: t,
            interval_s: 1.0,
            junction_id: if approach.starts_with('a') { "A" } else { "B" }.into(),
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

    fn cmd(junction: &str, phase: usize) -> Option<Command> {
        Some(Command {
            schema_version: "2.0".into(),
            time: 0.0,
            junction_id: junction.into(),
            phase_index: phase,
            phase_started_at: 0.0,
            controller_name: "test".into(),
            stage: Stage::Green,
            target_phase: phase,
            requested_phase: None,
            intervention: None,
        })
    }

    struct Rig {
        ix: CorridorIndex,
        guard: GuardV1,
    }

    impl Rig {
        fn new(trust: TrustConfig) -> Self {
            let ix = CorridorIndex::new(two_junctions()).unwrap();
            let guard = GuardV1::new(
                &ix,
                PCU,
                &EvidenceConfig::default(),
                &CombineConfig::default(),
                &trust,
                &VirtualSensorConfig::default(),
            )
            .unwrap();
            Rig { ix, guard }
        }

        /// One second: rates the given measurements as the engine does.
        fn step(&mut self, t: f64, ms: Vec<Measurement>, commands: &[Option<Command>]) -> Vec<Option<ApproachState>> {
            let mut slots = vec![None; 4];
            for m in ms {
                let a = self.ix.approach(&m.approach_id).unwrap();
                slots[a] = Some(m);
            }
            let mut states: Vec<Option<ApproachState>> = slots
                .iter()
                .enumerate()
                .map(|(a, m)| m.as_ref().map(|m| state::assess(t, m, a, &self.ix, &PCU)))
                .collect();
            let input = GuardInput { t, corridor: &self.ix, measurements: &slots, last_commands: commands };
            self.guard.rate(&input, &mut states);
            states
        }
    }

    fn green_a_n() -> [Option<Command>; 2] {
        [cmd("A", 1), cmd("B", 1)]
    }

    #[test]
    fn healthy_approach_stays_trusted_with_its_measured_queue() {
        let mut rig = Rig::new(TrustConfig::default());
        for t in 0..200 {
            // a_n under green: one car in and one out every 2 s, queue 2
            let moving = u32::from(t % 2 == 0);
            let s = rig.step(f64::from(t), vec![meas("a_n", f64::from(t), 2, 2, moving, moving)], &green_a_n());
            let s = s[A_N].as_ref().unwrap();
            assert_eq!((s.trust_state, s.queue_estimate, s.margin, s.override_flag), (TrustState::Trusted, 2.0, 0.0, false));
        }
    }

    #[test]
    fn frozen_camera_goes_trusted_suspect_faulty_and_back() {
        // a_n: healthy 100 s (queue 4, a car in and out every 2 s), then the
        // feed freezes under green, then it recovers
        let mut rig = Rig::new(TrustConfig::default());
        let healthy = |t: i32| {
            let moving = u32::from(t % 2 == 0);
            meas("a_n", f64::from(t), 4, 4, moving, moving)
        };
        let mut seen = vec![];
        for t in 0..400 {
            // frozen on a second with a car in and out: counts that cannot repeat
            let m = if (100..250).contains(&t) { healthy(98) } else { healthy(t) };
            let m = Measurement { time: f64::from(t), ..m };
            let s = rig.step(f64::from(t), vec![m], &green_a_n())[A_N].clone().unwrap();
            if seen.last() != Some(&s.trust_state) {
                seen.push(s.trust_state);
            }
            if s.trust_state == TrustState::Faulty {
                // the virtual queue, not the frozen camera
                assert!(s.override_flag);
                assert_eq!(s.queue_estimate, rig.guard.virtual_queue(A_N));
            }
        }
        assert_eq!(seen, vec![TrustState::Trusted, TrustState::Suspect, TrustState::Faulty, TrustState::Trusted]);
    }

    #[test]
    fn faulty_estimate_comes_from_upstream_departures() {
        // b_w goes black under green while a_w keeps sending a car every
        // 4 s toward it: the virtual queue fills from those departures
        let trust = TrustConfig { suspect_s: 1.0, faulty_s: 1.0, ..Default::default() };
        let mut rig = Rig::new(trust);
        // B shows b_w red, so nothing discharges; A shows a_w green
        let commands = [cmd("A", 0), cmd("B", 1)];
        let mut last = None;
        for t in 0..400 {
            let sent = u32::from(t % 4 == 0);
            let ms = vec![meas("a_w", f64::from(t), 3, 0, sent, sent), meas("b_w", f64::from(t), 0, 0, 0, 0)];
            last = rig.step(f64::from(t), ms, &commands)[B_W].clone();
        }
        let b_w = last.unwrap();
        assert_eq!(b_w.trust_state, TrustState::Faulty, "score {:?}", rig.guard.score(B_W));
        assert!(b_w.override_flag);
        assert!(b_w.queue_estimate > 20.0, "{}", b_w.queue_estimate);
        assert!(b_w.margin > 0.0);
    }

    #[test]
    fn suspect_keeps_the_camera_with_a_wider_margin() {
        let trust = TrustConfig { faulty_s: 1e9, ..Default::default() };
        let mut rig = Rig::new(trust);
        let mut last = None;
        for t in 0..60 {
            // a queue of 6 that never discharges under green: weak evidence only
            last = rig.step(f64::from(t), vec![meas("a_n", f64::from(t), 6, 6, 0, 0)], &green_a_n())[A_N].clone();
        }
        let s = last.unwrap();
        assert_eq!(s.trust_state, TrustState::Suspect);
        assert_eq!(s.queue_estimate, 6.0);
        assert!(!s.override_flag);
        assert!(s.margin > 0.0);
    }

    #[test]
    fn missing_measurement_holds_trust_and_advances_the_estimate() {
        let mut rig = Rig::new(TrustConfig::default());
        for t in 0..50 {
            rig.step(f64::from(t), vec![meas("a_n", f64::from(t), 3, 3, 0, 0)], &[cmd("A", 0), None]);
        }
        let before = rig.guard.trust_state(A_N);
        let q = rig.guard.virtual_queue(A_N);
        // a_n red: no measurement for 30 s; the estimate keeps running
        for t in 50..80 {
            let s = rig.step(f64::from(t), vec![], &[cmd("A", 0), None]);
            assert!(s[A_N].is_none());
        }
        assert_eq!(rig.guard.trust_state(A_N), before);
        assert_eq!(rig.guard.virtual_queue(A_N), q);
    }

    #[test]
    fn approaches_rate_independently() {
        let trust = TrustConfig { suspect_s: 1.0, faulty_s: 1.0, ..Default::default() };
        let mut rig = Rig::new(trust);
        let commands = [cmd("A", 0), cmd("B", 1)];
        for t in 0..400 {
            let sent = u32::from(t % 4 == 0);
            let ms = vec![
                meas("a_w", f64::from(t), 3, 0, sent, sent),
                meas("b_w", f64::from(t), 0, 0, 0, 0),
                meas("a_n", f64::from(t), 0, 0, 0, 0),
            ];
            rig.step(f64::from(t), ms, &commands);
        }
        assert_eq!(rig.guard.trust_state(B_W), TrustState::Faulty);
        assert_eq!(rig.guard.trust_state(A_W), TrustState::Trusted);
        assert_eq!(rig.guard.trust_state(A_N), TrustState::Trusted);
    }

    /// a_n under green: `healthy` seconds with a car through every `every` s,
    /// then all zeros; returns the first second it is FAULTY.
    fn entry_goes_silent(healthy: i32, every: i32, until: i32) -> Option<i32> {
        let mut rig = Rig::new(TrustConfig::default());
        let mut faulty = None;
        for t in 0..until {
            let car = u32::from(t < healthy && t % every == 0);
            let s = rig.step(f64::from(t), vec![meas("a_n", f64::from(t), 0, 0, car, car)], &green_a_n());
            if faulty.is_none() && s[A_N].as_ref().unwrap().trust_state == TrustState::Faulty {
                faulty = Some(t);
            }
        }
        faulty
    }

    #[test]
    fn blackout_on_a_busy_entry_road_becomes_faulty() {
        // the silence starts after the last car (t = 396); the 300 trusted
        // seconds before it held 75 cars, 0.25 PCU/s: score 0.9 (p below
        // 10^-4.8) after 45 s, FAULTY 10 s later, t = 450. No other check
        // sees an all-zero entry camera.
        assert_eq!(entry_goes_silent(400, 4, 600), Some(450));
    }

    #[test]
    fn quiet_entry_road_going_empty_stays_trusted() {
        // 0.02 PCU/s: five empty minutes are plausible (p of about 0.002)
        assert_eq!(entry_goes_silent(300, 50, 600), None);
    }

    #[test]
    fn silence_is_not_judged_before_a_full_trusted_window() {
        // the first car alone would make the rate 1 PCU/s
        assert_eq!(entry_goes_silent(1, 50, 299), None);
    }

    #[test]
    fn rejects_bad_config() {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        let bad_vs = VirtualSensorConfig { arrival_window_s: 0.0, ..Default::default() };
        let r = GuardV1::new(&ix, PCU, &EvidenceConfig::default(), &CombineConfig::default(), &TrustConfig::default(), &bad_vs);
        assert!(r.is_err());
        let bad_trust = TrustConfig { faulty_score: 1.5, ..Default::default() };
        let r = GuardV1::new(&ix, PCU, &EvidenceConfig::default(), &CombineConfig::default(), &bad_trust, &VirtualSensorConfig::default());
        assert!(r.is_err());
    }
}
