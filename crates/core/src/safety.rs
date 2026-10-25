//! Safety envelope: the only thing that decides what the signals show.
//!
//! Controllers only say which phase they would like green. The envelope turns
//! that into a legal sequence: minimum green, yellow then all-red on every
//! change, no phase red longer than max red, one phase lit at a time. Order of
//! rules each second (decisions.md D3):
//! 1. finish a yellow -> all-red -> green already under way; requests are
//!    ignored until done, and that is not counted as an intervention;
//! 2. if a red phase would pass max red once the clearance has run, start
//!    switching to the one red the longest;
//! 3. otherwise switch to the requested phase if the current green has run
//!    min green, else hold it.
//!
//! Min green is never cut short, not even for max red.

use std::collections::BTreeMap;

use crate::contracts::{Command, Intervention, SignalTiming, Stage, SCHEMA_VERSION};
use crate::error::{Error, Result};

impl SignalTiming {
    pub fn validate(&self) -> Result<()> {
        let all = [self.min_green_s, self.max_red_s, self.yellow_s, self.all_red_s];
        if all.iter().any(|v| !v.is_finite() || *v < 0.0) || self.yellow_s <= 0.0 || self.min_green_s <= 0.0 {
            return Err(Error::Config("timing values must be finite, >= 0, with yellow and min green > 0".into()));
        }
        // With two phases this is the condition under which a forced switch
        // never has to cut a green short.
        if self.max_red_s < self.min_green_s + self.yellow_s + 2.0 * self.all_red_s {
            return Err(Error::Config("max_red_s is too short for this min green and clearance".into()));
        }
        Ok(())
    }

    pub fn clearance_s(&self) -> f64 {
        self.yellow_s + self.all_red_s
    }
}

#[derive(Debug, Clone)]
struct Signal {
    junction_id: String,
    n_phases: usize,
    /// Phase that is green, or being cleared during yellow/all-red.
    phase: usize,
    stage: Stage,
    stage_started: f64,
    /// Phase we are changing to (== phase when settled).
    target: usize,
    /// (phase, time it went red) for phases not currently green, oldest
    /// insertion first. Order matters: ties in "red the longest" go to the
    /// first entry.
    red_since: Vec<(usize, f64)>,
}

#[derive(Debug, Clone)]
pub struct SafetyEnvelope {
    timing: SignalTiming,
    controller_name: String,
    signals: Vec<Signal>,
    interventions: BTreeMap<Intervention, u64>,
}

impl SafetyEnvelope {
    /// `junctions`: (id, number of phases), in output order. Every junction
    /// starts in phase 0 green at t = 0.
    pub fn new(junctions: &[(String, usize)], timing: SignalTiming, controller_name: &str) -> Result<Self> {
        timing.validate()?;
        let signals = junctions
            .iter()
            .map(|(id, n)| Signal {
                junction_id: id.clone(),
                n_phases: *n,
                phase: 0,
                stage: Stage::Green,
                stage_started: 0.0,
                target: 0,
                red_since: (1..*n).map(|p| (p, 0.0)).collect(),
            })
            .collect();
        Ok(SafetyEnvelope { timing, controller_name: controller_name.into(), signals, interventions: BTreeMap::new() })
    }

    pub fn interventions(&self) -> &BTreeMap<Intervention, u64> {
        &self.interventions
    }

    /// `requests`: junction id -> wanted phase. Missing = no preference.
    pub fn step(&mut self, t: f64, requests: &BTreeMap<String, usize>) -> Vec<Command> {
        let timing = self.timing;
        let mut out = Vec::with_capacity(self.signals.len());
        for s in &mut self.signals {
            let requested = requests.get(&s.junction_id).copied();
            let reason = step_one(s, &timing, t, requested);
            if let Some(r) = reason {
                *self.interventions.entry(r).or_insert(0) += 1;
            }
            out.push(Command {
                schema_version: SCHEMA_VERSION.into(),
                time: t,
                junction_id: s.junction_id.clone(),
                phase_index: s.phase,
                phase_started_at: s.stage_started,
                controller_name: self.controller_name.clone(),
                stage: s.stage,
                target_phase: s.target,
                requested_phase: requested,
                intervention: reason,
            });
        }
        out
    }
}

fn step_one(s: &mut Signal, timing: &SignalTiming, t: f64, requested: Option<usize>) -> Option<Intervention> {
    let mut reason = None;
    let mut wanted = requested;
    if wanted.is_some_and(|w| w >= s.n_phases) {
        reason = Some(Intervention::InvalidRequest);
        wanted = None;
    }

    // finish a change that is already under way
    if s.stage == Stage::Yellow && t - s.stage_started >= timing.yellow_s {
        s.stage = Stage::AllRed;
        s.stage_started = t;
        s.red_since.push((s.phase, t));
    }
    if s.stage == Stage::AllRed && t - s.stage_started >= timing.all_red_s {
        let target = s.target;
        s.red_since.retain(|&(p, _)| p != target);
        s.phase = target;
        s.stage = Stage::Green;
        s.stage_started = t;
    }

    if s.stage == Stage::Green {
        let green_for = t - s.stage_started;
        // the clearance still has to run after we start switching, so a
        // starving phase is caught clearance_s before it hits max red
        let mut starving: Option<(usize, f64)> = None;
        for &(p, since) in &s.red_since {
            let due = t - since + timing.clearance_s() >= timing.max_red_s;
            if due && starving.is_none_or(|(_, best)| since < best) {
                starving = Some((p, since));
            }
        }
        let wants_change = wanted.is_some_and(|w| w != s.phase);
        let target = match starving {
            Some((p, _)) => Some(p),
            None if wants_change => wanted,
            None => None,
        };

        if let Some(target) = target {
            if green_for < timing.min_green_s {
                if wants_change {
                    reason = Some(Intervention::MinGreen);
                }
            } else {
                s.stage = Stage::Yellow;
                s.stage_started = t;
                s.target = target;
                if wanted != Some(target) {
                    reason = Some(Intervention::MaxRed);
                }
            }
        }
    }
    reason
}

#[cfg(test)]
mod tests {
    use super::*;

    const TIMING: SignalTiming = SignalTiming { min_green_s: 10.0, max_red_s: 60.0, yellow_s: 3.0, all_red_s: 2.0 };

    /// Small deterministic generator so the tests need no extra crate.
    pub struct Rng(pub u64);
    impl Rng {
        pub fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
    }

    fn drive(mut request_at: impl FnMut() -> Option<usize>, seconds: usize) -> (SafetyEnvelope, Vec<Command>) {
        let mut env = SafetyEnvelope::new(&[("J".into(), 2)], TIMING, "test").unwrap();
        let mut cmds = Vec::new();
        for t in 0..seconds {
            let mut req = BTreeMap::new();
            if let Some(p) = request_at() {
                req.insert("J".to_string(), p);
            }
            cmds.push(env.step(t as f64, &req).remove(0));
        }
        (env, cmds)
    }

    fn random_requests(seed: u64) -> impl FnMut() -> Option<usize> {
        let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
        move || match rng.next() % 3 {
            0 => Some(0),
            1 => Some(1),
            _ => None,
        }
    }

    fn count(env: &SafetyEnvelope, i: Intervention) -> u64 {
        env.interventions().get(&i).copied().unwrap_or(0)
    }

    /// Longest unbroken run of seconds where `phase` is not lit (green or yellow).
    fn longest_red(cmds: &[Command], phase: usize) -> usize {
        let (mut red, mut longest) = (0, 0);
        for c in cmds {
            let lit = c.phase_index == phase && c.stage != Stage::AllRed;
            red = if lit { 0 } else { red + 1 };
            longest = longest.max(red);
        }
        longest
    }

    #[test]
    fn rejects_green_shorter_than_min() {
        let (env, cmds) = drive(|| Some(1), 30);
        assert!(cmds[..10].iter().all(|c| c.stage == Stage::Green && c.phase_index == 0));
        assert_eq!(cmds[10].stage, Stage::Yellow);
        assert_eq!(count(&env, Intervention::MinGreen), 10);
    }

    #[test]
    fn every_green_lasts_at_least_min_green() {
        for seed in 1..20 {
            let (_, cmds) = drive(random_requests(seed), 3000);
            let (mut run, mut lengths) = (0, vec![]);
            for w in cmds.windows(2) {
                run += usize::from(w[0].stage == Stage::Green);
                if w[0].stage == Stage::Green && w[1].stage != Stage::Green {
                    lengths.push(run);
                    run = 0;
                }
            }
            assert!(!lengths.is_empty() && lengths.iter().all(|&l| l >= 10), "seed {seed}");
        }
    }

    #[test]
    fn yellow_and_all_red_before_every_change() {
        let clearance = [Stage::Yellow, Stage::Yellow, Stage::Yellow, Stage::AllRed, Stage::AllRed];
        for seed in 1..20 {
            let (_, cmds) = drive(random_requests(seed), 3000);
            let mut changes = 0;
            for i in 6..cmds.len() {
                if cmds[i].stage == Stage::Green && cmds[i - 1].stage != Stage::Green {
                    changes += 1;
                    let stages: Vec<Stage> = cmds[i - 5..i].iter().map(|c| c.stage).collect();
                    assert_eq!(stages, clearance);
                    assert_ne!(cmds[i].phase_index, cmds[i - 6].phase_index);
                }
            }
            assert!(changes > 10, "seed {seed}");
        }
    }

    #[test]
    fn max_red_forces_a_switch() {
        let (env, cmds) = drive(|| Some(0), 600); // never lets the side street go
        assert!(count(&env, Intervention::MaxRed) > 0);
        assert!(longest_red(&cmds, 1) as f64 <= TIMING.max_red_s);
    }

    #[test]
    fn max_red_holds_under_random_requests() {
        for seed in 1..20 {
            let (_, cmds) = drive(random_requests(seed), 3000);
            for phase in 0..2 {
                assert!(longest_red(&cmds, phase) as f64 <= TIMING.max_red_s, "seed {seed} phase {phase}");
            }
        }
    }

    #[test]
    fn bad_phase_index_is_ignored() {
        let (env, cmds) = drive(|| Some(7), 20);
        assert!(cmds.iter().all(|c| c.phase_index == 0 && c.stage == Stage::Green));
        assert_eq!(count(&env, Intervention::InvalidRequest), 20);
    }

    #[test]
    fn timing_validation() {
        assert!(TIMING.validate().is_ok());
        assert!(SignalTiming { max_red_s: 16.0, ..TIMING }.validate().is_err());
        assert!(SignalTiming { yellow_s: 0.0, ..TIMING }.validate().is_err());
        assert!(SignalTiming { all_red_s: f64::NAN, ..TIMING }.validate().is_err());
    }
}
