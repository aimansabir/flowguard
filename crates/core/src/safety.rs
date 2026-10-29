//! Safety envelope: the only thing that decides what the signals show.
//!
//! Controllers only say which phase they would like green. The envelope turns
//! that into a legal sequence: minimum green, yellow then all-red on every
//! change, no phase red longer than max red, one phase lit at a time. Order of
//! rules each second (decisions.md D3):
//! 1. finish a yellow -> all-red -> green already under way; requests are
//!    ignored until done, and that is not counted as an intervention;
//! 2. if serving the red phases one after another, red the longest first,
//!    each at min green, would take any of them past max red, start switching
//!    to the one red the longest (with two phases: when the red one would pass
//!    max red once the clearance has run);
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

    /// Longest a phase can be kept red when each of the other `n_phases - 1`
    /// is served once at min green: its own all-red, then min green + clearance
    /// for each of the others. A max red below this cannot be honoured.
    pub fn worst_red_s(&self, n_phases: usize) -> f64 {
        self.all_red_s + n_phases.saturating_sub(1) as f64 * (self.min_green_s + self.clearance_s())
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
        for (id, n) in junctions {
            if timing.max_red_s < timing.worst_red_s(*n) {
                return Err(Error::Config(format!(
                    "{id}: max_red_s {} s is below {} s, the red a phase needs while the other {} are each \
                     served once at min green",
                    timing.max_red_s,
                    timing.worst_red_s(*n),
                    n - 1
                )));
            }
        }
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
        // Red phases in the order a forced switch serves them: red the longest
        // first (stable, so ties keep insertion order). The k-th of them gets
        // green no sooner than one clearance plus k x (min green + clearance)
        // after the current green may end, so it is due once that would reach
        // max red. Switching to the first as soon as any is due keeps every
        // phase within max red (checked feasible in `new`); with two phases
        // this is "red phase due clearance_s before max red".
        let mut queue = s.red_since.clone();
        queue.sort_by(|a, b| a.1.total_cmp(&b.1));
        let free_at = t + (timing.min_green_s - green_for).max(0.0);
        let slot = timing.min_green_s + timing.clearance_s();
        let due = queue.iter().enumerate().any(|(k, &(_, since))| {
            free_at + timing.clearance_s() + k as f64 * slot - since >= timing.max_red_s
        });
        let starving = queue.first().filter(|_| due);
        let wants_change = wanted.is_some_and(|w| w != s.phase);
        // A switch to the wanted phase commits its min green and a clearance
        // before anything else; take it only if every other red phase, served
        // after it in the same order, still makes max red. Otherwise serve the
        // one red the longest (with two phases the wanted one is always it).
        let fits = |w: usize| {
            queue.iter().filter(|&&(p, _)| p != w).enumerate().all(|(k, &(_, since))| {
                free_at + timing.clearance_s() + (k + 1) as f64 * slot - since < timing.max_red_s
            })
        };
        let target = match (starving, wanted) {
            (Some(&(p, _)), _) => Some(p),
            (None, Some(w)) if wants_change && fits(w) => Some(w),
            (None, Some(_)) if wants_change => queue.first().map(|&(p, _)| p),
            _ => None,
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

    fn drive_n(
        n: usize,
        timing: SignalTiming,
        mut request_at: impl FnMut() -> Option<usize>,
        seconds: usize,
    ) -> Vec<Command> {
        let mut env = SafetyEnvelope::new(&[("J".into(), n)], timing, "test").unwrap();
        (0..seconds)
            .map(|t| {
                let req: BTreeMap<String, usize> = request_at().map(|p| ("J".to_string(), p)).into_iter().collect();
                env.step(t as f64, &req).remove(0)
            })
            .collect()
    }

    #[test]
    fn max_red_holds_with_four_phases_when_the_controller_never_lets_go() {
        // three red phases fall due together; each must still be served within max red
        let cmds = drive_n(4, TIMING, || Some(0), 1200);
        for phase in 1..4 {
            assert!(longest_red(&cmds, phase) as f64 <= TIMING.max_red_s, "phase {phase}: {}", longest_red(&cmds, phase));
        }
    }

    #[test]
    fn max_red_holds_for_more_phases_at_the_tightest_valid_max_red() {
        for n in 3..=5 {
            let timing = SignalTiming { max_red_s: TIMING.worst_red_s(n), ..TIMING };
            for seed in 1..20u64 {
                let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
                let cmds = drive_n(n, timing, move || (rng.next() % (n as u64 + 1)).try_into().ok().filter(|&p| p < n), 3000);
                for phase in 0..n {
                    let red = longest_red(&cmds, phase) as f64;
                    assert!(red <= timing.max_red_s, "{n} phases, seed {seed}, phase {phase}: red {red}");
                }
            }
            let always_first = drive_n(n, timing, || Some(0), 2000);
            for phase in 1..n {
                assert!(longest_red(&always_first, phase) as f64 <= timing.max_red_s, "{n} phases, phase {phase}");
            }
        }
    }

    #[test]
    fn max_red_holds_when_the_controller_keeps_choosing_other_phases() {
        // A controller that only ever wants the main road (0 and 1, whichever is not
        // green) and never the side streets: a voluntary switch to 0 or 1 must not
        // be taken when it would push a waiting side street past max red.
        for (max_red, n) in [(60.0, 4), (90.0, 4), (90.0, 3), (75.0, 5)] {
            let timing = SignalTiming { max_red_s: max_red, ..TIMING };
            let mut env = SafetyEnvelope::new(&[("J".into(), n)], timing, "test").unwrap();
            let mut cmds: Vec<Command> = Vec::new();
            for t in 0..4000 {
                let want = match cmds.last() {
                    Some(c) if c.phase_index == 0 => 1,
                    _ => 0,
                };
                cmds.push(env.step(t as f64, &BTreeMap::from([("J".to_string(), want)])).remove(0));
            }
            for phase in 0..n {
                let red = longest_red(&cmds, phase) as f64;
                assert!(red <= max_red, "max red {max_red}, {n} phases, phase {phase}: red {red}");
            }
            for seed in 1..30u64 {
                let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
                let cmds = drive_n(n, timing, move || Some((rng.next() % n as u64) as usize), 3000);
                for phase in 0..n {
                    let red = longest_red(&cmds, phase) as f64;
                    assert!(red <= max_red, "max red {max_red}, {n} phases, seed {seed}, phase {phase}: red {red}");
                }
            }
        }
    }

    #[test]
    fn max_red_must_leave_room_for_every_phase() {
        // 4 phases: all-red 2 + 3 x (min green 10 + clearance 5) = 47 s
        assert_eq!(TIMING.worst_red_s(4), 47.0);
        let short = SignalTiming { max_red_s: 46.0, ..TIMING };
        assert!(SafetyEnvelope::new(&[("J".into(), 4)], short, "t").is_err());
        assert!(SafetyEnvelope::new(&[("J".into(), 2)], short, "t").is_ok());
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
