use std::collections::{BTreeMap, VecDeque};

use super::{Controller, DecisionInput};
use crate::contracts::{SignalTiming, TrustState};
use crate::corridor::CorridorIndex;
use crate::error::{Error, Result};

/// Fixed-time plan with splits re-computed from measured arrivals (D35).
///
/// Starts from a fixed plan (greens and offsets, normally Webster with the
/// green wave) and keeps its cycle and offsets, so coordination is kept.
/// Every `update_s` the greens are re-split with Webster's formula (as
/// `flowguard_sim/webster.py`): flow ratio of phase k
///     y_k = max over junctions and the phase's approaches of
///           arrival rate (PCU/h) / (lanes x saturation flow),
/// target green_k = (sum of greens) x y_k / sum(y). Each green moves at most
/// `max_step_s` toward its target, never below min green, rounded to whole
/// seconds, and the greens are then balanced so their sum, and so the cycle,
/// is unchanged. A split whose reds would pass max red is not used. A
/// junction takes a new split when its own cycle starts again, never in the
/// middle of one.
///
/// Arrival rate of an approach = PCU entering its camera zone over the last
/// `window_s`, counting only seconds it was TRUSTED; an approach that is not
/// TRUSTED at an update keeps its last trusted rate. Until every approach
/// has a rate the plan is not changed. Limit: a camera zone that is full
/// admits only what the queue moves up, so on a saturated approach the
/// measured rate is the served flow, not the demand.
pub struct AdaptiveSplit {
    junction_ids: Vec<String>,
    offsets: Vec<f64>,
    cycle: f64,
    clearance_s: f64,
    min_green_s: f64,
    worst_red_room: f64,
    plan: Vec<f64>,
    /// Junction index -> greens in use this cycle.
    active: Vec<Vec<f64>>,
    last_pos: Vec<Option<f64>>,
    update_s: f64,
    window_s: f64,
    max_step_s: f64,
    sat_pcu_h_lane: f64,
    lanes: Vec<f64>,
    phases: Vec<Vec<Vec<usize>>>,
    /// Approach index -> (time, interval, PCU) on trusted seconds in the window.
    window: Vec<VecDeque<(f64, f64, f64)>>,
    rate: Vec<Option<f64>>,
    last_t: Option<f64>,
    next_update: f64,
}

impl AdaptiveSplit {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        corridor: &CorridorIndex,
        greens: &[f64],
        offsets_s: &BTreeMap<String, f64>,
        timing: &SignalTiming,
        update_s: f64,
        window_s: f64,
        max_step_s: f64,
        saturation_pcu_per_h_per_lane: f64,
    ) -> Result<Self> {
        for j in &corridor.corridor.junctions {
            if j.phases.len() != greens.len() {
                return Err(Error::Config(format!("{} has {} phases, plan has {}", j.id, j.phases.len(), greens.len())));
            }
        }
        if greens.iter().any(|g| !g.is_finite() || *g < timing.min_green_s) {
            return Err(Error::Config("adaptive split greens must be finite and >= min green".into()));
        }
        let pos = |x: f64| x.is_finite() && x > 0.0;
        if !(pos(update_s) && pos(window_s) && pos(saturation_pcu_per_h_per_lane) && max_step_s.is_finite() && max_step_s >= 0.0) {
            return Err(Error::Config(
                "adaptive split needs update_s, window_s, saturation flow > 0 and max_step_s >= 0".into(),
            ));
        }
        let junction_ids: Vec<String> = corridor.corridor.junctions.iter().map(|j| j.id.clone()).collect();
        for (id, off) in offsets_s {
            if !junction_ids.contains(id) || !off.is_finite() {
                return Err(Error::Config(format!("adaptive split offset for {id}: unknown junction or not finite")));
            }
        }
        let clearance_s = timing.clearance_s();
        let cycle = greens.iter().sum::<f64>() + greens.len() as f64 * clearance_s;
        let n = junction_ids.len();
        Ok(AdaptiveSplit {
            offsets: junction_ids.iter().map(|id| offsets_s.get(id).copied().unwrap_or(0.0)).collect(),
            junction_ids,
            cycle,
            clearance_s,
            min_green_s: timing.min_green_s,
            // red of phase k = cycle - green_k - yellow, at most max red
            worst_red_room: timing.max_red_s + timing.yellow_s - cycle,
            plan: greens.to_vec(),
            active: vec![greens.to_vec(); n],
            last_pos: vec![None; n],
            update_s,
            window_s,
            max_step_s,
            sat_pcu_h_lane: saturation_pcu_per_h_per_lane,
            lanes: corridor.corridor.approaches.iter().map(|a| f64::from(a.lanes)).collect(),
            phases: corridor.phases.clone(),
            window: vec![VecDeque::new(); corridor.corridor.approaches.len()],
            rate: vec![None; corridor.corridor.approaches.len()],
            last_t: None,
            next_update: update_s,
        })
    }

    /// Greens in use at a junction (for tests and logs).
    pub fn greens(&self, junction: usize) -> &[f64] {
        &self.active[junction]
    }

    fn phase_at(greens: &[f64], clearance_s: f64, pos: f64) -> usize {
        let mut end = 0.0;
        for (k, g) in greens.iter().enumerate() {
            end += g;
            if pos < end {
                return k;
            }
            end += clearance_s;
        }
        0
    }

    /// Webster split of the same total, moved at most `max_step_s` per green;
    /// None if no such split keeps the total and every red within max red.
    fn resplit(&self, old: &[f64], y: &[f64]) -> Option<Vec<f64>> {
        let total: f64 = old.iter().sum();
        let y_sum: f64 = y.iter().sum();
        if y_sum.is_nan() || y_sum <= 0.0 {
            return None;
        }
        let target: Vec<f64> = y.iter().map(|yk| total * yk / y_sum).collect();
        let lo: Vec<f64> = old.iter().map(|g| (g - self.max_step_s).max(self.min_green_s)).collect();
        let hi: Vec<f64> = old.iter().map(|g| g + self.max_step_s).collect();
        let mut g: Vec<f64> = (0..old.len()).map(|k| target[k].round().clamp(lo[k], hi[k])).collect();
        let mut left = (total - g.iter().sum::<f64>()).round();
        while left != 0.0 {
            let step = left.signum();
            // add to the green furthest below its target, take from the one furthest above
            let k = (0..g.len())
                .filter(|&k| if step > 0.0 { g[k] + 1.0 <= hi[k] } else { g[k] - 1.0 >= lo[k] })
                .max_by(|&a, &b| ((target[a] - g[a]) * step).total_cmp(&((target[b] - g[b]) * step)).then(b.cmp(&a)))?;
            g[k] += step;
            left -= step;
        }
        g.iter().all(|gk| gk + self.worst_red_room >= 0.0).then_some(g)
    }

    fn record(&mut self, input: &DecisionInput<'_>) {
        let dt = self.last_t.map_or(1.0, |l| (input.t - l).max(0.0));
        self.last_t = Some(input.t);
        for (a, w) in self.window.iter_mut().enumerate() {
            if let Some(s) = input.states[a].as_ref().filter(|s| s.trust_state == TrustState::Trusted) {
                w.push_back((input.t, dt, s.arrivals_pcu));
            }
            while w.front().is_some_and(|&(t, _, _)| t <= input.t - self.window_s) {
                w.pop_front();
            }
        }
    }

    fn update(&mut self, input: &DecisionInput<'_>) {
        for (a, w) in self.window.iter().enumerate() {
            let trusted = input.states[a].as_ref().is_some_and(|s| s.trust_state == TrustState::Trusted);
            let secs: f64 = w.iter().map(|x| x.1).sum();
            if trusted && secs > 0.0 {
                self.rate[a] = Some(w.iter().map(|x| x.2).sum::<f64>() / secs);
            }
        }
        let Some(rates) = self.rate.iter().copied().collect::<Option<Vec<f64>>>() else { return };
        let n_phases = self.plan.len();
        let y: Vec<f64> = (0..n_phases)
            .map(|k| {
                self.phases
                    .iter()
                    .flat_map(|ph| ph[k].iter())
                    .map(|&a| rates[a] * 3600.0 / (self.lanes[a] * self.sat_pcu_h_lane))
                    .fold(0.0, f64::max)
            })
            .collect();
        if let Some(g) = self.resplit(&self.plan.clone(), &y) {
            self.plan = g;
        }
    }
}

impl Controller for AdaptiveSplit {
    fn name(&self) -> &str {
        "adaptive_split"
    }

    fn decide(&mut self, input: &DecisionInput<'_>) -> BTreeMap<String, usize> {
        self.record(input);
        if input.t >= self.next_update {
            self.update(input);
            while self.next_update <= input.t {
                self.next_update += self.update_s;
            }
        }
        let mut out = BTreeMap::new();
        for j in 0..self.junction_ids.len() {
            let pos = (input.t - self.offsets[j]).rem_euclid(self.cycle);
            if self.last_pos[j].is_some_and(|p| pos < p) {
                self.active[j] = self.plan.clone(); // a new cycle starts here
            }
            self.last_pos[j] = Some(pos);
            out.insert(self.junction_ids[j].clone(), Self::phase_at(&self.active[j], self.clearance_s, pos));
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contracts::ApproachState;
    use crate::corridor::tests::two_junctions;

    const TIMING: SignalTiming = SignalTiming { min_green_s: 10.0, max_red_s: 90.0, yellow_s: 3.0, all_red_s: 2.0 };

    fn ix() -> CorridorIndex {
        CorridorIndex::new(two_junctions()).unwrap()
    }

    fn split(update_s: f64) -> AdaptiveSplit {
        AdaptiveSplit::new(&ix(), &[30.0, 20.0], &BTreeMap::new(), &TIMING, update_s, 600.0, 3.0, 1800.0).unwrap()
    }

    fn st(id: &str, arr: f64, trust: TrustState) -> Option<ApproachState> {
        Some(ApproachState {
            schema_version: "2.0".into(),
            time: 0.0,
            junction_id: String::new(),
            approach_id: id.into(),
            measured_queue_pcu: 0.0,
            queue_estimate: 0.0,
            margin: 0.0,
            trust_state: trust,
            override_flag: false,
            arrivals_pcu: arr,
            departures_pcu: 0.0,
            exits_to: BTreeMap::new(),
        })
    }

    fn run(c: &mut AdaptiveSplit, from: i32, to: i32, a_w: f64, a_n: f64, trust_n: TrustState) {
        let ix = ix();
        for t in from..to {
            let states = [st("a_w", a_w, TrustState::Trusted), st("a_n", a_n, trust_n), st("b_w", 0.0, TrustState::Trusted), st("b_n", 0.0, TrustState::Trusted)];
            c.decide(&DecisionInput { t: f64::from(t), corridor: &ix, states: &states, last_commands: &[None, None] });
        }
    }

    #[test]
    fn resplit_keeps_the_cycle_and_limits_each_step() {
        let c = split(300.0);
        // demand wants 40/10 of the 50 s; one step moves 3 s
        assert_eq!(c.resplit(&[30.0, 20.0], &[0.8, 0.2]), Some(vec![33.0, 17.0]));
        // min green holds
        assert_eq!(c.resplit(&[39.0, 11.0], &[0.9, 0.1]), Some(vec![40.0, 10.0]));
        // nothing measured: no change
        assert_eq!(c.resplit(&[30.0, 20.0], &[0.0, 0.0]), None);
    }

    #[test]
    fn splits_follow_measured_rates_at_the_next_cycle() {
        // update at 310 s, 10 s into a 60 s cycle
        let mut c = split(310.0);
        assert_eq!(c.cycle, 60.0);
        // a_w 4 x busier than a_n (both 2 lanes)
        run(&mut c, 0, 311, 0.4, 0.1, TrustState::Trusted);
        assert_eq!(c.plan, vec![33.0, 17.0]);
        assert_eq!(c.greens(0), &[30.0, 20.0]); // not in the middle of a cycle
        run(&mut c, 311, 361, 0.4, 0.1, TrustState::Trusted);
        assert_eq!(c.greens(0), &[33.0, 17.0]); // taken when the cycle restarted at 360
    }

    #[test]
    fn untrusted_approach_keeps_its_last_rate() {
        let mut c = split(300.0);
        run(&mut c, 0, 301, 0.4, 0.1, TrustState::Trusted);
        let before = c.rate[1];
        // a_n turns FAULTY and reports a flood: ignored, its old rate kept
        run(&mut c, 301, 601, 0.4, 5.0, TrustState::Faulty);
        assert_eq!(c.rate[1], before);
        assert_eq!(c.plan, vec![36.0, 14.0]); // still moving toward 40/10
    }

    #[test]
    fn config_is_validated() {
        let none = BTreeMap::new();
        assert!(AdaptiveSplit::new(&ix(), &[30.0], &none, &TIMING, 300.0, 600.0, 3.0, 1800.0).is_err());
        assert!(AdaptiveSplit::new(&ix(), &[30.0, 5.0], &none, &TIMING, 300.0, 600.0, 3.0, 1800.0).is_err());
        assert!(AdaptiveSplit::new(&ix(), &[30.0, 20.0], &none, &TIMING, 0.0, 600.0, 3.0, 1800.0).is_err());
        let bad: BTreeMap<String, f64> = [("Z".to_string(), 1.0)].into_iter().collect();
        assert!(AdaptiveSplit::new(&ix(), &[30.0, 20.0], &bad, &TIMING, 300.0, 600.0, 3.0, 1800.0).is_err());
    }
}
