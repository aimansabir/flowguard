use std::collections::BTreeMap;

use super::{Controller, DecisionInput};
use crate::contracts::{Capacity, Command, Hysteresis, Stage};
use crate::corridor::CorridorIndex;
use crate::error::{Error, Result};

/// Max-pressure (Varaiya 2013), re-decided every second at every junction.
///
/// pressure(phase) = sum over its approaches of
///     queue on the approach - queue on the approach its through traffic joins
/// Approaches whose through traffic leaves the corridor count downstream as 0.
/// Queues are the Guard `queue_estimate` (PCU). Ties keep the current phase,
/// then go to the lowest index.
///
/// Without hysteresis this is the basic version (D12): it asks for the best
/// phase every second, also during min green and clearance, and the envelope
/// overrides it. With hysteresis (D20) it keeps asking for the current phase
/// until that green is settled and has run min green, and after that only
/// asks to switch when the best phase beats the current one by more than
/// `gain_pcu`.
///
/// With capacity (D22, spillback protection after Gregoire et al. 2015) an
/// approach whose downstream approach is at or above `spillback_occupancy`
/// of its storage adds no positive pressure: green there would only push
/// traffic into a full link. Its term becomes min(queue - downstream, 0).
/// Pressure stays in PCU, so hysteresis gains mean the same with or without.
///
/// Known simplification (next work, see docs/specs/control.md): turning
/// traffic is not split out; downstream is the straight-through approach.
pub struct MaxPressure {
    name: String,
    hysteresis: Option<Hysteresis>,
    capacity: Option<Capacity>,
    /// Approach index -> storage in PCU (length x lanes / jam spacing);
    /// empty without capacity.
    storage: Vec<f64>,
    min_green_s: f64,
}

impl MaxPressure {
    pub fn new(
        corridor: &CorridorIndex,
        hysteresis: Option<Hysteresis>,
        capacity: Option<Capacity>,
        min_green_s: f64,
    ) -> Result<Self> {
        if hysteresis.is_some_and(|h| !(h.gain_pcu.is_finite() && h.gain_pcu >= 0.0)) {
            return Err(Error::Config("max-pressure hysteresis gain_pcu must be finite and >= 0".into()));
        }
        if capacity.is_some_and(|c| {
            !(c.jam_spacing_m.is_finite() && c.jam_spacing_m > 0.0)
                || !(c.spillback_occupancy > 0.0 && c.spillback_occupancy <= 1.0)
        }) {
            return Err(Error::Config(
                "max-pressure capacity needs jam_spacing_m > 0 and 0 < spillback_occupancy <= 1".into(),
            ));
        }
        let storage = capacity.map_or_else(Vec::new, |c| {
            corridor.corridor.approaches.iter().map(|a| a.length_m * f64::from(a.lanes) / c.jam_spacing_m).collect()
        });
        let mut name = String::from("maxpressure");
        if hysteresis.is_some() {
            name.push_str("_hyst");
        }
        if capacity.is_some() {
            name.push_str("_cap");
        }
        Ok(MaxPressure { name, hysteresis, capacity, storage, min_green_s })
    }

    /// Pressure term of one approach.
    fn term(&self, input: &DecisionInput<'_>, a: usize) -> f64 {
        let Some(d) = input.corridor.downstream[a] else { return input.queue(a) };
        let diff = input.queue(a) - input.queue(d);
        match self.capacity {
            Some(c) if input.queue(d) >= c.spillback_occupancy * self.storage[d] => diff.min(0.0),
            _ => diff,
        }
    }

    /// True while the current green must not be given up: a change is under
    /// way, or the green has not run min green yet. The envelope starts every
    /// junction in phase 0 green at t = 0, so no command yet means holding.
    fn holding(&self, t: f64, last: Option<&Command>) -> bool {
        last.is_none_or(|c| {
            c.stage != Stage::Green || c.phase_index != c.target_phase || t - c.phase_started_at < self.min_green_s
        })
    }
}

impl Controller for MaxPressure {
    fn name(&self) -> &str {
        &self.name
    }

    fn decide(&mut self, input: &DecisionInput<'_>) -> BTreeMap<String, usize> {
        let ix = input.corridor;
        let mut requests = BTreeMap::new();
        for (j, junction) in ix.corridor.junctions.iter().enumerate() {
            let pressure = |phase: &Vec<usize>| -> f64 { phase.iter().map(|&a| self.term(input, a)).sum() };
            let pressures: Vec<f64> = ix.phases[j].iter().map(pressure).collect();
            let last = input.last_commands[j].as_ref();
            let current = last.map_or(0, |c| c.target_phase);

            let mut best = 0;
            for i in 1..pressures.len() {
                let better = pressures[i] > pressures[best]
                    || (pressures[i] == pressures[best] && i == current && best != current);
                if better {
                    best = i;
                }
            }
            let wish = match self.hysteresis {
                None => best,
                Some(_) if self.holding(input.t, last) => current,
                Some(h) if pressures[best] - pressures[current] <= h.gain_pcu => current,
                Some(_) => best,
            };
            requests.insert(junction.id.clone(), wish);
        }
        requests
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contracts::{ApproachState, TrustState};
    use crate::corridor::{tests::two_junctions, CorridorIndex};

    fn state(id: &str, q: f64) -> Option<ApproachState> {
        Some(ApproachState {
            schema_version: "2.0".into(),
            time: 0.0,
            junction_id: String::new(),
            approach_id: id.into(),
            measured_queue_pcu: q,
            queue_estimate: q,
            margin: 0.0,
            trust_state: TrustState::Trusted,
            override_flag: false,
            arrivals_pcu: 0.0,
            departures_pcu: 0.0,
            exits_to: BTreeMap::new(),
        })
    }

    /// Settled green in `phase` since `since`.
    fn green(phase: usize, since: f64) -> Option<Command> {
        Some(Command {
            schema_version: "2.0".into(),
            time: 0.0,
            junction_id: String::new(),
            phase_index: phase,
            phase_started_at: since,
            controller_name: "maxpressure".into(),
            stage: Stage::Green,
            target_phase: phase,
            requested_phase: None,
            intervention: None,
        })
    }

    fn cmd(target: usize) -> Option<Command> {
        green(target, 0.0)
    }

    fn ix() -> CorridorIndex {
        CorridorIndex::new(two_junctions()).unwrap()
    }

    fn basic() -> MaxPressure {
        MaxPressure::new(&ix(), None, None, 10.0).unwrap()
    }

    fn hyst(gain_pcu: f64) -> MaxPressure {
        MaxPressure::new(&ix(), Some(Hysteresis { gain_pcu }), None, 10.0).unwrap()
    }

    /// Test approaches are 300 m x 2 lanes: storage 80 PCU at 7.5 m.
    fn cap(spillback_occupancy: f64) -> MaxPressure {
        let c = Capacity { jam_spacing_m: 7.5, spillback_occupancy };
        MaxPressure::new(&ix(), None, Some(c), 10.0).unwrap()
    }

    fn decide_with(mp: &mut MaxPressure, t: f64, queues: [f64; 4], last: [Option<Command>; 2]) -> BTreeMap<String, usize> {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        let states = [state("a_w", queues[0]), state("a_n", queues[1]), state("b_w", queues[2]), state("b_n", queues[3])];
        let input = DecisionInput { t, corridor: &ix, states: &states, last_commands: &last };
        mp.decide(&input)
    }

    fn decide(queues: [f64; 4], last: [Option<Command>; 2]) -> BTreeMap<String, usize> {
        decide_with(&mut basic(), 0.0, queues, last)
    }

    #[test]
    fn downstream_queue_reduces_pressure() {
        // a_w: 10 waiting but b_w (its downstream) holds 8 -> pressure 2 < a_n 5
        let r = decide([10.0, 5.0, 8.0, 0.0], [None, None]);
        assert_eq!(r["A"], 1);
        // with b_w empty, a_w wins
        let r = decide([10.0, 5.0, 0.0, 0.0], [None, None]);
        assert_eq!(r["A"], 0);
    }

    #[test]
    fn ties_keep_current_phase() {
        let r = decide([3.0, 3.0, 0.0, 0.0], [cmd(1), cmd(0)]);
        assert_eq!(r["A"], 1);
        let r = decide([3.0, 3.0, 0.0, 0.0], [cmd(0), cmd(0)]);
        assert_eq!(r["A"], 0);
    }

    #[test]
    fn missing_state_counts_as_empty() {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        let states = [None, state("a_n", 1.0), None, None];
        let last = [None, None];
        let input = DecisionInput { t: 0.0, corridor: &ix, states: &states, last_commands: &last };
        assert_eq!(basic().decide(&input)["A"], 1);
    }

    #[test]
    fn basic_asks_to_switch_during_min_green() {
        // the behaviour hysteresis removes: phase 1 far ahead 3 s into phase 0
        let r = decide_with(&mut basic(), 3.0, [0.0, 9.0, 0.0, 0.0], [green(0, 0.0), green(0, 0.0)]);
        assert_eq!(r["A"], 1);
    }

    #[test]
    fn hysteresis_holds_until_min_green() {
        let mut mp = hyst(0.0);
        let queues = [0.0, 9.0, 0.0, 0.0];
        let last = || [green(0, 100.0), green(0, 100.0)];
        assert_eq!(decide_with(&mut mp, 100.0, queues, last())["A"], 0);
        assert_eq!(decide_with(&mut mp, 109.0, queues, last())["A"], 0);
        assert_eq!(decide_with(&mut mp, 110.0, queues, last())["A"], 1);
        // nothing issued yet: the envelope starts in phase 0 green at t = 0
        assert_eq!(decide_with(&mut mp, 0.0, queues, [None, None])["A"], 0);
    }

    #[test]
    fn hysteresis_needs_gain_above_threshold() {
        let mut mp = hyst(2.0);
        let last = || [green(0, 0.0), green(0, 0.0)];
        // phase 0 pressure 1, phase 1 pressure 3: a gain of 2 is not above 2
        assert_eq!(decide_with(&mut mp, 30.0, [1.0, 3.0, 0.0, 0.0], last())["A"], 0);
        assert_eq!(decide_with(&mut mp, 30.0, [1.0, 3.5, 0.0, 0.0], last())["A"], 1);
        // gain 0 still needs a strict gain: a tie keeps the current phase
        assert_eq!(decide_with(&mut hyst(0.0), 30.0, [3.0, 3.0, 0.0, 0.0], last())["A"], 0);
    }

    #[test]
    fn hysteresis_asks_for_target_during_clearance() {
        let mut yellow = green(0, 50.0).unwrap();
        yellow.stage = Stage::Yellow;
        yellow.target_phase = 1;
        // phase 0 is now far ahead, but the change to 1 is under way
        let r = decide_with(&mut hyst(0.0), 52.0, [9.0, 0.0, 0.0, 0.0], [Some(yellow), green(0, 0.0)]);
        assert_eq!(r["A"], 1);
    }

    #[test]
    fn full_downstream_stops_release() {
        // a_w 100 waiting, b_w holds 72 of its 80 PCU: 0.9 full
        let queues = [100.0, 5.0, 72.0, 0.0];
        assert_eq!(decide(queues, [None, None])["A"], 0); // basic: 100 - 72 = 28 > 5
        assert_eq!(decide_with(&mut cap(0.9), 0.0, queues, [None, None])["A"], 1); // gated to 0 < 5
        // just under the threshold the plain difference counts again
        let below = [100.0, 5.0, 71.9, 0.0];
        assert_eq!(decide_with(&mut cap(0.9), 0.0, below, [None, None])["A"], 0);
    }

    #[test]
    fn full_downstream_keeps_negative_pressure() {
        // downstream full and longer than the queue: the term stays negative,
        // so a phase with nothing waiting (0) still beats it
        let r = decide_with(&mut cap(0.5), 0.0, [30.0, 0.0, 60.0, 0.0], [None, None]);
        assert_eq!(r["A"], 1);
    }

    #[test]
    fn exits_are_never_full() {
        // b_w leaves the corridor: no downstream, no gate
        let r = decide_with(&mut cap(0.1), 0.0, [0.0, 0.0, 50.0, 3.0], [None, None]);
        assert_eq!(r["B"], 0);
    }

    #[test]
    fn names_and_validation() {
        assert_eq!(basic().name(), "maxpressure");
        assert_eq!(hyst(1.0).name(), "maxpressure_hyst");
        assert_eq!(cap(0.9).name(), "maxpressure_cap");
        let both = MaxPressure::new(
            &ix(),
            Some(Hysteresis { gain_pcu: 1.0 }),
            Some(Capacity { jam_spacing_m: 7.5, spillback_occupancy: 0.9 }),
            10.0,
        )
        .unwrap();
        assert_eq!(both.name(), "maxpressure_hyst_cap");
        assert!(MaxPressure::new(&ix(), Some(Hysteresis { gain_pcu: -1.0 }), None, 10.0).is_err());
        assert!(MaxPressure::new(&ix(), Some(Hysteresis { gain_pcu: f64::NAN }), None, 10.0).is_err());
        for (jam_spacing_m, spillback_occupancy) in [(0.0, 0.9), (7.5, 0.0), (7.5, 1.5), (f64::NAN, 0.9)] {
            let c = Capacity { jam_spacing_m, spillback_occupancy };
            assert!(MaxPressure::new(&ix(), None, Some(c), 10.0).is_err());
        }
    }
}
