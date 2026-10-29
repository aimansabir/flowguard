use std::collections::BTreeMap;

use super::{Controller, DecisionInput};
use crate::contracts::{Command, Stage};
use crate::corridor::CorridorIndex;
use crate::error::{Error, Result};

/// Fully actuated control (D34), after FHWA's actuated signal timing.
///
/// Phases are served in their fixed order. A green runs at least min green
/// (the envelope's), then is extended while one of its approaches had PCU
/// entering the camera zone within the last `gap_s` (the zone works as the
/// detector), and ends on gap-out or on reaching its max green (max-out).
/// It then goes to the next phase in order that has a vehicle waiting
/// (standing queue or an arrival this second); phases with none are skipped,
/// and with none anywhere the green rests where it is. The envelope's max red
/// still overrides any wish, and the order carries on from whatever phase is
/// green.
pub struct Actuated {
    max_green: Vec<f64>,
    gap_s: f64,
    min_green_s: f64,
    /// Approach index -> last time PCU entered its zone.
    last_arrival: Vec<Option<f64>>,
}

impl Actuated {
    pub fn new(corridor: &CorridorIndex, max_green_s: &[f64], gap_s: f64, min_green_s: f64) -> Result<Self> {
        for j in &corridor.corridor.junctions {
            if j.phases.len() != max_green_s.len() {
                return Err(Error::Config(format!(
                    "{} has {} phases, actuated max_green_s has {}",
                    j.id,
                    j.phases.len(),
                    max_green_s.len()
                )));
            }
        }
        if max_green_s.iter().any(|g| !g.is_finite() || *g < min_green_s) {
            return Err(Error::Config("actuated max_green_s must be finite and >= min green".into()));
        }
        if !(gap_s.is_finite() && gap_s > 0.0) {
            return Err(Error::Config("actuated gap_s must be > 0".into()));
        }
        Ok(Actuated {
            max_green: max_green_s.to_vec(),
            gap_s,
            min_green_s,
            last_arrival: vec![None; corridor.corridor.approaches.len()],
        })
    }

    fn has_demand(input: &DecisionInput<'_>, phase: &[usize]) -> bool {
        phase.iter().any(|&a| input.states[a].as_ref().is_some_and(|s| s.queue_estimate > 0.0 || s.arrivals_pcu > 0.0))
    }

    fn extending(&self, t: f64, phase: &[usize]) -> bool {
        phase.iter().any(|&a| self.last_arrival[a].is_some_and(|l| t - l < self.gap_s))
    }

    /// Seconds the current phase has been settled green, None during a change.
    fn green_for(t: f64, last: Option<&Command>) -> Option<f64> {
        last.filter(|c| c.stage == Stage::Green && c.phase_index == c.target_phase).map(|c| t - c.phase_started_at)
    }
}

impl Controller for Actuated {
    fn name(&self) -> &str {
        "actuated"
    }

    fn decide(&mut self, input: &DecisionInput<'_>) -> BTreeMap<String, usize> {
        for (a, s) in input.states.iter().enumerate() {
            if s.as_ref().is_some_and(|s| s.arrivals_pcu > 0.0) {
                self.last_arrival[a] = Some(input.t);
            }
        }
        let ix = input.corridor;
        let mut requests = BTreeMap::new();
        for (j, junction) in ix.corridor.junctions.iter().enumerate() {
            let phases = &ix.phases[j];
            let last = input.last_commands[j].as_ref();
            let current = last.map_or(0, |c| c.target_phase);
            let next = (1..phases.len()).map(|k| (current + k) % phases.len()).find(|&k| Self::has_demand(input, &phases[k]));
            let wish = match (Self::green_for(input.t, last), next) {
                (None, _) | (_, None) => current,
                (Some(g), Some(_)) if g < self.min_green_s => current,
                (Some(g), Some(n)) if g >= self.max_green[current] => n,
                (Some(_), Some(_)) if self.extending(input.t, &phases[current]) => current,
                (Some(_), Some(n)) => n,
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
    use crate::corridor::tests::two_junctions;

    fn st(id: &str, q: f64, arr: f64) -> Option<ApproachState> {
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
            arrivals_pcu: arr,
            departures_pcu: 0.0,
            exits_to: BTreeMap::new(),
        })
    }

    fn green(phase: usize, since: f64) -> Option<Command> {
        Some(Command {
            schema_version: "2.0".into(),
            time: 0.0,
            junction_id: String::new(),
            phase_index: phase,
            phase_started_at: since,
            controller_name: "actuated".into(),
            stage: Stage::Green,
            target_phase: phase,
            requested_phase: None,
            intervention: None,
        })
    }

    fn ix() -> CorridorIndex {
        CorridorIndex::new(two_junctions()).unwrap()
    }

    fn at(c: &mut Actuated, t: f64, a_w: (f64, f64), a_n: (f64, f64)) -> usize {
        let ix = ix();
        let states = [st("a_w", a_w.0, a_w.1), st("a_n", a_n.0, a_n.1), st("b_w", 0.0, 0.0), st("b_n", 0.0, 0.0)];
        let last = [green(0, 0.0), green(0, 0.0)];
        c.decide(&DecisionInput { t, corridor: &ix, states: &states, last_commands: &last })["A"]
    }

    #[test]
    fn extends_while_arrivals_keep_coming_and_gaps_out() {
        let mut c = Actuated::new(&ix(), &[40.0, 20.0], 3.0, 10.0).unwrap();
        assert_eq!(at(&mut c, 5.0, (4.0, 1.0), (3.0, 0.0)), 0); // min green
        assert_eq!(at(&mut c, 12.0, (4.0, 1.0), (3.0, 0.0)), 0); // arrival now: extend
        assert_eq!(at(&mut c, 14.0, (4.0, 0.0), (3.0, 0.0)), 0); // 2 s gap < 3 s
        assert_eq!(at(&mut c, 15.0, (4.0, 0.0), (3.0, 0.0)), 1); // gap-out
    }

    #[test]
    fn maxes_out() {
        let mut c = Actuated::new(&ix(), &[40.0, 20.0], 3.0, 10.0).unwrap();
        assert_eq!(at(&mut c, 39.0, (4.0, 1.0), (3.0, 0.0)), 0);
        assert_eq!(at(&mut c, 40.0, (4.0, 1.0), (3.0, 0.0)), 1);
    }

    #[test]
    fn skips_a_phase_without_demand_and_rests() {
        let mut c = Actuated::new(&ix(), &[40.0, 20.0], 3.0, 10.0).unwrap();
        // a_n has nothing waiting: a_w keeps the green even after max green
        assert_eq!(at(&mut c, 50.0, (0.0, 0.0), (0.0, 0.0)), 0);
        // an arrival on a_n is a call
        assert_eq!(at(&mut c, 51.0, (0.0, 0.0), (0.0, 0.5)), 1);
    }

    #[test]
    fn config_is_validated() {
        assert!(Actuated::new(&ix(), &[40.0], 3.0, 10.0).is_err());
        assert!(Actuated::new(&ix(), &[40.0, 5.0], 3.0, 10.0).is_err());
        assert!(Actuated::new(&ix(), &[40.0, 20.0], 0.0, 10.0).is_err());
    }
}
