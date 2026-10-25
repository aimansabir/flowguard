use std::collections::BTreeMap;

use super::{Controller, DecisionInput};

/// Basic max-pressure (Varaiya 2013), re-decided every second at every junction.
///
/// pressure(phase) = sum over its approaches of
///     queue on the approach - queue on the approach its through traffic joins
/// Approaches whose through traffic leaves the corridor count downstream as 0.
/// Queues are the Guard `queue_estimate` (PCU). Ties keep the current phase,
/// then go to the lowest index.
///
/// Known simplifications (next work, see docs/specs/control.md): turning
/// traffic is not split out, there is no capacity weighting (Gregoire et al.
/// 2015), and nothing damps switching, so it asks to switch during min green.
pub struct MaxPressure;

impl Controller for MaxPressure {
    fn name(&self) -> &str {
        "maxpressure"
    }

    fn decide(&mut self, input: &DecisionInput<'_>) -> BTreeMap<String, usize> {
        let ix = input.corridor;
        let mut requests = BTreeMap::new();
        for (j, junction) in ix.corridor.junctions.iter().enumerate() {
            let pressure = |phase: &Vec<usize>| -> f64 {
                phase.iter().map(|&a| input.queue(a) - ix.downstream[a].map_or(0.0, |d| input.queue(d))).sum()
            };
            let pressures: Vec<f64> = ix.phases[j].iter().map(pressure).collect();
            let current = input.last_commands[j].as_ref().map_or(0, |c| c.target_phase);

            let mut best = 0;
            for i in 1..pressures.len() {
                let better = pressures[i] > pressures[best]
                    || (pressures[i] == pressures[best] && i == current && best != current);
                if better {
                    best = i;
                }
            }
            requests.insert(junction.id.clone(), best);
        }
        requests
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contracts::{ApproachState, Command, Stage, TrustState};
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

    fn cmd(target: usize) -> Option<Command> {
        Some(Command {
            schema_version: "2.0".into(),
            time: 0.0,
            junction_id: String::new(),
            phase_index: target,
            phase_started_at: 0.0,
            controller_name: "maxpressure".into(),
            stage: Stage::Green,
            target_phase: target,
            requested_phase: None,
            intervention: None,
        })
    }

    fn decide(queues: [f64; 4], last: [Option<Command>; 2]) -> BTreeMap<String, usize> {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        let states = [state("a_w", queues[0]), state("a_n", queues[1]), state("b_w", queues[2]), state("b_n", queues[3])];
        let input = DecisionInput { t: 0.0, corridor: &ix, states: &states, last_commands: &last };
        MaxPressure.decide(&input)
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
        assert_eq!(MaxPressure.decide(&input)["A"], 1);
    }
}
