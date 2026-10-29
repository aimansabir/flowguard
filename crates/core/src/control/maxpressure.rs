use std::collections::BTreeMap;

use super::{Controller, DecisionInput};
use crate::contracts::{Capacity, Command, Hysteresis, ServeQueue, Stage};
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
/// With serve_queue (D32, actuated extension) a settled green that has run
/// min green is kept, whatever the pressures say, while one of its
/// approaches still has `hold_pcu_per_lane` x lanes queued and a positive
/// term, up to `max_green_s` of green: with one phase per approach every
/// switch costs a clearance, so a green is not cut while its queue still
/// discharges. The envelope's max red overrides it as it overrides any wish.
///
/// With lane_weight (D33) each approach's term is multiplied by its lanes:
/// the capacity weight of Varaiya's formula, so a 2-lane approach that
/// discharges twice as fast counts twice. With moving_count (D33) the queue
/// of an approach is a running count of the vehicles in its camera zone,
/// moving or not: n = max(queue_estimate, n_previous + arrivals - departures),
/// because a camera's standing queue drops as soon as a served queue starts
/// to move. Both are off unless asked for, and then the name says so.
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
    serve_queue: Option<ServeQueue>,
    /// Approach index -> lanes, for serve_queue and lane_weight.
    lanes: Vec<f64>,
    lane_weight: bool,
    /// Approach index -> running count (PCU), Some with moving_count.
    counts: Option<Vec<f64>>,
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
        let lanes = corridor.corridor.approaches.iter().map(|a| f64::from(a.lanes)).collect();
        Ok(MaxPressure {
            name,
            hysteresis,
            capacity,
            storage,
            min_green_s,
            serve_queue: None,
            lanes,
            lane_weight: false,
            counts: None,
        })
    }

    /// Adds the actuated extension (None leaves the controller as it is).
    pub fn serving(mut self, serve_queue: Option<ServeQueue>) -> Result<Self> {
        if let Some(q) = serve_queue {
            let ok = q.hold_pcu_per_lane.is_finite() && q.hold_pcu_per_lane > 0.0
                && q.max_green_s.is_finite() && q.max_green_s >= self.min_green_s;
            if !ok {
                return Err(Error::Config(
                    "max-pressure serve_queue needs hold_pcu_per_lane > 0 and max_green_s >= min green".into(),
                ));
            }
            self.name.push_str("_serve");
        }
        self.serve_queue = serve_queue;
        Ok(self)
    }

    /// Adds the D33 options (both false leaves the controller as it is).
    pub fn counting(mut self, lane_weight: bool, moving_count: bool) -> Self {
        if lane_weight {
            self.name.push_str("_lanes");
        }
        if moving_count {
            self.name.push_str("_moving");
        }
        self.lane_weight = lane_weight;
        self.counts = moving_count.then(|| vec![0.0; self.lanes.len()]);
        self
    }

    /// Queue the pressure uses: the running count with moving_count, else
    /// the Guard estimate.
    fn q(&self, input: &DecisionInput<'_>, a: usize) -> f64 {
        self.counts.as_ref().map_or_else(|| input.queue(a), |c| c[a])
    }

    fn update_counts(&mut self, input: &DecisionInput<'_>) {
        let Some(counts) = self.counts.as_mut() else { return };
        for (a, n) in counts.iter_mut().enumerate() {
            let q = input.queue(a);
            *n = match &input.states[a] {
                Some(s) => q.max(*n + s.arrivals_pcu - s.departures_pcu),
                None => q.max(*n),
            };
        }
    }

    /// True while the current settled green should be kept for its queue.
    fn still_serving(&self, input: &DecisionInput<'_>, j: usize, last: Option<&Command>) -> bool {
        let (Some(q), Some(c)) = (self.serve_queue, last) else { return false };
        c.stage == Stage::Green
            && c.phase_index == c.target_phase
            && input.t - c.phase_started_at < q.max_green_s
            && input.corridor.phases[j][c.target_phase]
                .iter()
                .any(|&a| self.term(input, a) > 0.0 && self.q(input, a) >= q.hold_pcu_per_lane * self.lanes[a])
    }

    /// Pressure term of one approach.
    fn term(&self, input: &DecisionInput<'_>, a: usize) -> f64 {
        let term = match input.corridor.downstream[a] {
            None => self.q(input, a),
            Some(d) => {
                let diff = self.q(input, a) - self.q(input, d);
                match self.capacity {
                    Some(c) if self.q(input, d) >= c.spillback_occupancy * self.storage[d] => diff.min(0.0),
                    _ => diff,
                }
            }
        };
        if self.lane_weight { term * self.lanes[a] } else { term }
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
        self.update_counts(input);
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
            let wish = if wish != current && self.still_serving(input, j, last) { current } else { wish };
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

    fn serve(hold_pcu_per_lane: f64, max_green_s: f64) -> MaxPressure {
        MaxPressure::new(&ix(), Some(Hysteresis { gain_pcu: 2.0 }), None, 10.0)
            .unwrap()
            .serving(Some(ServeQueue { hold_pcu_per_lane, max_green_s }))
            .unwrap()
    }

    #[test]
    fn serve_queue_keeps_a_green_while_its_queue_discharges() {
        // a_w (2 lanes) green since 0 with 6 PCU still queued; a_n leads by 14 PCU
        let q = [6.0, 20.0, 0.0, 0.0];
        let at = |t: f64, mp: &mut MaxPressure| decide_with(mp, t, q, [green(0, 0.0), green(0, 0.0)])["A"];
        assert_eq!(at(15.0, &mut hyst(2.0)), 1); // without the extension: switch
        let mut mp = serve(2.0, 40.0);
        assert_eq!(mp.name(), "maxpressure_hyst_serve");
        assert_eq!(at(15.0, &mut mp), 0); // 3 PCU per lane left: keep serving
        assert_eq!(at(40.0, &mut mp), 1); // max green reached
        let short = [3.0, 20.0, 0.0, 0.0]; // 1.5 PCU per lane: queue served
        assert_eq!(decide_with(&mut mp, 15.0, short, [green(0, 0.0), green(0, 0.0)])["A"], 1);
    }

    #[test]
    fn serve_queue_does_not_hold_a_green_that_cannot_discharge() {
        // a_w's queue feeds b_w, which holds more: term negative, nothing would move
        let q = [6.0, 20.0, 9.0, 0.0];
        assert_eq!(decide_with(&mut serve(2.0, 40.0), 15.0, q, [green(0, 0.0), green(0, 0.0)])["A"], 1);
    }

    #[test]
    fn serve_queue_is_validated() {
        let mp = || MaxPressure::new(&ix(), None, None, 10.0).unwrap();
        assert!(mp().serving(Some(ServeQueue { hold_pcu_per_lane: 0.0, max_green_s: 40.0 })).is_err());
        assert!(mp().serving(Some(ServeQueue { hold_pcu_per_lane: 2.0, max_green_s: 5.0 })).is_err());
        assert!(mp().serving(None).is_ok());
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
    fn lane_weight_counts_two_lanes_twice() {
        // a_w (2 lanes) 4 PCU vs a_n (1 lane) 6 PCU: plain picks a_n, lane-weighted a_w (8 > 6)
        let mut ix_c = two_junctions();
        ix_c.approaches[1].lanes = 1;
        let ix = CorridorIndex::new(ix_c).unwrap();
        let states = [state("a_w", 4.0), state("a_n", 6.0), state("b_w", 0.0), state("b_n", 0.0)];
        let last = [None, None];
        let input = DecisionInput { t: 0.0, corridor: &ix, states: &states, last_commands: &last };
        let mut plain = MaxPressure::new(&ix, None, None, 10.0).unwrap();
        assert_eq!(plain.decide(&input)["A"], 1);
        let mut weighted = MaxPressure::new(&ix, None, None, 10.0).unwrap().counting(true, false);
        assert_eq!(weighted.name(), "maxpressure_lanes");
        assert_eq!(weighted.decide(&input)["A"], 0); // 4 x 2 = 8 > 6 x 1
    }

    fn flow(id: &str, q: f64, arr: f64, dep: f64) -> Option<ApproachState> {
        let mut s = state(id, q);
        if let Some(s) = s.as_mut() {
            s.arrivals_pcu = arr;
            s.departures_pcu = dep;
        }
        s
    }

    #[test]
    fn moving_count_keeps_a_moving_queue() {
        let mut mp = MaxPressure::new(&ix(), None, None, 10.0).unwrap().counting(false, true);
        assert_eq!(mp.name(), "maxpressure_moving");
        let ix = ix();
        let last = [None, None];
        let run = |mp: &mut MaxPressure, s: [Option<ApproachState>; 4]| {
            mp.decide(&DecisionInput { t: 0.0, corridor: &ix, states: &s, last_commands: &last })
        };
        // 10 PCU queue on a_w, all stopped
        run(&mut mp, [flow("a_w", 10.0, 10.0, 0.0), flow("a_n", 0.0, 0.0, 0.0), state("b_w", 0.0), state("b_n", 0.0)]);
        assert_eq!(mp.counts.as_ref().unwrap()[0], 10.0);
        // green: the queue moves, the camera sees 1 standing, 2 PCU left the zone
        run(&mut mp, [flow("a_w", 1.0, 0.0, 2.0), flow("a_n", 5.0, 5.0, 0.0), state("b_w", 0.0), state("b_n", 0.0)]);
        assert_eq!(mp.counts.as_ref().unwrap()[0], 8.0); // 8 still to serve, not 1
        // never below the standing queue
        run(&mut mp, [flow("a_w", 9.0, 0.0, 8.0), flow("a_n", 5.0, 0.0, 0.0), state("b_w", 0.0), state("b_n", 0.0)]);
        assert_eq!(mp.counts.as_ref().unwrap()[0], 9.0);
    }

    #[test]
    fn options_off_change_nothing() {
        let queues = [7.0, 3.0, 2.0, 0.0];
        let a = decide_with(&mut basic(), 0.0, queues, [None, None]);
        let b = decide_with(&mut basic().counting(false, false), 0.0, queues, [None, None]);
        assert_eq!(a, b);
        assert_eq!(basic().counting(false, false).name(), "maxpressure");
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
