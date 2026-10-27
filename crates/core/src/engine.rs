//! `Core`: one call per interval runs Guard -> controller -> envelope.

use std::collections::BTreeMap;

use crate::contracts::{
    check_version, ApproachState, Command, CoreConfig, Corridor, GuardConfig, Intervention, Measurement,
    PcuWeights, StepOutput,
};
use crate::control::{make_controller, Controller, DecisionInput};
use crate::corridor::CorridorIndex;
use crate::error::{Error, Result};
use crate::guard::{Guard, GuardInput, PassThrough};
use crate::safety::SafetyEnvelope;
use crate::state;

pub struct Core {
    ix: CorridorIndex,
    pcu: PcuWeights,
    guard: Box<dyn Guard>,
    controller: Box<dyn Controller>,
    envelope: SafetyEnvelope,
    last_commands: Vec<Option<Command>>,
}

impl Core {
    pub fn new(corridor: Corridor, config: CoreConfig) -> Result<Self> {
        let ix = CorridorIndex::new(corridor)?;
        config.pcu.validate()?;
        config.timing.validate()?;
        let controller = make_controller(&config.controller, &ix, &config.timing)?;
        let guard: Box<dyn Guard> = match config.guard {
            GuardConfig::Passthrough => Box::new(PassThrough),
        };
        let junctions: Vec<(String, usize)> =
            ix.corridor.junctions.iter().map(|j| (j.id.clone(), j.phases.len())).collect();
        let envelope = SafetyEnvelope::new(&junctions, config.timing, controller.name())?;
        let n = junctions.len();
        Ok(Core { ix, pcu: config.pcu, guard, controller, envelope, last_commands: vec![None; n] })
    }

    pub fn from_json(corridor_json: &str, config_json: &str) -> Result<Self> {
        Core::new(serde_json::from_str(corridor_json)?, serde_json::from_str(config_json)?)
    }

    pub fn controller_name(&self) -> &str {
        self.controller.name()
    }

    pub fn guard_name(&self) -> &str {
        self.guard.name()
    }

    pub fn pcu(&self) -> &PcuWeights {
        &self.pcu
    }

    pub fn interventions(&self) -> &BTreeMap<Intervention, u64> {
        self.envelope.interventions()
    }

    /// One interval. Measurements may come in any order and may miss
    /// approaches (a missing approach has no state this second); unknown
    /// approaches, duplicates and wrong junction ids are errors.
    pub fn step(&mut self, t: f64, measurements: Vec<Measurement>) -> Result<StepOutput> {
        let slots = self.slot(measurements)?;
        let mut states: Vec<Option<ApproachState>> = slots
            .iter()
            .enumerate()
            .map(|(a, m)| m.as_ref().map(|m| state::assess(t, m, a, &self.ix, &self.pcu)))
            .collect();

        let guard_input = GuardInput { t, corridor: &self.ix, measurements: &slots, last_commands: &self.last_commands };
        self.guard.rate(&guard_input, &mut states);

        let decision = DecisionInput { t, corridor: &self.ix, states: &states, last_commands: &self.last_commands };
        let requests = self.controller.decide(&decision);
        let commands = self.envelope.step(t, &requests);
        self.last_commands = commands.iter().cloned().map(Some).collect();

        Ok(StepOutput { states: states.into_iter().flatten().collect(), requests, commands })
    }

    fn slot(&self, measurements: Vec<Measurement>) -> Result<Vec<Option<Measurement>>> {
        let mut slots = vec![None; self.ix.corridor.approaches.len()];
        for m in measurements {
            check_version(&m.schema_version)?;
            let a = self.ix.approach(&m.approach_id).ok_or_else(|| Error::Input(format!("unknown approach {}", m.approach_id)))?;
            let expected = &self.ix.corridor.approaches[a].junction;
            if &m.junction_id != expected {
                return Err(Error::Input(format!("{} belongs to {expected}, not {}", m.approach_id, m.junction_id)));
            }
            if !(m.interval_s > 0.0 && m.zone_length_m > 0.0) {
                return Err(Error::Input(format!("{}: interval_s and zone_length_m must be > 0", m.approach_id)));
            }
            if slots[a].is_some() {
                return Err(Error::Input(format!("two measurements for {}", m.approach_id)));
            }
            slots[a] = Some(m);
        }
        Ok(slots)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contracts::{
        ClassCounts, ControllerConfig, MovementCounts, Quality, SignalTiming, Source, Stage, TrustState,
    };
    use crate::corridor::tests::two_junctions;

    fn config(controller: ControllerConfig) -> CoreConfig {
        CoreConfig {
            pcu: PcuWeights { car: 1.0, motorcycle: 0.5, rickshaw: 1.0, qingqi: 1.0, bus: 3.0, truck: 3.0 },
            timing: SignalTiming { min_green_s: 10.0, max_red_s: 90.0, yellow_s: 3.0, all_red_s: 2.0 },
            controller,
            guard: GuardConfig::Passthrough,
        }
    }

    fn measurement(approach: &str, junction: &str, queued_cars: u32) -> Measurement {
        let cars = ClassCounts { car: queued_cars, ..Default::default() };
        Measurement {
            schema_version: "2.0".into(),
            time: 0.0,
            interval_s: 1.0,
            junction_id: junction.into(),
            approach_id: approach.into(),
            source: Source::Sim,
            zone_length_m: 60.0,
            zone_counts: cars,
            queue_counts: cars,
            arrivals: ClassCounts::default(),
            departures: MovementCounts { straight: ClassCounts { bus: 2, ..Default::default() }, ..Default::default() },
            quality: Quality::default(),
        }
    }

    #[test]
    fn step_weighs_rates_and_commands() {
        let mut core = Core::new(two_junctions(), config(ControllerConfig::MaxPressure { hysteresis: None, capacity: None })).unwrap();
        let out = core.step(0.0, vec![measurement("a_w", "A", 4), measurement("a_n", "A", 1)]).unwrap();
        assert_eq!(out.states.len(), 2);
        let a_w = &out.states[0];
        assert_eq!(a_w.queue_estimate, 4.0);
        assert_eq!(a_w.trust_state, TrustState::Trusted);
        assert_eq!(a_w.departures_pcu, 6.0); // two buses
        assert_eq!(a_w.exits_to["B"], 2); // straight from a_w joins b_w at junction B
        assert_eq!(out.states[1].exits_to["B"], 0); // a_n straight leaves the corridor
        assert_eq!(out.requests["A"], 0);
        assert_eq!(out.commands.len(), 2);
        assert!(out.commands.iter().all(|c| c.stage == Stage::Green && c.controller_name == "maxpressure"));
    }

    #[test]
    fn rejects_bad_measurements() {
        let mut core = Core::new(two_junctions(), config(ControllerConfig::MaxPressure { hysteresis: None, capacity: None })).unwrap();
        assert!(matches!(core.step(0.0, vec![measurement("nope", "A", 0)]), Err(Error::Input(_))));
        assert!(matches!(core.step(0.0, vec![measurement("a_w", "B", 0)]), Err(Error::Input(_))));
        let dup = vec![measurement("a_w", "A", 0), measurement("a_w", "A", 0)];
        assert!(matches!(core.step(0.0, dup), Err(Error::Input(_))));
        let mut old = measurement("a_w", "A", 0);
        old.schema_version = "1.0".into();
        assert!(matches!(core.step(0.0, vec![old]), Err(Error::Version(_))));
        let mut zero = measurement("a_w", "A", 0);
        zero.zone_length_m = 0.0;
        assert!(core.step(0.0, vec![zero]).is_err());
    }

    #[test]
    fn hysteresis_never_fights_min_green() {
        let run = |hysteresis| {
            let mut core = Core::new(two_junctions(), config(ControllerConfig::MaxPressure { hysteresis, capacity: None })).unwrap();
            let mut x: u64 = 0x2545_F491_4F6C_DD1D;
            for t in 0..600 {
                let mut queue = || {
                    x ^= x << 13;
                    x ^= x >> 7;
                    x ^= x << 17;
                    (x % 12) as u32
                };
                let ms = vec![
                    measurement("a_w", "A", queue()),
                    measurement("a_n", "A", queue()),
                    measurement("b_w", "B", queue()),
                    measurement("b_n", "B", queue()),
                ];
                core.step(f64::from(t), ms).unwrap();
            }
            core.interventions().get(&Intervention::MinGreen).copied().unwrap_or(0)
        };
        assert!(run(None) > 0);
        assert_eq!(run(Some(crate::contracts::Hysteresis { gain_pcu: 0.0 })), 0);
    }

    #[test]
    fn rejects_bad_config() {
        let wrong_plan = config(ControllerConfig::Fixed { green_s: vec![40.0], offsets_s: None });
        assert!(matches!(Core::new(two_junctions(), wrong_plan), Err(Error::Config(_))));
        let mut bad_pcu = config(ControllerConfig::MaxPressure { hysteresis: None, capacity: None });
        bad_pcu.pcu.bus = 0.0;
        assert!(Core::new(two_junctions(), bad_pcu).is_err());
    }

    #[test]
    fn from_json_round_trip() {
        let corridor = serde_json::to_string(&two_junctions()).unwrap();
        let cfg = r#"{"pcu":{"car":1,"motorcycle":0.5,"rickshaw":1,"qingqi":1,"bus":3,"truck":3},
                     "timing":{"min_green_s":10,"max_red_s":90,"yellow_s":3,"all_red_s":2},
                     "controller":{"kind":"fixed","green_s":[40,20]}}"#;
        let core = Core::from_json(&corridor, cfg).unwrap();
        assert_eq!(core.controller_name(), "fixed");
        assert_eq!(core.guard_name(), "passthrough");
        assert!(Core::from_json(&corridor, "{}").is_err());
    }
}
