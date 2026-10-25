//! Messages and static data shared by every component. These types are the
//! single source of truth: `gen_schema` turns them into
//! `contracts/flowguard.schema.json`, and the Python models are generated from
//! that file. Change a field here, regenerate, and bump SCHEMA_VERSION if a
//! reader of the old format would misread the new one. Units are in the docs.

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::error::{Error, Result};

pub const SCHEMA_VERSION: &str = "2.0";

fn current_version() -> String {
    SCHEMA_VERSION.to_string()
}

/// Rejects messages whose major version differs from ours.
pub fn check_version(version: &str) -> Result<()> {
    let major = |v: &str| v.split('.').next().unwrap_or("").to_string();
    if major(version) != major(SCHEMA_VERSION) {
        return Err(Error::Version(format!("{version} not compatible with {SCHEMA_VERSION}")));
    }
    Ok(())
}

// ---------------------------------------------------------------- counts

/// Vehicles per class. Exactly these six classes; anything else is rejected.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ClassCounts {
    pub car: u32,
    pub motorcycle: u32,
    pub rickshaw: u32,
    pub qingqi: u32,
    pub bus: u32,
    pub truck: u32,
}

/// Passenger car units per vehicle class. Calibration data, owned by the core.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PcuWeights {
    pub car: f64,
    pub motorcycle: f64,
    pub rickshaw: f64,
    pub qingqi: f64,
    pub bus: f64,
    pub truck: f64,
}

impl ClassCounts {
    pub fn total(&self) -> u32 {
        self.car + self.motorcycle + self.rickshaw + self.qingqi + self.bus + self.truck
    }

    /// PCU-weighted total.
    pub fn pcu(&self, w: &PcuWeights) -> f64 {
        f64::from(self.car) * w.car
            + f64::from(self.motorcycle) * w.motorcycle
            + f64::from(self.rickshaw) * w.rickshaw
            + f64::from(self.qingqi) * w.qingqi
            + f64::from(self.bus) * w.bus
            + f64::from(self.truck) * w.truck
    }

    pub fn add(&self, o: &ClassCounts) -> ClassCounts {
        ClassCounts {
            car: self.car + o.car,
            motorcycle: self.motorcycle + o.motorcycle,
            rickshaw: self.rickshaw + o.rickshaw,
            qingqi: self.qingqi + o.qingqi,
            bus: self.bus + o.bus,
            truck: self.truck + o.truck,
        }
    }
}

impl PcuWeights {
    pub fn validate(&self) -> Result<()> {
        let all = [self.car, self.motorcycle, self.rickshaw, self.qingqi, self.bus, self.truck];
        if all.iter().any(|w| !w.is_finite() || *w <= 0.0) {
            return Err(Error::Config("PCU weights must be finite and > 0".into()));
        }
        Ok(())
    }
}

/// Turning movement at a junction. U-turns are not modelled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Movement {
    Left,
    Straight,
    Right,
}

impl Movement {
    pub const ALL: [Movement; 3] = [Movement::Left, Movement::Straight, Movement::Right];
}

/// Counts split by the movement the vehicles took.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MovementCounts {
    pub left: ClassCounts,
    pub straight: ClassCounts,
    pub right: ClassCounts,
}

impl MovementCounts {
    pub fn get(&self, m: Movement) -> &ClassCounts {
        match m {
            Movement::Left => &self.left,
            Movement::Straight => &self.straight,
            Movement::Right => &self.right,
        }
    }

    pub fn total(&self) -> ClassCounts {
        self.left.add(&self.straight).add(&self.right)
    }
}

// ---------------------------------------------------------------- measurement

/// Where a measurement came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Source {
    /// SUMO ground truth, cut to the camera zone.
    Sim,
    /// Live or recorded camera through the vision pipeline.
    Camera,
    /// Historical counts replayed from a file.
    Replay,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DetectorConfidence {
    /// Mean detector score over the interval, 0-1.
    pub mean: f64,
    /// Standard deviation of detector scores over the interval.
    pub std: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ImageHealth {
    /// Consecutive frames near-identical.
    pub frozen: bool,
    /// 0 = sharp, 1 = unusable.
    pub blur: f64,
    /// Mean brightness, 0 = black, 1 = saturated.
    pub brightness: f64,
    /// Scene shift against the reference frame, pixels.
    pub displacement_px: f64,
}

/// The producer's own evidence about measurement quality. All optional: the
/// simulator leaves everything null, the camera pipeline fills what it can.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Quality {
    pub detector_confidence: Option<DetectorConfidence>,
    /// Tracks lost and restarted, per minute.
    pub track_breaks_per_min: Option<f64>,
    pub image_health: Option<ImageHealth>,
    /// Producer estimate of how wrong `zone_counts` might be, vehicles.
    pub count_error: Option<f64>,
}

/// Producer -> core. One per approach per interval. Raw, unweighted counts:
/// PCU weighting and every judgement about trust happen in the core.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Measurement {
    #[serde(default = "current_version")]
    pub schema_version: String,
    /// End of the interval, seconds since the start of the run.
    pub time: f64,
    /// Interval length, seconds. 1.0 in the simulator.
    pub interval_s: f64,
    pub junction_id: String,
    /// Incoming approach the producer watches (SUMO edge id in the simulator).
    pub approach_id: String,
    pub source: Source,
    /// Length of road the producer can see, metres back from the stop line.
    pub zone_length_m: f64,
    /// Vehicles inside the zone at `time`.
    pub zone_counts: ClassCounts,
    /// Of those, vehicles stopped (speed < 0.1 m/s).
    pub queue_counts: ClassCounts,
    /// Vehicles that entered the zone during the interval.
    pub arrivals: ClassCounts,
    /// Vehicles that crossed the stop line during the interval, by movement.
    pub departures: MovementCounts,
    #[serde(default)]
    pub quality: Quality,
}

// ---------------------------------------------------------------- core output

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "UPPERCASE")]
pub enum TrustState {
    Trusted,
    Suspect,
    Faulty,
}

/// The core view of one approach after the Guard. What controllers decide on.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ApproachState {
    #[serde(default = "current_version")]
    pub schema_version: String,
    pub time: f64,
    pub junction_id: String,
    pub approach_id: String,
    /// Queue as measured, PCU.
    pub measured_queue_pcu: f64,
    /// Queue controllers should use, PCU. Equals the measured queue when trusted.
    pub queue_estimate: f64,
    /// +/- uncertainty on `queue_estimate`, PCU.
    pub margin: f64,
    pub trust_state: TrustState,
    /// True when `queue_estimate` is not the measured queue.
    pub override_flag: bool,
    /// Arrivals into the zone this interval, PCU.
    pub arrivals_pcu: f64,
    /// Departures over the stop line this interval, PCU.
    pub departures_pcu: f64,
    /// Departures this interval heading to each neighbouring junction, vehicles.
    /// A key for every neighbour of this junction, 0 if none.
    pub exits_to: BTreeMap<String, u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Stage {
    Green,
    Yellow,
    AllRed,
}

/// Why the envelope did something other than what the controller asked.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Intervention {
    MinGreen,
    MaxRed,
    InvalidRequest,
}

/// Safety envelope -> signals, one per junction per second.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Command {
    #[serde(default = "current_version")]
    pub schema_version: String,
    /// Second this command applies to.
    pub time: f64,
    pub junction_id: String,
    /// Phase whose approaches are lit: the green phase, or the phase being cleared.
    pub phase_index: usize,
    /// When the current `stage` began, seconds.
    pub phase_started_at: f64,
    pub controller_name: String,
    pub stage: Stage,
    /// Phase being switched to; equals `phase_index` when settled.
    pub target_phase: usize,
    /// What the controller asked for this second, null if nothing.
    pub requested_phase: Option<usize>,
    pub intervention: Option<Intervention>,
}

// ---------------------------------------------------------------- corridor map

/// One signal link in SUMO link order: which approach it serves and its SUMO
/// direction code (s, l, r, L, R, t).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SignalLink {
    pub approach: String,
    pub direction: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Junction {
    pub id: String,
    /// Phase index -> approaches that get green in that phase.
    pub phases: Vec<Vec<String>>,
    /// Signal links in the order the signal state string uses.
    pub links: Vec<SignalLink>,
}

/// Edge each movement leads onto, null if the movement does not exist.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MovementTargets {
    pub left: Option<String>,
    pub straight: Option<String>,
    pub right: Option<String>,
}

impl MovementTargets {
    pub fn get(&self, m: Movement) -> Option<&String> {
        match m {
            Movement::Left => self.left.as_ref(),
            Movement::Straight => self.straight.as_ref(),
            Movement::Right => self.right.as_ref(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Approach {
    pub id: String,
    pub junction: String,
    /// Edge length up to the stop line, metres.
    pub length_m: f64,
    pub lanes: u32,
    pub movements: MovementTargets,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RoadKind {
    Main,
    Side,
}

/// Edge where traffic enters the corridor.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Entry {
    pub edge: String,
    pub kind: RoadKind,
    /// Exit edge reached by going straight at every junction.
    pub through_exit: String,
}

/// Edge where traffic leaves the corridor.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Exit {
    pub edge: String,
    pub kind: RoadKind,
}

/// Static map knowledge. Not live state, so every component may read it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Corridor {
    #[serde(default = "current_version")]
    pub schema_version: String,
    pub junctions: Vec<Junction>,
    /// Grouped by junction, in junction order.
    pub approaches: Vec<Approach>,
    pub entries: Vec<Entry>,
    pub exits: Vec<Exit>,
}

// ---------------------------------------------------------------- config

/// Safety envelope limits, seconds.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SignalTiming {
    pub min_green_s: f64,
    pub max_red_s: f64,
    pub yellow_s: f64,
    pub all_red_s: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ControllerConfig {
    /// Fixed-time plan: green seconds per phase, same at every junction.
    #[schemars(title = "FixedControllerConfig")]
    Fixed { green_s: Vec<f64> },
    /// Basic max-pressure, re-decided every second.
    #[serde(rename = "maxpressure")]
    #[schemars(title = "MaxPressureControllerConfig")]
    MaxPressure,
}

/// Which Guard rates the measurements.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum GuardConfig {
    /// Trust every measurement as it is (the naive adaptive baseline).
    #[default]
    #[schemars(title = "PassthroughGuardConfig")]
    Passthrough,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CoreConfig {
    pub pcu: PcuWeights,
    pub timing: SignalTiming,
    pub controller: ControllerConfig,
    #[serde(default)]
    pub guard: GuardConfig,
}

/// Everything `Core::step` produces for one second.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StepOutput {
    pub states: Vec<ApproachState>,
    /// Controller wish per junction, before the envelope.
    pub requests: BTreeMap<String, usize>,
    pub commands: Vec<Command>,
}

/// Wrapper so one schema file carries every contract type.
#[derive(JsonSchema)]
#[allow(dead_code)]
pub struct Contracts {
    measurement: Measurement,
    approach_state: ApproachState,
    command: Command,
    corridor: Corridor,
    core_config: CoreConfig,
    step_output: StepOutput,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_check_accepts_same_major_only() {
        assert!(check_version("2.0").is_ok());
        assert!(check_version("2.7").is_ok());
        assert!(check_version("1.0").is_err());
        assert!(check_version("").is_err());
    }

    #[test]
    fn pcu_weighting() {
        let w = PcuWeights { car: 1.0, motorcycle: 0.5, rickshaw: 1.0, qingqi: 1.0, bus: 3.0, truck: 3.0 };
        let c = ClassCounts { car: 4, motorcycle: 4, bus: 1, ..Default::default() };
        assert_eq!(c.pcu(&w), 9.0);
        assert_eq!(c.total(), 9);
    }

    #[test]
    fn rejects_unknown_class_and_fields() {
        let bad = r#"{"car":1,"motorcycle":0,"rickshaw":0,"qingqi":0,"bus":0,"truck":0,"tank":1}"#;
        assert!(serde_json::from_str::<ClassCounts>(bad).is_err());
        let missing = r#"{"car":1}"#;
        assert!(serde_json::from_str::<ClassCounts>(missing).is_err());
    }

    #[test]
    fn controller_config_is_tagged() {
        let f: ControllerConfig = serde_json::from_str(r#"{"kind":"fixed","green_s":[40,20]}"#).unwrap();
        assert_eq!(f, ControllerConfig::Fixed { green_s: vec![40.0, 20.0] });
        let m: ControllerConfig = serde_json::from_str(r#"{"kind":"maxpressure"}"#).unwrap();
        assert_eq!(m, ControllerConfig::MaxPressure);
    }
}
