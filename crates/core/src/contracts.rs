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
    /// `offsets_s` shifts a junction's plan later by that many seconds
    /// (green wave); junctions left out have offset 0 (D21).
    #[schemars(title = "FixedControllerConfig")]
    Fixed {
        green_s: Vec<f64>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        offsets_s: Option<BTreeMap<String, f64>>,
    },
    /// Max-pressure, re-decided every second. Without `hysteresis` this is the
    /// basic version (D12); with it, greens are held for min green and only
    /// given up for a clear pressure gain (D20).
    #[serde(rename = "maxpressure")]
    #[schemars(title = "MaxPressureControllerConfig")]
    MaxPressure {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        hysteresis: Option<Hysteresis>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        capacity: Option<Capacity>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        serve_queue: Option<ServeQueue>,
    },
}

/// Actuated extension for max-pressure (D32): keep the green on an approach
/// that is still discharging a queue. With one phase per approach every
/// switch costs a clearance, so ending a green while its queue still flows
/// at saturation throws capacity away. The envelope's max red still applies.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ServeQueue {
    /// Keep the current phase while one of its approaches still has at least
    /// this queue per lane (PCU) and a positive pressure term (it can move).
    pub hold_pcu_per_lane: f64,
    /// Longest green the extension keeps, s.
    pub max_green_s: f64,
}

/// Spillback protection for max-pressure (D22): an approach adds no
/// positive pressure while the approach it feeds is nearly full.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Capacity {
    /// Road length one queued PCU takes, m. Storage of an approach =
    /// length_m x lanes / jam_spacing_m, in PCU.
    pub jam_spacing_m: f64,
    /// Occupancy (queue / storage, 0-1) at or above which an approach is
    /// treated as full.
    pub spillback_occupancy: f64,
}

/// Switching damper for max-pressure.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Hysteresis {
    /// Switch only when the best phase beats the current one by more than
    /// this, PCU. 0 = any strict gain.
    pub gain_pcu: f64,
}

/// Which Guard rates the measurements.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
// read once when the core starts; boxing v1 would only add noise
#[allow(clippy::large_enum_variant)]
pub enum GuardConfig {
    /// Trust every measurement as it is (the naive adaptive baseline).
    #[default]
    #[schemars(title = "PassthroughGuardConfig")]
    Passthrough,
    /// Evidence (G1), trust state machine (G2) and virtual sensor (G3) of
    /// docs/specs/guard.md. Every block and field may be left out for its
    /// default: `{"kind": "v1"}` is the documented Guard (D26).
    #[serde(rename = "v1")]
    #[schemars(title = "GuardV1Config")]
    V1 {
        #[serde(default)]
        evidence: EvidenceConfig,
        #[serde(default)]
        combine: CombineConfig,
        #[serde(default)]
        trust: TrustConfig,
        #[serde(default)]
        virtual_sensor: VirtualSensorConfig,
    },
}

/// Guard evidence parameters, one block per check (D24, D25). All
/// provisional research values.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct EvidenceConfig {
    pub quality: QualityEvidence,
    pub frozen: FrozenEvidence,
    pub conservation: ConservationEvidence,
    pub link: LinkEvidence,
    pub response: ResponseEvidence,
    pub silence: SilenceEvidence,
}

/// E6: a camera reporting nothing at all for longer than its own trusted
/// arrival rate makes plausible (D28). Probability of the silence
/// p = exp(-rate x seconds); evidence from `p_start`, full at `p_full`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct SilenceEvidence {
    pub p_start: f64,
    pub p_full: f64,
}

impl Default for SilenceEvidence {
    fn default() -> Self {
        SilenceEvidence { p_start: 1e-3, p_full: 1e-5 }
    }
}

/// E1: the producer's quality report. Any bad flag scores `weight`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct QualityEvidence {
    /// Brightness at or below this is black, at or above 1 - this saturated.
    pub brightness_edge: f64,
    /// Blur above this (0 sharp, 1 unusable).
    pub blur_max: f64,
    /// Mean detector confidence below this.
    pub confidence_min: f64,
    /// Track breaks per minute above this.
    pub track_breaks_max: f64,
    /// Scene shift above this, pixels.
    pub displacement_max_px: f64,
    /// Score of a bad flag, below 1: a bad image is not proof of bad counts.
    pub weight: f64,
}

impl Default for QualityEvidence {
    fn default() -> Self {
        QualityEvidence {
            brightness_edge: 0.05,
            blur_max: 0.7,
            confidence_min: 0.3,
            track_breaks_max: 30.0,
            displacement_max_px: 20.0,
            weight: 0.5,
        }
    }
}

/// E2: counts repeating when they should change.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct FrozenEvidence {
    /// Seconds of suspicious repetition for a score of 1.
    pub frozen_s: f64,
    /// An upstream departure toward the approach counts as supply this long, s.
    pub supply_s: f64,
    /// Score cap for a repeated standing queue, which a blocked exit gives too.
    pub standing_weight: f64,
}

impl Default for FrozenEvidence {
    fn default() -> Self {
        FrozenEvidence { frozen_s: 20.0, supply_s: 60.0, standing_weight: 0.5 }
    }
}

/// E5: zone(t) - zone(t-1) - arrivals + departures.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct ConservationEvidence {
    /// Residuals kept, intervals.
    pub window: usize,
    /// Residuals summed over blocks this long, so late departures cancel.
    pub block: usize,
    /// Mean |block residual| tolerated, vehicles. Score 1 at twice this.
    pub tolerance: f64,
}

impl Default for ConservationEvidence {
    fn default() -> Self {
        ConservationEvidence { window: 60, block: 5, tolerance: 1.0 }
    }
}

/// E3: PCU balance on each link between junctions. `free_flow_mps`,
/// `saturation_flow` and `min_spacing_m` also drive the virtual sensor (G3).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct LinkEvidence {
    /// Free-flow speed on links, m/s (not in the corridor map).
    pub free_flow_mps: f64,
    /// The fastest drivers go this much over free-flow speed.
    pub fast_factor: f64,
    /// Extra seconds the slowest trip may take.
    pub slack_s: f64,
    /// Metres of lane one PCU takes in the densest standing queue; link
    /// storage = length x lanes / this. Not max-pressure's `jam_spacing_m`.
    pub min_spacing_m: f64,
    /// A zone counts as congested, a link as full, from this share of storage.
    pub congested_fraction: f64,
    /// PCU per second per lane a released queue discharges at.
    pub saturation_flow: f64,
    /// Seconds of corrections and sent PCU compared.
    pub window_s: f64,
    /// Sent PCU below this count as this much in a window.
    pub min_volume_pcu: f64,
    /// Share of sent PCU the balance may be corrected by. Score 1 at twice this.
    pub tolerance: f64,
}

impl Default for LinkEvidence {
    fn default() -> Self {
        LinkEvidence {
            free_flow_mps: 13.89,
            fast_factor: 1.2,
            slack_s: 30.0,
            min_spacing_m: 3.5,
            congested_fraction: 0.5,
            saturation_flow: 0.5,
            window_s: 120.0,
            min_volume_pcu: 10.0,
            tolerance: 0.2,
        }
    }
}

/// E4: queues discharge under green, nothing leaves under red.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct ResponseEvidence {
    /// Seconds into green before departures are expected.
    pub startup_s: f64,
    /// Seconds of stalled green for the full stall score.
    pub stall_s: f64,
    /// Cap on the stall score: gridlock and incidents stall queues too.
    pub stall_weight: f64,
    /// Seconds of history for red-light departures.
    pub red_window_s: f64,
    /// Red seconds below this count as this many.
    pub red_floor_s: f64,
    /// Red-light departures per minute of red tolerated.
    pub red_tolerance_per_min: PerMovement,
}

impl Default for ResponseEvidence {
    fn default() -> Self {
        ResponseEvidence {
            startup_s: 4.0,
            stall_s: 15.0,
            stall_weight: 0.5,
            red_window_s: 120.0,
            red_floor_s: 60.0,
            red_tolerance_per_min: PerMovement { left: 2.0, straight: 2.0, right: 2.0 },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PerMovement {
    pub left: f64,
    pub straight: f64,
    pub right: f64,
}

/// How the evidence becomes one score per approach (D26): noisy-OR,
/// `1 - product(1 - weight x score)` over the checks that could judge.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct CombineConfig {
    pub weights: CheckWeights,
}

/// 0-1 multiplier per check; 0 leaves a check out (ablations).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct CheckWeights {
    pub quality: f64,
    pub frozen: f64,
    pub conservation: f64,
    pub cross_junction: f64,
    pub stalled_green: f64,
    pub red_departures: f64,
    pub silence: f64,
}

impl Default for CheckWeights {
    fn default() -> Self {
        CheckWeights {
            quality: 1.0,
            frozen: 1.0,
            conservation: 1.0,
            cross_junction: 1.0,
            stalled_green: 1.0,
            red_departures: 1.0,
            silence: 1.0,
        }
    }
}

/// G2 hysteresis. TRUSTED -> SUSPECT once the score has been at least
/// `suspect_score` for `suspect_s` seconds; SUSPECT -> FAULTY once it has
/// been at least `faulty_score` for `faulty_s` seconds; back to TRUSTED
/// after `recover_s` judged seconds below `suspect_score` (D26).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct TrustConfig {
    pub suspect_score: f64,
    pub suspect_s: f64,
    pub faulty_score: f64,
    pub faulty_s: f64,
    pub recover_s: f64,
}

impl Default for TrustConfig {
    fn default() -> Self {
        TrustConfig { suspect_score: 0.5, suspect_s: 5.0, faulty_score: 0.9, faulty_s: 10.0, recover_s: 30.0 }
    }
}

/// G3 virtual sensor (D26). Discharge and travel time use
/// `evidence.link.saturation_flow` and `free_flow_mps`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct VirtualSensorConfig {
    /// Trusted seconds the arrival rate is averaged over (entry approaches,
    /// and internal ones whose upstream cannot be used).
    pub arrival_window_s: f64,
    /// `margin` growth per second since the last trusted reading, PCU/s.
    pub margin_growth_pcu_per_s: f64,
    /// While TRUSTED with a full zone, report the virtual queue when it is
    /// longer than the camera can see (the queue extends past the zone).
    pub extend_queue_beyond_zone: bool,
}

impl Default for VirtualSensorConfig {
    fn default() -> Self {
        VirtualSensorConfig { arrival_window_s: 300.0, margin_growth_pcu_per_s: 0.05, extend_queue_beyond_zone: false }
    }
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
        assert_eq!(f, ControllerConfig::Fixed { green_s: vec![40.0, 20.0], offsets_s: None });
        assert_eq!(serde_json::to_string(&f).unwrap(), r#"{"kind":"fixed","green_s":[40.0,20.0]}"#);
        let m: ControllerConfig = serde_json::from_str(r#"{"kind":"maxpressure"}"#).unwrap();
        assert_eq!(m, ControllerConfig::MaxPressure { hysteresis: None, capacity: None, serve_queue: None });
        let h: ControllerConfig =
            serde_json::from_str(r#"{"kind":"maxpressure","hysteresis":{"gain_pcu":2}}"#).unwrap();
        assert_eq!(h, ControllerConfig::MaxPressure { hysteresis: Some(Hysteresis { gain_pcu: 2.0 }), capacity: None, serve_queue: None });
        assert_eq!(serde_json::to_string(&m).unwrap(), r#"{"kind":"maxpressure"}"#);
    }

    #[test]
    fn guard_config_v1_is_additive() {
        let p: GuardConfig = serde_json::from_str(r#"{"kind":"passthrough"}"#).unwrap();
        assert_eq!(p, GuardConfig::Passthrough);
        assert_eq!(serde_json::to_string(&p).unwrap(), r#"{"kind":"passthrough"}"#);
        assert_eq!(GuardConfig::default(), GuardConfig::Passthrough);

        let v: GuardConfig = serde_json::from_str(r#"{"kind":"v1"}"#).unwrap();
        let defaults = GuardConfig::V1 {
            evidence: EvidenceConfig::default(),
            combine: CombineConfig::default(),
            trust: TrustConfig::default(),
            virtual_sensor: VirtualSensorConfig::default(),
        };
        assert_eq!(v, defaults);
        // a partial block keeps the other defaults
        let v: GuardConfig =
            serde_json::from_str(r#"{"kind":"v1","trust":{"faulty_s":20},"evidence":{"link":{"slack_s":40}}}"#).unwrap();
        let GuardConfig::V1 { evidence, trust, .. } = v else { panic!("v1") };
        assert_eq!((trust.faulty_s, trust.suspect_s), (20.0, 5.0));
        assert_eq!((evidence.link.slack_s, evidence.link.free_flow_mps), (40.0, 13.89));
        // round trip, and unknown fields rejected
        let back: GuardConfig = serde_json::from_str(&serde_json::to_string(&defaults).unwrap()).unwrap();
        assert_eq!(back, defaults);
        assert!(serde_json::from_str::<GuardConfig>(r#"{"kind":"v1","trust":{"faulty":1}}"#).is_err());
        assert!(serde_json::from_str::<GuardConfig>(r#"{"kind":"v1","nope":{}}"#).is_err());
    }
}
