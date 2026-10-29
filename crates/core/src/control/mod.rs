//! Controllers pick the phase they would like green at each junction. They
//! never touch signals: the safety envelope owns every transition.
//!
//! To add a controller: implement `Controller`, add a variant to
//! `ControllerConfig` (contracts.rs), and map it in `make_controller`.

mod fixed;
mod maxpressure;

use std::collections::BTreeMap;

pub use fixed::FixedTime;
pub use maxpressure::MaxPressure;

use crate::contracts::{ApproachState, Command, ControllerConfig, SignalTiming};
use crate::corridor::CorridorIndex;
use crate::error::Result;

/// What a controller sees each second. Only Guard output and the last
/// commands: never raw measurements, never simulator state.
pub struct DecisionInput<'a> {
    pub t: f64,
    pub corridor: &'a CorridorIndex,
    /// Approach index -> Guard output, None if no measurement this second.
    pub states: &'a [Option<ApproachState>],
    /// Junction index -> command issued last second, None on the first step.
    pub last_commands: &'a [Option<Command>],
}

impl DecisionInput<'_> {
    /// Queue estimate for an approach, 0 if it has no state this second.
    pub fn queue(&self, approach: usize) -> f64 {
        self.states[approach].as_ref().map_or(0.0, |s| s.queue_estimate)
    }
}

pub trait Controller: Send + Sync {
    fn name(&self) -> &str;
    /// Junction id -> wanted phase. Leaving a junction out = no preference.
    fn decide(&mut self, input: &DecisionInput<'_>) -> BTreeMap<String, usize>;
}

pub fn make_controller(
    cfg: &ControllerConfig,
    corridor: &CorridorIndex,
    timing: &SignalTiming,
) -> Result<Box<dyn Controller>> {
    Ok(match cfg {
        ControllerConfig::Fixed { green_s, offsets_s } => {
            Box::new(FixedTime::new(corridor, green_s, &offsets_s.clone().unwrap_or_default(), timing.clearance_s())?)
        }
        ControllerConfig::MaxPressure { hysteresis, capacity, serve_queue } => {
            Box::new(MaxPressure::new(corridor, *hysteresis, *capacity, timing.min_green_s)?.serving(*serve_queue)?)
        }
    })
}
