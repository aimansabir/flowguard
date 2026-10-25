//! The Guard decides how far each approach can be trusted and, when it can
//! not, what queue the controller should use instead. See docs/specs/guard.md.

use crate::contracts::{ApproachState, Command, Measurement};
use crate::corridor::CorridorIndex;

/// Everything the Guard may look at. Measurements and states are indexed by
/// approach (None = nothing arrived this second), commands by junction.
pub struct GuardInput<'a> {
    pub t: f64,
    pub corridor: &'a CorridorIndex,
    pub measurements: &'a [Option<Measurement>],
    /// Commands issued last second, so the Guard can check how queues
    /// responded to green.
    pub last_commands: &'a [Option<Command>],
}

pub trait Guard: Send {
    fn name(&self) -> &str;
    /// Adjust `states` in place: trust_state, queue_estimate, margin,
    /// override_flag. States arrive taken at face value (state::assess).
    fn rate(&mut self, input: &GuardInput<'_>, states: &mut [Option<ApproachState>]);
}

/// Baseline "naive adaptive" Guard: trusts every measurement as it is.
/// Kept after Guard v1 lands, as the comparison arm in experiments.
pub struct PassThrough;

impl Guard for PassThrough {
    fn name(&self) -> &str {
        "passthrough"
    }

    fn rate(&mut self, _input: &GuardInput<'_>, _states: &mut [Option<ApproachState>]) {}
}
