//! Measurement -> ApproachState before any trust judgement: PCU weighting and
//! which neighbour each departure heads to. Pure functions.

use std::collections::BTreeMap;

use crate::contracts::{ApproachState, Measurement, Movement, PcuWeights, TrustState, SCHEMA_VERSION};
use crate::corridor::CorridorIndex;

/// State taken at face value: TRUSTED, estimate = measured queue, margin 0.
pub fn assess(t: f64, m: &Measurement, approach: usize, ix: &CorridorIndex, pcu: &PcuWeights) -> ApproachState {
    let j = ix.junction_of(approach);
    let mut exits_to: BTreeMap<String, u32> = ix.neighbours[j].iter().map(|n| (n.clone(), 0)).collect();
    for mv in Movement::ALL {
        if let Some(n) = ix.neighbour_via(approach, mv) {
            *exits_to.get_mut(n).expect("neighbour listed") += m.departures.get(mv).total();
        }
    }
    let queue = m.queue_counts.pcu(pcu);
    ApproachState {
        schema_version: SCHEMA_VERSION.into(),
        time: t,
        junction_id: m.junction_id.clone(),
        approach_id: m.approach_id.clone(),
        measured_queue_pcu: queue,
        queue_estimate: queue,
        margin: 0.0,
        trust_state: TrustState::Trusted,
        override_flag: false,
        arrivals_pcu: m.arrivals.pcu(pcu),
        departures_pcu: m.departures.total().pcu(pcu),
        exits_to,
    }
}
