//! FlowGuard decision core. Takes raw per-approach Measurements, rates them
//! (Guard), picks phases (controller) and makes every change legal (safety
//! envelope). No I/O and no simulator access: the boundary is `Core::step`.

pub mod contracts;
pub mod control;
pub mod corridor;
pub mod engine;
pub mod error;
pub mod guard;
pub mod safety;
pub mod state;

pub use engine::Core;
pub use error::{Error, Result};

/// The JSON Schema for every contract type, as written to contracts/.
pub fn contract_schema() -> String {
    let schema = schemars::schema_for!(contracts::Contracts);
    serde_json::to_string_pretty(&schema).expect("schema serialises") + "\n"
}

#[cfg(test)]
mod tests {
    #[test]
    fn checked_in_schema_is_current() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../contracts/flowguard.schema.json");
        let on_disk = std::fs::read_to_string(path).unwrap_or_default().replace("\r\n", "\n");
        assert!(
            on_disk == super::contract_schema(),
            "contracts/flowguard.schema.json is stale: run `python scripts/gen_contracts.py`"
        );
    }
}
