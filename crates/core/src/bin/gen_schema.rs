//! Prints the contract JSON Schema. Used by scripts/gen_contracts.py.
fn main() {
    print!("{}", flowguard_core::contract_schema());
}
