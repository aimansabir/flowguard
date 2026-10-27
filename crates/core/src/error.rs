use std::fmt;

/// Everything the core can reject. Inputs are validated at the boundary, so a
/// bad corridor, config or measurement fails loudly instead of being guessed at.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// Malformed JSON or a message that doesn't match the contract.
    Contract(String),
    /// Schema major version this core can't read.
    Version(String),
    /// Corridor map is inconsistent.
    Corridor(String),
    /// Core config is unusable (timing, plan, weights).
    Config(String),
    /// A measurement refers to something the corridor doesn't have.
    Input(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Contract(m) => write!(f, "contract: {m}"),
            Error::Version(m) => write!(f, "schema version: {m}"),
            Error::Corridor(m) => write!(f, "corridor: {m}"),
            Error::Config(m) => write!(f, "config: {m}"),
            Error::Input(m) => write!(f, "input: {m}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<serde_json::Error> for Error {
    fn from(e: serde_json::Error) -> Self {
        Error::Contract(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, Error>;
