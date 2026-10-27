//! Python face of the core. Everything crosses as JSON text: the same
//! encoding the logs and any future network transport use, and the cost is
//! negligible at one call per second.

use fg::contracts::{ClassCounts, Measurement, SCHEMA_VERSION};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

fn err(e: impl std::fmt::Display) -> PyErr {
    PyValueError::new_err(e.to_string())
}

#[pyclass(name = "Core", module = "flowguard_core")]
struct PyCore {
    inner: fg::Core,
}

#[pymethods]
impl PyCore {
    #[new]
    fn new(corridor_json: &str, config_json: &str) -> PyResult<Self> {
        Ok(PyCore { inner: fg::Core::from_json(corridor_json, config_json).map_err(err)? })
    }

    /// Measurements JSON array in, StepOutput JSON out.
    fn step(&mut self, t: f64, measurements_json: &str) -> PyResult<String> {
        let ms: Vec<Measurement> = serde_json::from_str(measurements_json).map_err(err)?;
        let out = self.inner.step(t, ms).map_err(err)?;
        serde_json::to_string(&out).map_err(err)
    }

    /// PCU total of a ClassCounts JSON object, using the configured weights.
    fn pcu(&self, class_counts_json: &str) -> PyResult<f64> {
        let c: ClassCounts = serde_json::from_str(class_counts_json).map_err(err)?;
        Ok(c.pcu(self.inner.pcu()))
    }

    /// Envelope interventions so far as a JSON object, e.g. {"min_green": 12}.
    fn interventions_json(&self) -> PyResult<String> {
        serde_json::to_string(self.inner.interventions()).map_err(err)
    }

    #[getter]
    fn controller_name(&self) -> String {
        self.inner.controller_name().to_string()
    }

    #[getter]
    fn guard_name(&self) -> String {
        self.inner.guard_name().to_string()
    }
}

#[pymodule]
fn flowguard_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyCore>()?;
    m.add("SCHEMA_VERSION", SCHEMA_VERSION)?;
    m.add("CONTRACT_SCHEMA", fg::contract_schema())?;
    Ok(())
}
