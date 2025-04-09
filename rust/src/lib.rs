// Add to rust/src/lib.rs
use pyo3::prelude::*;
use std::io::{self, Read, Write};
use std::time::Duration;
use serialport;

#[pyfunction]
fn read_sensor_data(port_name: &str) -> PyResult<(u32, u32, u32)> {
    let port = serialport::new(port_name, 9600)
        .timeout(Duration::from_millis(1000))
        .open()
        .map_err(|e| PyErr::new::<pyo3::exceptions::PyIOError, _>(e.to_string()))?;
    
    let mut buf = [0; 1024];
    let mut reader = io::BufReader::new(port);
    
    // Read a line from the serial port
    let n = reader.read(&mut buf)
        .map_err(|e| PyErr::new::<pyo3::exceptions::PyIOError, _>(e.to_string()))?;
    
    let data = String::from_utf8_lossy(&buf[..n]);
    
    // Parse the data (assuming format "Red: X, IR: Y, Green: Z")
    // You might need more robust parsing in practice
    let parts: Vec<&str> = data.split(',').collect();
    if parts.len() < 3 {
        return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>("Invalid data format"));
    }
    
    let red = parts[0].trim_start_matches("Red: ").trim().parse::<u32>()
        .map_err(|_| PyErr::new::<pyo3::exceptions::PyValueError, _>("Invalid red value"))?;
    
    let ir = parts[1].trim_start_matches("IR: ").trim().parse::<u32>()
        .map_err(|_| PyErr::new::<pyo3::exceptions::PyValueError, _>("Invalid IR value"))?;
    
    let green = parts[2].trim_start_matches("Green: ").trim().parse::<u32>()
        .map_err(|_| PyErr::new::<pyo3::exceptions::PyValueError, _>("Invalid green value"))?;
    
    Ok((red, ir, green))
}

// Update the PyModule function
#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(hello_from_bin, m)?)?;
    m.add_function(wrap_pyfunction!(read_sensor_data, m)?)?;
    Ok(())
}