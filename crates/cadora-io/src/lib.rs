//! File format & I/O for CADORA CAD.
//!
//! Mirrors FreeCAD's import/export infrastructure:
//! - `.cadora` project files (ZIP archive with JSON metadata + binary shapes)
//! - STEP import/export via truck-stepio
//! - STL import/export (triangulated mesh)
//! - IGES import (legacy support)

mod project;
mod step;
mod stl;
mod iges;

pub use project::*;
pub use step::*;
pub use stl::*;
pub use iges::*;
