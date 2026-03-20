//! # cadora-sketch
//!
//! Sketch manager for CADORA — manages geometry elements, constraints,
//! and solver integration. Equivalent to FreeCAD's SketchObject + Sketch layer.

pub mod types;
pub mod geometry;
pub mod constraint;
pub mod sketch;
pub mod editing;
pub mod analysis;
pub mod operations;

pub use types::*;
pub use geometry::GeoDef;
pub use constraint::SketchConstraint;
pub use sketch::Sketch;
pub use analysis::DetectedConstraint;
pub use operations::GeoOpResult;
