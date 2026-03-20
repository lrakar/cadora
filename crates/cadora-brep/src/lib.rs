//! BRep shape kernel for CADORA CAD.
//!
//! Wraps the `truck` crate to provide a clean API for:
//! - BRep topology types (Vertex, Edge, Wire, Face, Shell, Solid)
//! - Primitive shape creation (box, cylinder, sphere, cone, torus)
//! - Shape operations (extrusion, revolution, booleans, fillet, chamfer)
//! - Tessellation for rendering

mod shape;
mod primitives;
mod builder;
mod operations;
mod tessellation;

pub use shape::*;
pub use primitives::*;
pub use builder::*;
pub use operations::*;
pub use tessellation::*;
