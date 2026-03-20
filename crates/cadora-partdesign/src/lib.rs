//! PartDesign feature framework for CADORA.
//!
//! Mirrors FreeCAD's PartDesign module:
//! - `Body`: ordered container of features with tip tracking
//! - `Feature`: base trait for all features
//! - `FeatureAddSub`: additive/subtractive features (boolean with base)
//! - `ProfileBased`: sketch → 3D operation pipeline

mod body;
mod feature;

pub use body::*;
pub use feature::*;
