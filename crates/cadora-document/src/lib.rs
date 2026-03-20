//! # cadora-document
//!
//! Document model for CADORA — properties, objects, dependency graph,
//! transactions (undo/redo), placement, and expression engine.
//! Equivalent to FreeCAD's App layer (Document + DocumentObject + Property system).

pub mod property;
pub mod placement;
pub mod object;
pub mod document;
pub mod transaction;
pub mod expression;

pub use property::{PropValue, Property, PropertyBag, PropertyStatus, Unit, Color};
pub use placement::{Placement, Rotation};
pub use object::{DocumentObject, ObjectId, ObjectStatus, ObjectType};
pub use document::Document;
pub use transaction::Transaction;
