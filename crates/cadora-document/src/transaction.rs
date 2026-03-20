//! Transaction system — records property changes for undo/redo.
//!
//! Mirrors FreeCAD's `App::Transaction`.  Each transaction stores
//! snapshots of property values taken *before* changes are applied,
//! allowing the document to revert or re-apply them.

use crate::object::ObjectId;
use crate::property::Property;

/// A snapshot of a single property value before a change.
#[derive(Debug, Clone)]
pub struct PropertySnapshot {
    pub object_id: ObjectId,
    pub prop_name: String,
    pub old_value: Property,
}

/// A transaction groups a set of property changes into a single
/// undoable/redoable operation.
#[derive(Debug, Clone)]
pub struct Transaction {
    name: String,
    id: u64,
    snapshots: Vec<PropertySnapshot>,
}

impl Transaction {
    pub fn new(name: String, id: u64) -> Self {
        Self { name, id, snapshots: Vec::new() }
    }

    /// Record one property change (the old value).
    pub fn record(&mut self, object_id: ObjectId, prop_name: String, old_value: Property) {
        self.snapshots.push(PropertySnapshot { object_id, prop_name, old_value });
    }

    pub fn name(&self) -> &str { &self.name }
    pub fn id(&self) -> u64 { self.id }
    pub fn snapshots(&self) -> &[PropertySnapshot] { &self.snapshots }
    pub fn is_empty(&self) -> bool { self.snapshots.is_empty() }
}
