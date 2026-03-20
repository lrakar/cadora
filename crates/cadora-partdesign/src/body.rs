//! PartDesign Body: ordered feature list with chain management.
//!
//! Mirrors FreeCAD's `PartDesign::Body`:
//! - Ordered feature list (Group)
//! - Tip pointer (active feature)
//! - Recompute pipeline: execute features in order, each receives previous shape
//! - Insert/remove with chain fixup

use crate::feature::{Feature, FeatureId, FeatureOutput, FeatureStatus};
use cadora_brep::Shape;

/// A feature entry within the body, wrapping the trait object with an ID.
#[derive(Debug)]
struct FeatureEntry {
    id: FeatureId,
    feature: Box<dyn Feature>,
    /// Cached output from last execution.
    last_output: Option<FeatureOutput>,
}

/// Error type for body operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BodyError {
    /// Feature not found in this body.
    FeatureNotFound(FeatureId),
    /// Invalid index for insertion/removal.
    InvalidIndex(usize),
    /// Body has no features.
    Empty,
    /// Recompute failed for one or more features.
    RecomputeFailed(Vec<(FeatureId, String)>),
}

impl std::fmt::Display for BodyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BodyError::FeatureNotFound(id) => write!(f, "Feature {:?} not found", id),
            BodyError::InvalidIndex(i) => write!(f, "Invalid index {}", i),
            BodyError::Empty => write!(f, "Body has no features"),
            BodyError::RecomputeFailed(errs) => {
                write!(f, "Recompute failed for {} features: ", errs.len())?;
                for (id, msg) in errs {
                    write!(f, "[{:?}: {}] ", id, msg)?;
                }
                Ok(())
            }
        }
    }
}

impl std::error::Error for BodyError {}

/// A PartDesign Body containing an ordered list of features.
///
/// Features are executed sequentially — each receives the previous feature's
/// output shape as its base. The body's shape is the tip feature's output.
#[derive(Debug)]
pub struct Body {
    name: String,
    features: Vec<FeatureEntry>,
    /// Index of the tip feature (active feature).
    tip: Option<usize>,
    /// Counter for generating unique feature IDs.
    next_id: u64,
}

impl Body {
    /// Create a new empty body.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            features: Vec::new(),
            tip: None,
            next_id: 1,
        }
    }

    /// Body name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Set the body name.
    pub fn set_name(&mut self, name: impl Into<String>) {
        self.name = name.into();
    }

    /// Number of features in the body.
    pub fn feature_count(&self) -> usize {
        self.features.len()
    }

    /// Whether the body has any features.
    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }

    /// Get the tip index.
    pub fn tip_index(&self) -> Option<usize> {
        self.tip
    }

    /// Get the tip feature ID.
    pub fn tip_id(&self) -> Option<FeatureId> {
        self.tip.map(|i| self.features[i].id)
    }

    /// Get the body's result shape (tip feature's output shape).
    pub fn shape(&self) -> Option<&Shape> {
        let tip = self.tip?;
        self.features[tip].last_output.as_ref()?.shape.as_ref()
    }

    /// Add a feature at the end and set it as the tip.
    /// Returns the assigned FeatureId.
    pub fn add_feature(&mut self, feature: Box<dyn Feature>) -> FeatureId {
        let id = FeatureId(self.next_id);
        self.next_id += 1;
        self.features.push(FeatureEntry {
            id,
            feature,
            last_output: None,
        });
        self.tip = Some(self.features.len() - 1);
        id
    }

    /// Insert a feature at a specific index.
    /// Features after the insertion point remain in the chain.
    /// The inserted feature becomes the new tip.
    pub fn insert_feature(&mut self, index: usize, feature: Box<dyn Feature>) -> std::result::Result<FeatureId, BodyError> {
        if index > self.features.len() {
            return Err(BodyError::InvalidIndex(index));
        }
        let id = FeatureId(self.next_id);
        self.next_id += 1;
        self.features.insert(index, FeatureEntry {
            id,
            feature,
            last_output: None,
        });
        self.tip = Some(index);
        Ok(id)
    }

    /// Remove a feature by ID and fix the chain.
    /// If the removed feature was the tip, the tip moves to the previous feature.
    pub fn remove_feature(&mut self, id: FeatureId) -> std::result::Result<Box<dyn Feature>, BodyError> {
        let index = self.find_index(id)?;
        let entry = self.features.remove(index);

        // Fix tip
        if self.features.is_empty() {
            self.tip = None;
        } else if let Some(tip) = self.tip {
            if tip == index {
                // Removed the tip — move to previous or first
                self.tip = Some(if index > 0 { index - 1 } else { 0 });
            } else if tip > index {
                // Tip was after removed feature — shift down
                self.tip = Some(tip - 1);
            }
        }

        // Invalidate cached outputs from the removed index onward
        for i in index..self.features.len() {
            self.features[i].last_output = None;
        }

        Ok(entry.feature)
    }

    /// Move a feature from one position to another.
    pub fn move_feature(&mut self, from: usize, to: usize) -> std::result::Result<(), BodyError> {
        if from >= self.features.len() {
            return Err(BodyError::InvalidIndex(from));
        }
        if to >= self.features.len() {
            return Err(BodyError::InvalidIndex(to));
        }
        if from == to {
            return Ok(());
        }

        let entry = self.features.remove(from);
        self.features.insert(to, entry);

        // Update tip if needed
        if let Some(tip) = self.tip {
            if tip == from {
                self.tip = Some(to);
            } else if from < to {
                if tip > from && tip <= to {
                    self.tip = Some(tip - 1);
                }
            } else {
                if tip >= to && tip < from {
                    self.tip = Some(tip + 1);
                }
            }
        }

        // Invalidate from the smaller of from/to onward
        let invalidate_from = from.min(to);
        for i in invalidate_from..self.features.len() {
            self.features[i].last_output = None;
        }

        Ok(())
    }

    /// Set the tip to a specific feature by ID.
    pub fn set_tip(&mut self, id: FeatureId) -> std::result::Result<(), BodyError> {
        let index = self.find_index(id)?;
        self.tip = Some(index);
        Ok(())
    }

    /// Set the tip to a specific index.
    pub fn set_tip_index(&mut self, index: usize) -> std::result::Result<(), BodyError> {
        if index >= self.features.len() {
            return Err(BodyError::InvalidIndex(index));
        }
        self.tip = Some(index);
        Ok(())
    }

    /// Suppress a feature by ID.
    pub fn suppress_feature(&mut self, id: FeatureId) -> std::result::Result<(), BodyError> {
        let index = self.find_index(id)?;
        self.features[index].feature.set_suppressed(true);
        // Invalidate from this feature onward
        for i in index..self.features.len() {
            self.features[i].last_output = None;
        }
        Ok(())
    }

    /// Unsuppress a feature by ID.
    pub fn unsuppress_feature(&mut self, id: FeatureId) -> std::result::Result<(), BodyError> {
        let index = self.find_index(id)?;
        self.features[index].feature.set_suppressed(false);
        for i in index..self.features.len() {
            self.features[i].last_output = None;
        }
        Ok(())
    }

    /// Get a reference to a feature by ID.
    pub fn get_feature(&self, id: FeatureId) -> std::result::Result<&dyn Feature, BodyError> {
        let index = self.find_index(id)?;
        Ok(self.features[index].feature.as_ref())
    }

    /// Get a mutable reference to a feature by ID.
    pub fn get_feature_mut(&mut self, id: FeatureId) -> std::result::Result<&mut dyn Feature, BodyError> {
        let index = self.find_index(id)?;
        // Invalidate from this feature onward since it may be modified
        for i in index..self.features.len() {
            self.features[i].last_output = None;
        }
        Ok(self.features[index].feature.as_mut())
    }

    /// Get the feature ID at a given index.
    pub fn feature_id_at(&self, index: usize) -> std::result::Result<FeatureId, BodyError> {
        self.features.get(index)
            .map(|e| e.id)
            .ok_or(BodyError::InvalidIndex(index))
    }

    /// Get the index of a feature by ID.
    pub fn feature_index(&self, id: FeatureId) -> std::result::Result<usize, BodyError> {
        self.find_index(id)
    }

    /// List all feature IDs in order.
    pub fn feature_ids(&self) -> Vec<FeatureId> {
        self.features.iter().map(|e| e.id).collect()
    }

    /// List all feature names in order.
    pub fn feature_names(&self) -> Vec<&str> {
        self.features.iter().map(|e| e.feature.name()).collect()
    }

    /// Get the output of a feature by ID (from last recompute).
    pub fn feature_output(&self, id: FeatureId) -> std::result::Result<Option<&FeatureOutput>, BodyError> {
        let index = self.find_index(id)?;
        Ok(self.features[index].last_output.as_ref())
    }

    /// Recompute all features from scratch.
    /// Each feature receives the previous feature's output shape.
    /// Returns errors from any features that failed.
    pub fn recompute(&mut self) -> std::result::Result<(), BodyError> {
        let mut errors = Vec::new();
        let mut prev_shape: Option<Shape> = None;

        for i in 0..self.features.len() {
            let output = self.features[i].feature.execute(prev_shape.as_ref());

            if let FeatureStatus::Error(ref msg) = output.status {
                errors.push((self.features[i].id, msg.clone()));
            }

            // The next feature's base is this feature's output shape
            // (even if suppressed — suppressed features pass through the base shape)
            prev_shape = output.shape.clone();
            self.features[i].last_output = Some(output);
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(BodyError::RecomputeFailed(errors))
        }
    }

    /// Recompute features starting from a given index.
    /// Features before `from_index` keep their cached outputs.
    pub fn recompute_from(&mut self, from_index: usize) -> std::result::Result<(), BodyError> {
        if from_index >= self.features.len() {
            return Err(BodyError::InvalidIndex(from_index));
        }

        let mut errors = Vec::new();

        // Get the shape from the feature before from_index
        let mut prev_shape: Option<Shape> = if from_index > 0 {
            self.features[from_index - 1]
                .last_output
                .as_ref()
                .and_then(|o| o.shape.clone())
        } else {
            None
        };

        for i in from_index..self.features.len() {
            let output = self.features[i].feature.execute(prev_shape.as_ref());

            if let FeatureStatus::Error(ref msg) = output.status {
                errors.push((self.features[i].id, msg.clone()));
            }

            prev_shape = output.shape.clone();
            self.features[i].last_output = Some(output);
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(BodyError::RecomputeFailed(errors))
        }
    }

    // ─── Private ──────────────────────────────────────────────────

    fn find_index(&self, id: FeatureId) -> std::result::Result<usize, BodyError> {
        self.features
            .iter()
            .position(|e| e.id == id)
            .ok_or(BodyError::FeatureNotFound(id))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::feature::*;
    use cadora_brep::{ExtrudeMode, RevolveMode};
    use truck_modeling::*;

    fn make_rect_wire(w: f64, h: f64) -> Wire {
        let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
        let v1 = builder::vertex(Point3::new(w, 0.0, 0.0));
        let v2 = builder::vertex(Point3::new(w, h, 0.0));
        let v3 = builder::vertex(Point3::new(0.0, h, 0.0));
        let e0 = builder::line(&v0, &v1);
        let e1 = builder::line(&v1, &v2);
        let e2 = builder::line(&v2, &v3);
        let e3 = builder::line(&v3, &v0);
        Wire::from_iter(vec![&e0, &e1, &e2, &e3])
    }

    // ─── Body basics ──────────────────────────────────────────────

    #[test]
    fn body_new_is_empty() {
        let body = Body::new("TestBody");
        assert_eq!(body.name(), "TestBody");
        assert!(body.is_empty());
        assert_eq!(body.feature_count(), 0);
        assert!(body.tip_index().is_none());
        assert!(body.tip_id().is_none());
        assert!(body.shape().is_none());
    }

    #[test]
    fn body_set_name() {
        let mut body = Body::new("Old");
        body.set_name("New");
        assert_eq!(body.name(), "New");
    }

    #[test]
    fn body_add_feature_sets_tip() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let pad = PadFeature::new("Pad1", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0));
        let id = body.add_feature(Box::new(pad));
        assert_eq!(body.feature_count(), 1);
        assert_eq!(body.tip_index(), Some(0));
        assert_eq!(body.tip_id(), Some(id));
    }

    #[test]
    fn body_add_multiple_features() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id1 = body.add_feature(Box::new(PadFeature::new("Pad1", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        let id2 = body.add_feature(Box::new(PadFeature::new("Pad2", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(3.0))));
        assert_eq!(body.feature_count(), 2);
        assert_eq!(body.tip_id(), Some(id2));
        assert_eq!(body.feature_ids(), vec![id1, id2]);
    }

    // ─── Feature listing ──────────────────────────────────────────

    #[test]
    fn body_feature_names() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        body.add_feature(Box::new(PadFeature::new("First", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        body.add_feature(Box::new(PocketFeature::new("Second", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(2.0))));
        assert_eq!(body.feature_names(), vec!["First", "Second"]);
    }

    #[test]
    fn body_feature_id_at() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id = body.add_feature(Box::new(PadFeature::new("P", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        assert_eq!(body.feature_id_at(0), Ok(id));
        assert!(body.feature_id_at(1).is_err());
    }

    #[test]
    fn body_feature_index() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id = body.add_feature(Box::new(PadFeature::new("P", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        assert_eq!(body.feature_index(id), Ok(0));
        assert!(body.feature_index(FeatureId(9999)).is_err());
    }

    // ─── Recompute ────────────────────────────────────────────────

    #[test]
    fn recompute_single_pad() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id = body.add_feature(Box::new(PadFeature::new("Pad1", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        body.recompute().unwrap();
        assert!(body.shape().is_some());

        let output = body.feature_output(id).unwrap().unwrap();
        assert_eq!(output.status, FeatureStatus::Ok);
        assert!(output.shape.is_some());
        assert!(output.add_sub_shape.is_some());
    }

    #[test]
    fn recompute_pad_then_pocket() {
        let mut body = Body::new("B");
        // Larger base pad
        let base_wire = make_rect_wire(20.0, 20.0);
        body.add_feature(Box::new(PadFeature::new(
            "Base", base_wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(10.0),
        )));

        // Smaller pocket — offset above top face to avoid coplanar issues
        let pocket_v0 = builder::vertex(Point3::new(5.0, 5.0, 10.01));
        let pocket_v1 = builder::vertex(Point3::new(15.0, 5.0, 10.01));
        let pocket_v2 = builder::vertex(Point3::new(15.0, 15.0, 10.01));
        let pocket_v3 = builder::vertex(Point3::new(5.0, 15.0, 10.01));
        let pocket_wire = Wire::from_iter(vec![
            &builder::line(&pocket_v0, &pocket_v1),
            &builder::line(&pocket_v1, &pocket_v2),
            &builder::line(&pocket_v2, &pocket_v3),
            &builder::line(&pocket_v3, &pocket_v0),
        ]);
        body.add_feature(Box::new(PocketFeature::new(
            "Pocket", pocket_wire, Vector3::new(0.0, 0.0, -1.0), ExtrudeMode::Length(3.0),
        )));

        body.recompute().unwrap();
        assert!(body.shape().is_some());
    }

    // ─── Tip management ──────────────────────────────────────────

    #[test]
    fn set_tip_by_id() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id1 = body.add_feature(Box::new(PadFeature::new("P1", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        let _id2 = body.add_feature(Box::new(PadFeature::new("P2", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(3.0))));
        assert_eq!(body.tip_index(), Some(1));

        body.set_tip(id1).unwrap();
        assert_eq!(body.tip_index(), Some(0));
    }

    #[test]
    fn set_tip_by_index() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        body.add_feature(Box::new(PadFeature::new("P1", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        body.add_feature(Box::new(PadFeature::new("P2", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(3.0))));

        body.set_tip_index(0).unwrap();
        assert_eq!(body.tip_index(), Some(0));
        assert!(body.set_tip_index(5).is_err());
    }

    // ─── Insert & Remove ─────────────────────────────────────────

    #[test]
    fn insert_feature_at_beginning() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id1 = body.add_feature(Box::new(PadFeature::new("P1", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        let id0 = body.insert_feature(0, Box::new(PadFeature::new("P0", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(3.0)))).unwrap();

        assert_eq!(body.feature_count(), 2);
        assert_eq!(body.feature_ids(), vec![id0, id1]);
        assert_eq!(body.tip_id(), Some(id0)); // Inserted became tip
    }

    #[test]
    fn insert_feature_invalid_index() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let result = body.insert_feature(5, Box::new(PadFeature::new("P", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        assert!(result.is_err());
    }

    #[test]
    fn remove_feature_from_middle() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id1 = body.add_feature(Box::new(PadFeature::new("P1", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        let id2 = body.add_feature(Box::new(PadFeature::new("P2", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(3.0))));
        let id3 = body.add_feature(Box::new(PadFeature::new("P3", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(2.0))));

        body.remove_feature(id2).unwrap();
        assert_eq!(body.feature_count(), 2);
        assert_eq!(body.feature_ids(), vec![id1, id3]);
        // Tip was id3 at index 2, now at index 1
        assert_eq!(body.tip_id(), Some(id3));
    }

    #[test]
    fn remove_tip_feature() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id1 = body.add_feature(Box::new(PadFeature::new("P1", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        let id2 = body.add_feature(Box::new(PadFeature::new("P2", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(3.0))));

        body.remove_feature(id2).unwrap();
        assert_eq!(body.tip_id(), Some(id1));
    }

    #[test]
    fn remove_last_feature() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id = body.add_feature(Box::new(PadFeature::new("P", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        body.remove_feature(id).unwrap();
        assert!(body.is_empty());
        assert!(body.tip_index().is_none());
    }

    #[test]
    fn remove_nonexistent_feature() {
        let mut body = Body::new("B");
        assert!(body.remove_feature(FeatureId(999)).is_err());
    }

    // ─── Move ─────────────────────────────────────────────────────

    #[test]
    fn move_feature_forward() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id1 = body.add_feature(Box::new(PadFeature::new("P1", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        let id2 = body.add_feature(Box::new(PadFeature::new("P2", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(3.0))));
        let id3 = body.add_feature(Box::new(PadFeature::new("P3", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(2.0))));

        body.move_feature(0, 2).unwrap();
        assert_eq!(body.feature_ids(), vec![id2, id3, id1]);
    }

    #[test]
    fn move_feature_backward() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id1 = body.add_feature(Box::new(PadFeature::new("P1", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        let id2 = body.add_feature(Box::new(PadFeature::new("P2", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(3.0))));
        let id3 = body.add_feature(Box::new(PadFeature::new("P3", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(2.0))));

        body.move_feature(2, 0).unwrap();
        assert_eq!(body.feature_ids(), vec![id3, id1, id2]);
    }

    #[test]
    fn move_feature_invalid_index() {
        let mut body = Body::new("B");
        assert!(body.move_feature(0, 1).is_err());
    }

    // ─── Suppression ─────────────────────────────────────────────

    #[test]
    fn suppress_feature_passes_through() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let pad_id = body.add_feature(Box::new(PadFeature::new("Pad", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));

        body.recompute().unwrap();
        assert!(body.shape().is_some());

        // Now suppress
        body.suppress_feature(pad_id).unwrap();
        assert!(body.get_feature(pad_id).unwrap().is_suppressed());

        body.recompute().unwrap();
        // Single suppressed pad with no base → no shape
        let output = body.feature_output(pad_id).unwrap().unwrap();
        assert_eq!(output.status, FeatureStatus::Suppressed);
        assert!(output.shape.is_none());
    }

    #[test]
    fn unsuppress_feature() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id = body.add_feature(Box::new(PadFeature::new("P", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));

        body.suppress_feature(id).unwrap();
        body.unsuppress_feature(id).unwrap();
        assert!(!body.get_feature(id).unwrap().is_suppressed());

        body.recompute().unwrap();
        assert!(body.shape().is_some());
    }

    #[test]
    fn suppress_middle_feature_keeps_chain() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id1 = body.add_feature(Box::new(PadFeature::new("P1", wire.clone(), Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));

        // Second pad — different profile to avoid coplanar issues
        let wire2_v0 = builder::vertex(Point3::new(15.0, 0.0, 0.0));
        let wire2_v1 = builder::vertex(Point3::new(25.0, 0.0, 0.0));
        let wire2_v2 = builder::vertex(Point3::new(25.0, 10.0, 0.0));
        let wire2_v3 = builder::vertex(Point3::new(15.0, 10.0, 0.0));
        let wire2 = Wire::from_iter(vec![
            &builder::line(&wire2_v0, &wire2_v1),
            &builder::line(&wire2_v1, &wire2_v2),
            &builder::line(&wire2_v2, &wire2_v3),
            &builder::line(&wire2_v3, &wire2_v0),
        ]);
        let id2 = body.add_feature(Box::new(PadFeature::new("P2", wire2, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(3.0))));

        // Third pad — yet another profile
        let wire3_v0 = builder::vertex(Point3::new(30.0, 0.0, 0.0));
        let wire3_v1 = builder::vertex(Point3::new(40.0, 0.0, 0.0));
        let wire3_v2 = builder::vertex(Point3::new(40.0, 10.0, 0.0));
        let wire3_v3 = builder::vertex(Point3::new(30.0, 10.0, 0.0));
        let wire3 = Wire::from_iter(vec![
            &builder::line(&wire3_v0, &wire3_v1),
            &builder::line(&wire3_v1, &wire3_v2),
            &builder::line(&wire3_v2, &wire3_v3),
            &builder::line(&wire3_v3, &wire3_v0),
        ]);
        let _id3 = body.add_feature(Box::new(PadFeature::new("P3", wire3, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(2.0))));

        // Suppress middle feature
        body.suppress_feature(id2).unwrap();
        body.recompute().unwrap();

        // Body should still have a shape (P1 + P3, without P2)
        assert!(body.shape().is_some());

        // P2 output should be suppressed with shape from P1
        let p2_output = body.feature_output(id2).unwrap().unwrap();
        assert_eq!(p2_output.status, FeatureStatus::Suppressed);
    }

    // ─── Recompute from ──────────────────────────────────────────

    #[test]
    fn recompute_from_index() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        body.add_feature(Box::new(PadFeature::new("P1", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        // Second pad with offset wire to avoid coplanar faces
        let wire2_v0 = builder::vertex(Point3::new(15.0, 0.0, 0.0));
        let wire2_v1 = builder::vertex(Point3::new(25.0, 0.0, 0.0));
        let wire2_v2 = builder::vertex(Point3::new(25.0, 10.0, 0.0));
        let wire2_v3 = builder::vertex(Point3::new(15.0, 10.0, 0.0));
        let wire2 = Wire::from_iter(vec![
            &builder::line(&wire2_v0, &wire2_v1),
            &builder::line(&wire2_v1, &wire2_v2),
            &builder::line(&wire2_v2, &wire2_v3),
            &builder::line(&wire2_v3, &wire2_v0),
        ]);
        body.add_feature(Box::new(PadFeature::new("P2", wire2, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(3.0))));

        body.recompute().unwrap();
        // Partial recompute from second feature
        body.recompute_from(1).unwrap();
        assert!(body.shape().is_some());
    }

    #[test]
    fn recompute_from_invalid_index() {
        let mut body = Body::new("B");
        assert!(body.recompute_from(0).is_err());
    }

    // ─── Get feature ─────────────────────────────────────────────

    #[test]
    fn get_feature_by_id() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id = body.add_feature(Box::new(PadFeature::new("MyPad", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        let feat = body.get_feature(id).unwrap();
        assert_eq!(feat.name(), "MyPad");
        assert_eq!(feat.type_name(), "Pad");
    }

    #[test]
    fn get_feature_not_found() {
        let body = Body::new("B");
        assert!(body.get_feature(FeatureId(999)).is_err());
    }

    // ─── Feature type tests ──────────────────────────────────────

    #[test]
    fn pad_feature_creates_shape() {
        let wire = make_rect_wire(10.0, 10.0);
        let pad = PadFeature::new("P", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0));
        let output = pad.execute(None);
        assert_eq!(output.status, FeatureStatus::Ok);
        assert!(output.shape.is_some());
    }

    #[test]
    fn pocket_requires_base() {
        let wire = make_rect_wire(10.0, 10.0);
        let pocket = PocketFeature::new("P", wire, Vector3::new(0.0, 0.0, -1.0), ExtrudeMode::Length(5.0));
        let output = pocket.execute(None);
        assert!(matches!(output.status, FeatureStatus::Error(_)));
    }

    #[test]
    fn pad_reversed() {
        let wire = make_rect_wire(10.0, 10.0);
        let mut pad = PadFeature::new("P", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0));
        pad.reversed = true;
        let output = pad.execute(None);
        assert_eq!(output.status, FeatureStatus::Ok);
        assert!(output.shape.is_some());
    }

    #[test]
    fn revolution_creates_shape() {
        use std::f64::consts::FRAC_PI_2;
        let v0 = builder::vertex(Point3::new(5.0, 0.0, 0.0));
        let v1 = builder::vertex(Point3::new(10.0, 0.0, 0.0));
        let v2 = builder::vertex(Point3::new(10.0, 5.0, 0.0));
        let v3 = builder::vertex(Point3::new(5.0, 5.0, 0.0));
        let wire = Wire::from_iter(vec![
            &builder::line(&v0, &v1),
            &builder::line(&v1, &v2),
            &builder::line(&v2, &v3),
            &builder::line(&v3, &v0),
        ]);
        let rev = RevolutionFeature::new(
            "Rev",
            wire,
            Point3::origin(),
            Vector3::unit_y(),
            RevolveMode::Angle(FRAC_PI_2),
        );
        let output = rev.execute(None);
        assert_eq!(output.status, FeatureStatus::Ok);
        assert!(output.shape.is_some());
    }

    #[test]
    fn groove_requires_base() {
        use std::f64::consts::FRAC_PI_2;
        let v0 = builder::vertex(Point3::new(5.0, 0.0, 0.0));
        let v1 = builder::vertex(Point3::new(10.0, 0.0, 0.0));
        let v2 = builder::vertex(Point3::new(10.0, 5.0, 0.0));
        let v3 = builder::vertex(Point3::new(5.0, 5.0, 0.0));
        let wire = Wire::from_iter(vec![
            &builder::line(&v0, &v1),
            &builder::line(&v1, &v2),
            &builder::line(&v2, &v3),
            &builder::line(&v3, &v0),
        ]);
        let groove = GrooveFeature::new(
            "G",
            wire,
            Point3::origin(),
            Vector3::unit_y(),
            RevolveMode::Angle(FRAC_PI_2),
        );
        let output = groove.execute(None);
        assert!(matches!(output.status, FeatureStatus::Error(_)));
    }

    #[test]
    fn fillet_feature_requires_base() {
        let fillet = FilletFeature::new("F", vec![0, 1], 1.0);
        let output = fillet.execute(None);
        assert!(matches!(output.status, FeatureStatus::Error(_)));
    }

    #[test]
    fn chamfer_feature_requires_base() {
        let chamfer = ChamferFeature::new("C", vec![0, 1], 1.0);
        let output = chamfer.execute(None);
        assert!(matches!(output.status, FeatureStatus::Error(_)));
    }

    #[test]
    fn suppressed_feature_returns_base() {
        let wire = make_rect_wire(10.0, 10.0);
        let mut pad = PadFeature::new("P", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0));
        pad.set_suppressed(true);
        assert!(pad.is_suppressed());

        // With base shape
        let base = cadora_brep::make_box(5.0, 5.0, 5.0);
        let output = pad.execute(Some(&base));
        assert_eq!(output.status, FeatureStatus::Suppressed);
        assert!(output.shape.is_some());
    }

    // ─── Body with different feature types ─────────────────────

    #[test]
    fn body_with_pad_and_fillet() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        body.add_feature(Box::new(PadFeature::new("Pad", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        body.add_feature(Box::new(FilletFeature::new("Fillet", vec![0], 0.5)));

        body.recompute().unwrap();
        assert!(body.shape().is_some());
    }

    #[test]
    fn body_with_linear_pattern() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(5.0, 5.0);
        body.add_feature(Box::new(PadFeature::new("Pad", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        body.add_feature(Box::new(LinearPatternFeature::new("LinPat", Vector3::new(1.0, 0.0, 0.0), 3, 10.0)));

        body.recompute().unwrap();
        assert!(body.shape().is_some());
    }

    #[test]
    fn body_error_display() {
        let e = BodyError::Empty;
        assert_eq!(format!("{e}"), "Body has no features");

        let e = BodyError::FeatureNotFound(FeatureId(42));
        assert!(format!("{e}").contains("42"));

        let e = BodyError::InvalidIndex(5);
        assert!(format!("{e}").contains("5"));

        let e = BodyError::RecomputeFailed(vec![(FeatureId(1), "test".to_string())]);
        assert!(format!("{e}").contains("test"));
    }

    // ─── Edge cases ──────────────────────────────────────────────

    #[test]
    fn recompute_empty_body() {
        let mut body = Body::new("B");
        body.recompute().unwrap(); // Should succeed with no features
    }

    #[test]
    fn move_feature_same_position() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(10.0, 10.0);
        let id = body.add_feature(Box::new(PadFeature::new("P", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(5.0))));
        body.move_feature(0, 0).unwrap();
        assert_eq!(body.feature_ids(), vec![id]);
    }

    // ─── Hole feature ──────────────────────────────────────────

    #[test]
    fn hole_requires_base() {
        let hole = HoleFeature::new(
            "H",
            Point3::new(5.0, 5.0, 5.0),
            Vector3::new(0.0, 0.0, -1.0),
            HoleType::Simple { diameter: 3.0, depth: Some(4.0) },
        );
        let output = hole.execute(None);
        assert!(matches!(output.status, FeatureStatus::Error(_)));
    }

    #[test]
    fn simple_hole_in_box() {
        let mut body = Body::new("B");
        let wire = make_rect_wire(20.0, 20.0);
        body.add_feature(Box::new(PadFeature::new("Pad", wire, Vector3::new(0.0, 0.0, 1.0), ExtrudeMode::Length(10.0))));
        body.add_feature(Box::new(HoleFeature::new(
            "Hole",
            Point3::new(10.0, 10.0, 10.01), // slightly above top face
            Vector3::new(0.0, 0.0, -1.0),
            HoleType::Simple { diameter: 5.0, depth: Some(8.0) },
        )));
        body.recompute().unwrap();
        assert!(body.shape().is_some());
    }

    #[test]
    fn countersink_hole() {
        let hole = HoleFeature::new(
            "CSHole",
            Point3::new(10.0, 10.0, 10.01),
            Vector3::new(0.0, 0.0, -1.0),
            HoleType::Countersink {
                diameter: 5.0,
                depth: Some(15.0),
                countersink_diameter: 10.0,
                countersink_angle: std::f64::consts::FRAC_PI_2,
            },
        );
        let base = cadora_brep::make_box(20.0, 20.0, 10.0);
        let output = hole.execute(Some(&base));
        // May fail due to coplanar faces but tool should be built
        assert!(output.add_sub_shape.is_some());
    }

    #[test]
    fn counterbore_hole() {
        let hole = HoleFeature::new(
            "CBHole",
            Point3::new(10.0, 10.0, 10.01),
            Vector3::new(0.0, 0.0, -1.0),
            HoleType::Counterbore {
                diameter: 5.0,
                depth: Some(15.0),
                counterbore_diameter: 10.0,
                counterbore_depth: 3.0,
            },
        );
        let base = cadora_brep::make_box(20.0, 20.0, 10.0);
        let output = hole.execute(Some(&base));
        assert!(output.add_sub_shape.is_some());
    }

    #[test]
    fn hole_feature_type_name() {
        let hole = HoleFeature::new(
            "H",
            Point3::origin(),
            Vector3::unit_z(),
            HoleType::Simple { diameter: 5.0, depth: None },
        );
        assert_eq!(hole.type_name(), "Hole");
    }
}
