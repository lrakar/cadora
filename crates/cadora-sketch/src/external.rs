//! External geometry references (Phase A7).
//!
//! Manages geometry projected from 3D model edges/vertices onto the sketch
//! plane. Mirrors FreeCAD's external geometry system where external GeoIds
//! are `<= -3` with the mapping `index = -(geo_id + 3)`.

use crate::geometry::{GeoDef, GeoType};
use crate::sketch::Sketch;
use crate::types::*;

/// Flags for an external geometry reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExternalFlags {
    /// External geometry contributes to output shape.
    pub defining: bool,
    /// Frozen: don't update from external source.
    pub frozen: bool,
    /// Detached from external reference (now standalone).
    pub detached: bool,
    /// External reference is missing/broken.
    pub missing: bool,
}

impl Default for ExternalFlags {
    fn default() -> Self {
        Self {
            defining: false,
            frozen: false,
            detached: false,
            missing: false,
        }
    }
}

/// An external geometry reference — links a source identifier to projected
/// sketch geometry.
#[derive(Debug, Clone)]
pub struct ExternalRef {
    /// Source identifier (e.g., "Part.Edge3", "Body.Face1").
    pub source_key: String,
    /// Flags controlling update behavior.
    pub flags: ExternalFlags,
    /// The GeoId assigned to this external geometry in the sketch.
    pub geo_id: GeoId,
}

impl Sketch {
    /// Add an external geometry reference.
    ///
    /// The geometry is stored in the external slot and assigned a negative
    /// GeoId (`<= -3`). Constraints can reference it like any other geometry.
    ///
    /// `source_key` is an opaque identifier for the external shape (used for
    /// rebuild tracking).
    ///
    /// Returns the GeoId of the external geometry.
    pub fn add_external(
        &mut self,
        source_key: String,
        geo: GeoType,
    ) -> GeoId {
        let geo_id = self.store.add_external(GeoDef::new(geo));
        self.external_refs.push(ExternalRef {
            source_key,
            flags: ExternalFlags::default(),
            geo_id,
        });
        self.last_status = None;
        self.last_diagnosis = None;
        geo_id
    }

    /// Remove an external geometry reference by GeoId.
    ///
    /// Also removes all constraints referencing this geometry.
    /// Returns true if the geometry existed and was removed.
    pub fn del_external(&mut self, geo_id: GeoId) -> bool {
        if geo_id > -3 {
            return false; // not an external GeoId
        }

        // Remove constraints referencing this external geometry
        self.constraints
            .retain(|c| !c.elements.iter().any(|e| e.geo_id == geo_id));

        // Remove the external ref record
        self.external_refs.retain(|r| r.geo_id != geo_id);

        // Remove from store
        let removed = self.store.remove_external(geo_id);
        if removed {
            // Remap constraints: external GeoIds below this one shift up
            for c in &mut self.constraints {
                for e in &mut c.elements {
                    if e.geo_id < geo_id {
                        e.geo_id += 1;
                    }
                }
            }
            // Remap external_refs too
            for r in &mut self.external_refs {
                if r.geo_id < geo_id {
                    r.geo_id += 1;
                }
            }
        }

        self.last_status = None;
        self.last_diagnosis = None;
        removed
    }

    /// Update an external geometry's projected shape (e.g., after the 3D
    /// model changed).
    ///
    /// Preserves constraints referencing this geometry.
    pub fn update_external(&mut self, geo_id: GeoId, new_geo: GeoType) -> bool {
        if let Some(g) = self.store.get_mut(geo_id) {
            g.geo = new_geo;
            self.last_status = None;
            self.last_diagnosis = None;
            true
        } else {
            false
        }
    }

    /// Rebuild all external geometry from source keys using a callback.
    ///
    /// The callback receives the source key and should return the updated
    /// geometry, or `None` if the source is missing.
    pub fn rebuild_external<F>(&mut self, mut resolver: F)
    where
        F: FnMut(&str) -> Option<GeoType>,
    {
        for ext_ref in &mut self.external_refs {
            if ext_ref.flags.frozen || ext_ref.flags.detached {
                continue;
            }
            match resolver(&ext_ref.source_key) {
                Some(new_geo) => {
                    ext_ref.flags.missing = false;
                    if let Some(g) = self.store.get_mut(ext_ref.geo_id) {
                        g.geo = new_geo;
                    }
                }
                None => {
                    ext_ref.flags.missing = true;
                }
            }
        }
        self.last_status = None;
        self.last_diagnosis = None;
    }

    /// Get external geometry references.
    pub fn external_refs(&self) -> &[ExternalRef] {
        &self.external_refs
    }

    /// Number of external geometry elements.
    pub fn external_count(&self) -> usize {
        self.store.external_len()
    }

    /// Set flags on an external geometry reference.
    pub fn set_external_flags(&mut self, geo_id: GeoId, flags: ExternalFlags) -> bool {
        if let Some(r) = self.external_refs.iter_mut().find(|r| r.geo_id == geo_id) {
            r.flags = flags;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_external_geometry() {
        let mut sketch = Sketch::new();
        let ext_id = sketch.add_external(
            "Part.Edge1".to_string(),
            GeoType::Line {
                x1: 0.0,
                y1: 0.0,
                x2: 10.0,
                y2: 0.0,
            },
        );
        assert!(ext_id <= -3);
        assert_eq!(sketch.external_count(), 1);

        let g = sketch.get_geometry(ext_id).unwrap();
        assert!(matches!(g.geo, GeoType::Line { .. }));
    }

    #[test]
    fn del_external_removes_constraints() {
        let mut sketch = Sketch::new();
        let ext_id = sketch.add_external(
            "Part.Edge1".to_string(),
            GeoType::Point { x: 5.0, y: 5.0 },
        );
        let int_id = sketch.add_geometry(GeoDef::new(GeoType::Point {
            x: 0.0,
            y: 0.0,
        }));

        // Constrain internal point coincident with external
        sketch.add_constraint(crate::constraint::SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::mid(int_id), GeoElementId::mid(ext_id)],
        ));
        assert_eq!(sketch.constraint_count(), 1);

        sketch.del_external(ext_id);
        assert_eq!(sketch.external_count(), 0);
        assert_eq!(sketch.constraint_count(), 0);
    }

    #[test]
    fn update_external_geometry() {
        let mut sketch = Sketch::new();
        let ext_id = sketch.add_external(
            "Part.Edge1".to_string(),
            GeoType::Point { x: 1.0, y: 2.0 },
        );

        sketch.update_external(
            ext_id,
            GeoType::Point { x: 3.0, y: 4.0 },
        );

        let g = sketch.get_geometry(ext_id).unwrap();
        if let GeoType::Point { x, y } = &g.geo {
            assert!((*x - 3.0).abs() < 1e-10);
            assert!((*y - 4.0).abs() < 1e-10);
        }
    }

    #[test]
    fn rebuild_external_with_resolver() {
        let mut sketch = Sketch::new();
        sketch.add_external(
            "Part.Edge1".to_string(),
            GeoType::Point { x: 0.0, y: 0.0 },
        );
        sketch.add_external(
            "Part.Edge2".to_string(),
            GeoType::Point { x: 1.0, y: 1.0 },
        );

        // Rebuild: Edge1 updates, Edge2 is missing
        sketch.rebuild_external(|key| match key {
            "Part.Edge1" => Some(GeoType::Point { x: 10.0, y: 20.0 }),
            _ => None,
        });

        let refs = sketch.external_refs();
        assert!(!refs[0].flags.missing);
        assert!(refs[1].flags.missing);
    }

    #[test]
    fn external_geo_ids_are_negative() {
        let mut sketch = Sketch::new();
        let id1 = sketch.add_external("A".to_string(), GeoType::Point { x: 0.0, y: 0.0 });
        let id2 = sketch.add_external("B".to_string(), GeoType::Point { x: 1.0, y: 1.0 });
        assert_eq!(id1, -3);
        assert_eq!(id2, -4);
    }
}
