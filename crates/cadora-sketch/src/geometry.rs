//! Geometry storage and identification for sketches.
//!
//! Manages a list of geometry elements with stable [`GeoId`] addressing
//! and built-in reference axes. Equivalent to FreeCAD's `Geoms` vector.

use crate::types::{GeoId, GeometryMode, InternalType, PointPos, GeoElementId,
                   GEO_ID_H_AXIS, GEO_ID_V_AXIS};

/// The kind of geometry primitive.
#[derive(Debug, Clone)]
pub enum GeoType {
    Point { x: f64, y: f64 },
    Line { x1: f64, y1: f64, x2: f64, y2: f64 },
    Circle { cx: f64, cy: f64, radius: f64 },
    Arc { cx: f64, cy: f64, radius: f64, start_angle: f64, end_angle: f64 },
    Ellipse { cx: f64, cy: f64, fx: f64, fy: f64, radmin: f64 },
    ArcOfEllipse { cx: f64, cy: f64, fx: f64, fy: f64, radmin: f64, start_angle: f64, end_angle: f64 },
    Hyperbola { cx: f64, cy: f64, fx: f64, fy: f64, radmin: f64 },
    ArcOfHyperbola { cx: f64, cy: f64, fx: f64, fy: f64, radmin: f64, start_angle: f64, end_angle: f64 },
    Parabola { vx: f64, vy: f64, fx: f64, fy: f64 },
    ArcOfParabola { vx: f64, vy: f64, fx: f64, fy: f64, start_angle: f64, end_angle: f64 },
    BSpline {
        poles: Vec<(f64, f64)>,
        weights: Vec<f64>,
        knots: Vec<f64>,
        multiplicities: Vec<usize>,
        degree: usize,
        periodic: bool,
    },
}

impl GeoType {
    /// Returns the start point of a geometry, if applicable.
    pub fn start_point(&self) -> Option<(f64, f64)> {
        match self {
            GeoType::Line { x1, y1, .. } => Some((*x1, *y1)),
            GeoType::Arc { cx, cy, radius, start_angle, .. } => {
                Some((cx + radius * start_angle.cos(), cy + radius * start_angle.sin()))
            }
            _ => None,
        }
    }

    /// Returns the end point of a geometry, if applicable.
    pub fn end_point(&self) -> Option<(f64, f64)> {
        match self {
            GeoType::Line { x2, y2, .. } => Some((*x2, *y2)),
            GeoType::Arc { cx, cy, radius, end_angle, .. } => {
                Some((cx + radius * end_angle.cos(), cy + radius * end_angle.sin()))
            }
            _ => None,
        }
    }

    /// Returns the center/midpoint of a geometry, if applicable.
    pub fn mid_point(&self) -> Option<(f64, f64)> {
        match self {
            GeoType::Point { x, y } => Some((*x, *y)),
            GeoType::Circle { cx, cy, .. }
            | GeoType::Arc { cx, cy, .. }
            | GeoType::Ellipse { cx, cy, .. }
            | GeoType::ArcOfEllipse { cx, cy, .. }
            | GeoType::Hyperbola { cx, cy, .. }
            | GeoType::ArcOfHyperbola { cx, cy, .. } => Some((*cx, *cy)),
            GeoType::Parabola { vx, vy, .. }
            | GeoType::ArcOfParabola { vx, vy, .. } => Some((*vx, *vy)),
            GeoType::Line { x1, y1, x2, y2 } => Some(((x1 + x2) / 2.0, (y1 + y2) / 2.0)),
            _ => None,
        }
    }

    /// Returns the point for a given [`PointPos`].
    pub fn point_at(&self, pos: PointPos) -> Option<(f64, f64)> {
        match pos {
            PointPos::None => None,
            PointPos::Start => self.start_point(),
            PointPos::End => self.end_point(),
            PointPos::Mid => self.mid_point(),
        }
    }
}

/// A geometry element stored in the sketch.
#[derive(Debug, Clone)]
pub struct GeoDef {
    /// The geometric primitive and its parameter values.
    pub geo: GeoType,
    /// Display/behavior flags (construction, blocked).
    pub mode: GeometryMode,
    /// Classification for internal alignment geometry.
    pub internal_type: InternalType,
    /// Unique identifier within the sketch (monotonically increasing).
    pub(crate) id: u64,
}

impl GeoDef {
    pub fn new(geo: GeoType) -> Self {
        Self {
            geo,
            mode: GeometryMode::default(),
            internal_type: InternalType::None,
            id: 0,
        }
    }

    pub fn with_construction(mut self, construction: bool) -> Self {
        self.mode.construction = construction;
        self
    }
}

/// Manages the geometry list for a sketch, including built-in axes.
pub(crate) struct GeometryStore {
    /// User geometry, indexed by position (GeoId = index).
    geometries: Vec<GeoDef>,
    /// External geometry, indexed as -(geo_id + 3).
    external: Vec<GeoDef>,
    /// Monotonically increasing ID counter for unique geometry IDs.
    next_id: u64,
}

impl GeometryStore {
    pub fn new() -> Self {
        Self {
            geometries: Vec::new(),
            external: Vec::new(),
            next_id: 0,
        }
    }

    /// Number of user geometries.
    pub fn len(&self) -> usize {
        self.geometries.len()
    }

    /// Add a geometry element, returning its GeoId.
    pub fn add(&mut self, mut geo_def: GeoDef) -> GeoId {
        geo_def.id = self.next_id;
        self.next_id += 1;
        let id = self.geometries.len() as GeoId;
        self.geometries.push(geo_def);
        id
    }

    /// Remove a geometry by GeoId. Returns the removed element.
    /// Note: this shifts subsequent GeoIds — callers must remap constraints.
    pub fn remove(&mut self, geo_id: GeoId) -> Option<GeoDef> {
        let idx = geo_id as usize;
        if idx < self.geometries.len() {
            Some(self.geometries.remove(idx))
        } else {
            None
        }
    }

    /// Get a geometry by GeoId.
    pub fn get(&self, geo_id: GeoId) -> Option<&GeoDef> {
        if geo_id >= 0 {
            self.geometries.get(geo_id as usize)
        } else if geo_id <= -3 {
            let ext_idx = (-(geo_id + 3)) as usize;
            self.external.get(ext_idx)
        } else {
            // Axis geometry — synthesized on demand
            None
        }
    }

    /// Get a mutable reference to a geometry by GeoId.
    pub fn get_mut(&mut self, geo_id: GeoId) -> Option<&mut GeoDef> {
        if geo_id >= 0 {
            self.geometries.get_mut(geo_id as usize)
        } else if geo_id <= -3 {
            let ext_idx = (-(geo_id + 3)) as usize;
            self.external.get_mut(ext_idx)
        } else {
            None
        }
    }

    /// Iterate over all user geometries with their GeoIds.
    pub fn iter(&self) -> impl Iterator<Item = (GeoId, &GeoDef)> {
        self.geometries.iter().enumerate().map(|(i, g)| (i as GeoId, g))
    }

    /// Get the point coordinates referenced by a GeoElementId.
    pub fn get_point(&self, elem: GeoElementId) -> Option<(f64, f64)> {
        // Special handling for axes
        if elem.geo_id == GEO_ID_H_AXIS {
            return match elem.pos {
                PointPos::Start => Some((0.0, 0.0)), // origin
                PointPos::End => Some((1.0, 0.0)),    // +X direction
                PointPos::Mid => Some((0.0, 0.0)),    // origin
                PointPos::None => None,
            };
        }
        if elem.geo_id == GEO_ID_V_AXIS {
            return match elem.pos {
                PointPos::Start => Some((0.0, 0.0)),
                PointPos::End => Some((0.0, 1.0)),    // +Y direction
                PointPos::Mid => Some((0.0, 0.0)),
                PointPos::None => None,
            };
        }

        self.get(elem.geo_id).and_then(|gd| gd.geo.point_at(elem.pos))
    }

    /// Clear all geometry.
    pub fn clear(&mut self) {
        self.geometries.clear();
        self.external.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_and_get_geometry() {
        let mut store = GeometryStore::new();
        let id = store.add(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        assert_eq!(id, 0);
        assert_eq!(store.len(), 1);

        let g = store.get(id).unwrap();
        assert!(matches!(g.geo, GeoType::Line { .. }));
    }

    #[test]
    fn sequential_geo_ids() {
        let mut store = GeometryStore::new();
        let id0 = store.add(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        let id1 = store.add(GeoDef::new(GeoType::Point { x: 1.0, y: 1.0 }));
        let id2 = store.add(GeoDef::new(GeoType::Circle { cx: 0.0, cy: 0.0, radius: 5.0 }));
        assert_eq!(id0, 0);
        assert_eq!(id1, 1);
        assert_eq!(id2, 2);
    }

    #[test]
    fn remove_geometry() {
        let mut store = GeometryStore::new();
        store.add(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        store.add(GeoDef::new(GeoType::Point { x: 1.0, y: 1.0 }));
        assert_eq!(store.len(), 2);

        let removed = store.remove(0);
        assert!(removed.is_some());
        assert_eq!(store.len(), 1);
    }

    #[test]
    fn axis_points() {
        let store = GeometryStore::new();

        // Origin via H-axis start
        let origin = store.get_point(GeoElementId::start(GEO_ID_H_AXIS));
        assert_eq!(origin, Some((0.0, 0.0)));

        // H-axis end = (1, 0)
        let hend = store.get_point(GeoElementId::end(GEO_ID_H_AXIS));
        assert_eq!(hend, Some((1.0, 0.0)));

        // V-axis end = (0, 1)
        let vend = store.get_point(GeoElementId::end(GEO_ID_V_AXIS));
        assert_eq!(vend, Some((0.0, 1.0)));
    }

    #[test]
    fn line_points() {
        let mut store = GeometryStore::new();
        store.add(GeoDef::new(GeoType::Line {
            x1: 1.0, y1: 2.0, x2: 3.0, y2: 4.0,
        }));

        let start = store.get_point(GeoElementId::start(0));
        assert_eq!(start, Some((1.0, 2.0)));

        let end = store.get_point(GeoElementId::end(0));
        assert_eq!(end, Some((3.0, 4.0)));

        let mid = store.get_point(GeoElementId::mid(0));
        assert_eq!(mid, Some((2.0, 3.0)));
    }

    #[test]
    fn construction_mode() {
        let mut store = GeometryStore::new();
        let id = store.add(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 1.0, y2: 0.0,
        }).with_construction(true));

        assert!(store.get(id).unwrap().mode.construction);
    }

    #[test]
    fn get_nonexistent_returns_none() {
        let store = GeometryStore::new();
        assert!(store.get(0).is_none());
        assert!(store.get(99).is_none());
        assert!(store.get(-3).is_none()); // no external geometry
    }

    #[test]
    fn clear_removes_all() {
        let mut store = GeometryStore::new();
        store.add(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        store.add(GeoDef::new(GeoType::Point { x: 1.0, y: 1.0 }));
        assert_eq!(store.len(), 2);
        store.clear();
        assert_eq!(store.len(), 0);
    }

    #[test]
    fn iter_yields_all_geometries() {
        let mut store = GeometryStore::new();
        store.add(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        store.add(GeoDef::new(GeoType::Line { x1: 0.0, y1: 0.0, x2: 1.0, y2: 1.0 }));

        let items: Vec<_> = store.iter().collect();
        assert_eq!(items.len(), 2);
        assert_eq!(items[0].0, 0);
        assert_eq!(items[1].0, 1);
    }

    #[test]
    fn unique_ids_persist_after_remove() {
        let mut store = GeometryStore::new();
        store.add(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 })); // id=0, uniqueId=0
        store.add(GeoDef::new(GeoType::Point { x: 1.0, y: 1.0 })); // id=1, uniqueId=1
        store.remove(0);
        let new_id = store.add(GeoDef::new(GeoType::Point { x: 2.0, y: 2.0 }));
        // GeoId recycles (shifted) but internal unique id keeps incrementing
        let g = store.get(new_id).unwrap();
        assert_eq!(g.id, 2); // third geometry ever created
    }
}
