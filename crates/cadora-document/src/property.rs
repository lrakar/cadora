//! Property system — typed values, status flags, and property containers.
//!
//! Mirrors FreeCAD's App::Property hierarchy using Rust enums instead of
//! C++ class inheritance. Every DocumentObject stores its data as named
//! properties in a PropertyBag.

use crate::placement::Placement;
use crate::object::ObjectId;

// ── Colour ─────────────────────────────────────────────────────────

/// RGBA colour with f32 channels in \[0,1\].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Color {
    pub fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }
    pub fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b, a: 1.0 }
    }
}

impl Default for Color {
    fn default() -> Self {
        Self { r: 0.0, g: 0.0, b: 0.0, a: 1.0 }
    }
}

// ── Unit system ────────────────────────────────────────────────────

/// Physical-quantity unit categories (mirrors FreeCAD's 59+ PropertyUnits
/// collapsed into a single enum — concrete types are all PropertyQuantity +
/// this discriminator).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Unit {
    None,
    Length,
    Angle,
    Area,
    Volume,
    Mass,
    Time,
    Speed,
    Acceleration,
    Force,
    Pressure,
    Stiffness,
    Power,
    Frequency,
    Temperature,
}

// ── PropValue ──────────────────────────────────────────────────────

/// Dynamically-typed property value.
///
/// Covers the most-used FreeCAD property types as enum variants instead of
/// separate C++ classes. This keeps storage homogeneous while still allowing
/// typed access through pattern matching.
#[derive(Debug, Clone)]
pub enum PropValue {
    Integer(i64),
    Float(f64),
    String(String),
    Bool(bool),

    IntegerList(Vec<i64>),
    FloatList(Vec<f64>),
    StringList(Vec<String>),

    /// 3-component vector.
    Vector3([f64; 3]),

    /// Position + rotation in 3-D space.
    Placement(Placement),

    /// Single link to another object (None = unlinked).
    Link(Option<ObjectId>),

    /// Multiple links.
    LinkList(Vec<ObjectId>),

    /// Link + sub-element names (e.g. "Face1", "Edge3").
    LinkSub(Option<ObjectId>, Vec<String>),

    /// Multiple links each with sub-element names.
    LinkSubList(Vec<(ObjectId, Vec<String>)>),

    /// Enumeration (list of allowed strings + selected index).
    Enumeration {
        options: Vec<String>,
        selected: usize,
    },

    /// Physical quantity (value + unit).
    Quantity { value: f64, unit: Unit },

    /// RGBA colour.
    Color(Color),
}

// ── PropertyStatus ─────────────────────────────────────────────────

/// Bit-flags mirroring FreeCAD Property::Status.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PropertyStatus(u32);

impl PropertyStatus {
    pub const TOUCHED: u32       = 1 << 0;
    pub const IMMUTABLE: u32     = 1 << 1;
    pub const READ_ONLY: u32     = 1 << 2;
    pub const HIDDEN: u32        = 1 << 3;
    pub const TRANSIENT: u32     = 1 << 4;
    pub const OUTPUT: u32        = 1 << 7;
    pub const NO_RECOMPUTE: u32  = 1 << 11;

    pub fn new() -> Self { Self(0) }

    pub fn set(&mut self, flag: u32, on: bool) {
        if on { self.0 |= flag; } else { self.0 &= !flag; }
    }
    pub fn test(&self, flag: u32) -> bool {
        self.0 & flag != 0
    }
    pub fn bits(&self) -> u32 { self.0 }
}

// ── Property ───────────────────────────────────────────────────────

/// A named, status-tracked property value.
#[derive(Debug, Clone)]
pub struct Property {
    pub value: PropValue,
    pub status: PropertyStatus,
}

impl Property {
    pub fn new(value: PropValue) -> Self {
        Self { value, status: PropertyStatus::new() }
    }

    pub fn touch(&mut self) {
        self.status.set(PropertyStatus::TOUCHED, true);
    }
    pub fn is_touched(&self) -> bool {
        self.status.test(PropertyStatus::TOUCHED)
    }
    pub fn purge_touched(&mut self) {
        self.status.set(PropertyStatus::TOUCHED, false);
    }

    /// Clone this property for undo/redo snapshots — equivalent to
    /// FreeCAD Property::Copy().
    pub fn snapshot(&self) -> Self {
        self.clone()
    }

    /// Restore value from a snapshot — equivalent to FreeCAD Property::Paste().
    pub fn restore(&mut self, from: &Property) {
        self.value = from.value.clone();
        // Keep current status, only copy Touched bit
        self.status.set(PropertyStatus::TOUCHED, from.status.test(PropertyStatus::TOUCHED));
    }
}

// ── PropertyBag ────────────────────────────────────────────────────

/// Ordered map of named properties — the property container for a
/// DocumentObject.  Uses a Vec for insertion-order + O(n) lookup by name
/// (property counts per object are small, typically < 20).
#[derive(Debug, Clone, Default)]
pub struct PropertyBag {
    entries: Vec<(String, Property)>,
}

impl PropertyBag {
    pub fn new() -> Self { Self { entries: Vec::new() } }

    /// Insert or replace a property.  Returns the old property if replaced.
    pub fn set(&mut self, name: impl Into<String>, value: PropValue) -> Option<Property> {
        let name = name.into();
        for entry in &mut self.entries {
            if entry.0 == name {
                let old = entry.1.clone();
                entry.1 = Property::new(value);
                entry.1.touch();
                return Some(old);
            }
        }
        let mut prop = Property::new(value);
        prop.touch();
        self.entries.push((name, prop));
        None
    }

    /// Get a property by name.
    pub fn get(&self, name: &str) -> Option<&Property> {
        self.entries.iter().find(|(n, _)| n == name).map(|(_, p)| p)
    }

    /// Get a mutable reference to a property by name.
    pub fn get_mut(&mut self, name: &str) -> Option<&mut Property> {
        self.entries.iter_mut().find(|(n, _)| n == name).map(|(_, p)| p)
    }

    /// Remove a property by name.
    pub fn remove(&mut self, name: &str) -> Option<Property> {
        if let Some(pos) = self.entries.iter().position(|(n, _)| n == name) {
            Some(self.entries.remove(pos).1)
        } else {
            None
        }
    }

    /// Number of properties.
    pub fn len(&self) -> usize { self.entries.len() }
    pub fn is_empty(&self) -> bool { self.entries.is_empty() }

    /// Iterate over (name, property) pairs.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &Property)> {
        self.entries.iter().map(|(n, p)| (n.as_str(), p))
    }

    /// Iterate mutably.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (&str, &mut Property)> {
        self.entries.iter_mut().map(|(n, p)| (n.as_str(), p))
    }

    /// Check if any property is touched.
    pub fn any_touched(&self) -> bool {
        self.entries.iter().any(|(_, p)| p.is_touched())
    }

    /// Purge all touched flags.
    pub fn purge_all_touched(&mut self) {
        for (_, p) in &mut self.entries {
            p.purge_touched();
        }
    }

    /// Property names.
    pub fn names(&self) -> Vec<&str> {
        self.entries.iter().map(|(n, _)| n.as_str()).collect()
    }

    // ── typed getters ──────────────────────────────────────────────

    pub fn get_integer(&self, name: &str) -> Option<i64> {
        match self.get(name)?.value {
            PropValue::Integer(v) => Some(v),
            _ => None,
        }
    }
    pub fn get_float(&self, name: &str) -> Option<f64> {
        match self.get(name)?.value {
            PropValue::Float(v) => Some(v),
            PropValue::Quantity { value, .. } => Some(value),
            _ => None,
        }
    }
    pub fn get_string(&self, name: &str) -> Option<&str> {
        match &self.get(name)?.value {
            PropValue::String(s) => Some(s.as_str()),
            _ => None,
        }
    }
    pub fn get_bool(&self, name: &str) -> Option<bool> {
        match self.get(name)?.value {
            PropValue::Bool(v) => Some(v),
            _ => None,
        }
    }
    pub fn get_vector3(&self, name: &str) -> Option<[f64; 3]> {
        match self.get(name)?.value {
            PropValue::Vector3(v) => Some(v),
            _ => None,
        }
    }
    pub fn get_placement(&self, name: &str) -> Option<&Placement> {
        match &self.get(name)?.value {
            PropValue::Placement(p) => Some(p),
            _ => None,
        }
    }
    pub fn get_link(&self, name: &str) -> Option<Option<ObjectId>> {
        match &self.get(name)?.value {
            PropValue::Link(id) => Some(*id),
            _ => None,
        }
    }
    pub fn get_link_list(&self, name: &str) -> Option<&[ObjectId]> {
        match &self.get(name)?.value {
            PropValue::LinkList(ids) => Some(ids.as_slice()),
            _ => None,
        }
    }

    /// Collect all ObjectIds referenced by Link/LinkList/LinkSub/LinkSubList
    /// properties — used for building the dependency graph.
    pub fn referenced_objects(&self) -> Vec<ObjectId> {
        let mut refs = Vec::new();
        for (_, prop) in &self.entries {
            match &prop.value {
                PropValue::Link(Some(id)) => refs.push(*id),
                PropValue::LinkList(ids) => refs.extend(ids),
                PropValue::LinkSub(Some(id), _) => refs.push(*id),
                PropValue::LinkSubList(entries) => {
                    for (id, _) in entries {
                        refs.push(*id);
                    }
                }
                _ => {}
            }
        }
        refs
    }

    // ── typed setters (touch automatically) ────────────────────────

    pub fn set_integer(&mut self, name: &str, val: i64) -> bool {
        if let Some(prop) = self.get_mut(name) {
            if !matches!(prop.value, PropValue::Integer(_)) { return false; }
            prop.value = PropValue::Integer(val);
            prop.touch();
            true
        } else {
            false
        }
    }
    pub fn set_float(&mut self, name: &str, val: f64) -> bool {
        if let Some(prop) = self.get_mut(name) {
            match &mut prop.value {
                PropValue::Float(v) => *v = val,
                PropValue::Quantity { value, .. } => *value = val,
                _ => return false,
            }
            prop.touch();
            true
        } else {
            false
        }
    }
    pub fn set_string(&mut self, name: &str, val: impl Into<String>) -> bool {
        if let Some(prop) = self.get_mut(name) {
            if !matches!(prop.value, PropValue::String(_)) { return false; }
            prop.value = PropValue::String(val.into());
            prop.touch();
            true
        } else {
            false
        }
    }
    pub fn set_bool(&mut self, name: &str, val: bool) -> bool {
        if let Some(prop) = self.get_mut(name) {
            if !matches!(prop.value, PropValue::Bool(_)) { return false; }
            prop.value = PropValue::Bool(val);
            prop.touch();
            true
        } else {
            false
        }
    }
    pub fn set_link(&mut self, name: &str, target: Option<ObjectId>) -> bool {
        if let Some(prop) = self.get_mut(name) {
            if !matches!(prop.value, PropValue::Link(_)) { return false; }
            prop.value = PropValue::Link(target);
            prop.touch();
            true
        } else {
            false
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prop_value_variants() {
        let v = PropValue::Integer(42);
        assert!(matches!(v, PropValue::Integer(42)));
        let v = PropValue::Float(3.14);
        assert!(matches!(v, PropValue::Float(f) if (f - 3.14).abs() < 1e-10));
        let v = PropValue::String("hello".into());
        assert!(matches!(v, PropValue::String(ref s) if s == "hello"));
        let v = PropValue::Bool(true);
        assert!(matches!(v, PropValue::Bool(true)));
        let v = PropValue::Vector3([1.0, 2.0, 3.0]);
        assert!(matches!(v, PropValue::Vector3([1.0, 2.0, 3.0])));
        let v = PropValue::Color(Color::rgb(1.0, 0.0, 0.0));
        assert!(matches!(v, PropValue::Color(Color { r, .. }) if (r - 1.0).abs() < 1e-6));
    }

    #[test]
    fn prop_value_link_types() {
        let id = ObjectId(5);
        let v = PropValue::Link(Some(id));
        assert!(matches!(v, PropValue::Link(Some(ObjectId(5)))));

        let v = PropValue::LinkList(vec![ObjectId(1), ObjectId(2)]);
        if let PropValue::LinkList(ids) = v {
            assert_eq!(ids.len(), 2);
        }

        let v = PropValue::LinkSub(Some(ObjectId(3)), vec!["Face1".into()]);
        assert!(matches!(v, PropValue::LinkSub(Some(ObjectId(3)), _)));
    }

    #[test]
    fn prop_value_enumeration() {
        let v = PropValue::Enumeration {
            options: vec!["A".into(), "B".into(), "C".into()],
            selected: 1,
        };
        if let PropValue::Enumeration { options, selected } = v {
            assert_eq!(options[selected], "B");
        }
    }

    #[test]
    fn prop_value_quantity() {
        let v = PropValue::Quantity { value: 10.0, unit: Unit::Length };
        assert!(matches!(v, PropValue::Quantity { value, unit: Unit::Length } if (value - 10.0).abs() < 1e-10));
    }

    #[test]
    fn property_status_flags() {
        let mut s = PropertyStatus::new();
        assert!(!s.test(PropertyStatus::TOUCHED));
        s.set(PropertyStatus::TOUCHED, true);
        assert!(s.test(PropertyStatus::TOUCHED));
        s.set(PropertyStatus::READ_ONLY, true);
        assert!(s.test(PropertyStatus::TOUCHED));
        assert!(s.test(PropertyStatus::READ_ONLY));
        s.set(PropertyStatus::TOUCHED, false);
        assert!(!s.test(PropertyStatus::TOUCHED));
        assert!(s.test(PropertyStatus::READ_ONLY));
    }

    #[test]
    fn property_touch_purge() {
        let mut p = Property::new(PropValue::Integer(10));
        assert!(!p.is_touched());
        p.touch();
        assert!(p.is_touched());
        p.purge_touched();
        assert!(!p.is_touched());
    }

    #[test]
    fn property_snapshot_restore() {
        let mut p = Property::new(PropValue::Float(1.0));
        p.touch();
        let snap = p.snapshot();
        p.value = PropValue::Float(2.0);
        p.purge_touched();
        assert!(!p.is_touched());
        p.restore(&snap);
        assert!(matches!(p.value, PropValue::Float(v) if (v - 1.0).abs() < 1e-10));
        assert!(p.is_touched()); // restored with Touched bit from snap
    }

    #[test]
    fn property_bag_crud() {
        let mut bag = PropertyBag::new();
        assert!(bag.is_empty());

        bag.set("Width", PropValue::Float(100.0));
        bag.set("Height", PropValue::Float(200.0));
        bag.set("Name", PropValue::String("Box".into()));
        assert_eq!(bag.len(), 3);

        assert_eq!(bag.get_float("Width"), Some(100.0));
        assert_eq!(bag.get_string("Name"), Some("Box"));
        assert_eq!(bag.get_float("Missing"), None);

        // Replace
        let old = bag.set("Width", PropValue::Float(150.0));
        assert!(old.is_some());
        assert_eq!(bag.get_float("Width"), Some(150.0));
        assert_eq!(bag.len(), 3);

        // Remove
        let removed = bag.remove("Height");
        assert!(removed.is_some());
        assert_eq!(bag.len(), 2);
        assert!(bag.get("Height").is_none());
    }

    #[test]
    fn property_bag_typed_setters() {
        let mut bag = PropertyBag::new();
        bag.set("count", PropValue::Integer(0));
        bag.set("enabled", PropValue::Bool(false));
        bag.set("label", PropValue::String(String::new()));
        bag.set("ref", PropValue::Link(None));

        assert!(bag.set_integer("count", 42));
        assert!(bag.set_bool("enabled", true));
        assert!(bag.set_string("label", "hello"));
        assert!(bag.set_link("ref", Some(ObjectId(7))));

        assert_eq!(bag.get_integer("count"), Some(42));
        assert_eq!(bag.get_bool("enabled"), Some(true));
        assert_eq!(bag.get_string("label"), Some("hello"));
        assert_eq!(bag.get_link("ref"), Some(Some(ObjectId(7))));

        // Wrong type returns false
        assert!(!bag.set_integer("enabled", 5));
    }

    #[test]
    fn property_bag_touch_tracking() {
        let mut bag = PropertyBag::new();
        bag.set("x", PropValue::Float(1.0));
        bag.set("y", PropValue::Float(2.0));
        assert!(bag.any_touched()); // newly added props are touched

        bag.purge_all_touched();
        assert!(!bag.any_touched());

        bag.set_float("x", 5.0);
        assert!(bag.any_touched());
        assert!(bag.get("x").unwrap().is_touched());
        assert!(!bag.get("y").unwrap().is_touched());
    }

    #[test]
    fn property_bag_referenced_objects() {
        let mut bag = PropertyBag::new();
        bag.set("src", PropValue::Link(Some(ObjectId(1))));
        bag.set("deps", PropValue::LinkList(vec![ObjectId(2), ObjectId(3)]));
        bag.set("sub", PropValue::LinkSub(Some(ObjectId(4)), vec!["Face1".into()]));
        bag.set("multi", PropValue::LinkSubList(vec![
            (ObjectId(5), vec!["Edge1".into()]),
        ]));
        bag.set("nolink", PropValue::Float(0.0));
        bag.set("empty", PropValue::Link(None));

        let refs = bag.referenced_objects();
        assert_eq!(refs.len(), 5);
        assert!(refs.contains(&ObjectId(1)));
        assert!(refs.contains(&ObjectId(2)));
        assert!(refs.contains(&ObjectId(3)));
        assert!(refs.contains(&ObjectId(4)));
        assert!(refs.contains(&ObjectId(5)));
    }

    #[test]
    fn property_bag_names_and_iter() {
        let mut bag = PropertyBag::new();
        bag.set("a", PropValue::Integer(1));
        bag.set("b", PropValue::Integer(2));
        bag.set("c", PropValue::Integer(3));

        let names = bag.names();
        assert_eq!(names, vec!["a", "b", "c"]);

        let sum: i64 = bag.iter().filter_map(|(_, p)| {
            if let PropValue::Integer(v) = p.value { Some(v) } else { None }
        }).sum();
        assert_eq!(sum, 6);
    }

    #[test]
    fn unit_enum() {
        assert_ne!(Unit::Length, Unit::Angle);
        assert_eq!(Unit::Length, Unit::Length);
        let u = Unit::Pressure;
        assert!(matches!(u, Unit::Pressure));
    }

    #[test]
    fn color_constructors() {
        let c = Color::rgb(1.0, 0.5, 0.0);
        assert!((c.a - 1.0).abs() < 1e-6);
        let c2 = Color::new(1.0, 0.5, 0.0, 0.8);
        assert!((c2.a - 0.8).abs() < 1e-6);
        let c3 = Color::default();
        assert!((c3.r).abs() < 1e-6);
    }
}
