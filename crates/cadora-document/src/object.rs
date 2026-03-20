//! DocumentObject — the core data-bearing entity in a Document.
//!
//! Mirrors FreeCAD's `App::DocumentObject` (name/label/id triple,
//! properties, status flags, touch tracking).

use crate::placement::Placement;
use crate::property::{PropValue, PropertyBag};

// ── ObjectId ───────────────────────────────────────────────────────

/// Unique numeric identifier for a DocumentObject within a Document.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ObjectId(pub u64);

// ── ObjectStatus ───────────────────────────────────────────────────

/// Status bit-flags for a DocumentObject (mirrors FreeCAD ObjectStatus).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ObjectStatus(u32);

impl ObjectStatus {
    pub const TOUCH: u32          = 1 << 0;
    pub const ERROR: u32          = 1 << 1;
    pub const NEW: u32            = 1 << 2;
    pub const RECOMPUTE: u32      = 1 << 3;
    pub const RESTORE: u32        = 1 << 4;
    pub const REMOVE: u32         = 1 << 5;
    pub const DESTROY: u32        = 1 << 7;
    pub const ENFORCE: u32        = 1 << 8;
    pub const PENDING: u32        = 1 << 11;
    pub const NO_TOUCH: u32       = 1 << 14;
    pub const FREEZE: u32         = 1 << 21;

    pub fn new() -> Self { Self(0) }

    pub fn set(&mut self, flag: u32, on: bool) {
        if on { self.0 |= flag; } else { self.0 &= !flag; }
    }
    pub fn test(&self, flag: u32) -> bool {
        self.0 & flag != 0
    }
    pub fn bits(&self) -> u32 { self.0 }
}

// ── ObjectType trait ───────────────────────────────────────────────

/// Trait that concrete feature types implement to provide custom
/// execution behaviour.  Analogous to FreeCAD's virtual methods on
/// DocumentObject (execute, mustExecute, viewProviderName, etc.).
pub trait ObjectType: Send + Sync {
    /// Type identifier string (e.g. "Sketcher::SketchObject", "PartDesign::Pad").
    fn type_name(&self) -> &str;

    /// Recompute the object.  Receives mutable access to the object's
    /// properties so it can read inputs and write outputs.
    fn execute(&self, props: &mut PropertyBag) -> Result<(), String>;

    /// Returns true if the object should be recomputed.  Default: check
    /// if any property is touched.
    fn must_execute(&self, props: &PropertyBag) -> bool {
        props.any_touched()
    }

    /// Called when a property has changed (for side-effects like updating
    /// internal caches).  Default: no-op.
    fn on_changed(&self, _prop_name: &str, _props: &PropertyBag) {}
}

/// A default/generic object type used when no specialized behavior is needed.
#[derive(Debug)]
pub struct GenericObject {
    type_name: String,
}

impl GenericObject {
    pub fn new(type_name: impl Into<String>) -> Self {
        Self { type_name: type_name.into() }
    }
}

impl ObjectType for GenericObject {
    fn type_name(&self) -> &str { &self.type_name }
    fn execute(&self, _props: &mut PropertyBag) -> Result<(), String> { Ok(()) }
}

// ── DocumentObject ─────────────────────────────────────────────────

/// A document object with properties, identity, and type-specific behavior.
pub struct DocumentObject {
    /// Immutable ASCII name, unique within document.
    name: String,
    /// Mutable user-visible label (UTF-8).
    label: String,
    /// Numeric ID, unique within document.
    id: ObjectId,
    /// Status flags.
    status: ObjectStatus,
    /// The object's properties.
    pub properties: PropertyBag,
    /// Type-specific behavior handler.
    handler: Box<dyn ObjectType>,
    /// Error message from last failed execute(), if any.
    error_msg: Option<String>,
}

impl DocumentObject {
    pub fn new(name: String, id: ObjectId, handler: Box<dyn ObjectType>) -> Self {
        let mut obj = Self {
            label: name.clone(),
            name,
            id,
            status: ObjectStatus::new(),
            properties: PropertyBag::new(),
            handler,
            error_msg: None,
        };
        obj.status.set(ObjectStatus::NEW, true);
        obj.status.set(ObjectStatus::TOUCH, true);
        obj
    }

    /// Create a GeoFeature-style object that automatically includes a Placement property.
    pub fn new_geo_feature(name: String, id: ObjectId, handler: Box<dyn ObjectType>) -> Self {
        let mut obj = Self::new(name, id, handler);
        obj.properties.set("Placement", PropValue::Placement(Placement::identity()));
        obj.properties.set("Label2", PropValue::String(String::new()));
        obj
    }

    // ── Identity ───────────────────────────────────────────────────

    pub fn name(&self) -> &str { &self.name }
    pub fn label(&self) -> &str { &self.label }
    pub fn set_label(&mut self, label: impl Into<String>) { self.label = label.into(); }
    pub fn id(&self) -> ObjectId { self.id }
    pub fn type_name(&self) -> &str { self.handler.type_name() }

    // ── Status ─────────────────────────────────────────────────────

    pub fn status(&self) -> &ObjectStatus { &self.status }
    pub fn set_status(&mut self, flag: u32, on: bool) { self.status.set(flag, on); }

    pub fn touch(&mut self) {
        if !self.status.test(ObjectStatus::NO_TOUCH) {
            self.status.set(ObjectStatus::TOUCH, true);
        }
    }

    pub fn is_touched(&self) -> bool {
        self.status.test(ObjectStatus::TOUCH) || self.status.test(ObjectStatus::ENFORCE)
    }

    pub fn purge_touched(&mut self) {
        self.status.set(ObjectStatus::TOUCH, false);
        self.status.set(ObjectStatus::ENFORCE, false);
        self.properties.purge_all_touched();
    }

    pub fn enforce_recompute(&mut self) {
        self.status.set(ObjectStatus::ENFORCE, true);
    }

    pub fn is_frozen(&self) -> bool {
        self.status.test(ObjectStatus::FREEZE)
    }

    pub fn error(&self) -> Option<&str> {
        self.error_msg.as_deref()
    }

    // ── Execution ──────────────────────────────────────────────────

    pub fn must_execute(&self) -> bool {
        self.handler.must_execute(&self.properties)
    }

    /// Execute the object's handler, storing any error message.
    pub fn execute(&mut self) -> Result<(), String> {
        self.status.set(ObjectStatus::RECOMPUTE, true);
        self.status.set(ObjectStatus::ERROR, false);
        self.error_msg = None;

        let result = self.handler.execute(&mut self.properties);

        self.status.set(ObjectStatus::RECOMPUTE, false);
        match &result {
            Ok(()) => {
                self.purge_touched();
            }
            Err(msg) => {
                self.status.set(ObjectStatus::ERROR, true);
                self.error_msg = Some(msg.clone());
            }
        }
        result
    }

    // ── Dependencies ───────────────────────────────────────────────

    /// Objects this object depends on (from Link properties).
    pub fn out_list(&self) -> Vec<ObjectId> {
        self.properties.referenced_objects()
    }

    // ── Placement convenience ──────────────────────────────────────

    /// Get placement (if this is a GeoFeature with a Placement property).
    pub fn placement(&self) -> Option<&Placement> {
        self.properties.get_placement("Placement")
    }

    /// Set placement.
    pub fn set_placement(&mut self, p: Placement) {
        self.properties.set("Placement", PropValue::Placement(p));
    }
}

impl std::fmt::Debug for DocumentObject {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DocumentObject")
            .field("name", &self.name)
            .field("label", &self.label)
            .field("id", &self.id)
            .field("type", &self.type_name())
            .field("status", &self.status)
            .field("properties", &self.properties.names())
            .finish()
    }
}

// ── Tests ──────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_obj(name: &str, id: u64) -> DocumentObject {
        DocumentObject::new(
            name.into(),
            ObjectId(id),
            Box::new(GenericObject::new("App::FeatureTest")),
        )
    }

    #[test]
    fn object_identity() {
        let obj = make_obj("Box", 1);
        assert_eq!(obj.name(), "Box");
        assert_eq!(obj.label(), "Box"); // label defaults to name
        assert_eq!(obj.id(), ObjectId(1));
        assert_eq!(obj.type_name(), "App::FeatureTest");
    }

    #[test]
    fn object_label_change() {
        let mut obj = make_obj("Box", 1);
        obj.set_label("My Box 🎁");
        assert_eq!(obj.label(), "My Box 🎁");
        assert_eq!(obj.name(), "Box"); // name unchanged
    }

    #[test]
    fn object_status_flags() {
        let mut obj = make_obj("S", 1);
        assert!(obj.status().test(ObjectStatus::NEW));
        assert!(obj.is_touched());

        obj.purge_touched();
        assert!(!obj.is_touched());

        obj.touch();
        assert!(obj.is_touched());

        obj.set_status(ObjectStatus::FREEZE, true);
        assert!(obj.is_frozen());
    }

    #[test]
    fn object_no_touch_flag() {
        let mut obj = make_obj("S", 1);
        obj.purge_touched();
        obj.set_status(ObjectStatus::NO_TOUCH, true);
        obj.touch(); // should be suppressed
        assert!(!obj.is_touched());
    }

    #[test]
    fn object_enforce_recompute() {
        let mut obj = make_obj("S", 1);
        obj.purge_touched();
        assert!(!obj.is_touched());
        obj.enforce_recompute();
        assert!(obj.is_touched()); // ENFORCE makes it touched
    }

    #[test]
    fn object_execute_success() {
        let mut obj = make_obj("S", 1);
        assert!(obj.is_touched());
        let result = obj.execute();
        assert!(result.is_ok());
        assert!(!obj.is_touched()); // purged after success
        assert!(!obj.status().test(ObjectStatus::ERROR));
    }

    #[test]
    fn object_execute_failure() {
        struct FailHandler;
        impl ObjectType for FailHandler {
            fn type_name(&self) -> &str { "App::FailTest" }
            fn execute(&self, _: &mut PropertyBag) -> Result<(), String> {
                Err("something went wrong".into())
            }
        }

        let mut obj = DocumentObject::new(
            "Bad".into(), ObjectId(1), Box::new(FailHandler),
        );
        let result = obj.execute();
        assert!(result.is_err());
        assert!(obj.status().test(ObjectStatus::ERROR));
        assert_eq!(obj.error(), Some("something went wrong"));
    }

    #[test]
    fn object_properties() {
        let mut obj = make_obj("Box", 1);
        obj.properties.set("Width", PropValue::Float(100.0));
        obj.properties.set("Height", PropValue::Float(200.0));
        obj.properties.set("Label", PropValue::String("mybox".into()));

        assert_eq!(obj.properties.get_float("Width"), Some(100.0));
        assert_eq!(obj.properties.get_string("Label"), Some("mybox"));
        assert_eq!(obj.properties.len(), 3);
    }

    #[test]
    fn object_out_list() {
        let mut obj = make_obj("Pad", 1);
        obj.properties.set("Sketch", PropValue::Link(Some(ObjectId(5))));
        obj.properties.set("Deps", PropValue::LinkList(vec![ObjectId(2), ObjectId(3)]));

        let deps = obj.out_list();
        assert_eq!(deps.len(), 3);
        assert!(deps.contains(&ObjectId(5)));
        assert!(deps.contains(&ObjectId(2)));
        assert!(deps.contains(&ObjectId(3)));
    }

    #[test]
    fn geo_feature_has_placement() {
        let obj = DocumentObject::new_geo_feature(
            "Body".into(),
            ObjectId(1),
            Box::new(GenericObject::new("PartDesign::Body")),
        );
        assert!(obj.placement().is_some());
        assert!(obj.placement().unwrap().is_identity(1e-12));
    }

    #[test]
    fn geo_feature_set_placement() {
        let mut obj = DocumentObject::new_geo_feature(
            "Body".into(),
            ObjectId(1),
            Box::new(GenericObject::new("PartDesign::Body")),
        );
        obj.set_placement(Placement::from_translation(1.0, 2.0, 3.0));
        let p = obj.placement().unwrap();
        assert!((p.position[0] - 1.0).abs() < 1e-12);
    }
}
