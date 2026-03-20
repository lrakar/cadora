//! Document — the top-level container that owns objects, manages
//! the dependency graph, and orchestrates recomputation.
//!
//! Mirrors FreeCAD's `App::Document`.

use std::collections::HashMap;
use petgraph::graph::{DiGraph, NodeIndex};
use petgraph::algo::toposort;

use crate::object::{DocumentObject, GenericObject, ObjectId, ObjectType};
use crate::property::PropValue;
use crate::transaction::Transaction;

// ── Document ───────────────────────────────────────────────────────

/// Document — owns objects, tracks dependencies, supports undo/redo.
pub struct Document {
    /// All objects in insertion order.
    objects: Vec<DocumentObject>,
    /// Name → index in `objects`.
    name_map: HashMap<String, usize>,
    /// ObjectId → index in `objects`.
    id_map: HashMap<ObjectId, usize>,
    /// Auto-increment counter for object IDs.
    next_id: u64,

    /// Document label (user-visible name).
    pub label: String,
    /// File path (if saved).
    pub file_name: String,

    // ── Undo/Redo ──────────────────────────────────────────────────

    /// Currently recording transaction (None if no transaction open).
    active_transaction: Option<Transaction>,
    undo_stack: Vec<Transaction>,
    redo_stack: Vec<Transaction>,
    max_undo: usize,
    next_transaction_id: u64,
}

impl Document {
    pub fn new(label: impl Into<String>) -> Self {
        Self {
            objects: Vec::new(),
            name_map: HashMap::new(),
            id_map: HashMap::new(),
            next_id: 1,
            label: label.into(),
            file_name: String::new(),
            active_transaction: None,
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            max_undo: 50,
            next_transaction_id: 1,
        }
    }

    // ── Object Management ──────────────────────────────────────────

    /// Generate a unique object name from a proposed base name.
    pub fn unique_name(&self, proposed: &str) -> String {
        if !self.name_map.contains_key(proposed) {
            return proposed.to_string();
        }
        for i in 1u64.. {
            let candidate = format!("{proposed}{i:03}");
            if !self.name_map.contains_key(&candidate) {
                return candidate;
            }
        }
        unreachable!()
    }

    /// Add a new object with the given type handler.
    /// Returns the assigned ObjectId. Name is auto-uniquified.
    pub fn add_object(
        &mut self,
        proposed_name: &str,
        handler: Box<dyn ObjectType>,
    ) -> ObjectId {
        let name = self.unique_name(proposed_name);
        let id = ObjectId(self.next_id);
        self.next_id += 1;

        let obj = DocumentObject::new(name.clone(), id, handler);
        let idx = self.objects.len();
        self.name_map.insert(name, idx);
        self.id_map.insert(id, idx);
        self.objects.push(obj);
        id
    }

    /// Add a GeoFeature (object with Placement property).
    pub fn add_geo_feature(
        &mut self,
        proposed_name: &str,
        handler: Box<dyn ObjectType>,
    ) -> ObjectId {
        let name = self.unique_name(proposed_name);
        let id = ObjectId(self.next_id);
        self.next_id += 1;

        let obj = DocumentObject::new_geo_feature(name.clone(), id, handler);
        let idx = self.objects.len();
        self.name_map.insert(name, idx);
        self.id_map.insert(id, idx);
        self.objects.push(obj);
        id
    }

    /// Add a generic object (no special behavior).
    pub fn add_generic_object(&mut self, proposed_name: &str, type_name: &str) -> ObjectId {
        self.add_object(proposed_name, Box::new(GenericObject::new(type_name)))
    }

    /// Remove an object by name. Returns true if removed.
    /// Also clears any Link properties in other objects that pointed to this one.
    pub fn remove_object(&mut self, name: &str) -> bool {
        let Some(&idx) = self.name_map.get(name) else { return false };
        let removed_id = self.objects[idx].id();

        // Remove the object
        self.objects.remove(idx);
        self.name_map.remove(name);
        self.id_map.remove(&removed_id);

        // Rebuild index maps (indices shifted)
        self.name_map.clear();
        self.id_map.clear();
        for (i, obj) in self.objects.iter().enumerate() {
            self.name_map.insert(obj.name().to_string(), i);
            self.id_map.insert(obj.id(), i);
        }

        // Clean up dangling links in other objects
        for obj in &mut self.objects {
            Self::clear_link_refs(&mut obj.properties, removed_id);
        }
        true
    }

    /// Clear any link references to `removed` in a PropertyBag.
    fn clear_link_refs(bag: &mut crate::property::PropertyBag, removed: ObjectId) {
        let names: Vec<String> = bag.names().iter().map(|s| s.to_string()).collect();
        for name in names {
            let needs_update = match &bag.get(&name).unwrap().value {
                PropValue::Link(Some(id)) if *id == removed => true,
                PropValue::LinkList(ids) if ids.contains(&removed) => true,
                PropValue::LinkSub(Some(id), _) if *id == removed => true,
                PropValue::LinkSubList(entries) if entries.iter().any(|(id, _)| *id == removed) => true,
                _ => false,
            };
            if needs_update {
                let new_val = match &bag.get(&name).unwrap().value {
                    PropValue::Link(_) => PropValue::Link(None),
                    PropValue::LinkList(ids) => PropValue::LinkList(
                        ids.iter().copied().filter(|id| *id != removed).collect()
                    ),
                    PropValue::LinkSub(_, _) => PropValue::LinkSub(None, vec![]),
                    PropValue::LinkSubList(entries) => PropValue::LinkSubList(
                        entries.iter().filter(|(id, _)| *id != removed).cloned().collect()
                    ),
                    other => other.clone(),
                };
                bag.set(name, new_val);
            }
        }
    }

    // ── Lookup ─────────────────────────────────────────────────────

    pub fn get_object(&self, name: &str) -> Option<&DocumentObject> {
        self.name_map.get(name).map(|&i| &self.objects[i])
    }
    pub fn get_object_mut(&mut self, name: &str) -> Option<&mut DocumentObject> {
        self.name_map.get(name).copied().map(|i| &mut self.objects[i])
    }
    pub fn get_object_by_id(&self, id: ObjectId) -> Option<&DocumentObject> {
        self.id_map.get(&id).map(|&i| &self.objects[i])
    }
    pub fn get_object_by_id_mut(&mut self, id: ObjectId) -> Option<&mut DocumentObject> {
        self.id_map.get(&id).copied().map(|i| &mut self.objects[i])
    }
    pub fn objects(&self) -> &[DocumentObject] { &self.objects }
    pub fn count(&self) -> usize { self.objects.len() }

    /// Get objects of a specific type.
    pub fn objects_of_type(&self, type_name: &str) -> Vec<&DocumentObject> {
        self.objects.iter().filter(|o| o.type_name() == type_name).collect()
    }

    // ── Dependency Graph ───────────────────────────────────────────

    /// Build the dependency DAG from all objects' link properties.
    /// Returns (graph, id-to-node map).
    fn build_dag(&self) -> (DiGraph<ObjectId, ()>, HashMap<ObjectId, NodeIndex>) {
        let mut graph = DiGraph::new();
        let mut node_map = HashMap::new();

        // Add a node for every object
        for obj in &self.objects {
            let node = graph.add_node(obj.id());
            node_map.insert(obj.id(), node);
        }

        // Add edges: obj → dependency  (obj depends on dep)
        for obj in &self.objects {
            let obj_node = node_map[&obj.id()];
            for dep_id in obj.out_list() {
                if let Some(&dep_node) = node_map.get(&dep_id) {
                    // Edge from dependency to dependent (dep must recompute first)
                    graph.add_edge(dep_node, obj_node, ());
                }
            }
        }

        (graph, node_map)
    }

    /// Get topologically sorted object IDs.
    /// Dependencies come before dependents.  Returns Err if cycle detected.
    pub fn topological_sort(&self) -> Result<Vec<ObjectId>, String> {
        let (graph, _) = self.build_dag();
        match toposort(&graph, None) {
            Ok(nodes) => Ok(nodes.into_iter().map(|n| graph[n]).collect()),
            Err(_) => Err("Dependency cycle detected".into()),
        }
    }

    /// Return root objects (no incoming edges — not depended upon by others).
    pub fn root_objects(&self) -> Vec<ObjectId> {
        // Objects that are NOT in anyone's out_list
        let mut depended_on = std::collections::HashSet::new();
        for obj in &self.objects {
            for dep in obj.out_list() {
                depended_on.insert(dep);
            }
        }
        self.objects.iter()
            .filter(|o| !depended_on.contains(&o.id()))
            .map(|o| o.id())
            .collect()
    }

    /// Check if adding a link from `from` to `to` would create a cycle.
    /// `from` wants to depend on `to`. This creates a cycle if `from` can
    /// already be reached from `to` (i.e., `to` already depends on `from`
    /// transitively).
    pub fn would_create_cycle(&self, from: ObjectId, to: ObjectId) -> bool {
        let (graph, node_map) = self.build_dag();
        let Some(&from_node) = node_map.get(&from) else { return false };
        let Some(&to_node) = node_map.get(&to) else { return false };

        // BFS forward from `from` in the DAG: if we can reach `to`,
        // then adding to→from edge would close a cycle.
        use petgraph::visit::Bfs;
        let mut bfs = Bfs::new(&graph, from_node);
        while let Some(node) = bfs.next(&graph) {
            if node == to_node {
                return true;
            }
        }
        false
    }

    /// Get objects that `obj_id` depends on (outgoing links).
    pub fn get_out_list(&self, obj_id: ObjectId) -> Vec<ObjectId> {
        self.get_object_by_id(obj_id)
            .map(|o| o.out_list())
            .unwrap_or_default()
    }

    /// Get objects that depend on `obj_id` (incoming links - who references us).
    pub fn get_in_list(&self, obj_id: ObjectId) -> Vec<ObjectId> {
        self.objects.iter()
            .filter(|o| o.out_list().contains(&obj_id))
            .map(|o| o.id())
            .collect()
    }

    // ── Recompute ──────────────────────────────────────────────────

    /// Recompute all touched objects in dependency order.
    /// Returns the number of objects recomputed and whether any errors occurred.
    pub fn recompute(&mut self) -> Result<usize, String> {
        let order = self.topological_sort()?;

        // Collect IDs that need recompute
        let to_recompute: Vec<ObjectId> = order.iter().copied()
            .filter(|id| {
                self.get_object_by_id(*id)
                    .is_some_and(|o| o.is_touched() && !o.is_frozen())
            })
            .collect();

        let mut count = 0;
        let mut first_error = None;

        for id in &to_recompute {
            // We need index-based access to get &mut
            let idx = self.id_map[id];
            if let Err(e) = self.objects[idx].execute() {
                if first_error.is_none() {
                    first_error = Some(format!("{}: {e}", self.objects[idx].name()));
                }
            }
            count += 1;

            // Touch dependents if this object was recomputed
            let dependents = self.get_in_list(*id);
            for dep_id in dependents {
                if let Some(dep_idx) = self.id_map.get(&dep_id) {
                    self.objects[*dep_idx].touch();
                }
            }
        }

        if let Some(err) = first_error {
            Err(err)
        } else {
            Ok(count)
        }
    }

    /// Recompute a single object (and optionally its dependents recursively).
    pub fn recompute_object(&mut self, id: ObjectId, recursive: bool) -> Result<(), String> {
        let idx = *self.id_map.get(&id).ok_or("Object not found")?;
        self.objects[idx].execute()?;

        if recursive {
            let deps = self.get_in_list(id);
            for dep_id in deps {
                self.recompute_object(dep_id, true)?;
            }
        }
        Ok(())
    }

    // ── Transaction / Undo / Redo ──────────────────────────────────

    /// Open a new transaction for recording changes.
    pub fn open_transaction(&mut self, name: impl Into<String>) -> u64 {
        let tid = self.next_transaction_id;
        self.next_transaction_id += 1;

        // If there's already an open transaction, commit it first
        if self.active_transaction.is_some() {
            self.commit_transaction();
        }

        self.active_transaction = Some(Transaction::new(name.into(), tid));
        tid
    }

    /// Commit the current transaction to the undo stack.
    pub fn commit_transaction(&mut self) {
        if let Some(txn) = self.active_transaction.take() {
            if !txn.is_empty() {
                self.undo_stack.push(txn);
                self.redo_stack.clear(); // new changes invalidate redo
                // Enforce max undo
                while self.undo_stack.len() > self.max_undo {
                    self.undo_stack.remove(0);
                }
            }
        }
    }

    /// Abort the current transaction (discard recorded changes).
    pub fn abort_transaction(&mut self) {
        if let Some(txn) = self.active_transaction.take() {
            // Restore all snapshots in reverse
            for snap in txn.snapshots().iter().rev() {
                if let Some(obj) = self.get_object_by_id_mut(snap.object_id) {
                    if let Some(prop) = obj.properties.get_mut(&snap.prop_name) {
                        prop.restore(&snap.old_value);
                    }
                }
            }
        }
    }

    /// Record a property change in the active transaction.
    pub fn record_change(&mut self, object_id: ObjectId, prop_name: &str) {
        if let Some(txn) = &mut self.active_transaction {
            if let Some(obj) = self.id_map.get(&object_id).and_then(|&i| self.objects.get(i)) {
                if let Some(prop) = obj.properties.get(prop_name) {
                    txn.record(object_id, prop_name.to_string(), prop.snapshot());
                }
            }
        }
    }

    /// Undo the last transaction.
    pub fn undo(&mut self) -> bool {
        let Some(txn) = self.undo_stack.pop() else { return false };

        // Build the redo transaction (current state before reverting)
        let mut redo = Transaction::new(txn.name().to_string(), txn.id());
        for snap in txn.snapshots() {
            if let Some(obj) = self.get_object_by_id(snap.object_id) {
                if let Some(prop) = obj.properties.get(&snap.prop_name) {
                    redo.record(snap.object_id, snap.prop_name.clone(), prop.snapshot());
                }
            }
        }

        // Apply old values
        for snap in txn.snapshots().iter().rev() {
            if let Some(obj) = self.get_object_by_id_mut(snap.object_id) {
                if let Some(prop) = obj.properties.get_mut(&snap.prop_name) {
                    prop.restore(&snap.old_value);
                }
                obj.touch();
            }
        }

        self.redo_stack.push(redo);
        true
    }

    /// Redo the last undone transaction.
    pub fn redo(&mut self) -> bool {
        let Some(txn) = self.redo_stack.pop() else { return false };

        // Build undo from current state
        let mut undo = Transaction::new(txn.name().to_string(), txn.id());
        for snap in txn.snapshots() {
            if let Some(obj) = self.get_object_by_id(snap.object_id) {
                if let Some(prop) = obj.properties.get(&snap.prop_name) {
                    undo.record(snap.object_id, snap.prop_name.clone(), prop.snapshot());
                }
            }
        }

        // Apply redo values
        for snap in txn.snapshots().iter().rev() {
            if let Some(obj) = self.get_object_by_id_mut(snap.object_id) {
                if let Some(prop) = obj.properties.get_mut(&snap.prop_name) {
                    prop.restore(&snap.old_value);
                }
                obj.touch();
            }
        }

        self.undo_stack.push(undo);
        true
    }

    pub fn can_undo(&self) -> bool { !self.undo_stack.is_empty() }
    pub fn can_redo(&self) -> bool { !self.redo_stack.is_empty() }
    pub fn available_undos(&self) -> usize { self.undo_stack.len() }
    pub fn available_redos(&self) -> usize { self.redo_stack.len() }
    pub fn set_max_undo(&mut self, n: usize) { self.max_undo = n; }
}

// ── Tests ──────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    fn new_doc() -> Document {
        Document::new("TestDoc")
    }

    #[test]
    fn add_objects() {
        let mut doc = new_doc();
        let id1 = doc.add_generic_object("Box", "Part::Feature");
        let id2 = doc.add_generic_object("Cylinder", "Part::Feature");
        assert_eq!(doc.count(), 2);
        assert_eq!(doc.get_object("Box").unwrap().id(), id1);
        assert_eq!(doc.get_object("Cylinder").unwrap().id(), id2);
    }

    #[test]
    fn unique_naming() {
        let mut doc = new_doc();
        doc.add_generic_object("Box", "Part::Feature");
        let id2 = doc.add_generic_object("Box", "Part::Feature");
        // Second should be Box001
        let obj2 = doc.get_object_by_id(id2).unwrap();
        assert_eq!(obj2.name(), "Box001");
    }

    #[test]
    fn remove_object() {
        let mut doc = new_doc();
        doc.add_generic_object("A", "T");
        doc.add_generic_object("B", "T");
        doc.add_generic_object("C", "T");
        assert_eq!(doc.count(), 3);

        assert!(doc.remove_object("B"));
        assert_eq!(doc.count(), 2);
        assert!(doc.get_object("B").is_none());
        assert!(doc.get_object("A").is_some());
        assert!(doc.get_object("C").is_some());
    }

    #[test]
    fn remove_cascades_links() {
        let mut doc = new_doc();
        let id1 = doc.add_generic_object("Source", "T");
        let _id2 = doc.add_generic_object("Dep", "T");
        doc.get_object_mut("Dep").unwrap()
            .properties.set("ref", PropValue::Link(Some(id1)));

        assert!(doc.remove_object("Source"));
        // Dep's link should be cleared
        let dep = doc.get_object("Dep").unwrap();
        assert_eq!(dep.properties.get_link("ref"), Some(None));
    }

    #[test]
    fn object_lookup_by_id() {
        let mut doc = new_doc();
        let id = doc.add_generic_object("Sketch", "Sketcher::SketchObject");
        assert_eq!(doc.get_object_by_id(id).unwrap().name(), "Sketch");
    }

    #[test]
    fn objects_of_type() {
        let mut doc = new_doc();
        doc.add_generic_object("Box", "Part::Feature");
        doc.add_generic_object("Cyl", "Part::Feature");
        doc.add_generic_object("Sketch", "Sketcher::SketchObject");

        let parts = doc.objects_of_type("Part::Feature");
        assert_eq!(parts.len(), 2);
        let sketches = doc.objects_of_type("Sketcher::SketchObject");
        assert_eq!(sketches.len(), 1);
    }

    #[test]
    fn dependency_dag_topo_sort() {
        let mut doc = new_doc();
        let id_sketch = doc.add_generic_object("Sketch", "S");
        let id_pad = doc.add_generic_object("Pad", "P");
        let id_fillet = doc.add_generic_object("Fillet", "F");

        // Pad depends on Sketch
        doc.get_object_mut("Pad").unwrap()
            .properties.set("Profile", PropValue::Link(Some(id_sketch)));
        // Fillet depends on Pad
        doc.get_object_mut("Fillet").unwrap()
            .properties.set("Base", PropValue::Link(Some(id_pad)));

        let order = doc.topological_sort().unwrap();
        let pos_sketch = order.iter().position(|id| *id == id_sketch).unwrap();
        let pos_pad = order.iter().position(|id| *id == id_pad).unwrap();
        let pos_fillet = order.iter().position(|id| *id == id_fillet).unwrap();
        assert!(pos_sketch < pos_pad);
        assert!(pos_pad < pos_fillet);
    }

    #[test]
    fn cycle_detection() {
        let mut doc = new_doc();
        let id_a = doc.add_generic_object("A", "T");
        let id_b = doc.add_generic_object("B", "T");

        // A → B
        doc.get_object_mut("A").unwrap()
            .properties.set("dep", PropValue::Link(Some(id_b)));
        // B → A would create cycle
        assert!(doc.would_create_cycle(id_b, id_a));
        // A → B already exists, no new cycle
        assert!(!doc.would_create_cycle(id_a, id_b));
    }

    #[test]
    fn in_list_out_list() {
        let mut doc = new_doc();
        let id_a = doc.add_generic_object("A", "T");
        let id_b = doc.add_generic_object("B", "T");
        let id_c = doc.add_generic_object("C", "T");

        // B depends on A; C depends on A
        doc.get_object_mut("B").unwrap()
            .properties.set("dep", PropValue::Link(Some(id_a)));
        doc.get_object_mut("C").unwrap()
            .properties.set("dep", PropValue::Link(Some(id_a)));

        assert_eq!(doc.get_out_list(id_b), vec![id_a]);
        let in_list = doc.get_in_list(id_a);
        assert!(in_list.contains(&id_b));
        assert!(in_list.contains(&id_c));
    }

    #[test]
    fn recompute_order() {
        // Use atomics to track execution order
        static COUNTER: AtomicU32 = AtomicU32::new(0);

        struct TrackingHandler {
            name: String,
        }
        impl ObjectType for TrackingHandler {
            fn type_name(&self) -> &str { &self.name }
            fn execute(&self, props: &mut crate::property::PropertyBag) -> Result<(), String> {
                let order = COUNTER.fetch_add(1, Ordering::SeqCst);
                props.set("exec_order", PropValue::Integer(order as i64));
                Ok(())
            }
        }

        COUNTER.store(0, Ordering::SeqCst);

        let mut doc = new_doc();
        let id_a = doc.add_object("A", Box::new(TrackingHandler { name: "A".into() }));
        let id_b = doc.add_object("B", Box::new(TrackingHandler { name: "B".into() }));
        let _id_c = doc.add_object("C", Box::new(TrackingHandler { name: "C".into() }));

        // C depends on B, B depends on A
        doc.get_object_mut("C").unwrap()
            .properties.set("dep", PropValue::Link(Some(id_b)));
        doc.get_object_mut("B").unwrap()
            .properties.set("dep", PropValue::Link(Some(id_a)));

        let result = doc.recompute();
        assert!(result.is_ok());

        // Verify execution order: A before B before C
        let order_a = doc.get_object("A").unwrap().properties.get_integer("exec_order").unwrap();
        let order_b = doc.get_object("B").unwrap().properties.get_integer("exec_order").unwrap();
        let order_c = doc.get_object("C").unwrap().properties.get_integer("exec_order").unwrap();
        assert!(order_a < order_b, "A ({order_a}) should execute before B ({order_b})");
        assert!(order_b < order_c, "B ({order_b}) should execute before C ({order_c})");
    }

    #[test]
    fn root_objects() {
        let mut doc = new_doc();
        let id_a = doc.add_generic_object("Root1", "T");
        let id_b = doc.add_generic_object("Root2", "T");
        let _id_c = doc.add_generic_object("Child", "T");

        doc.get_object_mut("Child").unwrap()
            .properties.set("dep", PropValue::Link(Some(id_a)));

        let roots = doc.root_objects();
        // Root2 is not depended on by anyone
        assert!(roots.contains(&id_b));
        // Child is not depended on by anyone either (it's a leaf)
        // Root1 IS depended on, but root_objects means "not in anyone's out_list"
        // Actually "root" here means "no one depends on them" - leaf nodes
        // Let me reconsider: in FreeCAD, "root objects" are those with no parents
        // i.e., they don't appear in anyone else's InList.
        // An object with no InList (no one references it) is a "root".
        // Wait, root means top of tree = things that have no inputs.
        // Actually FreeCAD's getRootObjects returns objects that don't depend on anything else.
        // Let me check: roots = objects with empty out_list (no dependencies).
        // But our implementation returns objects not in anyone's out_list (no dependents on them).
        // Both Root1 and Child are "leaves" (Root2 has no dependents).
        // Let me just verify the implementation is consistent.
        assert!(!roots.contains(&id_a)); // A is referenced by Child
    }

    #[test]
    fn geo_feature_in_document() {
        let mut doc = new_doc();
        let id = doc.add_geo_feature("Body", Box::new(GenericObject::new("PartDesign::Body")));
        let obj = doc.get_object_by_id(id).unwrap();
        assert!(obj.placement().is_some());
    }

    // ── Transaction tests ──────────────────────────────────────────

    #[test]
    fn undo_redo_basic() {
        let mut doc = new_doc();
        let id = doc.add_generic_object("Box", "Part::Feature");
        doc.get_object_mut("Box").unwrap()
            .properties.set("Width", PropValue::Float(100.0));

        // Record initial state
        doc.recompute().ok();

        // Transaction: change width
        doc.open_transaction("Change Width");
        doc.record_change(id, "Width");
        doc.get_object_mut("Box").unwrap()
            .properties.set("Width", PropValue::Float(200.0));
        doc.commit_transaction();

        assert_eq!(doc.get_object("Box").unwrap().properties.get_float("Width"), Some(200.0));
        assert!(doc.can_undo());

        // Undo
        assert!(doc.undo());
        assert_eq!(doc.get_object("Box").unwrap().properties.get_float("Width"), Some(100.0));
        assert!(doc.can_redo());

        // Redo
        assert!(doc.redo());
        assert_eq!(doc.get_object("Box").unwrap().properties.get_float("Width"), Some(200.0));
    }

    #[test]
    fn undo_stack_limit() {
        let mut doc = new_doc();
        let id = doc.add_generic_object("X", "T");
        doc.get_object_mut("X").unwrap()
            .properties.set("v", PropValue::Integer(0));
        doc.set_max_undo(3);

        for i in 1..=5 {
            doc.open_transaction(&format!("T{i}"));
            doc.record_change(id, "v");
            doc.get_object_mut("X").unwrap()
                .properties.set_integer("v", i);
            doc.commit_transaction();
        }

        assert_eq!(doc.available_undos(), 3); // capped at 3
    }

    #[test]
    fn abort_transaction() {
        let mut doc = new_doc();
        let id = doc.add_generic_object("Box", "T");
        doc.get_object_mut("Box").unwrap()
            .properties.set("Width", PropValue::Float(50.0));

        doc.open_transaction("Change");
        doc.record_change(id, "Width");
        doc.get_object_mut("Box").unwrap()
            .properties.set("Width", PropValue::Float(999.0));
        doc.abort_transaction();

        // Should be back to 50
        assert_eq!(doc.get_object("Box").unwrap().properties.get_float("Width"), Some(50.0));
        assert!(!doc.can_undo()); // aborted transaction not on stack
    }

    #[test]
    fn new_changes_clear_redo() {
        let mut doc = new_doc();
        let id = doc.add_generic_object("X", "T");
        doc.get_object_mut("X").unwrap()
            .properties.set("v", PropValue::Integer(1));

        doc.open_transaction("T1");
        doc.record_change(id, "v");
        doc.get_object_mut("X").unwrap().properties.set_integer("v", 2);
        doc.commit_transaction();

        doc.undo();
        assert!(doc.can_redo());

        // New change clears redo
        doc.open_transaction("T2");
        doc.record_change(id, "v");
        doc.get_object_mut("X").unwrap().properties.set_integer("v", 3);
        doc.commit_transaction();

        assert!(!doc.can_redo());
    }
}
