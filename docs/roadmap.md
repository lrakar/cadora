# CADORA Full Roadmap — From Sketch to Parametric 3D CAD

## Current State

| Phase | Module | Tasks | Tests | Status |
|-------|--------|-------|-------|--------|
| A | GCS Constraint Solver | 79 | 178 | ✅ Complete |
| B | Sketch Manager | 54 | 282 | ✅ Complete |
| **Total completed** | | **133** | **460** | |

---

## Remaining Subsystems

Research based on comprehensive analysis of the FreeCAD source code at `src/App/`, `src/Mod/Part/`, `src/Mod/PartDesign/`, `src/Mod/Assembly/`, and `src/Gui/`.

---

## Phase C: Document Model & Property System

**FreeCAD sources**: `src/App/Document.h/.cpp`, `src/App/DocumentObject.h/.cpp`, `src/App/Property*.h/.cpp`, `src/App/Transactions.h/.cpp`, `src/App/PropertyExpressionEngine.h/.cpp`

### What it does
The parametric dependency graph — the core data model of the entire application. Every object (sketch, pad, fillet, body) is a `DocumentObject` living in a `Document`. Objects have typed `Property` values. When a property changes, dependent objects are topologically sorted and re-executed.

### FreeCAD core classes
- **`App::Document`** — Container for all objects. Manages save/restore (.FCStd), undo/redo transactions, topological recompute, signals.
- **`App::DocumentObject`** — Base class for every modeled entity. Has properties, status flags (Touch/Error/Freeze), an `execute()` method, and dependency links.
- **`App::Property`** (and ~30 subtypes) — `PropertyFloat`, `PropertyInteger`, `PropertyString`, `PropertyBool`, `PropertyLength`, `PropertyAngle`, `PropertyEnumeration`, `PropertyLink`, `PropertyLinkSub`, `PropertyLinkList`, `PropertyLinkSubList`, `PropertyPlacement`, `PropertyColor`, `PropertyPartShape`, `PropertyGeometryList`, `PropertyConstraintList`, etc.
- **`App::PropertyContainer`** — Base for anything that holds properties. Provides serialization, iteration, dynamic property support.
- **`App::Transaction`** — Undo/redo system. `openTransaction()` → mutations → `commitTransaction()` or `abortTransaction()`.
- **`App::PropertyExpressionEngine`** — Parametric formula engine. Allows `Pad.Length = Sketch.Constraints.width * 2`.
- **`App::GeoFeature`** — `DocumentObject` with a `Placement` property (position/orientation in 3D).
- **`App::Origin`** — XY/XZ/YZ planes + X/Y/Z axes. Every Body has one.

### CADORA design (Rust)
```
cadora-document
├── Document        — object registry, dependency graph, recompute
├── DocumentObject  — trait with execute(), properties, status
├── Property<T>     — generic typed property with change notification
├── PropertyLink    — reference to another DocumentObject
├── Transaction     — undo/redo snapshots
├── ExpressionEngine — formula parser & evaluator
├── Placement       — position + rotation in 3D
└── Origin          — standard reference planes/axes
```

### Task estimate: **~45 tasks**

| Sub-area | Tasks | Notes |
|----------|-------|-------|
| Property system (typed properties, serialization) | 12 | ~15 property types needed initially |
| DocumentObject trait + registry | 6 | ID management, naming, type system |
| Document (object storage, add/remove/find) | 5 | |
| Dependency graph + topological recompute | 8 | DAG, cycle detection, partial recompute |
| Transaction/Undo/Redo | 6 | Snapshot + rollback |
| Expression engine (parser, evaluator, deps) | 8 | Formula language, cell references |

### Dependencies
- None (foundational layer)

### Priority: **1 (First)**

---

## Phase D: Part/Shape Kernel (BRep Geometry)

**FreeCAD sources**: `src/Mod/Part/App/TopoShape.h/.cpp`, `src/Mod/Part/App/TopoShapeExpansion.cpp`, `src/Mod/Part/App/Geometry.h/.cpp`, `src/Mod/Part/App/PartFeature.h/.cpp`, `src/Mod/Part/App/BodyBase.h/.cpp`

### What it does
Wraps OpenCASCADE (OCCT) boundary representation (BRep) geometry. This is the 3D shape kernel — every solid, face, edge, vertex in the CAD model is a `TopoShape`. All PartDesign operations produce and consume `TopoShape` objects.

### FreeCAD core classes

**Shape hierarchy** (from OCCT `TopoDS`):
- **`TopoDS_Vertex`** — a point in 3D space
- **`TopoDS_Edge`** — a curve segment between vertices
- **`TopoDS_Wire`** — ordered sequence of connected edges forming a loop
- **`TopoDS_Face`** — a surface bounded by wires
- **`TopoDS_Shell`** — connected set of faces
- **`TopoDS_Solid`** — enclosed volume bounded by shells
- **`TopoDS_Compound`** — collection of any shapes
- **`TopoDS_CompSolid`** — set of solids sharing faces

**FreeCAD wrapper**: `Part::TopoShape` (~2800 lines header) adds:
- Element naming (topological naming / TNP) for tracking faces/edges through operations
- 34+ `make*` operations: `makeFuse`, `makeCut`, `makeElementPrism`, `makeElementRevolve`, `makeElementFillet`, `makeElementChamfer`, `makeElementDraft`, `makeElementThickSolid`, `makeElementOffset`, `makeElementSolid`, `makeElementFace`, `makeElementWires`, `makeElementMirror`, `makeElementSlice`, `makeElementBSplineFace`, `makeElementBoolean`, `makeElementTransform`, `makeElementCopy`, `makeElementRefine`, etc.
- `PropertyPartShape` — serializes shapes (BREP format)
- Boolean operations (fuse, cut, common, section)
- Tessellation for display
- Import/Export (STEP, IGES, BREP)

**2D Geometry** (shared with Sketcher):
- `Part::Geometry` base → `GeomPoint`, `GeomLineSegment`, `GeomCircle`, `GeomArcOfCircle`, `GeomEllipse`, `GeomArcOfEllipse`, `GeomBSplineCurve`, `GeomHyperbola`, `GeomParabola`
- `Part::Part2DObject` — shape constrained to a plane (sketch output)

### CADORA design (Rust)

For the BRep kernel, CADORA has a critical architectural decision:

**Option A: Wrap an existing kernel** (truck-factor, e.g. `opencascade-rs` bindings to OCCT, or use the `fornjot` Rust CAD kernel)
**Option B: Build a pure-Rust BRep kernel** (massive effort, multi-year, but full control)
**Recommended: Option A** — use `opencascade-rs` FFI bindings for the modeling kernel, with a Rust-idiomatic API layer on top.

```
cadora-shape
├── TopoShape       — Rust wrapper around OCCT TopoDS_Shape
├── ShapeType       — enum { Vertex, Edge, Wire, Face, Shell, Solid, Compound }
├── BooleanOps      — fuse, cut, common (wrapping BRepAlgoAPI)
├── PrismOps        — extrude, revolve, sweep, loft
├── DressUpOps      — fillet, chamfer, draft, thickness
├── FaceBuilder     — wire → face (FaceMaker)
├── ShapeIO         — STEP/BREP import/export
├── ElementMap      — topological naming persistence
└── Tessellation    — triangulation for rendering
```

### Task estimate: **~55 tasks**

| Sub-area | Tasks | Notes |
|----------|-------|-------|
| OCCT FFI setup + build system | 5 | opencascade-rs or custom bindings |
| TopoShape wrapper (all 8 shape types) | 8 | Rust-safe ownership of TopoDS handles |
| Boolean operations (fuse/cut/common/section) | 5 | BRepAlgoAPI wrappers |
| Prism/extrude/revolve | 5 | BRepPrimAPI, BRepFeat |
| Sweep/loft operations | 5 | BRepOffsetAPI_MakePipeShell, ThruSections |
| Fillet/chamfer/draft/thickness | 6 | BRepFilletAPI, BRepOffsetAPI |
| Face/wire/solid building | 5 | BRepBuilderAPI, FaceMaker logic |
| Element naming (TNP) | 6 | Map elements through operations |
| Tessellation/meshing | 3 | BRepMesh for GPU rendering |
| STEP/BREP I/O | 4 | STEPControl, BRepTools |
| Offset/shell/refine | 3 | Remaining operations |

### Dependencies
- Phase C (Document Model) — shapes are stored as `PropertyPartShape` in `DocumentObject`s

### Priority: **2 (Second)**

---

## Phase E: PartDesign Body & Feature Framework

**FreeCAD sources**: `src/Mod/PartDesign/App/Body.h/.cpp`, `src/Mod/PartDesign/App/Feature.h/.cpp`, `src/Mod/PartDesign/App/FeatureAddSub.h/.cpp`, `src/Mod/PartDesign/App/FeatureSketchBased.h/.cpp`, `src/Mod/PartDesign/App/FeatureRefine.h/.cpp`, `src/Mod/Part/App/BodyBase.h/.cpp`

### What it does
The `Body` is the central container for Part Design modeling. It holds an ordered list of features (pad, pocket, fillet, etc.) that are applied sequentially to build up a solid. The "Tip" marks the last active feature — features after the Tip are suppressed.

### FreeCAD class hierarchy
```
App::DocumentObject
  └── App::GeoFeature            (has Placement)
       └── Part::Feature          (has Shape)
            └── Part::BodyBase    (Group of features, Tip pointer)
                 └── PartDesign::Body  (the PartDesign body)

Part::Feature
  └── PartDesign::Feature         (has BaseFeature link, _Body link)
       └── PartDesign::FeatureRefine
            ├── PartDesign::FeatureAddSub   (Additive/Subtractive ops)
            │    └── ProfileBased           (sketch-based: Pad, Pocket, etc.)
            │    └── FeaturePrimitive       (Box, Cylinder, Sphere, etc.)
            │    └── DressUp                (Fillet, Chamfer, Draft, Thickness)
            └── PartDesign::Transformed     (LinearPattern, PolarPattern, etc.)
            └── PartDesign::Boolean         (Boolean operations)
```

### Key concepts
- **BaseFeature chain**: Each feature's `BaseFeature` property points to the previous solid feature. `execute()` takes the base shape + sketch profile → produces new shape.
- **Tip**: The feature whose output shape represents the Body's current state. Moving the Tip rolls back/forward the model history.
- **isSolidFeature()**: Returns true for features that modify the solid (skips sketches, datums).
- **addObject()**: Inserts at the current insert point (after Tip).
- **SingleSolidRule**: Enforces that the body produces exactly one contiguous solid.

### CADORA design (Rust)
```
cadora-partdesign
├── Body             — feature list, tip management, insert point
├── Feature trait     — base for all PD features (BaseFeature, execute)
├── FeatureAddSub    — additive/subtractive base (fuse/cut with base)
├── ProfileBased     — sketch-based features (profile + direction)
└── FeatureRefine    — optional shape refinement after boolean ops
```

### Task estimate: **~20 tasks**

| Sub-area | Tasks | Notes |
|----------|-------|-------|
| Body container (ordered feature list, Tip) | 5 | Insert/remove/reorder features |
| Feature trait + BaseFeature chain | 4 | execute() pipeline |
| FeatureAddSub (fuse/cut with base) | 4 | Boolean combination logic |
| ProfileBased (sketch → profile extraction) | 4 | Wire→Face, placement, normal |
| FeatureRefine | 1 | Shape cleanup after booleans |
| SingleSolidRule validation | 2 | |

### Dependencies
- Phase C (Document Model) — Body and Features are DocumentObjects
- Phase D (Shape Kernel) — features produce and transform shapes

### Priority: **3 (Third)**

---

## Phase F: PartDesign Operations

**FreeCAD sources**: `src/Mod/PartDesign/App/Feature*.h/.cpp` (one file pair per operation)

### What it does
The actual parametric modeling operations that users perform. Each is a `DocumentObject` with typed properties that calls the shape kernel to transform geometry.

### Complete operation inventory

#### Sketch-based (inherit ProfileBased → FeatureAddSub)
| Operation | FreeCAD class | Key properties | Description |
|-----------|---------------|----------------|-------------|
| **Pad** | `FeaturePad` → `FeatureExtrude` | Type, Length, TaperAngle, Direction, Midplane, Reversed, UpToFace | Extrude sketch profile to add material |
| **Pocket** | `FeaturePocket` → `FeatureExtrude` | Same as Pad | Extrude sketch profile to remove material |
| **Revolution** | `FeatureRevolution` | Axis, Base, Angle, Angle2, ReferenceAxis | Revolve profile around axis to add material |
| **Groove** | `FeatureGroove` | Same as Revolution | Revolve profile to remove material |
| **Loft** | `FeatureLoft` | Sections, Ruled, Closed | Sweep through multiple cross-sections |
| **Pipe/Sweep** | `FeaturePipe` | Spine, AuxiliarySpine, Mode, Transition, Sections | Sweep profile along a path |
| **Helix** | `FeatureHelix` | Pitch, Height, Turns, Angle, Growth, LeftHanded, Mode | Sweep profile along a helix |
| **Hole** | `FeatureHole` | Threaded, ThreadType, ThreadSize, Diameter, Depth, DrillPoint, HoleCutType | Parametric hole with thread standards |

**Extrude types**: `Dimension`, `UpToLast`, `UpToFirst`, `UpToFace`, `ThroughAll`, `TwoLengths`

#### Dress-up (inherit DressUp → FeatureAddSub)
| Operation | FreeCAD class | Key properties |
|-----------|---------------|----------------|
| **Fillet** | `FeatureFillet` | Radius, UseAllEdges |
| **Chamfer** | `FeatureChamfer` | ChamferType, Size, Size2, Angle, FlipDirection |
| **Draft** | `FeatureDraft` | Angle, NeutralPlane, PullDirection, Reversed |
| **Thickness** | `FeatureThickness` | Value, Reversed, Mode, Join |

#### Primitives (inherit FeaturePrimitive → FeatureAddSub)
| Primitive | Properties |
|-----------|-----------|
| **Box** | Length, Width, Height |
| **Cylinder** | Radius, Height, Angle |
| **Sphere** | Radius, Angle1, Angle2, Angle3 |
| **Cone** | Radius1, Radius2, Height, Angle |
| **Ellipsoid** | Radius1, Radius2, Radius3 |
| **Torus** | Radius1, Radius2, Angle1, Angle2, Angle3 |
| **Prism** | Polygon (sides), Circumradius, Height |
| **Wedge** | Xmin/Xmax/Ymin/Ymax/Zmin/Zmax/X2min/X2max/Z2min/Z2max |

#### Transformations (inherit Transformed → FeatureRefine)
| Operation | FreeCAD class | Key properties |
|-----------|---------------|----------------|
| **LinearPattern** | `FeatureLinearPattern` | Direction, Length, Occurrences, Mode |
| **PolarPattern** | `FeaturePolarPattern` | Axis, Angle, Occurrences, Mode |
| **Mirrored** | `FeatureMirrored` | MirrorPlane |
| **Scaled** | `FeatureScaled` | Factor, Occurrences |
| **MultiTransform** | `FeatureMultiTransform` | Transformations (list of sub-transforms) |

#### Other
| Operation | FreeCAD class | Notes |
|-----------|---------------|-------|
| **Boolean** | `FeatureBoolean` | Union/Cut/Intersection with other bodies |
| **ShapeBinder** | `ShapeBinder` / `SubShapeBinder` | Reference external geometry into body |
| **Base** | `FeatureBase` | Import external shape as body base |

### Task estimate: **~50 tasks**

| Sub-area | Tasks | Notes |
|----------|-------|-------|
| Pad/Pocket (FeatureExtrude) | 6 | All 6 type modes + taper + custom direction |
| Revolution/Groove | 4 | Axis computation, angle modes |
| Loft | 3 | Multi-section sweep |
| Pipe/Sweep | 4 | Spine, auxiliary, binormal, transition modes |
| Helix | 3 | 4 parameter modes |
| Hole | 5 | Thread tables, drill point, countersink |
| Fillet | 2 | Edge selection + radius |
| Chamfer | 3 | 3 chamfer types |
| Draft | 2 | Neutral plane computation |
| Thickness | 2 | Shell operation |
| Primitives (8 types) | 4 | Each is a small shape builder |
| LinearPattern | 3 | Direction + spacing modes |
| PolarPattern | 3 | Axis + angle modes |
| Mirrored | 2 | Mirror plane |
| MultiTransform | 2 | Composition of transforms |
| Boolean | 2 | Inter-body booleans |
| ShapeBinder | 2 | External reference resolution |

### Dependencies
- Phase E (Body framework) — all operations are features in a Body
- Phase D (Shape kernel) — all operations call shape kernel methods

### Priority: **4 (Fourth)**

---

## Phase G: Datum Geometry & Attachment

**FreeCAD sources**: `src/Mod/PartDesign/App/Datum*.h/.cpp`, `src/Mod/Part/App/DatumFeature.h/.cpp`, `src/Mod/Part/App/Attacher.h/.cpp`, `src/Mod/Part/App/AttachExtension.h/.cpp`

### What it does
Datum features are infinite reference geometry (planes, lines, points, coordinate systems) used to position sketches and features. The Attacher system computes their placement from reference geometry using ~40 attachment modes.

### FreeCAD core classes
- **`Part::Datum`** — abstract base (has `MapMode`, `MapPathParameter`, `MapReversed`, support references)
- **`PartDesign::Plane`** — infinite reference plane (ResizeMode, Length, Width)
- **`PartDesign::Line`** — infinite reference line (ResizeMode, Length)
- **`PartDesign::Point`** — reference point
- **`PartDesign::CoordinateSystem`** — reference coordinate frame
- **`Attacher::AttachEngine`** — computes `Placement` from support geometry + attachment mode
- **`Attacher::eMapMode`** (~40 modes) — `mmFlatFace`, `mmNormalToPath`, `mmThreePointsPlane`, `mmConcentric`, `mmTranslate`, `mmObjectXY/XZ/YZ`, `mmFrenetNB/TN/TB`, etc.

### CADORA design (Rust)
```
cadora-datum
├── DatumPlane       — infinite plane with display bounds
├── DatumLine        — infinite line with display length
├── DatumPoint       — reference point
├── DatumCS          — coordinate system (3 axes + origin)
└── AttachEngine     — compute placement from references + mode
```

### Task estimate: **~18 tasks**

| Sub-area | Tasks | Notes |
|----------|-------|-------|
| DatumPlane/Line/Point/CS | 5 | 4 datum types with properties |
| AttachEngine core | 4 | Placement from references |
| Attachment modes (most-used ~15 of 40) | 6 | Implement incrementally |
| Origin (standard XY/XZ/YZ planes + axes) | 3 | Per-body origin |

### Dependencies
- Phase C (Document Model) — datums are DocumentObjects
- Phase D (Shape Kernel) — geometric computations for attachment

### Priority: **5 (Fifth)**

---

## Phase H: File Format & Persistence

**FreeCAD sources**: `src/App/ProjectFile.h/.cpp`, `src/App/Document.cpp` (Save/Restore methods), `src/Base/Writer.h`, `src/Base/Reader.h`

### What it does
FreeCAD uses `.FCStd` files which are **ZIP archives** containing:
- `Document.xml` — main XML describing all objects and their properties
- `GuiDocument.xml` — view provider settings (colors, visibility)
- Binary shape files (`*.brp`) — OCCT BRep shapes
- Embedded files (images, etc.)
- `Metadata.xml` — project metadata

### FreeCAD mechanism
1. **Save**: `Document::Save()` → `Base::Writer` → `XMLWriter` + `ZipWriter`. Each `DocumentObject` calls `Save()` on all its `Property` objects. Shapes are serialized separately as `.brp` binary streams in the ZIP.
2. **Restore**: `Document::Restore()` → parse `Document.xml` → create `DocumentObject` instances → call `Restore()` on each → `afterRestore()` triggers re-execution.

### CADORA design (Rust)
CADORA should define its own project format (`.cadora`), likely also ZIP-based:
```
cadora-project
├── ProjectFile       — ZIP archive read/write
├── XmlSerializer     — object/property → XML
├── ShapeSerializer   — TopoShape → binary (BREP or STEP)
├── Metadata          — project info, version, author
└── Migration         — version compatibility
```

### Task estimate: **~15 tasks**

| Sub-area | Tasks | Notes |
|----------|-------|-------|
| ZIP archive read/write | 2 | Use `zip` crate |
| XML serialization (properties) | 5 | All property types to/from XML |
| Shape binary serialization | 3 | BREP stream format |
| Project metadata | 2 | Version, author, etc. |
| Migration/compatibility | 3 | Version evolution |

### Dependencies
- Phase C (Document Model) — serializes Documents and DocumentObjects
- Phase D (Shape Kernel) — serializes shapes

### Priority: **6 (Sixth)**

---

## Phase I: GUI & 3D Viewport

**FreeCAD sources**: `src/Gui/View3DInventor*.h/.cpp`, `src/Gui/ViewProvider*.h/.cpp`, `src/Gui/Selection/`, `src/Gui/Document.h/.cpp`, `src/Gui/Tree.h/.cpp`, coin3d (external)

### What it does
FreeCAD uses **Coin3D** (an OpenInventor scene graph library) for 3D rendering, with **Qt** for the UI framework. Each `DocumentObject` has a corresponding `ViewProvider` that manages its visual representation.

### Key components
- **View3DInventorViewer** — the 3D viewport (camera, lighting, rendering)
- **ViewProvider** hierarchy — mirrors the `DocumentObject` hierarchy, one VP per object type
- **Selection system** — point-and-click selection of faces, edges, vertices with preselection highlighting
- **Tree View** — hierarchical model browser
- **Task Panels** — parameter entry dialogs for each operation
- **NaviCube** — 3D navigation widget

### CADORA design (Rust)
For a Rust rewrite, the rendering stack would differ fundamentally:

**Recommended**: Use **wgpu** (Rust-native GPU abstraction) + **egui** or **iced** for UI.

```
cadora-gui
├── Viewport3D        — wgpu-based 3D renderer
│    ├── Camera       — orbit/pan/zoom controls
│    ├── SceneGraph   — shape mesh rendering
│    ├── Selection    — ray-casting, hit testing, highlighting
│    └── NaviCube     — orientation widget
├── ViewProvider      — trait: object → visual representation
├── ModelTree         — hierarchical feature browser
├── PropertyPanel     — property editor for selected object
├── TaskPanel         — operation-specific parameter UI
└── SketchView        — 2D sketch editing overlay
```

### Task estimate: **~65 tasks**

| Sub-area | Tasks | Notes |
|----------|-------|-------|
| wgpu renderer setup | 5 | Pipeline, shaders, window |
| Camera controls (orbit/pan/zoom) | 4 | |
| Mesh rendering (solid + wireframe) | 6 | From tessellated shapes |
| Lighting + materials | 3 | |
| Selection (ray-cast, hover, multi-select) | 8 | Face/edge/vertex picking |
| Model tree view | 5 | Expand/collapse, drag/drop, context menu |
| Property editor panel | 6 | All property types |
| Task panels (per-operation dialogs) | 12 | ~20 operations need dialogs |
| Sketch 2D view | 8 | Grid, constraints display, interactive drawing |
| NaviCube + view orientations | 3 | |
| Status bar + notifications | 2 | |
| Undo/redo UI | 3 | |

### Dependencies
- Phase C (Document Model) — objects to display
- Phase D (Shape Kernel) — tessellation for rendering
- All other phases for their ViewProviders

### Priority: **7 (Seventh)** — Can develop in parallel with backend phases using stub geometry

---

## Phase J: Assembly (Optional / Future)

**FreeCAD sources**: `src/Mod/Assembly/App/AssemblyObject.h/.cpp`, `src/Mod/Assembly/JointObject.py`, `src/Mod/Assembly/CommandCreateJoint.py`

### What it does
FreeCAD's Assembly workbench (introduced 2023/2024) uses the **OndselSolver** (MbD multibody dynamics library) to solve joint constraints between parts. It supports:
- Fixed, Revolute, Cylindrical, Slider, Ball, Distance, Parallel, Perpendicular, Angle, RackPinion, Screw, Gears, Belt joints
- Grounded parts
- Exploded views
- Bill of Materials (BOM)
- Simulation

### FreeCAD core classes
- **`Assembly::AssemblyObject`** — inherits `App::Part`, contains joints and parts
- **`Assembly::JointGroup`** — container for joints
- **`Assembly::AssemblyLink`** — link to external part
- Joint types defined in Python (`JointObject.py`), solved via MbD C++ library
- Uses `ASMTAssembly`, `ASMTPart`, `ASMTJoint`, `ASMTMarker` from OndselSolver

### Task estimate: **~35 tasks** (if implementing)

| Sub-area | Tasks | Notes |
|----------|-------|-------|
| Assembly container | 4 | |
| Joint types (10+ types) | 12 | One per joint type |
| Assembly solver (MbD integration or custom) | 10 | Position solving from constraints |
| Exploded views | 3 | |
| BOM generation | 3 | |
| Simulation | 3 | Deferred |

### Dependencies
- Phases C, D, E (full modeling stack)

### Priority: **10 (Future)**

---

## Summary: Estimated Task Counts

| Phase | Module | Est. Tasks | Est. Tests | Priority |
|-------|--------|-----------|-----------|----------|
| A | GCS Solver | 79 | 178 | ✅ Done |
| B | Sketch Manager | 54 | 282 | ✅ Done |
| C | Document Model & Properties | ~45 | ~120 | 1 |
| D | Part/Shape Kernel (BRep) | ~55 | ~150 | 2 |
| E | Body & Feature Framework | ~20 | ~60 | 3 |
| F | PartDesign Operations | ~50 | ~140 | 4 |
| G | Datum Geometry & Attachment | ~18 | ~50 | 5 |
| H | File Format & Persistence | ~15 | ~40 | 6 |
| I | GUI & 3D Viewport | ~65 | ~80 | 7 |
| J | Assembly (future) | ~35 | ~80 | 10 |
| **Total** | | **~436** | **~1180** | |

---

## Implementation Order & Dependency Graph

```
Phase A: GCS Solver ✅
Phase B: Sketch Manager ✅
         │
         ▼
Phase C: Document Model ◄──────────────────────────────────┐
         │                                                   │
         ▼                                                   │
Phase D: Shape Kernel (BRep) ◄── OCCT bindings              │
         │                                                   │
         ├──────────────┐                                    │
         ▼              ▼                                    │
Phase E: Body/Feature   Phase H: File Format                 │
         │                                                   │
         ▼                                                   │
Phase F: PartDesign Operations                               │
         │                                                   │
         ▼                                                   │
Phase G: Datums & Attachment                                 │
                                                             │
Phase I: GUI ◄── Can start early with stubs ─────────────────┘
         │
         ▼
Phase J: Assembly (future)
```

### Parallel work streams
1. **Core backend**: C → D → E → F → G → H (sequential, each builds on prior)
2. **GUI**: Can start Phase I in parallel once Phase C is minimally viable, using mock shapes
3. **Integration testing**: After Phase F, end-to-end tests (sketch → pad → fillet → save → reload)

---

## Key Architectural Decisions for CADORA

### 1. BRep Kernel Strategy
**Decision needed**: Wrap OCCT via FFI vs. pure-Rust kernel vs. use `fornjot`/`truck`

| Option | Pros | Cons |
|--------|------|------|
| **OCCT via `opencascade-rs`** | Production-proven, complete, STEP support | Large C++ dependency, FFI complexity |
| **truck (Rust BRep)** | Pure Rust, growing ecosystem | Less mature, missing some operations |
| **Custom** | Full control | Multi-year effort for a usable BRep kernel |

**Recommendation**: Start with OCCT bindings for correctness, evaluate `truck` as it matures.

### 2. GUI Framework
**Decision needed**: egui vs. iced vs. slint vs. tauri

| Option | Pros | Cons |
|--------|------|------|
| **egui + wgpu** | Immediate mode, easy 3D integration | Less native look |
| **iced** | Elm-like, native rendering | 3D viewport integration harder |
| **slint** | Declarative, native look | Commercial license for some uses |
| **tauri + web** | Cross-platform, rich UI | Performance overhead, JS/Rust boundary |

### 3. File Format
**Decision needed**: Custom `.cadora` vs. FreeCAD-compatible `.FCStd`

**Recommendation**: Define own format (simpler, no legacy), but support STEP import/export for interoperability.

### 4. Expression Language
Keep it simple initially — support basic arithmetic with property references (`Pad.Length = width * 2 + 5mm`). FreeCAD's expression engine is ~5000 lines with a full parser.

---

## Milestone Targets

| Milestone | Phases needed | What it demonstrates |
|-----------|---------------|---------------------|
| **M1: Parametric Sketch** | A+B | Create, constrain, solve sketches |
| **M2: First Extrusion** | +C+D+E+F(partial) | Sketch → Pad → 3D solid |
| **M3: Multi-feature Part** | +F(more)+G | Pad → Pocket → Fillet workflow |
| **M4: Save/Load** | +H | Persist and restore projects |
| **M5: Interactive Modeling** | +I | Full GUI with 3D viewport |
| **M6: Assembly** | +J | Multi-part assembly |

You are currently at **M1**. The next critical milestone is **M2** (first extrusion), which requires Phases C, D, E, and partial F.
