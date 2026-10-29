# Mesh Cut: Technical Documentation

[Project overview](../README.md) · [中文](TECHNICAL_CN.md) · [日本語](TECHNICAL_JP.md) · **English**

## 1. Overview and scope

Mesh Cut is an interactive mesh-slicing demonstration built with C++, DirectX 11, and DirectXMath. A mouse stroke defines a cutting plane. The CPU clips the input triangles, reconstructs section contours, builds caps for both halves, and prepares replacement rendering resources.

The current implementation supports plane slicing, intersection-attribute interpolation, capping of simple concave contours, multiple separate non-nested contours, and repeated cuts of the resulting mesh. OBJ drag and drop replaces the model; F1 switches solid/wireframe rendering, and F3 toggles the cutting-plane visualization. Two-stack snapshot history supports undo/redo, with on-screen controls in both Debug and Release.

A cut reconstructs triangle surfaces and offsets both sides. It does not generate collision shapes or simulate separate rigid bodies.

## 2. Modules and data flow

| Module | Responsibility |
|---|---|
| `main.cpp` | Drop requests, display-mode input, and the update/render loop |
| `ObjImporter` | OBJ/MTL parsing and CPU vertex, index, subset, and material data |
| `player.cpp` | Stroke interaction, plane conversion, algorithm integration, staging, and model updates |
| `MeshCutter` | Math helpers, interpolation, clipping, intersection topology, contour triangulation, and caps |
| `ModelSnapshot.h / .cpp` | Independent model snapshots, texture-reference ownership, and CPU/GPU restoration |
| `ModelHistory.h / .cpp` | Two history stacks, cut-history commits, undo/redo, and clearing |
| `renderer.cpp` | DirectX resources and combined fill/cull rasterizer states |
| `camera.cpp` | Perspective camera, matrices, and reset behavior |

`MeshCutter.h` declares the types and interfaces. Non-template methods live in `.cpp`, while template definitions live in `.inl`, included at the end of the header. Geometry processing operates on CPU data, but the current class also exposes DirectXMath coordinate helpers; its public header is not completely independent of DirectX.

The pipeline is:

1. Construct a world-space plane from the screen stroke.
2. Convert the plane to model-local space.
3. Expand subset triangles and organize them by persistent piece IDs.
4. Clip across materials within each piece, sharing intersections and collecting section segments.
5. Merge intersection nodes, recover closed contours, and project them to 2D.
6. Reject unsupported nested contours and triangulate caps with ear clipping.
7. Generate cap attributes, offset only split pieces, validate final connectivity, and assign new IDs.
8. Stage every subset, capture the pre-cut state, prepare CPU/GPU resources, and commit geometry and history.

## 3. Screen input and the cutting plane

### 3.1 Constructing a world-space ray

Screen coordinates $(s_x,s_y)$ are mapped to normalized device coordinates:

$$
x_{ndc}=\frac{2s_x}{w}-1,\qquad y_{ndc}=1-\frac{2s_y}{h}
$$

The implementation uses `SCREEN_WIDTH / SCREEN_HEIGHT`. Y is inverted because window coordinates increase downward, whereas projected coordinates increase upward.

For the current symmetric perspective projection, with vertical field of view $\theta_y$ and aspect ratio $a$:

$$
P_{11}=\frac{1}{\tan(\theta_y/2)},\qquad P_{00}=\frac{P_{11}}{a}
$$

Choosing view-space $z=1$ gives a direction through the cursor:

$$
D_{view}=\left(\frac{x_{ndc}}{P_{00}},\frac{y_{ndc}}{P_{11}},1\right)
$$

`CreateWorldRayDirection()` transforms this direction by the inverse view matrix and normalizes it. Translation is excluded for the direction. The origin is obtained separately by transforming the view-space origin into world space.

With zero-based subscripts, $P_{00}$ and $P_{11}$ are the projection matrix’s X and Y scale terms. Setting both to 1 corresponds to a symmetric perspective model with a $90^\circ$ vertical field of view and aspect ratio 1, not orthographic projection. The direction is constructed with `w=0` and transformed using `XMVector3TransformNormal()`, excluding camera translation.

Omitting division by $P_{00}/P_{11}$ does not produce orthographic rays: they still diverge from one origin, but no longer match the actual camera projection. This construction assumes a conventional symmetric perspective camera and a full-window viewport, not orthographic or off-center projection.

### 3.2 A plane through two rays

Let $D_s,D_e$ be the unit directions for the stroke endpoints, with shared camera origin $O$:

$$
N=\mathrm{normalize}(D_e\times D_s),\qquad Q=O
$$

The cutting plane satisfies:

$$
(P-Q)\cdot N=0
$$

Strokes shorter than two pixels are ignored. The operation also stops when the squared cross-product length is below $10^{-12}$, rather than normalizing an unstable near-zero vector.

### 3.3 Stroke acceptance

The mathematical plane is infinite, so `IsInsideStroke()` also evaluates section-segment endpoints. Each point is transformed by world, view, and projection matrices, divided by homogeneous W once, and mapped back to screen coordinates. It must lie in the axis-aligned rectangle formed by the stroke endpoints, expanded by a two-pixel margin.

This is not an exact point-to-stroke-segment distance test. Points with $w\leq10^{-5}$ are rejected. If an actual section endpoint fails acceptance, the current piece returns `Unchanged`; other pieces may still be cut independently. Acceptance applies to the piece-wide section, not separately to material groups.

`SliceModel()` receives this acceptance rule as a callback. When no screen restriction is required, the caller can pass a callback that always returns true.

## 4. Converting the plane to model-local space

Rendering and cutting use the same world transform. With row vectors:

$$
W=SRT
$$

The current model orientation includes an additional $\pi$ rotation around Y. A plane point and its normal require different transformations:

$$
Q_{local}=Q_{world}W^{-1}
$$

$$
N_{local}=\mathrm{normalize}(N_{world}W^T)
$$

Normals use the inverse transpose of the matrix that performs the coordinate transformation. Here that transformation is $W^{-1}$, so:

$$
\left((W^{-1})^{-1}\right)^T=W^T
$$

This preserves the normal's perpendicularity under non-uniform scaling. The world transform is assumed to be invertible; zero scale is not a supported operation.

## 5. Triangle input, pieces, and signed distance

`CutMeshes()` resolves `IndexArray` into expanded triangles and passes all material subsets to `SliceModel()`. A subset is a rendering-material group, not a closed solid. One piece can span several subsets.

For an initial model, `InitializePieceIds()` identifies geometric connectivity across materials. Subsequent operations group triangles by their persistent piece IDs instead of reconnecting the entire scene by position. Separate pieces therefore retain their identities after displacement makes them overlap. Cutting only the middle piece leaves the others' positions, attributes, and IDs intact.

Shared position IDs are built within each piece. For the unit cutting-plane normal:

$$
d=(P-Q)\cdot N
$$

Positive and negative distances identify the two sides. This normal belongs to the cutting plane, not the mesh's lighting normals; every subset uses the same plane. The default positional tolerance is $\epsilon=10^{-4}$ in model-local units. Distances are computed once per shared position in double precision. When $|d|\leq\epsilon$, classification becomes zero and the shared output position is projected onto the plane. Input arrays remain unchanged.

A piece without vertices strictly on both sides is not cut. The tolerance depends on asset scale and is not a universal precision guarantee.

## 6. Half-space clipping and attributes

### 6.1 Retaining each half-space

`ClipSubset()` processes a material group's triangles. `AppendClippedTriangle()` generates each side, retaining vertices satisfying:

$$
d\times side\geq0
$$

`side=+1` keeps the positive side; `side=-1` keeps the negative side. On-plane vertices belong to both boundaries. Edges A→B follow the original triangle order. For ordinary endpoints strictly away from the plane:

| A is inside the retained half-space | B is inside | Vertices appended for this edge |
|---|---|---|
| Yes | Yes | A |
| Yes | No | A, intersection |
| No | Yes | Intersection |
| No | No | None |

An original on-plane vertex is appended when it is the edge's starting vertex. New interior intersections are generated only for strictly opposite distance signs, avoiding repeated endpoint interpolation. Both sides reuse the precomputed intersections.

The resulting polygon preserves boundary order and is convex, so it can be triangulated as a fan $(P_0,P_i,P_{i+1})$. A typical crossing yields a triangle on the single-vertex side and a quadrilateral split into two triangles on the other. An un-crossed triangle belongs to its retained side. Fan triangulation handles clipped original surfaces; potentially concave caps use ear clipping later.

### 6.2 Interpolation and shared positions

For endpoint distances $d_A,d_B$, distance along the edge is $d(t)=d_A+t(d_B-d_A)$. Setting it to zero gives:

$$
t=\frac{d_A}{d_A-d_B},\qquad P=A+t(B-A)
$$

The same parameter interpolates position, UVs, RGBA color, and normals; interpolated normals are normalized. Parameters 0 and 1 return the corresponding original vertex directly, avoiding a rounded interpolation result that differs from the endpoint.

`InterpolateSharedEdge()` uses canonical endpoint order and double precision for the position, caching it by shared edge IDs. Surfaces and caps use the same representative position. Normals, UVs, and colors are still interpolated from each triangle's own rendering vertices, preserving material and attribute seams. Unhandled extra fields inherit endpoint A; tangent or skinning interpolation is not implied.

### 6.3 Triangle registration

`Triangle()` preserves supplied winding and computes the cross product in double precision, filtering actual zero-area faces. It does not independently discard a valid small face merely because its area falls below the squared positional tolerance, which could leave a gap. Nonzero area alone does not establish validity; final connectivity is also checked.

## 7. Section segments and shared topology

Surface triangles and cap contours are separate outputs sharing intersection results. The triangle-local `hits` in `ClipSubset()` stores cutting-node IDs, not cap triangles or a complete loop. `AppendUniqueIndex()` deduplicates node IDs.

| Level | Identity and purpose |
|---|---|
| Original shared vertex | Shared position ID gives adjacent triangles consistent classification |
| Edge intersection | Undirected endpoint-ID pair caches the shared intersection position |
| Cutting node | A representative position within tolerance is used by both surface and cap |
| Section segment | Undirected node pair accumulates which sides its incident faces occupy |

### 7.1 Cuts versus same-side contact

A segment crossing a triangle interior directly contributes both side flags. When the plane follows an existing edge, a single triangle contributes only one side. After processing all material groups in the piece, flags for that edge are combined. Only edges with both positive and negative incidence enter contour adjacency. An edge whose neighboring faces lie on the same side is contact, not a cut boundary. An isolated point-contact node need not form a loop.

Existing-edge cuts can therefore succeed, while exterior tangency can return `Unchanged`. Entire coplanar triangles during a genuine split, or otherwise ambiguous degenerate connectivity, can still reject the update. Temporary output is adopted only after validation.

## 8. Adjacency and closed contours

`nodes[i]` stores a section vertex; `adjacency[i]` stores connected node indices. Each segment AB is registered in both directions.

`BuildClosedContours()` ignores zero-degree contact nodes outside actual section segments and requires exactly two neighbors per participating node. It starts at an unvisited node and follows the neighbor that is not the previous node.

- Returning to the start completes a contour.
- Reaching another visited node before closing fails.
- A degree other than two, or fewer than three contour nodes, fails.
- Remaining unvisited nodes start additional independent contours.

Each traversal registers a new node or terminates at a visited node, so it cannot traverse indefinitely with valid node indices. Degree-two connectivity does not establish that the contour is geometrically free of self-intersections.

## 9. Planar coordinates and nesting

Projection and orientation use double-precision intermediates. A reference axis R is selected to avoid near-parallel alignment with the unit normal: X when $|N_x|<0.9$, otherwise Y.

$$
U=\mathrm{normalize}(R\times N),\qquad V=N\times U
$$

With $U\times V=N$, points are projected as:

$$
p_u=(P-Q)\cdot U,\qquad p_v=(P-Q)\cdot V
$$

`projected[i]` corresponds to `nodes[i]`; `loops` stores ordered node indices, not reordered coordinate arrays.

`IsPointInsideContour()` uses the odd/even rule for intersections with a horizontal ray to the right. `HasNestedContours()` tests a representative node from each contour against the others.

This assumes simple contours that neither intersect nor touch each other. It is not a complete contour-intersection test. Nesting returns `UnsupportedHole`, rather than incorrectly filling a hollow section with independent solid caps.

## 10. Winding and ear clipping

### 10.1 Counterclockwise order

Twice the signed area is:

$$
A_2=\sum_i(x_i y_{i+1}-y_i x_{i+1})
$$

A negative value reverses the contour. The implementation accumulates relative to the first contour point to reduce cancellation at larger coordinates.

### 10.2 Choosing an ear

For consecutive A, B, C:

$$
\mathrm{Orient}(A,B,C)=(B_x-A_x)(C_y-A_y)-(B_y-A_y)(C_x-A_x)
$$

A positive value identifies a convex corner in counterclockwise order. An ear must have a reliably positive orientation and contain no other contour vertex; otherwise removing it could cross the boundary or cover another region. Among valid ears, the algorithm favors greater minimum triangle altitude to reduce thin faces. It saves ABC, removes only B from the working contour, and continues.

Orientation and point-in-triangle tests use double precision and determinant-dependent error bounds, rather than treating squared positional tolerance as a universal orientation threshold. The contour is copied; original nodes remain available. A successful triangulation of an n-node simple boundary yields n−2 triangles. Failure to find valid ears or finish triangulation returns `TriangulationFailed`.

`TriangulateContours()` outputs `TriangleIndices`, triples of node IDs, separately from cap-vertex generation.

### 10.3 Boundary preservation and numerical stability

Nearly collinear boundary chains may be simplified temporarily to improve ear clipping. Omitted nodes are then restored into the triangulation so that surface and cap boundary segmentation agrees. Bounded internal-edge improvement preserves the boundary rather than hiding connectivity errors by deleting short edges.

After separation is stored in float coordinates, cap facing is checked and, when needed, retriangulated using the actual displaced positions. Existing planar surfaces are rebuilt only when reference facing, planarity, uniform normals, approximately affine UVs, and uniform vertex color permit it, protecting meaningful attribute seams.

On `OpenOrAmbiguousContour` or `TriangulationFailed`, the piece may retry the same input and plane with positional tolerance halved at most twice. Refinement is bounded by float coordinate spacing and a distance-arithmetic error floor. The cutting plane is never shifted; `Unchanged`, invalid input, and holes do not trigger this retry.

## 11. Caps, normals, and UVs

`AppendCapTriangles()` turns these triples into caps for both halves. `CreateCapVertex()` copies the intersection vertex and replaces its normal and UV.

| Half | Outward cap normal | Triangle order |
|---|---|---|
| Negative, $d\leq0$ | $+N$ | A, B, C |
| Positive, $d\geq0$ | $-N$ | C, B, A |

Winding controls geometric orientation; the stored `Normal` controls the lighting attribute. Both are assigned. Original surface and cap vertices may share a position but have different normals to represent a hard cut edge.

Planar UVs are:

$$
(UV_x,UV_y)=(p_u,p_v)\times uvScale
$$

The default is `uvScale=0.1`. Coordinates are not normalized to fit the cap into [0,1]; material and sampler behavior determine the appearance outside that interval. Caps are output separately and assigned an existing material by the application. No dedicated cap material is generated, and a cross-material cap need not be partitioned along the original material boundaries.

Surface and cap vertices of an actually split piece are offset together; uncut pieces stay in place:

$$
P_-=P-N\times separation,\qquad P_+=P+N\times separation
$$

The default is 1.5 local units per side, producing a three-unit gap between originally coincident cut faces. The completed result returns `Status::Cut`. Subsequent cuts operate directly on the generated, offset vertices.

## 12. Piece results, materials, and model updates

`SliceModel()` reconstructs contours across subsets within each persistent piece. `ModelResult::subsets` stores cut surfaces under their original material groups, `caps` stores caps separately, and `unchangedSubsets` preserves uncut pieces with per-triangle IDs. An individual material group need not have a closed contour; the related faces of the whole piece provide it.

- `Cut`: adopt new surfaces and caps; separate only pieces actually split.
- Piece `Unchanged`: retain original geometry, attributes, and IDs without separation.
- Any other piece failure: cancel the entire model update rather than commit partial success.
- No changes: skip resource reconstruction and history insertion.

At final displaced coordinates, each undirected edge in generated geometry must have exactly two incident faces. This requirement is not relaxed to accept a failed result. New piece IDs are staged with geometry and installed only on successful model replacement.

`CutMeshes()` preserves original surface/material associations. Caps reuse an existing material, without creating a dedicated cap material. After capacity checks, `TryUpdateModel()` creates expanded vertices, sequential indices, subset/material records, and independent GPU buffers. Only full preparation releases the old model. The current 16-bit representation accepts at most 65,535 expanded vertices.

| Status | Meaning |
|---|---|
| `Unchanged` | Retain the original piece or model |
| `Cut` | New surfaces, caps, and piece IDs are prepared |
| `InvalidInput` | Input layout, IDs, or settings violate requirements |
| `OpenOrAmbiguousContour` | Open, branching, coplanar-ambiguous, or invalid generated connectivity |
| `UnsupportedHole` | Unsupported nested contours |
| `TriangulationFailed` | Contour triangulation or required safe retriangulation could not finish |

These are algorithm results. Vertex-capacity and GPU-allocation failures occur during application submission. Failure installs no temporary geometry and creates no successful-cut history entry.

## 13. Ownership and failure handling

| Resource | Owner |
|---|---|
| Subset arrays before handoff | `unique_ptr<SimpleArray<VERTEX_3D>>` |
| Temporary arrays in update queues | `QueueCleanup`, which deletes them and clears the queues on exit |
| Replacement CPU arrays before installation | `unique_ptr<T[]>` |
| Replacement GPU buffers before installation | Local `BufferOwner`, releasing them on failure |
| Installed model | `UnloadModel()`, releasing texture references, buffers, arrays, and the outer `MODEL` |

Temporary material queues borrow texture pointers and do not release them. Each final subset receives an `AddRef()` before old-model release, preserving shared texture lifetime while old subset references are removed.

CPU arrays transfer through `release()`; temporary GPU owners are cleared after handoff. A cleanup object must actually be instantiated: a type declaration alone does not execute a destructor.

The path handles existing algorithm failure results, invalid arguments/capacity, allocation failure, and buffer-creation failure. It is not comprehensive recovery for every malformed asset or possible exception. Internal `SimpleArray` storage release relies on that container's existing destructor contract.

## 14. Snapshots, undo, and redo

### 14.1 Snapshot contents

`ModelSnapshot` stores independent vertex, index, and subset/material arrays. Positions, normals, colors, UVs, previously generated caps, and baked separation offsets are part of the saved model state. Per-triangle `TrianglePieceIds` and `NextPieceId` are also saved so undo/redo restores piece identity. GPU buffers, the camera, presentation transforms, fill mode, and the plane-visibility setting are not captured.

`Capture()` deep-copies CPU arrays without duplicating texture pixels. It calls `AddRef()` for each saved subset texture and matches those references with `Release()` in the snapshot destructor. Snapshots cannot be copied and are owned through `unique_ptr<ModelSnapshot>`.

`Restore()` copies replacement CPU arrays with the saved indices and subset ranges, then creates replacement vertex and index buffers. It does not use the cutting queues or rerun slicing. After preparation succeeds, it acquires texture references for the restored model, releases the old model through `UnloadModel()`, and installs the resources. Allocation or buffer-creation failure leaves the displayed model intact.

### 14.2 Two stacks

`ModelHistory` maintains undo and redo histories using two `std::stack<std::unique_ptr<ModelSnapshot>>` containers. The currently displayed model is separate; stack entries represent other recoverable states.

| Operation | Restore from | Save the current state to |
|---|---|---|
| Undo | Undo-stack top | Redo stack |
| Redo | Redo-stack top | Undo stack |

After two successful cuts A → B → C, the display shows C and the undo stack contains A, B from bottom to top. Undo displays B, leaving A on the undo stack and C on the redo stack. A second undo displays A and leaves C, B on the redo stack from bottom to top. Redo restores B and then C.

This does not move the same snapshot between stacks: restoring B consumes B, while a newly captured snapshot of C is saved to the opposite stack.

### 14.3 Committing geometry and history

After slicing and capacity checks, `PrepareCut()` captures the old model and temporarily pushes it onto the undo stack. `TryUpdateModel()` attempts the replacement; a local `CutHistoryGuard` invokes `FinishCut()` when its scope ends.

Success retains the undo entry and clears redo. Failure removes the temporary entry and preserves existing redo history. Missed cuts, algorithm rejection, and capacity rejection add no entry. If the old state cannot be saved, the cut is not committed.

Undo/redo also capture and push the current state before changing the model, because a stack push may allocate memory. Restore failure removes the newly pushed entry and leaves the source stack intact. Success pops the consumed source entry. Empty history returns `S_FALSE`, while an actual restore returns `S_OK`; neither is an HRESULT failure.

A successful model import clears both stacks; a failed import retains them. A successful new cut after undo creates a new editing branch and discards the old redo history. Shutdown explicitly clears history to release snapshots and their texture references.

### 14.4 Input and resource scope

`Ctrl+Z` undoes; `Ctrl+Y` or `Ctrl+Shift+Z` redoes. Either Ctrl/Shift key is accepted. Z/Y press edges trigger an operation, so holding a key does not repeat it every frame. History shortcuts are ignored while dragging a cutting stroke. A successful restore invalidates the old cutting-plane preview while retaining its visibility setting, the fill mode, and the camera.

History is memory-only and is not serialized. Every entry stores complete CPU geometry rather than a delta. There is currently no entry-count or memory-budget limit, so memory use grows with editing history. Textures are shared through COM references; history does not duplicate texture pixels or retain a separate GPU mesh buffer for every saved state.

## 15. OBJ drag and drop

`DragAcceptFiles()` and `WM_DROPFILES` receive one OBJ path. The window callback stores the request; the next update performs synchronous loading.

The wide path is converted to the Windows ANSI code page used by the loader, rejecting unrepresentable characters. Separators are normalized to `/` for associated-resource path handling. This is not complete Unicode import. Avoid spaces in referenced MTL/texture filenames and use files compatible with the existing parser.

`ReplacePlayerModel()` loads into a temporary model. Only success swaps CPU data and both GPU buffers into the displayed model, then releases the old resources. Failure keeps the current model and history. Success clears both history stacks.

Success resets position, rotation, scale, movement inertia, drag state, plane data, and the camera. Dropping the same OBJ again restores its uncut geometry. R resets presentation only, without reloading geometry. The drop gesture is canceled and left-button release is consumed to prevent an accidental cut.

Loading is synchronous, without a background thread. There is no automatic model centering or camera fitting to its bounds.

## 16. Display modes and on-screen controls

F1 switches `D3D11_FILL_SOLID / D3D11_FILL_WIREFRAME`. Two fill modes and three cull modes form six pre-created rasterizer states. Both selections are tracked and applied together, so `SetCullingMode()` does not overwrite wireframe mode.

F3 toggles the cutting-plane preview. Drawing requires successfully initialized preview vertices. Four points at distances 15 and 300 along the two rays form the preview quad; those distances do not bound the mathematical cutting plane.

The preview represents the most recently attempted plane and may update even when slicing is rejected. Model reset and successful undo/redo invalidate the old preview while preserving its visibility setting. The selected fill mode also affects the preview.

`PrintDebugProc()` registers per-frame control text; `DrawDebugProc()` calls `DebugTextOut()` to render it. F1/F3 settings and history shortcuts are displayed in both Debug and Release. Text support uses `DEBUG_DISP_TEXTOUT`, a BGRA back buffer, and a GDI-compatible swap chain. The D3D debug layer remains Debug-only. Camera controls and R reset are also available in both Debug and Release.

| Input | Action |
|---|---|
| Left drag and release | Attempt a cut |
| Drop one OBJ | Replace/reset the model; clear history on success |
| Ctrl + Z | Undo |
| Ctrl + Y / Ctrl + Shift + Z | Redo |
| F1 | Solid / wireframe |
| F3 | Cutting-plane visibility |
| Arrow keys | Move the model relative to camera heading |
| A / D | orbit the camera horizontally |
| W / N | raise / lower the camera |
| U / M | move the camera closer / farther |
| R | reset model transform and camera, retaining cut geometry |
| Esc | Exit |

Camera orbit uses A/D and upward movement uses W, leaving Z/Y for history shortcuts. The camera therefore needs no Ctrl-conflict check.

## 17. Environment, cost, and limitations

Build the Windows Visual Studio project with C++17 and its DirectX 11, DirectXMath, input, and D3DX11 dependencies. Shader/asset paths must match the working directory. Use triangulated OBJ assets with normals, UVs, and valid MTL/texture references.

| Area | Current scope |
|---|---|
| Input geometry | Closed, consistently outward-wound, non-self-intersecting manifolds are intended; this is not general mesh repair or proof of all self-intersection detection |
| Sections | Simple concave contours and separate non-nested loops; no holes, curved blades, or arbitrary solid Boolean operations |
| Degeneracy and scale | Tangency may be unchanged; full-face coplanarity or ambiguous connectivity during a split may be rejected. Float storage and finite tolerances do not guarantee arbitrary scales, narrow features, or unlimited cuts; projection/merging may lose sub-tolerance details |
| Attributes | A required repair may be refused if it would flatten UV seams beyond tolerance, non-affine UVs, or nonuniform colors; ordinary multi-material meshes and UV seams remain supported |
| Piece identity | IDs preserve identified pieces. Initial connectivity is inferred geometrically, without guaranteeing distinct identities for coincident input; no collision or Boolean union of overlapping pieces |
| Capacity and history | At most 65,535 expanded vertices in this application. Complete CPU snapshots grow with history, without a budget or persistence; the vertex cap is not a general D3D11 limit |

Clipping visits triangles; maps track shared edges and segments, while some position matching still uses linear search. Ear-candidate testing for an n-point contour can take $O(n^3)$ in the worst case. Boundary restoration, quality improvement, and bounded retries add work, so the full pipeline is not claimed to be linear. Temporary memory grows with input/output geometry, topology maps, and contours. Replacement holds old and new resources simultaneously; history additionally owns CPU geometry and IDs while sharing texture references.

Validation covers multi-material meshes, concave sections, existing edges/vertices, targeted piece cuts, overlapping piece identities, and repeated cuts, checking edge incidence, winding, and volume error. Passing these cases does not guarantee success on every input.

Suggested demonstration: cube cut → cross-material cap → concave L-prism section → cut only the middle piece → undo/redo and branching → F1/F3 inspection → OBJ reload.
