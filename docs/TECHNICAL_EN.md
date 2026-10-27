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
3. Resolve indices into expanded triangles for each subset.
4. Clip the original surface and collect section segments.
5. Merge intersection nodes, recover closed contours, and project them to 2D.
6. Reject unsupported nested contours and triangulate caps with ear clipping.
7. Generate cap attributes and offset both halves.
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

This is not an exact point-to-stroke-segment distance test. Points with $w\leq10^{-5}$ are rejected. If a section endpoint fails acceptance, the current subset returns `Unchanged`; other subsets may still produce independent cuts.

`Slice()` receives this acceptance rule as a callback. An overload that accepts every point is available when no screen restriction is required.

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

## 5. Triangle input and signed distance

`CutMeshes()` resolves vertices through `IndexArray`, producing three expanded vertices per triangle. It does not assume that an index-buffer position is itself a vertex index.

For a unit plane normal:

$$
d=(P-Q)\cdot N
$$

Positive and negative values identify the two half-spaces. The default positional tolerance is $\epsilon=10^{-4}$. Distances satisfying $|d|\leq\epsilon$ are set to zero for classification.

This changes the classification value, not the vertex position: near-plane vertices are not explicitly projected onto the plane. Tolerances are measured in local-space units and depend on asset scale. If the input has no vertices on both strict sides, or no section nodes, the operation returns `Unchanged`.

## 6. Half-space clipping and attributes

### 6.1 Two Clip calls

One `Clip()` invocation retains one half-space:

$$
d\times side\geq0
$$

`side=+1` retains the positive side; `side=-1` retains the negative side. On-plane vertices belong to both boundaries. Edges A→B are visited in the original triangle order:

| A is inside the retained half-space | B is inside | Vertices appended for this edge |
|---|---|---|
| Yes | Yes | A |
| Yes | No | A, intersection |
| No | Yes | Intersection |
| No | No | None |

Each edge handles its starting vertex. The resulting convex polygon is triangulated as a fan $(P_0,P_i,P_{i+1})$.

In a typical crossing, the side with one original vertex becomes a triangle, and the opposite side becomes a quadrilateral split into two triangles. Two calls produce both halves. A triangle not crossed by the plane is copied into its retained side.

Fan triangulation is valid here because clipping a triangle against a half-space produces a convex region. Section caps can be concave and use ear clipping instead.

### 6.2 Intersection parameter

For endpoint distances $d_A,d_B$:

$$
t=\frac{d_A}{d_A-d_B}
$$

This follows from $d(t)=d_A+t(d_B-d_A)=0$. The intersection is:

$$
P=A+t(B-A)
$$

The same parameter interpolates position, UVs, RGBA vertex color, and normals. Interpolated normals are normalized. There is no second length-ratio calculation. Additional vertex fields that are not explicitly handled inherit endpoint A's values; this does not imply support for tangent or skinning interpolation.

### 6.3 Triangle registration

`Triangle()` preserves the supplied winding and registers only faces satisfying:

$$
\left\|(B-A)\times(C-A)\right\|^2>\epsilon^4
$$

The cross-product magnitude is twice the area, so the threshold corresponds to area greater than $\epsilon^2/2$. Degenerate and very small faces are filtered, but this is not general mesh repair or a guarantee of watertightness. On-plane cases can produce duplicate polygon positions; their zero-area fan triangles are filtered here.

## 7. Section segments and deduplication

Surface clipping and cap-boundary collection have separate outputs. `Clip()` writes surface triangles, while a triangle-local `hits` array collects section points. Some intersection and interpolation work is currently repeated between these stages.

Original on-plane vertices are registered directly. Strictly opposite endpoint signs produce an interpolated edge intersection. Two distinct hits contribute one section segment.

| Operation | Scope | Purpose |
|---|---|---|
| `AddUniqueIntersection()` | One triangle | Merge hits within positional tolerance |
| `FindOrAddCutNode()` | The current subset | Assign the same node index to shared section points |
| Connection deduplication | A node pair | Avoid registering the same undirected segment twice |

Equal positions with different node indices would leave segments disconnected. Node merging is therefore part of contour-topology reconstruction. It currently uses linear search, keeps the first node's attributes, and does not average positions.

Zero or one hit does not form a segment; two hits do. Three distinct hits are treated as a coplanar/ambiguous case and reject the operation. Positional tolerance affects these classifications.

### 7.1 Existing-edge cuts and coplanar triangles

Two hits alone do not establish a split. When the plane passes through an existing edge, a triangle may produce two hits while its third vertex lies entirely on one side. That edge can still contribute a section segment. If the complete input contains vertices strictly on both sides and the segments form supported closed contours, a cut can follow existing mesh edges without crossing triangle interiors. Merely touching an exterior surface without vertices on both sides does not commit a cut.

A nondegenerate triangle with three distinct vertices classified on the plane produces three hits and returns `OpenOrAmbiguousContour`. This is a retained limitation: even if other regions extend to both sides, the coplanar triangle cancels the attempted model update. The implementation does not resolve which side should own such a face or whether to retain or replace it.

`Clip()` may already have generated temporary surfaces before rejection. They are not committed. Rejecting a coplanar triangle does not imply that clipping produced no output; it means that the attempted operation is not accepted.

## 8. Adjacency and closed contours

`nodes[i]` stores a section vertex; `adjacency[i]` stores connected node indices. Each segment AB is registered in both directions.

`BuildClosedContours()` requires exactly two neighbors per node. It starts at an unvisited node and follows the neighbor that is not the previous node.

- Returning to the start completes a contour.
- Reaching another visited node before closing fails.
- A degree other than two, or fewer than three contour nodes, fails.
- Remaining unvisited nodes start additional independent contours.

Each traversal registers a new node or terminates at a visited node, so it cannot traverse indefinitely with valid node indices. Degree-two connectivity does not establish that the contour is geometrically free of self-intersections.

## 9. Planar coordinates and nesting

A reference axis R is selected to avoid near-parallel alignment with the unit normal: X when $|N_x|<0.9$, otherwise Y.

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

Twice the signed polygon area is:

$$
A_2=\sum_i(x_i y_{i+1}-y_i x_{i+1})
$$

A negative value reverses the contour. Adjacency traversal can choose either initial direction, so winding is normalized before ear clipping.

### 10.2 Finding an ear

For consecutive A, B, C:

$$
Orient(A,B,C)=(B_x-A_x)(C_y-A_y)-(B_y-A_y)(C_x-A_x)
$$

A positive value is a convex corner in a counterclockwise contour. Candidates must satisfy $Orient>\epsilon^2$.

Every other contour vertex P is tested against:

$$
Orient(A,B,P)\geq-\epsilon^2
$$
$$
Orient(B,C,P)\geq-\epsilon^2
$$
$$
Orient(C,A,P)\geq-\epsilon^2
$$

If all three hold, P is inside or within the tolerated boundary of the candidate triangle, so the candidate is rejected. Otherwise ABC is saved and only B is removed from the working contour. The process repeats until the final three nodes remain and are registered.

The working contour is a copy; original nodes remain intact. Successful triangulation of an n-node simple contour produces n−2 triangles. A pass with no ear, or an invalid final triangle area, returns `TriangulationFailed`.

`TriangulateContours()` returns `TriangleIndices`: triples of node indices. It does not generate side-specific vertex attributes.

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

The default is `uvScale=0.1`. Coordinates are not normalized to fit the cap into [0,1]; material and sampler behavior determine the appearance outside that interval. Caps reuse the subset material, with no dedicated cap material generated.

Finally, surface and cap vertices are offset together:

$$
P_-=P-N\times separation,\qquad P_+=P+N\times separation
$$

The default is 1.5 local units per side, producing a three-unit gap between originally coincident cut faces. The completed result returns `Status::Cut`. Subsequent cuts operate directly on the generated, offset vertices.

## 12. Subset staging and model updates

Slicing is evaluated per subset. `PendingSubset::primaryVertices` always exists: it contains the negative half after a cut, or original vertices when unchanged. `additionalVertices` contains the positive half only after a cut. These names describe staging roles rather than universal geometric sides.

All subsets are staged locally:

- `Cut`: retain both halves.
- `Unchanged`: retain the original subset so it survives whole-model reconstruction.
- Other algorithm failures: cancel the entire model update.
- No changed subset: skip resource reconstruction.

After capacity checks, final arrays are handed to the update queues. `TryUpdateModel()` counts nonempty arrays, creates expanded vertex data, sequential indices, subset/material records, and separate replacement vertex/index buffers. The old model is released only after preparation succeeds.

Indices become 0, 1, 2, and so on. The current 16-bit representation limits replacement geometry to 65,535 expanded vertices; repeated cuts are not unbounded.

### Result status

| Status | Meaning |
|---|---|
| `Unchanged` | Do not apply the attempted cut; retain the subset |
| `Cut` | Both surfaces and caps have been generated |
| `InvalidInput` | Cutting input or settings violate the current requirements |
| `OpenOrAmbiguousContour` | Open, branching, coplanar, or otherwise ambiguous section connectivity |
| `UnsupportedHole` | Unsupported nested contours were detected |
| `TriangulationFailed` | Ear clipping could not finish, or the final triangle failed the area condition |

A failure result may contain partial temporary vertices, but they are not committed as a complete mesh.

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

`ModelSnapshot` stores independent vertex, index, and subset/material arrays. Positions, normals, colors, UVs, previously generated caps, and baked separation offsets are part of the saved model state. GPU buffers, the camera, presentation transforms, fill mode, and the plane-visibility setting are not captured.

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

`PrintDebugProc()` registers per-frame control text; `DrawDebugProc()` calls `DebugTextOut()` to render it. F1/F3 settings and history shortcuts are displayed in both Debug and Release. Text support uses `DEBUG_DISP_TEXTOUT`, a BGRA back buffer, and a GDI-compatible swap chain. The D3D debug layer remains Debug-only. Camera orbit, height, distance controls, and R reset are enabled in both Debug and Release. 

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

## 17. Environment and limitations

Build the Windows Visual Studio project with its DirectX 11, DirectXMath, input, and D3DX11 dependencies available. Use the current C++17 configuration and ensure shader/asset relative paths match the working directory. Triangulated OBJ assets with normals, UVs, explicit material groups, and valid MTL/texture references are recommended.

- Every subset being capped must provide a closed section graph. Material groups that split a closed object into open surface patches can cause rejection.
- Simple concave contours are supported; holes, self-intersection handling, and general mesh repair are not.
- A coplanar triangle with three distinct on-plane vertices rejects the entire update, even if other regions lie on both sides. Very small geometry and tolerance-induced merging may also cause rejection. Rejection is not mesh repair.
- History stores complete CPU geometry, with no entry-count/memory-budget limit or persistence.
- Wireframe displays triangle edges, not only the object's silhouette.
- Curved blades, arbitrary solid Boolean operations, collision generation, and rigid-body simulation are outside the scope.
- Node merging uses linear search. Repeated ear-candidate searches with scans of other vertices can take cubic time in the worst case. No large-mesh performance optimization claim is made.
- The 16-bit capacity, asset scale, and numerical tolerances limit the depth and stability of repeated cuts.

Suggested demonstration: straight and diagonal cube cuts → repeated undo/redo → a new cut after undo to demonstrate branching → an L-prism concave section → F1 triangulation inspection → F3 plane inspection → reload the asset to restore geometry and clear history.
