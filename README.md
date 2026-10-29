# Mesh Cut

**Interactive mesh slicing with C++ and DirectX 11.**

**English** · [日本語](README_JP.md) · [中文](README_CN.md)

Drag across a 3D model to split its triangle mesh. Closed pieces receive reconstructed section caps; open surfaces are clipped without caps. Undo and redo cuts, drop another OBJ into the window, and inspect the result in solid or wireframe mode.

## Features

- **Interactive plane slicing:** a mouse stroke defines a plane through the perspective camera.
- **Surface reconstruction:** half-space clipping generates both halves and interpolates positions, normals, vertex colors, and UVs.
- **Contour-based caps:** shared intersection nodes and adjacency recover closed contours; ear clipping supports simple concave contours and separate non-nested loops.
- **Open surfaces and mixed cuts:** each connected piece is classified automatically. Open surfaces are clipped without caps; a single operation can also cut and cap closed pieces.
- **Materials and pieces:** contours span material groups within each piece. Persistent IDs support targeted piece cuts while uncut pieces stay in place; generated surfaces and caps can be cut again.
- **Undo / redo:** two snapshot stacks restore geometry and materials; a successful new cut after undo clears the redo branch.
- **OBJ drag and drop:** successful loading replaces the model, resets its presentation and camera, and clears editing history. Failed loading retains the current model and history.
- **Inspection:** F1 toggles solid/wireframe; F3 toggles the cutting-plane preview. On-screen control hints appear in both Debug and Release.
- **Model statistics:** displays successful cuts represented by the current state, independent shapes, and expanded vertex count. Statistics follow undo/redo; failed cuts do not increase the cut count.
- **Staged replacement:** replacement CPU arrays and GPU buffers are prepared before the displayed model is released.

## Controls

| Input | Action |
|---|---|
| Left mouse drag, then release | Attempt a cut |
| Drop one `.obj` into the window | Replace/reset the model and clear history on success |
| Drop the same OBJ again | Reload its original geometry |
| Ctrl + Z | Undo |
| Ctrl + Y / Ctrl + Shift + Z | Redo |
| F1 | Toggle solid / wireframe |
| F3 | Show / hide the cutting-plane preview |
| Arrow keys | Move the model relative to camera heading |
| Esc | Exit |

Camera and presentation controls (Debug and Release):

| Input | Action |
|---|---|
| A / D | Orbit the camera horizontally |
| W / N | Raise / lower the camera |
| U / M | Move the camera closer / farther |
| R | Reset model transform and camera, retaining cut geometry |

Either Ctrl/Shift key is accepted. History shortcuts trigger once per Z/Y press and are ignored while dragging a cutting stroke. Undo/redo restore mesh state, not the camera or presentation transform. A successful restore clears the old plane preview while retaining its visibility setting.

F3 shows the most recently attempted plane; a visible preview does not guarantee that a cut was accepted. R resets presentation only; reload the OBJ to restore its original geometry.

## Getting started

1. Open the Windows Visual Studio solution/project and use C++17.
2. Configure the project's DirectX 11, DirectXMath, input, and D3DX11 dependencies. D3DX11 headers, libraries, and matching runtime dependencies must be available.
3. Ensure shader and asset paths resolve from the configured working directory, then build and run.
4. Drop a triangulated OBJ with normals, UVs, and valid MTL/texture references into the window.
5. Cut the model, inspect its triangles with F1, and use Ctrl+Z / Ctrl+Y to compare editing states.

Keep MTL and texture files at the paths referenced by the assets. The importer expects its supported OBJ/MTL format; ASCII paths and referenced filenames without spaces are the simplest choice. Dropped paths are converted to the loader's Windows ANSI code page, rather than imported through a full Unicode path pipeline.

## Technical documentation

The detailed documents describe the same implementation in three languages:

| Language | Project overview | Technical documentation |
|---|---|---|
| 中文 | [README](README_CN.md) | [技术说明](docs/TECHNICAL_CN.md) |
| 日本語 | [README](README_JP.md) | [技術解説](docs/TECHNICAL_JP.md) |
| English | [README](README.md) | [Technical documentation](docs/TECHNICAL_EN.md) |

Topics include screen rays, plane transforms, clipping, attribute interpolation, section topology, contour reconstruction, ear clipping, cap normals/UVs, CPU/GPU ownership, snapshot history, and failure handling.

## Implementation map

| Module | Responsibility |
|---|---|
| `MeshCutter.h / .cpp / .inl` | Geometry, coordinate helpers, clipping, contours, triangulation, and caps |
| `ObjImporter` | OBJ/MTL parsing and CPU model data |
| `ModelSnapshot.h / .cpp` | Capture/restore model state and retain texture references |
| `ModelHistory.h / .cpp` | Two stacks, cut-history commits, undo/redo, and clearing |
| `player.cpp` | Stroke input, subset staging, model updates, and history integration |
| `renderer.cpp` | Rendering resources, fill/cull states, and text rendering |
| `camera.cpp` | Perspective camera and reset behavior |

## Scope and limitations

- Supports closed pieces and open surfaces. Closed pieces require consistent winding; open surfaces retain input face orientation. Nonmanifold edges remain unsupported; this is not general mesh repair.
- Closed-piece caps support simple concave sections and separate non-nested loops, without holes. Open surfaces are not thickened or capped, and missing faces are not repaired.
- Tangency may leave geometry unchanged. Full-face coplanarity during a split, ambiguous connectivity, or unsafe triangulation retains the current valid model.
- Vertices use float storage. Finite tolerances and at most two local refinement steps do not guarantee arbitrary scales, narrow features, or unlimited cuts. Required planar repair may also be refused when UV/color features would be altered beyond tolerance.
- The application accepts at most 65,535 expanded vertices. History stores full CPU geometry and piece IDs without a count/memory budget or persistence; texture references are shared.
- IDs keep identified pieces independent; overlapping pieces are not unioned, and collision or rigid-body simulation is not provided.

Suggested demo: cube cut → cross-material cap → concave L-prism → cut only the middle piece → undo/redo → F1/F3 inspection → OBJ reload.
