# Mesh Cut

**Interactive plane-based mesh slicing with C++ and DirectX 11.**

Drag across a 3D model to split its triangle mesh, reconstruct the cut contours, and generate caps for both sides. Load another OBJ directly into the window, or inspect the result with solid, wireframe, and cutting-plane views.

## Highlights

- **Interactive slicing:** a mouse stroke defines a plane through the perspective camera.
- **Mesh reconstruction:** triangle clipping generates both halves with interpolated positions, normals, vertex colors, and UVs.
- **Contour-based capping:** intersection segments are connected into closed contours and triangulated with ear clipping, including simple concave contours.
- **Repeated cuts:** the generated surface and caps become the input for subsequent cuts.
- **OBJ drag and drop:** replace the current model and reset its transform, camera, and cutting interaction after a successful load.
- **Inspection controls:** switch between solid and wireframe rendering with **F1**, and toggle the cutting-plane visualization with **F3**.
- **Staged model updates:** prepare replacement CPU arrays and GPU buffers before releasing the displayed model.

## Technical documentation

The following documents describe the same implementation in three languages:

| Language | Document |
|---|---|
| 中文 | [技术说明](docs/TECHNICAL_CN.md) |
| 日本語 | [技術解説](docs/TECHNICAL_JP.md) |
| English | [Technical documentation](docs/TECHNICAL_EN.md) |

They cover ray construction, coordinate transforms, half-space clipping, intersection topology, contour reconstruction, ear clipping, cap attributes, and CPU/GPU resource ownership.

## Controls

| Input | Action |
|---|---|
| Left mouse drag, then release | Attempt a cut defined by the stroke |
| Drop one `.obj` into the window | Load and replace the current model |
| Drop the same OBJ again | Reload its original geometry and reset the presentation |
| **F1** | Toggle solid / wireframe rendering |
| **F3** | Show / hide the cutting-plane visualization |
| Arrow keys | Move the displayed model relative to the camera heading |
| Esc | Exit |

Additional controls available in the current **Debug build**:

| Input | Action |
|---|---|
| Z / C | Orbit the camera horizontally |
| Y / N | Raise / lower the camera |
| U / M | Move the camera closer / farther |
| R | Reset model transform and camera; does not restore already-cut geometry |

F3 controls visibility of the most recently generated cutting-plane preview. A preview is available after a valid drag; it is a visualization of the attempted plane, not a guarantee that the mesh cut was accepted. Reloading a model clears the old preview while retaining the visibility setting. Debug text displays the F1/F3 controls and their selected modes.

## Getting started

1. Open the existing Visual Studio solution/project on Windows.
2. Use the project's DirectX 11 build configuration and required library paths. The rendering path uses D3DX11 APIs, so its matching headers, libraries, and runtime dependencies must be available.
3. Keep the shader and asset directories accessible from the configured working directory, then build and run.
4. Drop a prepared OBJ into the window. Keep its referenced MTL and textures at the paths specified by the asset files.
5. Drag across the model, release to cut, and press F1 to inspect the resulting triangles.

Use triangulated, closed geometry with normals, UVs, and valid material references for predictable testing. The loader is a targeted OBJ/MTL import path rather than a general asset converter. ASCII asset paths are the simplest option; current drag-and-drop paths are converted to the Windows ANSI code page used by the loader.

## Implementation map

| Area | Responsibility |
|---|---|
| `MeshCutter.h / .cpp / .inl` | Geometry types, coordinate helpers, clipping, contours, and cap construction |
| `ObjImporter` | OBJ/MTL parsing and CPU model data |
| `player.cpp` | Stroke interaction, model replacement, subset staging, and model update integration |
| `renderer.cpp` | DirectX resources and solid/wireframe rasterizer-state selection |
| `camera.cpp` | Perspective camera and reset behavior |

## Current scope

- Cutting uses a **plane**, not a curved blade or arbitrary Boolean volume.
- Capping supports **simple contours without holes**, including concave contours and multiple separate, non-nested loops.
- Slicing is evaluated **per subset**. A closed object whose surface spans material subsets can produce open per-subset contours and be rejected.
- Fully coplanar triangles, invalid contour connectivity, unsupported nested contours, or failed triangulation can cause the operation to be rejected.
- The model format retains **16-bit indices and counts**; replacement geometry is limited to 65,535 expanded vertices.
- Cuts rebuild mesh geometry; they do not create collision shapes or simulated rigid bodies.

See the technical documents for the precise acceptance rules, numerical assumptions, and resource-update behavior.
