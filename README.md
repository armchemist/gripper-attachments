# OMX-F Gripper Attachments

3D-printed attachments for the OpenMANIPULATOR-X follower arm (OMX-F) gripper.

## Files

```
gripper-attachments/
├── GRIPPER_DIMENSIONS.md       measured finger geometry — start here before modeling
├── analyze_gripper.py          STL → cross-section extractor (re-run to re-measure)
├── gripper_sleeve.scad         generic parameterized slide-on sleeve
├── omx_f_gripper_sleeve.scad   sleeve fitted to OMX-F blade cross-sections
├── omx_f_tube_insert.scad      tip insert for gripping test tubes
├── omx_f_tube_insert2~5.scad   iterated versions of the tube insert
└── model/
    └── omx_f_mesh/
        └── follower_07_gripper_motorized.stl   original link6 finger mesh
```

## Designing a new attachment

Open `GRIPPER_DIMENSIONS.md` first — it contains the blade cross-section table, pivot positions, and inner/outer face conventions measured from the original STLs. Use those numbers directly instead of re-analyzing the mesh.

Key dimensions to design around:

| Zone | X range | Notes |
|---|---|---|
| Housing block | 0 – 22 mm | DO NOT cover (motor/gear body) |
| Grip blade | 22 – 65 mm | attach here |
| Tip | 65 mm | blade end |

Wall thickness `1.8 mm` + clearance `0.3 mm` is a good starting point for FDM on the blade taper.

## Checking fit with the original gripper in OpenSCAD

All `.scad` files have a `show_phantom` parameter. Set it to `true` to render a translucent hull of the finger blade alongside the attachment — useful for a quick cross-section check.

For an exact fit check against the real mesh, add an `import()` with the `%` (background) modifier at the bottom of any `.scad` file:

```scad
// Paste at the end of the file — remove before exporting STL
%import("model/omx_f_mesh/follower_07_gripper_motorized.stl");
```

The `%` modifier renders the STL as a translucent ghost in preview (F5) but excludes it from the final render and export (F6/F7), so it never ends up in your print file.

## Export workflow

1. Open the target `.scad` in OpenSCAD
2. Set `show_phantom = true` (or add `%import(...)`) and press **F5** to preview fit
3. Set `show_phantom = false` (and remove any `%import` lines) before exporting
4. Press **F6** to render, then **F7** to export STL
5. Print orientation: lay flat (X axis along print bed length)
6. Recommended material: PETG or TPU 95A; 3–4 walls, 20% infill
