# Repository layout and migration

This is a separate repository prepared from `Matthew-Garcia/SCARA-Robot` and the added Humble workspace. The source commit and file hashes are recorded in the manifest. Reorganization moves original files without editing their contents.

| Previous location | New location |
| --- | --- |
| `3D_Models/*.SLDPRT`, `3D_Models/*.SLDASM` | `cad/solidworks/` |
| `3D_Models/*.STEP`, `3D_Models/*.step` | `cad/step/` |
| `Code/SCARA_Robot.ino` | `firmware/arduino/SCARA_Robot/SCARA_Robot.ino` |
| `Code/GUI_for_SCARA_Robot.pde` | `software/processing/GUI_for_SCARA_Robot/GUI_for_SCARA_Robot.pde` |
| `Images/` | `media/original/` |
| Original `README.md` | `docs/archive/original-readme.md` |
| `ROS2_WS/` | `ros2_ws/` |
| Workspace export/render/integration scripts | `tools/` |
| Workspace hardware/validation notes | `docs/` |
| Workspace Dockerfile | `docker/Dockerfile` |

Original SolidWorks part filenames remain intact so the assembly can resolve its neighboring native parts. SolidWorks may request file-location repair if it has cached absolute paths from another machine. The complete STEP assembly is the portable source used for ROS mesh export.

Generated STL meshes are checked in under `ros2_ws/src/scara_description/meshes/`; ordinary ROS builds do not require SolidWorks or CadQuery. Do not rename ROS package directories or move installed resources without updating their references.

`source_manifest.json` gives the complete original-to-new mapping. The provenance test validates all 77 original files against the recorded hashes.

The processing and Arduino sketch folders match their main filenames. Open the `.pde` or `.ino` in its respective IDE; this reorganization does not change their serial behavior or perform hardware calibration.
