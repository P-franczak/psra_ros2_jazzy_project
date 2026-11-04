# Rally track for Gazebo Sim

## Contents
- `generate_track.py`: generates the road and barrier STL meshes.
- `models/rally_track/model.sdf`: Gazebo model.
- `models/rally_track/model.config`: model metadata.
- `models/rally_track/meshes/`: generated STL files (after running the generator).
- `world_include_snippet.sdf`: snippet to include in a world.

## 1. Generate the meshes

From this package directory:

```bash
python3 generate_track.py --output models/rally_track/meshes
```

This creates:
- `road.stl`
- `barriers.stl`

## 2. Make Gazebo find the model

The directory that contains `rally_track` must be in `GZ_SIM_RESOURCE_PATH`.

For example, if your project has:
`/home/developer/ros2_ws/src/psra_ros2_jazzy_project/models/rally_track`

then add its parent directory:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/developer/ros2_ws/src/psra_ros2_jazzy_project/models
```

The model lookup uses `model://rally_track`.

## 3. Include it in your world

Copy the contents of `world_include_snippet.sdf` inside the `<world>` element in your world file.

## Notes
- The track starts at `(0, 0)` and initially points along +X.
- Road top is at `z = 0.05 m`; road thickness is `0.10 m`.
- Barriers are `0.50 m` high and `0.15 m` thick.
- This is a first geometric prototype. Inspect it in Gazebo before driving on it.
- If your world has a ground plane at z=0, the road overlaps it slightly by design.
- The return section is a cubic Bezier curve, with tangent continuity at both ends.
