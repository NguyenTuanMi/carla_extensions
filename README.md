# carla_extensions
An extension of carla ros2 published topics

## Extension publisher

One node publishes the hero vehicle's map pose and republishes configured camera
views with their alpha channels set to 255. This is useful when CARLA publishes
`bgra8` images with zero alpha, which some viewers render as transparent.

```bash
ros2 run carla_extensions extension_publisher
```

The node publishes pose messages on `/carla/hero/odom`. By default, it subscribes
to the `image` topic for `Center`, `Rearview`, `Right`, and `Left`, publishing the
corrected images as `/carla/hero/<View>/image_opaque`. In Foxglove, select the
corrected image and use `/carla/hero/<View>/camera_info` for calibration.

The `role_name`, `frequency`, and `camera_views` ROS parameters configure the
vehicle namespace, pose publishing rate, and camera views.

## Build and source

After changing this package, rebuild and source the workspace overlay before
running its executables:

```bash
cd ~/carla_ws
colcon build --packages-select carla_extensions
source install/setup.bash
```

The CARLA 1.0 Python API still supports `carla.Client.get_world()`.
