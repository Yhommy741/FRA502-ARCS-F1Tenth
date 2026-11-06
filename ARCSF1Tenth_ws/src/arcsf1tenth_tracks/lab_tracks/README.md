# Lab tracks

Our own tracks (real lab track and its Gazebo replica). One folder per track:

```
<track_name>/
├── track.yaml          # name, description, start_pose {x, y, yaw}
├── world/world.sdf     # Gazebo world (CAD export or map_to_world.py)
└── maps/
    ├── sim/            # SLAM map built in Gazebo (+ raceline.csv for this map)
    └── real/           # SLAM map built on the real car (+ raceline.csv for this map)
```

Launch with `ros2 launch arcsf1tenth_bringup simulation.launch.py track:=<track_name>`.
