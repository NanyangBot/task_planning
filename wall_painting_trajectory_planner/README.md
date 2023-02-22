# How to run

1. Build `task_msgs` and `wall_painting_trajectory_planner` packages and source workspace

2. Run node:
```
ros2 run wall_painting_trajectory_planner start
```

3. Open `rviz2` for visualization

4. Send distance map with desired drawing (in grid map format)
### Flat wall
```
ros2 topic pub --once /task task_msgs/msg/DistanceMapSlice "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, resolution: 1.0, width: 3, height: 3, origin: {x: 0.0, y: 0.0, z: 0.0}, unknown_value: 0.0, data: [2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0], task: [false,false,true,false,true,false,true,false,false]}"
```

### Wall with horizontal slope
```
ros2 topic pub --once /task task_msgs/msg/DistanceMapSlice "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, resolution: 1.0, width: 3, height: 3, origin: {x: 0.0, y: 0.0, z: 0.0}, unknown_value: 0.0, data: [1,2,3,1,2,3,1,2,3], task: [false,false,true,false,true,false,true,false,false]}"
```

### Curved wall
```
ros2 topic pub --once /task task_msgs/msg/DistanceMapSlice "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, resolution: 0.5, width: 8, height: 8, origin: {x: 0.0, y: 0.0, z: 0.0}, unknown_value: 0.0, data: [2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2], task: [True, False, False, False, False, False, False, False, False, True, False, False, False, False, False, False, False, False, True, False, False, False, False, False, False, False, False, True, False, False, False, False, False, False, False, False, True, False, False, False, False, False, False, False, False, True, False, False, False, False, False, False, False, False, True, False, False, False, False, False, False, False, False, True]}"
```
