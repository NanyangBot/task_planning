# How to run RELEASE mode

1. Build `task_msgs` and `wall_painting_trajectory_planner` packages and source workspace

2. Run node:
```
ros2 run wall_painting_trajectory_planner start
```

3. Trigger planner with a task request, which includes the path to the image file containing the line to draw. For example, you can use the `freehand.png` image saved in the `images` folder within this package by going to the image location and calling the `TaskPlanning` service using the following command:
```
ros2 service call /wall_painting_trajectory_planner/trigger task_msgs/srv/TaskPlanning "{image_path: '${YOUR_WORKSPACE}/src/task_planning/images/freehand.png'}"
```

**Note:** Make sure that the required `DistanceMapSrv` service is enabled by the vision module.

# How to run DEBUG mode

1. Build `task_msgs` and `wall_painting_trajectory_planner` packages and source workspace

2. Run node:
```
ros2 run wall_painting_trajectory_planner debug
```

3. Open `rviz2` for visualization. Look for the topics `/wall` and `/path` and set the fixed frame to `map`.

4. Publish a message that includes the path to the image file containing the line to draw. For example, you can use the `freehand.png` image saved in the `images` folder using the following command:
```
ros2 topic pub --once /image std_msgs/msg/String "{data: '${YOUR_WORKSPACE}/src/task_planning/images/freehand.png'}"
```

5. Publish a message that includes the 2D distance map. For example, you can use the following command:
```
ros2 topic pub --once /dmap task_msgs/msg/DistanceMapMsg "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, resolution: 0.5, width: 8, height: 8, origin: {x: 0.0, y: 2.0, z: 4.0}, canvas_width: 8, canvas_height: 8, canvas_origin: {x: 0.0, y: 0.0, z: 0.0}, unknown_value: 0.0, data: [2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2, 2.2, 1.6, 1.2, 1.0, 1.0, 1.2, 1.6, 2.2]}"
```

