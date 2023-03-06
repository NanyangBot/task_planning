# How to run

1. Build `task_msgs` and `wall_painting_trajectory_planner` packages and source workspace

2. Run node:
```
ros2 run wall_painting_trajectory_planner start
```

3. Open `rviz2` for visualization

4. Trigger planner with a task request, which includes the path to the image file containing the line to draw. For example, you can use the `freehand.png` image saved in the `images` folder within this package by going to the image location and calling the `Trigger` service using the following command:
```
ros2 service call /wall_painting_trajectory_planner/trigger task_msgs/srv/Trigger "{image_path: '$(pwd)/freehand.png'}"
```

**Note:** Make sure that the required `DistanceMap` service is enabled by the vision module.
