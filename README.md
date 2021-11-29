# ros2_qbot_1

## Summary
A ROS 2 package for a differential drive, wheeled robot.

## Nodes
More details to follow, but will include:
```
Camera node
Base controller
Local planner
Odometry
URDF
Meshes
Rviz config
3D information extraction from monocular images
```

## Build Requirements

On the Coral Board, you will need to run the following commands to allow colcon to find OpenCV:
```
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

