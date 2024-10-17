# Real-Time Path Planning In Large LIDAR Mapped Environments

![demo_animation](https://github.com/user-attachments/assets/7a2d8236-9694-470f-bec2-4e8abacafd9a)

This ROS2 node subscribes to a stream of OctoMaps and vehicle poses, and generates a smooth, obstacle-free trajectory to a goal pose in real-time. The node handles dynamic environments by continuously checking for collisions and re-generating a path with real-time speed whenever updated map data or vehicle positions are received.

## Dependencies

- [ROS2](https://docs.ros.org/en/humble/index.html)
- [Flexible Collision Library (FCL)](https://github.com/flexible-collision-library/fcl)
- [Octomap](https://github.com/OctoMap/octomap)
- [OMPL (Open Motion Planning Library)](https://ompl.kavrakilab.org/)

## How it works

* The map of obstacles is represented using Octomap, which is a highly efficient probabilistic mapping framework
* [Informed RRT*](https://arxiv.org/abs/1404.2334) find near-optimal paths more quickly by reducing the search area, sampling an elliptical sub-region of the map
* FCL is used to perform collision checking between the Octomap and a bounding box representing the vehicle
* The vehicle travels through the environment as its being mapped by checking for collisions and replanning if necessary every time the Octomap is updated

## Usage

Variables to modify:

- Quadcopter collision geometry (Bounding box dimensions) [path_planner.cpp line 72]
- Planner parameters listed below

Current values:
```
delay_collision_checking = 1
goal_bias = 0.05
number_sampling_attempts = 100
ordered_sampling = 0
ordering_batch_size = 1
prune_threshold = 0.05
range = 6.09689
rewire_factor = 1.1
use_k_nearest = 1
```

This tool can be tested with the example ros2 bag `december_test_0.db3` which is a recording of a quadrotor flight carrying the Ouster OS1 LIDAR and running [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) 

```bash
ros2 bag play december_test_0.db3
source install/setup.sh
ros2 run nist_slam_path_planner planner  
```

You can use it with live data by running the following command before running this node
```bash
ros2 launch direct_lidar_inertial_odometry dlio.launch.py rviz:=true pointcloud_topic:=/ouster/points imu_topic:=/imu/data
```

For visualization,
* Add the "plan_path" topic in Rviz
* Use the goal pose selection tool in Rviz
