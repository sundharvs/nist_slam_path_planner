Dependencies:

ROS2 Humble

FCL (https://github.com/flexible-collision-library/fcl)

OMPL (https://ompl.kavrakilab.org/core/installation.html)

Variables to modify based on the environment:

- Quadcopter collision geometry (Box dimensions) [path_planner.cpp line 72]
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

Usage:
```
ros2 bag play december_test_0.db3
source install/setup.sh
ros2 launch direct_lidar_inertial_odometry dlio.launch.py rviz:=true pointcloud_topic:=/ouster/points imu_topic:=/imu/data
ros2 run nist_slam_path_planner planner  
```
Add the "plan_path" topic in Rviz
Use the goal pose selection tool in Rviz
