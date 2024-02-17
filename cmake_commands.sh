/usr/bin/cmake /home/sundharvs/ros2_ws/src/slam_path -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja -DCMAKE_INSTALL_PREFIX=/home/sundharvs/ros2_ws/install/slam_path
/usr/bin/cmake --build /home/sundharvs/ros2_ws/build/slam_path -- -j8 -l8
/usr/bin/cmake --install /home/sundharvs/ros2_ws/build/slam_path
