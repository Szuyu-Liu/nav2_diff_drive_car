# nav2_diff_drive_car
An implementation of autonomous navigation with ROS2 Jazzy, Nav2, and slam toolbox on differential drive robots

- workflow
- IMU
- Lidar launch, param
- teensy subscriber launch (sensor fusion)

The logic of the odometry is in src/python_parameters/python_parameters/teensy_subscriber.py, in which we fused the feedback from IMU and encoders on servo motors.

IMU drifting is compensated for by ignoring the changes smaller than 0.1 degrees.
An entire transformation tree must be completed: map->odom->base_link->laser_frame
Map is published by the SLAM Toolbox or AMCL algorithm in Nav2
Odom is computed and published by the Python code teensy_subscriber.py
Base_link and laser_frame are published by YDlidar

The best performance is conducted with the Dijkstra plan-planning algorithm and MPPI controller algorithm.
