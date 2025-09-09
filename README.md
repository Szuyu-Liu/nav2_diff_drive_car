# nav2_diff_drive_car  

An implementation of **autonomous navigation** for differential drive robots using **ROS 2 Jazzy**, **Nav2**, and the **SLAM Toolbox**.  

This is the [link](https://drive.google.com/file/d/1UGohlF4hFTtLeBsVapYi-5T-qgDYO2yT/view?usp=sharing) to the video of the performance of the autonomous car in a dynamic environment:

https://drive.google.com/file/d/1UGohlF4hFTtLeBsVapYi-5T-qgDYO2yT/view?usp=sharing

Hardware architecture of the system
<img width="1073" height="658" alt="image" src="https://github.com/user-attachments/assets/aa925437-fc4d-4c58-aeb7-8a68cda480c0" />

Software dataflow architecture of the system
<img width="1717" height="825" alt="image" src="https://github.com/user-attachments/assets/2d56be06-0d57-41cd-99bd-9a12b879d63a" />

## Features  
- End-to-end workflow for differential-drive robot navigation  
- Integration of **IMU** and **servo motor encoders** for odometry  
- **Lidar-based localization and mapping** with configurable launch and parameter files  
- **Sensor fusion** via a Teensy subscriber node  

## Odometry Logic  
The odometry logic is implemented in:  
src/python_parameters/python_parameters/teensy_subscriber.py

This node fuses IMU data with encoder feedback from the servo motors to compute odometry.  

- **IMU drift compensation**: Changes smaller than **0.1°** are ignored to reduce noise.  
- **TF tree**: A complete transformation chain is required:  
map → odom → base_link → laser_frame
- `map`: published by **SLAM Toolbox** or **AMCL** in Nav2  
- `odom`: computed and published by `teensy_subscriber.py`  
- `base_link` and `laser_frame`: published by **YDLidar**  

## Navigation  
- Best results are achieved with:  
- **Dijkstra global planner**  
- **MPPI local controller**  

