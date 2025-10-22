# nav2_diff_drive_car  

An implementation of **autonomous navigation** for differential drive robots using **ROS 2 Jazzy**, **Nav2**, and the **SLAM Toolbox**.  

This is the [link](https://drive.google.com/file/d/1UGohlF4hFTtLeBsVapYi-5T-qgDYO2yT/view?usp=sharing) to the video of the performance of the autonomous car in a dynamic environment:

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

### 🔹1. Odometry - IMU and Encoder Fusion

- Encoders compute the robot’s pose from wheel diameter and wheel bases (distance between two wheels)
- IMU provides orientation, but can drift due to noise and vibration.

To improve the accuracy of heading estimation:
- Filtering out small IMU drifts (< 0.1°)
- Fusing IMU angles with encoder feedback from the servo motors to compute odometry.
 
The odometry logic is implemented in:  
src/python_parameters/python_parameters/teensy_subscriber.py

```
        if abs(heading_change) >= 0.1:
            # ignore the imu noise around 8.0 and current_heading around 248
            if current_heading >= 360:
                current_heading = self.last_heading
            else:
                self.theta_imu += (current_heading - self.last_heading)
                if self.use_imu:
                    self.orientation_change = heading_change
                
        self.last_heading = current_heading
```

### 🔹2. Odometry Publisher
Odometry is published on /odom using:

- Encoder displacement for x, y
- Fused theta for orientation

TF transformation from odom → base_link
```
odom.header.stamp = self.get_clock().now().to_msg()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"
TF is also broadcasted at the same time.
```

### 🔹3. TF Transformations
*Transform Flow:*
```
map → odom → base_link → laser_frame
```
- base_link → laser_frame: Static transform set via launch file.
```
tf2_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_pub_laser',
    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
)
```
- odom → base_link: Published dynamically with odometry
```
       tf_msg = TransformStamped()
        tf_msg.header.stamp = odom.header.stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tf_msg)
```
- map → odom: Published by SLAM Toolbox to correct drift

Make sure:
- TFs are continuously broadcast (even when stopped)
- Timestamps are in sync

### 🔹4. LiDAR Parameters (YDLidar)
Configured in ydlidar_ros2_driver_node:
`inverted = true`: Countercloclwise; `inverted = false`: Clockwise
`reversion = true`: angle starts from 180; `reversion = false`: angle starts from 0 when facing forward
```
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    ignore_array: ""
    baudrate: 128000
    lidar_type: 1
    device_type: 0
    sample_rate: 5
    abnormal_check_count: 4
    fixed_resolution: true
    reversion: true
    inverted: true 
    auto_reconnect: true
    isSingleChannel: false
    intensity: false
    support_motor_dtr: true
    angle_max: 180.0
    angle_min: -180.0
    range_max: 12.0
    range_min: 0.1
    frequency: 10.0
    invalid_range_is_inf: false
```
*⚠️Note: Mismatched scanning direction causes rotated maps and TF errors.*

### 🔹5. SLAM Toolbox Launch
- Install slam toolbox
```
sudo apt install ros-jazzy-slam-toolbox
```
online: Working on the life data streams rather than recorded logs
async: Always process the most scan to avoid lagging, even if that means skipping.
```
ros2 launch slam_toolbox online_async_launch.py
```
Ensure:
- LiDAR frame is correctly transformed into base_link
- Odometry is publishing properly
TF tree is complete and coherent (check via `ros2 run tf2_tools view_frames`)
- Save the map
```
ros2 run nav2_map_server map_saver_cli -f ~/map

```

### 🔹6. Setup UI for Ubuntu 24.04 on Raspberry pi
- Step 1: install a lightweight desktop
```
sudo apt update
sudo apt install xfce4 xfce4-goodies
```
- Step 2: reconfigure the desktop, choose LightDM
```
 sudo dpkg-reconfigure gdm3
```
- Step3: Reboot and select the new desktop in the log in section
```
sudo reboot
```

### 🔹7. Launch navigation2
1. Launch with the saved map
```
ros2 launch nav2_bringup localization_launch.py use_sim_time:=false map:=/absolute/path/to/your_map.yaml
```
2. Launch without the saved map
```
ros2 launch nav2_bringup navigation_launch.py 
```
   

### 📍Others
- Change the Domain ID
- Works in the current terminal:
```
export ROS_DOMAIN_ID=<your_domain_id>

```
- Write into the .bashrc file
```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```
- Source the environment
```
source /~.bashrc 
```


## Navigation  
- Best results are achieved with:  
- **Dijkstra global planner**  
- **MPPI local controller**  

