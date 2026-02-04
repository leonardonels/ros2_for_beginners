# mmr_ekf_odometry

1. Download [common messages](https://github.com/MMR-Electric-Driverless/common_msgs)
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/MMR-Electric-Driverless/common_msgs
   ```
2. Build common_msgs
   ``` bash
   cd ~/ros2_ws
   colcon build --packages-select common_msgs
   ```
3. Source common mesages
   ```bash
   source install/setup.bash
   ```
4. Download [mmr_ekf_odometry](https://github.com/leonardonels/mmr_ekf_odometry.git)
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/leonardonels/mmr_ekf_odometry.git
   ```
5. Build mmr_ekf_odometry
   ```bash
   colcon build --packages-select mmr_ekf_odometry --symlink-install
   source install/setup.bash
   ```
7. Download the bags to test it from [google-drive](https://drive.google.com/drive/folders/1XP3lOW1AS15QlEyR_MPKyaI0VNsc3VF8?hl=it): rosbag2_2026_02_04-14_35_36 (bean circuit) and rosbag2_2026_02_04-15_45_27 (long circuit)

8. Launch mmr_ekf_odometry and run one of the bags
   ```bash
   ros2 launch mmr_ekf_odometry mmr_ekf_odometry.launch.py
   ```
## notes:
Ignore any reference to gazebo or simulation files since they are ment for a gazebo simulator

*mmr_ekf_odometry.yaml*
```yml
/mmr_ekf_odometry_node:
  ros__parameters:
    generic:
      cones_topic: "/clusters"
      imu_topic: "/synced/imu/data"
      input_odom_topic: "/fast_limo/state"
      output_odom_topic: "/Odometry"
      gps_speed_topic: "/speed/gps"
      race_status_topic: "/planning/race_status"
      enable_logging: false
      cone_time_seen_th: 4
      is_skidpad_mission: false

    frequency:
      imu_fs: 100
      gps_fs: 10

    noises:
      proc_noise:  [1.0, 0.5]        # Sigma range [m] and bearing [rad]
      meas_noise:  [0.02, 0.02, 0.07]  # Gaussian error of the odometry X,Y position [m] and Yaw angle [rad]
      min_new_cone_distance: 2.0      # Minimum distance to create new cone
      
```
