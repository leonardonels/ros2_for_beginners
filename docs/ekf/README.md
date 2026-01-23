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

6. Download the bag to test it from [google-drive](soon_tm) or [nextcloud.leonardonels](https://nextcloud.leonardonels.com/s/wPgrT9xLMYcKeWG)

## notes:
ignore any reference to gazebo or simulation files since they are ment for a gazebo simulator

```yml
/mmr_ekf_odometry_node:
  ros__parameters:
    generic:
      cones_topic: "/perception/colorblind_cones"
      imu_topic: "/imu/data"
      input_odom_topic: "/Odometry/fastLioOdom"
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
