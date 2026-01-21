# mmr_ekf_odometry

1. install [mmr_ekf_odometry](https://github.com/leonardonels/mmr_ekf_odometry.git)

2. this time the bag is only one and can be downloaded from [google-drive](soon_tm) or [nextcloud.leonardonels](https://nextcloud.leonardonels.com/s/wPgrT9xLMYcKeWG)

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