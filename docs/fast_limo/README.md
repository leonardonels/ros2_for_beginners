# fast limo

1. install [fast limo](https://github.com/MMR-Electric-Driverless/fast_LIMO.git)
    remember to switch to the ros2-humble branch with `git checkout ros2-humble`

2. download at least one [bag](https://unimore365.sharepoint.com/:f:/s/Driverless.MMR/EoqO8fp94YVPhkEVe6Hw9Z0BVyHfHU-v_fH6nWD6oCa-6A?e=IoMbzz) and [pcap](https://drive.google.com/drive/folders/1lLVYiEECPaj96yk6V_bwePHW_RUXzy7w?usp=sharing) file
    suggested bag file: rosbag2_2025_11_18-16_53_08-5c-0.95
    suggested pcap file: test_1_5deg_0.95m

3. test the following setups and report back what you found, successes, issues and solutions
    - lidar_points + lidar_imu (hesai imu) from the rosbag file
    - lidar_points + /imu/data (xsens imu) from the rosbag file
    - lidar_points + lidar_imu (hesai imu) from the pcap file

4. test the influence of `estimate_extrinsics=True` in fast_limo->config

## notes:
- **ADD THE RIGHT PATH TO THE CORRECTION AND FIRETIMES FILE**:
    - *correction_file* -> YOUR_PATH_TO/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0/correction/angle_correction/OT128_Angle Correction File.csv
    - *firetimes_file* -> YOUR_PATH_TO/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0/correction/firetime_correction/OT128_Firetime Correction File.csv
- when testing could be helpful the following command `ros2 launch fast_limo fast_limo.launch.py rviz:=True` to launch fast limo alongside a preconfigured rviz2 instance
- to run pcap file download the [Hesai ROS 2](https://github.com/leonardonels/HesaiLidar_ROS_2.0.git) driver and customize the config file like is shown below
- the HESAI lidar is positioned ~0.9m above the center of mass (base_link)
- the XSENS Imu is placed ~0.4m in front of the center of mass (base_link)

config files

- fast_limo config file
```yml
# General config

fast_limo:
  ros__parameters:
    topics:
      input:
        lidar: /lidar_points    # <- make sure this topic is correct
        imu: /imu/data          # <- make sure this topic is correct
      # output:
                                # <- currently we dont have any output topic

    frames: # output frames (the transform between frames will be broadcasted)
      world: map                # <- global frame, let's use map
      body: base_link           # <- reference frame for the car (center mass of the car), let's use base_link
      tf_pub: true

    num_threads: 20 # OpenMP threads (will be equal to $(nproc) if it is set higher)

    sensor_type: 2  # LiDAR type (0: OUSTER 1: VELODYNE 2: HESAI 3: LIVOX)     <- make sure hesai is selected

    debug: true     # fill useful intermediate pcl (deskewed, final_raw...) for visualizing purposes
    verbose: true   # print debug/performance board

    estimate_extrinsics: true   # continuous estimation of LiDAR-IMU extrinsics 

    # -------------------------- no need to customize the following options -------------------------- #

    time_offset: true           # whether to take into account a possible sync offset between IMU and LiDAR (set to true if they are not properly in sync)
    end_of_sweep: false         # whether the sweep reference time is w.r.t. the start or the end of the scan (only applies to VELODYNE/OUSTER)

    calibration:  # automatic IMU calibration (if all set to false, no calibration will be done)
      gravity_align: true     # estimate gravity vector
      accel: true             # estimate lin. accel. bias
      gyro: true              # estimate ang. vel. bias
      time: 3.0               # [s] time to estimate (during this time, the robot must be at stand still)

    # ------------------------------------------------------------------------------------------------ #

    extrinsics: # w.r.t baselink [SI]
      imu:
        t: [ 0., 0., 0. ]
        R: [ 1.,  0.,  0.,
              0.,  1.,  0.,
              0.,  0.,  1. ]
      lidar:
        t: [ 0.,  0.,  0. ]
        R: [ 1.,  0.,  0.,
              0.,  1.,  0.,
              0.,  0.,  1. ]

    # -------------------------- no need to customize the following options -------------------------- #

    intrinsics:
      accel:
        bias: [ 0.01, 0.01, 0.01 ]  # [m/s^2]
        sm:   [ 1.,  0.,  0.,
                0.,  1.,  0.,
                0.,  0.,  1. ]
      gyro:
        bias: [ 0.01, 0.01, 0.01 ]  # [rad/s]

    filters:
      cropBox: # prism crop
        active: true
        box:
          min: [ -1.0, -1.0, -1.0 ]  # [m]
          max: [ 1.0, 1.0, 1.0 ]     # [m]

      voxelGrid:
        active: true
        leafSize: [ 0.5, 0.5, 0.5 ]

      minDistance: # sphere crop
        active: true
        value: 3.0  # [m]

      FoV: # crop field of view
        active: false
        value: 180.0  # [deg]
      
      rateSampling: # quick downsample
        active: true
        value: 4

    iKFoM:  # Iterative Kalman Filter on Manifolds lib
      MAX_NUM_ITERS: 2                # max num+1 of iterations of the KF (limits comp. load)
      MAX_NUM_MATCHES: 5000           # max num of matches to account for when computing Jacobians (limits comp. load)
      MAX_NUM_PC2MATCH: 10000         # max num of points to consider when matching with map (limits comp. load)
      LIMITS: 0.001                     

      Mapping:
        NUM_MATCH_POINTS: 5           # num of points that constitute a match
        MAX_DIST_PLANE: 2.0           # [m] max distance between points of every match
        PLANES_THRESHOLD: 5.0e-2      # [m] threshold to consider if match points are part of a plane 
        Octree: # incremental OcTree
          bucket_size: 2      # maximum number of points allowed in an octant before it gets subdivided
          min_extent: 0.2     # minimum extent of the octant (used to stop subdividing)
          downsampling: true  # whether to downsample the octree

      covariance:
        gyro: 0.01 
        accel: 0.01
        bias_gyro: 0.001
        bias_accel: 0.001

    # ------------------------------------------------------------------------------------------------ #
```

- hesai_ros2_driver
```yml

lidar:
  - driver:
      use_gpu: true     # <- if left on the driver will automatically switch to False if needed
      
      source_type: 2    # <- select 2 to read the pcap file                                  
      # The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag, 4: serial    
      # Depending on the type of source_type, fill in the corresponding configuration block (lidar_udp_type, pcap_type, serial_type)

    # -------------------------- no need to customize the following options -------------------------- #
      
      lidar_udp_type:
        device_ip_address: 192.168.1.201              # host_ip_address. If empty(""), the source ip of the udp point cloud is used
        udp_port: 2368                                # UDP destination port
        ptc_port: 9347                                # PTC port of lidar
        multicast_ip_address: 255.255.255.255

        use_ptc_connected: true                       # Set to false when ptc connection is not used
        host_ptc_port: 0                              # PTC source port, 0 means auto
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        recv_point_cloud_timeout: -1                  # The timeout of receiving point cloud
        ptc_connect_timeout: -1                       # The timeout of ptc connection

        host_ip_address: ""
        fault_message_port: 0

        standby_mode: -1                              # The standby mode: [-1] is invalid [0] in operation [1] standby
        speed: -1                                     # The speed: [-1] invalid, you must make sure your set has been supported by the lidar you are using
        ptc_mode: 0                                   # The ptc mode: [0] tcp [1] tcp_ssl
        # tcp_ssl use
        certFile: ""                                  # Represents the path of the user's certificate
        privateKeyFile: ""                            # Represents the path of the user's private key 
        caFile: ""                                    # Represents the path of the CA certificate 

    # ------------------------------------------------------------------------------------------------ #

      pcap_type:
        pcap_path: "/file.pcap"                         # The path of pcap file
        correction_file_path: "/home/orin/ros2_ws/src/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0/correction/angle_correction/OT128_Angle Correction File.csv"   # The path of correction file
        firetimes_path: "/home/orin/ros2_ws/src/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0/correction/firetime_correction/OT128_Firetime Correction File.csv"           # The path of firetimes file

        pcap_play_synchronization: true                     # pcap play rate synchronize with the host time
        pcap_play_in_loop: true
        play_rate_: 1.0                                     # pcap play rate 

    # -------------------------- no need to customize the following options -------------------------- #

      rosbag_type:
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

      serial_type:
        rs485_com: "Your serial port name for receiving point cloud"  # if using JT16, Port to receive the point cloud
        rs232_com: "Your serial port name for sending cmd"            # if using JT16, Port to send cmd
        point_cloud_baudrate: 3125000
        correction_save_path: ""                                      # turn on when you need to store angle calibration files(from lidar)
        correction_file_path: "Your correction file path"             # The path of correction file

      # public module
      use_timestamp_type: 0                 # 0 use point cloud timestamp; 1 use receive timestamp
      frame_start_azimuth: 0                # Frame azimuth for Pandar128, range from 1 to 359, set it less than 0 if you do not want to use it      
      
    # ------------------------------------------------------------------------------------------------ #

      #transform param
      transform_flag: true  # <- set to true to use these options
      x: 0                  # <- translation of the lidar/imu
      y: 0
      z: 1.1
      roll: 0.087           # <- rotation of the lidar/imu
      pitch: 0
      yaw: 0

    # -------------------------- no need to customize the following options -------------------------- #

      # fov config, [fov_start, fov_end] range [1, 359], [-1, -1]means use default
      fov_start: -1
      fov_end:  -1
      channel_fov_filter_path: "Your channel fov filter file path"  # The path of channel_fov_filter file, like channel 0 filter [10, 20] and [30,40]
      multi_fov_filter_ranges: "" # compare to channel_fov_filter_path, multi_fov_filter_ranges for all channels. example config "[20,30];[40,200]"
      # other config
      enable_packet_loss_tool: true         # enable the udp packet loss detection tool
      distance_correction_flag: false       # set to true when optical centre correction needs to be turned on
      xt_spot_correction: false             # Set to TRUE when XT S point cloud layering correction is required
      device_udp_src_port: 0                # Filter point clouds for specified source ports in case of multiple lidar, setting >=1024
      device_fault_port: 0                  # Filter fault message for specified source ports in case of multiple lidar, setting >=1024
      frame_frequency: 0                    # The frequency that point cloud sends.
      default_frame_frequency: 10.0         # The default frequency that point cloud sends.
      echo_mode_filter: 0                   # return mode filter

    ros:
      ros_frame_id: hesai_lidar                       # Frame id of packet message and point cloud message
      ros_recv_packet_topic: /lidar_packets           # Topic used to receive lidar packets from rosbag
      ros_send_packet_topic: /lidar_packets           # Topic used to send lidar raw packets through ROS
      ros_send_point_cloud_topic: /lidar_points       # Topic used to send point cloud through ROS
      ros_send_imu_topic: /lidar_imu                  # Topic used to send lidar imu message
      ros_send_packet_loss_topic: /lidar_packets_loss # Topic used to monitor packets loss condition through ROS
        # ros_recv_correction_topic: /lidar_corrections # Topic used to receive corrections file from rosbag
        # ros_send_correction_topic: /lidar_corrections # Topic used to send correction through ROS
        # ros_send_firetime_topic: /lidar_firetime
        # ros_send_ptp_topic: /lidar_ptp                # Topic used to send PTP lock status, offset through ROS
      send_packet_ros: false                          # true: Send packets through ROS 
      send_point_cloud_ros: true                      # true: Send point cloud through ROS    
      send_imu_ros: true                              # true: Send imu through ROS    
        # car points filter options
      bubble_filter: true                            # Enable bubble filter
      car_filter_distance: 2.0                        # Car filter distance for bubble filter
      cube_filter: false                              # Enable cube filter
      car_filter_distance_x: 0.0                      # Car filter distance in x direction for cube filter
      car_filter_distance_y: 0.0                      # Car filter distance in y direction for cube filter
      car_filter_distance_z: 0.0                      # Car filter distance in z direction for cube filter

# ------------------------------------------------------------------------------------------------ #
```
