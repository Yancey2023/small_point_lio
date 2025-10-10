# Small Point-LIO

Small Point-LIO is an advanced implementation of the [Point-LIO algorithm](https://github.com/hku-mars/Point-LIO), delivering a 2-3x speed improvement over the original.

The default branch is for ROS2. If you want to run it without ros, please checkout the `main` branch.

If you want to know why it so fast, please read [this](https://bbs.robomaster.com/article/813022).

## Contact

QQ group: 1070252119

Email: 1709185482@qq.com

## Param

here are some parameters you can set in config file:

```yaml
small_point_lio:
    lidar_topic: /livox/lidar                  # LiDAR topic name
    imu_topic: /livox/imu                      # IMU topic name
    save_pcd: false                            # Whether to save point cloud

    # Point Cloud Filtering
    point_filter_num: 1                        # keep one point every N points
    min_distance: 0.5                          # Minimum point cloud radius; points closer than this will be filtered
    max_distance: 1000                         # Maximum point cloud radius; points farther than this will be filtered
    space_downsample: true                     # Whether to enable point cloud downsampling
    space_downsample_leaf_size: 0.2            # Voxel size used for point cloud downsampling

    # IMU Processing
    gravity: [0.0, 0.0, -9.810]                # Gravity vector
    fix_gravity_direction: true                # Whether to use the first 200 IMU data points to correct gravity direction (magnitude still from gravity parameter)
    check_satu: true                           # Whether to enable IMU data saturation check
    satu_acc: 3.0                              # IMU acceleration saturation threshold
    satu_gyro: 35.0                            # IMU angular velocity saturation threshold
    acc_norm: 1.0                              # IMU acceleration unit (multiple of gravity)

    # Map
    map_resolution: 0.2                        # Map resolution
    init_map_size: 10                          # Number of points required to initialize the map

    # LiDAR-IMU Extrinsic Calibration
    extrinsic_est_en: false                    # Whether to estimate LiDAR-IMU extrinsic transformation online
    extrinsic_T: [-0.011, -0.02329, 0.04412]
    extrinsic_R: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]

    # Kalman Filter Parameters
    # R
    laser_point_cov: 0.01                      # Laser point covariance
    imu_meas_acc_cov: 0.01                     # IMU measured acceleration covariance
    imu_meas_omg_cov: 0.01                     # IMU measured angular velocity covariance
    # Q
    velocity_cov: 20.0                         # Velocity covariance
    acceleration_cov: 500.0                    # Acceleration covariance
    omg_cov: 1000.0                            # Angular velocity covariance
    ba_cov: 0.0001                             # Acceleration bias covariance
    bg_cov: 0.0001                             # Gyroscope bias covariance
    plane_thr: 0.1                             # Plane matching threshold (smaller value = stricter)
    match_s: 81.0                              # Point-to-plane association threshold (smaller value = stricter)

    # Data Publishing
    publish_odometry_without_downsample: false # Whether to publish high-frequency odometry. Note that this does not enhance the real-time nature of the odometry and but degrades performance. It is recommended to increase the point cloud publishing rate to achieve highly real-time odometry.
```

## Save map

**Step 1**: set `pcd_save_enable` to `true` in config file.

**Step 2**: run small point lio until the map is finished.

**Step 3**: save map by calling service:

```cpp
ros2 service call /map_save std_srvs/srv/Trigger
```

> Note: Please make sure you have enough memery to save map. Don't forget to set `pcd_save_enable` to `false` after saving.

## Third-party

Small Point-LIO is built in on and with the aid of the following open source projects. Credits are given to these pojects.

|                     project                     |                description                |                                            license                                            |
| :---------------------------------------------: | :---------------------------------------: | :-------------------------------------------------------------------------------------------: |
|   [Eigen](https://gitlab.com/libeigen/eigen)    | A C++ template library for linear algebra | [Mozilla Public License Version 2.0](https://gitlab.com/libeigen/eigen/-/blob/master/LICENSE) |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp)  |     A YAML parser and emitter in C++      |             [MIT License](https://github.com/jbeder/yaml-cpp/blob/master/LICENSE)             |
|   [spdlog](https://github.com/gabime/spdlog)    |         Fast C++ logging library          |               [MIT License](https://github.com/gabime/spdlog/blob/v1.x/LICENSE)               |
| [PCL](https://github.com/PointCloudLibrary/pcl) |            Point Cloud Library            |        [BSD License](https://github.com/PointCloudLibrary/pcl/blob/master/LICENSE.txt)        |

## License

Copyright (C) 2025 Yingjie Huang

Licensed under the MIT License. See License.txt in the project root for license information.
