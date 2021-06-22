# bebop2navi

The autonomous navigation package for [Parrot BEBOP 2](https://hitecrcd.co.jp/products/bebop2power-packfpv/). This package includes the monocular-camera-based localization and path recording, saving, and following programs. Simple launch scripts are summarized under ros/launch/.



# Characteristics

The main characteristics of the package is the localization program. We use [orbslam-map-saving-extension](https://github.com/TUMFTM/orbslam-map-saving-extension), that is extension of [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2), as a main localization program; however, we extended it to fuse the estimates by ORB SLAM2 and Odometry with Bayesian filtering manner. To fuse them, uncertainty of the ORB SLAM2's estimate is determined while considering Hessian that is used in the optimization process to estimate the camera pose. The ORB SLAM2's estimate is fused using Kalman filter. This fusion enables to estimate smooth trajectory more than the original ORB SLAM2's estimate.

The modules contained in the package are independent. If you want to use the localization module, please copy the orbslam-map-saving-extension and pose_fuser modeuls to your workspace.

We are planning to present the method in both International and Japanese conferences.



# Note

[bebop_autonomy](https://github.com/AutonomyLab/bebop_autonomy) is necessary to communicate with Parrot BEBOP 2. We confirmed that our package works on Ubuntu 16.04 and 18.04; however, bebop_autonomy only works on Ubuntu 16.04.

In our experience, the communication via bebop_autonomy is sometimes refused even though the Wi-Fi connection between the laptop and Parrot BEBOP 2 is still connecting. In this case, the bebop autonomy driver must be launched again. To ensure safety of experiments, it would be better to keep the distance within 5 m. We consider that this distance depends on environments.