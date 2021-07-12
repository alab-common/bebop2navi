# bebop2navi

The ROS-based autonomous navigation package for [Parrot BEBOP 2](https://hitecrcd.co.jp/products/bebop2power-packfpv/). This package includes the monocular-camera-based localization and path recording, saving, and following programs. Simple launch scripts are summarized under **ros/launch/**.

Demo video on YouTube: [Autonomous flight of Parrot BEBOP 2](https://www.youtube.com/watch?v=-uUoH1lWyTU)



# Note

[bebop_autonomy](https://github.com/AutonomyLab/bebop_autonomy) is necessary to communicate with Parrot BEBOP 2 on ROS. We confirmed that our package works on Ubuntu 16.04 and 18.04; however, **bebop_autonomy only works on Ubuntu 16.04**.



# Requirements and install

- OpenCV
- Eigen3
- Boost
- octomap-ros

You first need to do some preparations for **orbslam-uncertainty-extension**. Please into **ros/src/orbslam-uncertainty-extension/** and see README.md.



# Characteristics

The main characteristics of the package is the localization program. We use [orbslam-map-saving-extension](https://github.com/TUMFTM/orbslam-map-saving-extension), that is extension of [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) by [Institute of Automotive Technology](https://www.ftm.mw.tum.de/en/home/) of the Technical University of Munich, as a main localization program. We further extended it to fuse the estimates by ORB SLAM2 and Odometry with Bayesian filtering manner. To fuse them, uncertainty of the ORB SLAM2's estimate is determined while considering Hessian that is used in the optimization process to track the camera pose. The ORB SLAM2's estimate is fused with Odometry using Kalman filter. This fusion enables to estimate smooth trajectory more than the original ORB SLAM2's estimate.

The modules contained in the package are independent. If you want to use only the localization module, please copy the **orbslam-uncertainty-extension** and **pose_fuser** modules to your workspace. You can apply the localization programs to other vehicles, e.g., wheeled robots.



# Contributes

The main developers

- Naoki Akai ([home page](https://sites.google.com/view/naokiakaigoo/home))
- Kazuya Arashi
- Koki Yasui
- Kane Saliou

We are planning to present this project in both International and Japanese conferences.

[Japanese preprint](https://www.researchgate.net/publication/353186082_Fusion_of_Optimization-Based_Monocular_Visual_Localization_with_Bayesian_Filter_and_Autonomous_Quadcopter_Navigation)

