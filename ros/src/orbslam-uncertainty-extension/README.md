# ORB SLAM2 with pose uncertainty

We extended [orbslam-map-saving-extension](https://github.com/TUMFTM/orbslam-map-saving-extension) to publish the estimated pose with uncertainty represented by the covariance matrix. In below, we describe important parts of this package. Determination way of the uncertainty can be seen in **doc/how_to_determine_uncertainty.pdf**.



# Preparation

Decompress ORBVoc.txt file in orb_slam2_lib/Vocabulary

```
$ cd src/orbslam-uncertainty-extension/orb_slam2_lib/Vocabulary
$ tar -xf ORBvoc.txt.tar.gz
```

Convert  the vocabulary file ORBvoc from .txt to .bin for faster loading

```
$ ./bin_vocabulary
```

See [orbslam-map-saving-extension](https://github.com/TUMFTM/orbslam-map-saving-extension) for more details such as how to install and software dependencies.



# Parameters

In the orb_slam2_ros/launch/ directory, we prepared two launch scripts, **bebop_localization.launch** and **bebop_slam.launch**. Followings are the important parameters in these launch scripts.



**pose_covariance_scale_factor** (type: double)

The scale factor for the pose covariance matrix (see doc/how_to_determine_uncertainty.pdf for more details).



**use_static_covariance_matrix** (type: bool)

If it is true, the uncertainty determination described above is used. If not, diagonal constant covariance matrix will be set.



**static_covariance_matrix_value** (type: double)

If use_static_covariance_matrix is false, this value will be set to the pose covariance.



These launch scripts load yaml files, **bebop_localization.yaml** and **bebop_slam.yaml** located at orb_slam2_ros/settings/. You can change parameters related to ORB SLAM2 such as name of a map by editing the yaml files.

