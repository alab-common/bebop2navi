/****************************************************************************
 * Copyright (C) 2021 Koki Yasui.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <array>
#include <vector>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pose_fuser/matrix_generator.h>

class PoseFuser {
public:
    ros::NodeHandle nh_;
    ros::Subscriber odomSub_, slamSub_, cmdVelSub_;
    ros::Publisher fusedPosePub_;
    std::string odomTopic_, cmdVelTopic_, slamTopic_, fusedTopic_;
    std::string cameraFrame_, fusedFrame_, mapFrame_, slamFrame_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> slamMsgs_;
    Eigen::VectorXd pose_;
    Eigen::MatrixXd covMat_;
    Eigen::VectorXd cmdVel_;
    std::vector<double> noiseParam_;
    bool isNotSlamMsg_;
    double fusedPoseHz_;
    double slamLostTime_;
    double scaleFactor_;
    double initialCovXX_, initialCovYY_, initialCovZZ_;
    double initialCovRollRoll_, initialCovPitchPitch_, initialCovYawYaw_;
    double initialCov_;

    PoseFuser();
    void odomCB(const nav_msgs::Odometry::ConstPtr &msg);
    void slamCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void cmdVelCB(const geometry_msgs::Twist::ConstPtr &msg);
    void spin(void);
    void publishMassages(void);
    void KalmanFilter(Eigen::VectorXd currPose, Eigen::Matrix<double, 6, 6> currCovMat, geometry_msgs::PoseWithCovarianceStamped slamMsg);
    void modifyAngle(double *angle);
    void modifyRPY(double *roll, double *pitch, double *yaw);
    int get_slam_vectors_index(double currTime);
};

PoseFuser::PoseFuser():
    nh_("~"),
    odomTopic_("/bebop/odom"),
    slamTopic_("/orb_slam2/pose"),
    cmdVelTopic_("/bebop/cmd_vel"),
    fusedTopic_("/fused_pose"),
    cameraFrame_("/camera_world"),
    fusedFrame_("/fused_frame"),
    mapFrame_("/map"),
    isNotSlamMsg_(true),
    fusedPoseHz_(20.0f),
    slamLostTime_(0.2f),
    scaleFactor_(0.11f),
    cmdVel_(6),
    initialCovXX_(2.0f),
    initialCovYY_(2.0f),
    initialCovZZ_(1.0f),
    initialCovRollRoll_(2.0f),
    initialCovPitchPitch_(2.0f),
    initialCovYawYaw_(2.0f),
    initialCov_(std::pow(10.0f, -6))
{   
    noiseParam_ = 
        {0.6, 0.6, 0.6, 0.8, 0.8, 0.8,
        0.6, 0.6, 0.6, 0.8, 0.8, 0.8,
        0.6, 0.6, 0.6, 0.8, 0.8, 0.8,
        0.6, 0.6, 0.6, 0.8, 0.8, 0.8,
        0.6, 0.6, 0.6, 0.8, 0.8, 0.8,
        0.6, 0.6, 0.6, 0.8, 0.8, 0.8};
    
    // set parameter
    nh_.param("odom_topic_name", odomTopic_, odomTopic_);
    nh_.param("slam_topic_name", slamTopic_, slamTopic_);
    nh_.param("cmd_vel_topic_name", cmdVelTopic_, cmdVelTopic_);
    nh_.param("fused_pose_topic_name", fusedTopic_, fusedTopic_);
    nh_.param("map_frame", mapFrame_, mapFrame_);
    nh_.param("camera_frame", cameraFrame_, cameraFrame_);
    nh_.param("fused_frame", fusedFrame_, fusedFrame_);
    nh_.param("fused_pose_hz", fusedPoseHz_, fusedPoseHz_);
    nh_.param("slam_lost_time", slamLostTime_, slamLostTime_);
    nh_.param("scale_factor", scaleFactor_, scaleFactor_);
    nh_.param("initial_cov_xx", initialCovXX_, initialCovXX_);
    nh_.param("initial_cov_yy", initialCovYY_, initialCovYY_);
    nh_.param("initial_cov_zz", initialCovZZ_, initialCovZZ_);
    nh_.param("initial_cov_rollroll", initialCovRollRoll_, initialCovPitchPitch_);
    nh_.param("initial_cov_pitchpitch", initialCovPitchPitch_, initialCovPitchPitch_);
    nh_.param("initial_cov_yawyaw", initialCovYawYaw_, initialCovYawYaw_);
    nh_.param("initial_cov", initialCov_, initialCov_);
    nh_.getParam("noise_param", noiseParam_);

    // set initial pose
    pose_ = Eigen::VectorXd::Zero(6);
    pose_(0) = 0.1;
    // set initial covariance matrix of pose
    covMat_ = initialCov_ * Eigen::MatrixXd::Ones(6,6);
    covMat_(0,0) = initialCovXX_;
    covMat_(1,1) = initialCovYY_;
    covMat_(2,2) = initialCovZZ_;
    covMat_(3,3) = initialCovRollRoll_;
    covMat_(4,4) = initialCovPitchPitch_;
    covMat_(5,5) = initialCovYawYaw_;

    // subscriber
    odomSub_ = nh_.subscribe(odomTopic_, 1, &PoseFuser::odomCB, this);
    slamSub_ = nh_.subscribe(slamTopic_, 1, &PoseFuser::slamCB, this);
    cmdVelSub_ = nh_.subscribe(cmdVelTopic_, 1, &PoseFuser::cmdVelCB, this);
    // publisher
    fusedPosePub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(fusedTopic_, 1);
}

void PoseFuser::odomCB(const nav_msgs::Odometry::ConstPtr &msg){
    static bool isFirst = true;
    static double prevTime;
    static Eigen::VectorXd prevAttitude(3);
    
    if (isFirst) {
        // set initial values
        prevTime = msg->header.stamp.toSec();

        tf::Quaternion q(
        msg->pose.pose.orientation.x, 
        msg->pose.pose.orientation.y, 
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(prevAttitude(0), prevAttitude(1), prevAttitude(2));
        
        isFirst = false;
        return;
    }

    double currTime = msg->header.stamp.toSec();
    double deltaTime = currTime - prevTime;
    Eigen::VectorXd u(6), omega(3), omegaRPY(3), currAttitude(3), pose(6); 
    Eigen::MatrixXd R(6,6), V(6,6), M(6,6), G(6,6), covMat(6,6);

    // get current pose & covariance matrix
    pose = pose_;
    covMat = covMat_;

    // get current attitude
    tf::Quaternion q(
        msg->pose.pose.orientation.x, 
        msg->pose.pose.orientation.y, 
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(currAttitude(0), currAttitude(1), currAttitude(2));
    // calculate angular velocity
    omegaRPY = (currAttitude - prevAttitude) / deltaTime;
    omega(0) = omegaRPY(0) + std::sin(currAttitude(1)) * omegaRPY(2);
    omega(1) = std::cos(currAttitude(0)) * omegaRPY(1) - std::sin(currAttitude(0)) * std::cos(currAttitude(1)) * omegaRPY(2);
    omega(2) = std::sin(currAttitude(0)) * omegaRPY(1) + std::cos(currAttitude(0)) * std::cos(currAttitude(1)) * omegaRPY(2);

    // set veloity & angular velocity
    u <<
        scaleFactor_ * msg->twist.twist.linear.x,
        scaleFactor_ * msg->twist.twist.linear.y,
        scaleFactor_ * msg->twist.twist.linear.z,
        omega(0),
        omega(1),
        omega(2);
    
    //  get matrices
    R = mat_generator::get_transformation_matrix_map_to_world(currAttitude);
    G = mat_generator::get_Jacobian_matrix_state(deltaTime, currAttitude, u);
    V = mat_generator::get_Jacobian_matrix_input(deltaTime, currAttitude);
    M = mat_generator::get_input_error_matrix(noiseParam_, u);
       
    // update odometry & covariance matrix
    pose = pose + deltaTime * R * u;
    covMat = G * covMat * G.transpose() + V * M * V.transpose();
    modifyRPY(&pose(3), &pose(4), &pose(5));
    
    pose_ = pose;
    covMat_ = covMat;
    
    prevTime = currTime;
    prevAttitude = currAttitude;
}

void PoseFuser::slamCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    // input ORB-SLAM messages into vector
    slamMsgs_.insert(slamMsgs_.begin(), *msg);
    if (slamMsgs_.size() >= 10)
        slamMsgs_.resize(10);
    isNotSlamMsg_ = false;
}

void PoseFuser::cmdVelCB(const geometry_msgs::Twist::ConstPtr &msg){
    // get command value
    cmdVel_(0) = msg->linear.x;
    cmdVel_(1) = msg->linear.y;
    cmdVel_(2) = msg->linear.z;
    cmdVel_(3) = msg->angular.x;
    cmdVel_(4) = msg->angular.y;
    cmdVel_(5) = msg->angular.z;
}

void PoseFuser::KalmanFilter(Eigen::VectorXd currPose, Eigen::Matrix<double, 6, 6> currCovMat, geometry_msgs::PoseWithCovarianceStamped slamMsg){
    static tf::Quaternion q_camera2Map;
    static Eigen::Matrix3d transformPosition;
    static bool isFirst = true;
    
    if (isFirst){
        // transform from camera world to map
        tf::TransformListener tfListener;
        tf::StampedTransform camera2Map;

        try{
            ros::Time now = slamMsg.header.stamp + ros::Duration(1.0);
            tfListener.waitForTransform(cameraFrame_, mapFrame_, now, ros::Duration(1.0));
            tfListener.lookupTransform(cameraFrame_, mapFrame_, now, camera2Map); 
        } catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }

        q_camera2Map[0] = camera2Map.getRotation().x();
        q_camera2Map[1] = camera2Map.getRotation().y();
        q_camera2Map[2] = camera2Map.getRotation().z();
        q_camera2Map[3] = camera2Map.getRotation().w();

        tf::Matrix3x3 m(q_camera2Map);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // transformation matrix from camera world to map
        transformPosition = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).inverse()
                          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).inverse()
                          * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).inverse();
        
        isFirst = false;
    }
    
    // when trace of covariance matix is small, exit Kalman filter
    if (currCovMat.trace() < 0.0001f){
        return;
    }

    Eigen::MatrixXd KalmanGain(6,6), outputMat(6,6), slamCovMat(6,6), fusedCovMat(6,6);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
    Eigen::VectorXd slamPose(6), fusedPose(6);

    // transform position & attitude from camera world to map
    Eigen::VectorXd position(3);
    position(0) = slamMsg.pose.pose.position.x;
    position(1) = slamMsg.pose.pose.position.y;
    position(2) = slamMsg.pose.pose.position.z;
    position = transformPosition * position;
    slamPose(0) = position(0);
    slamPose(1) = position(1);
    slamPose(2) = position(2);
    tf::Quaternion q(
        slamMsg.pose.pose.orientation.x, 
        slamMsg.pose.pose.orientation.y, 
        slamMsg.pose.pose.orientation.z,
        slamMsg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    Eigen::Vector3d slamAttitude;
    m.getRPY(slamAttitude(0), slamAttitude(1), slamAttitude(2));

    slamPose(3) = atan2(std::cos(slamAttitude(0))*std::sin(slamAttitude(2)), std::sin(slamAttitude(0))*std::sin(slamAttitude(1))*std::sin(slamAttitude(2))+std::cos(slamAttitude(0))*std::cos(slamAttitude(2)));
    slamPose(4) = asin(std::cos(slamAttitude(0))*std::sin(slamAttitude(1))*std::sin(slamAttitude(2))-std::sin(slamAttitude(0))*std::cos(slamAttitude(2)));
    slamPose(5) = atan2(-std::cos(slamAttitude(0))*std::sin(slamAttitude(1))*std::cos(slamAttitude(2))-std::sin(slamAttitude(0))*std::sin(slamAttitude(2)),std::cos(slamAttitude(0))*std::cos(slamAttitude(1)));
    
    // for debug
    tf::Quaternion q2;
    q2.setRPY(slamPose(3),slamPose(4),slamPose(5));
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(slamPose(0), slamPose(1), slamPose(2)));
    transform.setRotation(q2);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), mapFrame_, "/base_link3"));
    // for debug

    // set matrices
    outputMat = I;
    // covariance matrix of pose from slam
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++)
        slamCovMat(i, j) = slamMsg.pose.covariance[i * 6 + j];
    }
    // slamCovMat = 0.1 * I;

    // transform covariance matrix from camera to map
    Eigen::Matrix3d JacobiMatRPY = mat_generator::get_transform_matrix_RPY(slamAttitude);
    Eigen::MatrixXd transformMat = Eigen::MatrixXd::Zero(6,6);
    // transform matrix
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            transformMat(i,j) = transformPosition(i,j);
            transformMat(3+i,3+j) = JacobiMatRPY(i,j);
        }
    }
    slamCovMat = transformMat * slamCovMat * transformMat.transpose();

    // get Kalman Gain
    KalmanGain = currCovMat * outputMat.transpose() 
                * (outputMat * currCovMat * outputMat.transpose() + slamCovMat).inverse();
    
    Eigen::VectorXd diffVector = slamPose - outputMat * currPose;
    modifyRPY(&diffVector(3), &diffVector(4), &diffVector(5));

    // update pose and covariance matrix
    pose_ = currPose + KalmanGain * diffVector;
    modifyRPY(&pose_(3), &pose_(4), &pose_(5));
    
    covMat_ = (I - KalmanGain * outputMat) * currCovMat;
}

void PoseFuser::publishMassages(void){
    tf::Quaternion q;
    q.setRPY(pose_(3), pose_(4), pose_(5));
    
    geometry_msgs::PoseWithCovarianceStamped fusedPose;
    fusedPose.header.stamp = ros::Time::now();
    fusedPose.header.frame_id = mapFrame_;
    fusedPose.pose.pose.position.x = pose_(0);
    fusedPose.pose.pose.position.y = pose_(1);
    fusedPose.pose.pose.position.z = pose_(2);
    fusedPose.pose.pose.orientation.x = q[0];
    fusedPose.pose.pose.orientation.y = q[1];
    fusedPose.pose.pose.orientation.z = q[2];
    fusedPose.pose.pose.orientation.w = q[3];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++)
        fusedPose.pose.covariance[i * 6 + j] = covMat_(i,j);
    }
    fusedPosePub_.publish(fusedPose);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose_(0), pose_(1), pose_(2)));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), mapFrame_, fusedFrame_));
}

int PoseFuser::get_slam_vectors_index(double currTime) {
    if (isNotSlamMsg_){
        return -1;
    }
    for (int i = 0; i < (int)slamMsgs_.size(); i++) {
        double time = slamMsgs_[i].header.stamp.toSec();
        if (time < currTime && fabs(currTime - time) < slamLostTime_)
            return i;
    }
    return -1;
}

void PoseFuser::spin(void){
    ros::Rate rate(fusedPoseHz_);
    geometry_msgs::PoseWithCovarianceStamped currSlamMsg;
    Eigen::MatrixXd currCovMat(6,6);
    Eigen::VectorXd currPose(6);
    double currTime;
    
    while(ros::ok()){
        // get current values
        currTime = ros::Time::now().toSec();
        currPose = pose_;
        currCovMat = covMat_;
    
        int i = get_slam_vectors_index(currTime);
        if (i >= 0){
            currSlamMsg = slamMsgs_[i];
            KalmanFilter(currPose, currCovMat, currSlamMsg);
        }
        
        publishMassages();
        ros::spinOnce();
        rate.sleep();
    }
}

void PoseFuser::modifyAngle(double *angle){
    while (*angle < -M_PI)
        *angle += 2.0 * M_PI;
    while (*angle > M_PI)
        *angle -= 2.0 * M_PI;
}

void PoseFuser::modifyRPY(double *roll, double *pitch, double *yaw){
    modifyAngle(roll);
    modifyAngle(pitch);
    modifyAngle(yaw);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pose_fuser");
    PoseFuser node;

    node.spin();
    return 0;
}
