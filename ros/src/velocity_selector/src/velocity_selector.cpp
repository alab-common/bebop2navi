/****************************************************************************
 * Copyright (C) 2021 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

class VelocitySelector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber joyCmdSub_, pathFollowCmdSub_, orbSLAMTrackingFlagSub_;
    ros::Publisher cmdPub_;
    std::string joyCmdName_, pathFollowCmdName_, orbSLAMTrackingFlagName_;
    std::string pubCmdName_;
    bool useORBSLAMTrackingFlag_;
    double cmdPublishHz_, watchDogTime_;

    geometry_msgs::TransformStamped joyCmd_, pathFollowCmd_;
    std_msgs::Char orbSLAMTrackingFlag_;
    double lastTimeOfTrackingFlag_;

public:
    VelocitySelector(void):
        nh_("~"),
        joyCmdName_("/cmd_vel_joy"),
        pathFollowCmdName_("/cmd_vel_path_follow"),
        orbSLAMTrackingFlagName_("/orb_slam_tracking_flag"),
        pubCmdName_("/cmd_vel"),
        useORBSLAMTrackingFlag_(false),
        cmdPublishHz_(20.0),
        watchDogTime_(3.0)
    {
        nh_.param("joy_cmd_name", joyCmdName_, joyCmdName_);
        nh_.param("path_follow_cmd_name", pathFollowCmdName_, pathFollowCmdName_);
        nh_.param("orb_slam_tracking_flag", orbSLAMTrackingFlagName_, orbSLAMTrackingFlagName_);
        nh_.param("use_orb_slam_tracking_flag", useORBSLAMTrackingFlag_, useORBSLAMTrackingFlag_);
        nh_.param("cmd_name", pubCmdName_, pubCmdName_);
        nh_.param("cmd_publish_hz", cmdPublishHz_, cmdPublishHz_);
        nh_.param("watch_dog_time", watchDogTime_, watchDogTime_);

        joyCmdSub_ = nh_.subscribe(joyCmdName_, 1, &VelocitySelector::joyCmdCB, this);
        pathFollowCmdSub_ = nh_.subscribe(pathFollowCmdName_, 1, &VelocitySelector::pathFollowCmdCB, this);
        orbSLAMTrackingFlagSub_ = nh_.subscribe(orbSLAMTrackingFlagName_, 1, &VelocitySelector::orbSLAMTrackingFlagCB, this);

        cmdPub_ = nh_.advertise<geometry_msgs::Twist>(pubCmdName_, 10);

        ros::Rate loopRate(cmdPublishHz_);
        while (ros::ok()) {
            ros::spinOnce();
            geometry_msgs::Twist cmd = getCmd();
            cmdPub_.publish(cmd);
            loopRate.sleep();
        }
    }

    inline void joyCmdCB(const geometry_msgs::TransformStamped::ConstPtr &msg) {
        joyCmd_ = *msg;
    }

    inline void pathFollowCmdCB(const geometry_msgs::TransformStamped::ConstPtr &msg) {
        pathFollowCmd_ = *msg;
    }

    inline void orbSLAMTrackingFlagCB(const std_msgs::Char::ConstPtr &msg) {
        orbSLAMTrackingFlag_ = *msg;
        lastTimeOfTrackingFlag_ = ros::Time::now().toSec();
    }

    geometry_msgs::Twist getCmd(void) {
        double currTime = ros::Time::now().toSec();
        geometry_msgs::Twist cmd;

        // The joy stick command is definitely used if its priority is positive
        if (joyCmd_.transform.rotation.w > 0.0) {
            double deltaTime = currTime - joyCmd_.header.stamp.toSec();
            if (deltaTime > watchDogTime_) {
                ROS_WARN("The joy stick command is not renewed. \
                    %lf second past from the last update.", deltaTime);
                cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
                cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;
                return cmd;
            } else {
                cmd.linear.x = joyCmd_.transform.translation.x;
                cmd.linear.y = joyCmd_.transform.translation.y;
                cmd.linear.z = joyCmd_.transform.translation.z;
                cmd.angular.x = joyCmd_.transform.rotation.x;
                cmd.angular.y = joyCmd_.transform.rotation.y;
                cmd.angular.z = joyCmd_.transform.rotation.z;
                return cmd;
            }
        }

        if (useORBSLAMTrackingFlag_) {
            double deltaTime = currTime - lastTimeOfTrackingFlag_;
            if (deltaTime > watchDogTime_) {
                ROS_WARN("ORB SLAM tracking flag is not renewed. ORB SLAM might not work.\
                    %lf second past from the last update.", deltaTime);
                cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
                cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;
                return cmd;
            }
            if (orbSLAMTrackingFlag_.data != 3) {
                ROS_WARN("ORB SLAM failed to track. The joy stick command is only acceptable.");
                cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
                cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;
                return cmd;
            }
        }

        if (pathFollowCmd_.transform.rotation.w > 0.0) {
            double deltaTime = currTime - pathFollowCmd_.header.stamp.toSec();
            if (deltaTime > watchDogTime_) {
                ROS_WARN("The command from the path follower is not renewed. \
                    %lf second past from the last update.", deltaTime);
                cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
                cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;
                return cmd;
            }

            cmd.linear.x = pathFollowCmd_.transform.translation.x;
            cmd.linear.y = pathFollowCmd_.transform.translation.y;
            cmd.linear.z = pathFollowCmd_.transform.translation.z;
            cmd.angular.x = pathFollowCmd_.transform.rotation.x;
            cmd.angular.y = pathFollowCmd_.transform.rotation.y;
            cmd.angular.z = pathFollowCmd_.transform.rotation.z;
            return cmd;
        } else {
            ROS_INFO("No available commands. Set zero velocities.");
            cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
            cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;
            return cmd;
        }
    }

}; // class VelocitySelector

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_selector");
    VelocitySelector node;
    return 0;
}