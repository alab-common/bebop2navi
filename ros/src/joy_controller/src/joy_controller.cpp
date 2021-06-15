/****************************************************************************
 * Copyright (C) 2021 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>

class JoyController {
public:
    ros::NodeHandle nh_;
    ros::Subscriber joySub_;
    ros::Publisher cmdPub_, twistPub_;
    std::string joyTopicName_, cmdTopicName_;
    bool publishTwist_;
    double publishHz_, velocitiesCoef_;
    geometry_msgs::Twist cmd_;
    double priority_;

    JoyController(void):
        nh_("~"),
        joyTopicName_("/joy"),
        cmdTopicName_("/cmd_vel_joy"),
        publishTwist_(false),
        publishHz_(10.0),
        velocitiesCoef_(0.1),
        priority_(0.0)
    {
        nh_.param("joy_topic_name", joyTopicName_, joyTopicName_);
        nh_.param("joy_controller_topic_name", cmdTopicName_, cmdTopicName_);
        nh_.param("publish_twist", publishTwist_, publishTwist_);
        nh_.param("cmd_publish_hz", publishHz_, publishHz_);
        nh_.param("joy_velocities_coef", velocitiesCoef_, velocitiesCoef_);

        joySub_ = nh_.subscribe(joyTopicName_, 1, &JoyController::joyCB, this);

        cmdPub_ = nh_.advertise<geometry_msgs::Transform>(cmdTopicName_, 10);
        if (publishTwist_)
            twistPub_ = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 10);
    }

    void joyCB(const sensor_msgs::Joy::ConstPtr &msg) {
        cmd_.linear.x = cmd_.linear.y = cmd_.linear.z = 0.0;
        cmd_.angular.x = cmd_.angular.y = cmd_.angular.z = 0.0;
        if (msg->buttons[5] == 1) {
            cmd_.linear.x = msg->axes[1] * velocitiesCoef_;
            cmd_.linear.y = msg->axes[0] * velocitiesCoef_;
            cmd_.linear.z = msg->axes[5] * velocitiesCoef_;
            cmd_.angular.z = msg->axes[2] * velocitiesCoef_;

            if (msg->buttons[8] == 1)
                priority_ += 1.0;
            else if (msg->buttons[9] == 1)
                priority_ -= 1.0;

            if (msg->buttons[3] == 1) {
                printf("takeoff\n");
                int retVal = system("rostopic pub --once /bebop/takeoff std_msgs/Empty");
            } else if (msg->buttons[1] == 1) {
                printf("land\n");
                int retVal = system("rostopic pub --once /bebop/land std_msgs/Empty");
            } else {
                printf("x = %.3lf, y = %.3lf, z = %.3lf, yaw = %.3lf, priority = %.1f\n",
                    cmd_.linear.x, cmd_.linear.y, cmd_.linear.z, cmd_.angular.z, priority_);
            }
        }
    }

    geometry_msgs::Transform getJoyCmd(void) {
        geometry_msgs::Transform cmd;
        cmd.translation.x = cmd_.linear.x;
        cmd.translation.y = cmd_.linear.y;
        cmd.translation.z = cmd_.linear.z;
        cmd.rotation.x = cmd_.angular.x;
        cmd.rotation.y = cmd_.angular.y;
        cmd.rotation.z = cmd_.angular.z;
        cmd.rotation.w = priority_;
        return cmd;
    }

    void spin(void) {
        ros::Rate loopRate(publishHz_);
        while (ros::ok()) {
            ros::spinOnce();
            geometry_msgs::Transform cmd = getJoyCmd();
            cmdPub_.publish(cmd);
            if (publishTwist_)
                twistPub_.publish(cmd_);
            loopRate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_controller_node");
    JoyController node;
    node.spin();
    return 0;
}
