/****************************************************************************
 * Copyright (C) 2021 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class SimpleSimulator
{
private:
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub;
    std::string map_frame, base_link_frame;
    std::string input_cmd_topic_name;
    double update_hz, translation_scale;
    double initial_x, initial_y, initial_z, initial_roll, initial_pitch, initial_yaw;
    geometry_msgs::TransformStamped cmd;

public:
    SimpleSimulator();
    ~SimpleSimulator() {};
    void cmd_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void spin(void);
};

SimpleSimulator::SimpleSimulator(void):
    nh("~"),
    map_frame("/map"),
    base_link_frame("/base_link2"),
    input_cmd_topic_name("/cmd_vel_path_follow"),
    update_hz(20.0),
    translation_scale(0.05),
    initial_x(0.0),
    initial_y(0.0),
    initial_z(0.0),
    initial_roll(0.0),
    initial_pitch(0.0),
    initial_yaw(0.0)
{
    // read parameters
    nh.param("map_frame", map_frame, map_frame);
    nh.param("base_link_frame", base_link_frame, base_link_frame);
    nh.param("input_cmd_topic_name", input_cmd_topic_name, input_cmd_topic_name);
    nh.param("update_hz", update_hz, update_hz);
    nh.param("translation_scale", translation_scale, translation_scale);
    nh.param("initial_x", initial_x, initial_x);
    nh.param("initial_y", initial_y, initial_y);
    nh.param("initial_z", initial_z, initial_z);
    nh.param("initial_roll", initial_roll, initial_roll);
    nh.param("initial_pitch", initial_pitch, initial_pitch);
    nh.param("initial_yaw", initial_yaw, initial_yaw);
    // subscriber
    cmd_sub = nh.subscribe(input_cmd_topic_name, 1, &SimpleSimulator::cmd_callback, this);
}

void SimpleSimulator::cmd_callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
    cmd = *msg;
}

void SimpleSimulator::spin(void) {
    double x = initial_x;
    double y = initial_y;
    double z = initial_z;
    double roll = initial_roll;
    double pitch = initial_pitch;
    double yaw = initial_yaw;
    tf::TransformBroadcaster br;
    ros::Rate loop_rate(update_hz);
    while (ros::ok()) {
        ros::spinOnce();
        // update the pose
        double cr = cos(roll);
        double sr = sin(roll);
        double cp = cos(pitch);
        double sp = sin(pitch);
        double tp = tan(pitch);
        double cy = cos(yaw);
        double sy = sin(yaw);
        double vx = cmd.transform.translation.x;
        double vy = cmd.transform.translation.y;
        double vz = cmd.transform.translation.z;
        double wx = cmd.transform.rotation.x;
        double wy = cmd.transform.rotation.y;
        double wz = cmd.transform.rotation.z;
        x += translation_scale * (cp * cy * vx + (sr * sp * sy - cr * sy) * vy + (cr * sp * cy + sr * sy) * vz);
        y += translation_scale * (cp * sy * vx + (sr * sp * sy + cr * cy) * vy + (cr * sp * sy - sr * cy) * vz);
        z += translation_scale * (-sp * vx + sr * cp * vy + cr * cp * vz);
        roll += 1.0 * wx + sr * tp * wy + cr * tp * wz;
        pitch += 0.0 * wx + cr * wy - sr * wz;
        yaw += 0.0 * wx + sr / cp * wy + cr / cp * wz;
        while (roll > M_PI)
            roll -= 2.0 * M_PI;
        while (roll < -M_PI)
            roll += 2.0 * M_PI;
        while (pitch > M_PI)
            pitch -= 2.0 * M_PI;
        while (pitch < -M_PI)
            pitch += 2.0 * M_PI;
        while (yaw > M_PI)
            yaw -= 2.0 * M_PI;
        while (yaw < -M_PI)
            yaw += 2.0 * M_PI;
        // broadcast tf
        tf::Transform tf;
        tf::Quaternion q;
        tf.setOrigin(tf::Vector3(x, y, z));
        q.setRPY(roll, pitch, yaw);
        tf.setRotation(q);
        br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), map_frame, base_link_frame));
        // print info
        printf("x = %.3lf [m], y = %.3lf [m], z = %.3lf [m]\n", x, y, z);
        printf("roll = %.3lf [deg], pitch = %.3lf [deg], yaw = %.3lf [deg]\n",
            roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
        printf("\n");
        // sleep
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_simulator");
    SimpleSimulator node;
    node.spin();
    return 0;
}