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
    tf::TransformListener tf_listener;
    double update_hz;
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
    tf_listener()
{
    // read parameters
    nh.param("map_frame", map_frame, map_frame);
    nh.param("base_link_frame", base_link_frame, base_link_frame);
    nh.param("input_cmd_topic_name", input_cmd_topic_name, input_cmd_topic_name);
    nh.param("update_hz", update_hz, update_hz);
    // subscriber
    cmd_sub = nh.subscribe(input_cmd_topic_name, 1, &SimpleSimulator::cmd_callback, this);
}

void SimpleSimulator::cmd_callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
    cmd = *msg;
}

void SimpleSimulator::spin(void) {
    tf::TransformBroadcaster br;
    ros::Rate loop_rate(update_hz);
    while (ros::ok()) {
        ros::spinOnce();
        ros::spinOnce();
        // read robot pose (base_link_frame) from tf tree in map_frame
        ros::Time now = ros::Time::now();
        tf::StampedTransform map2base_link;
        try
        {
            tf_listener.waitForTransform(map_frame, base_link_frame, now, ros::Duration(1.0));
            tf_listener.lookupTransform(map_frame, base_link_frame, now, map2base_link);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            continue;
        }
        // read the 3d robot pose
        double x = map2base_link.getOrigin().x();
        double y = map2base_link.getOrigin().y();
        double z = map2base_link.getOrigin().z();
        tf::Quaternion q(map2base_link.getRotation().x(),
            map2base_link.getRotation().y(),
            map2base_link.getRotation().z(),
            map2base_link.getRotation().w());
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
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
        x += cp * cy * vx + (sr * sp * sy - cr * sy) * vy + (cr * sp * cy + sr * sy) * vz;
        y += cp * sy * vx + (sr * sp * sy + cr * cy) * vy + (cr * sp * sy - sr * cy) * vz;
        z += -sp * vx + sr * cp * vy + cr * cp * vz;
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
        tf.setOrigin(tf::Vector3(x, y, z));
        q.setRPY(roll, pitch, yaw);
        tf.setRotation(q);
        br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), map_frame, base_link_frame));
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
