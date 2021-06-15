/****************************************************************************
 * Copyright (C) 2021 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class PathFollower
{
private:
    ros::NodeHandle nh;
    ros::Subscriber path_sub;
    ros::Publisher cmd_pub, twist_pub;
    std::string map_frame, base_link_frame;
    std::string input_path_topic_name, output_cmd_topic_name;
    nav_msgs::Path path;
    tf::TransformListener tf_listener;
    double look_ahead_dist, max_vel, kv;
    double cmd_publish_hz;
    bool iterate_following, publish_twist;
    int nearest_path_index, prev_nearest_path_index;

public:
    PathFollower();
    ~PathFollower() {};
    void path_callback(const nav_msgs::Path::ConstPtr& msg);
    void spin(void);
};

PathFollower::PathFollower():
    nh("~"),
    map_frame("/map"),
    base_link_frame("/base_link2"),
    input_path_topic_name("/target_path"),
    output_cmd_topic_name("/cmd_vel_path_follow"),
    look_ahead_dist(0.2),
    max_vel(0.1),
    kv(0.06),
    cmd_publish_hz(20.0),
    prev_nearest_path_index(-1),
    iterate_following(false),
    publish_twist(false),
    tf_listener()
{
    // read parameters
    nh.param("map_frame", map_frame, map_frame);
    nh.param("base_link_frame", base_link_frame, base_link_frame);
    nh.param("input_path_topic_name", input_path_topic_name, input_path_topic_name);
    nh.param("output_cmd_topic_name", output_cmd_topic_name, output_cmd_topic_name);
    nh.param("look_ahead_dist", look_ahead_dist, look_ahead_dist);
    nh.param("max_vel", max_vel, max_vel);
    nh.param("kv", kv, kv);
    nh.param("cmd_publish_hz", cmd_publish_hz, cmd_publish_hz);
    nh.param("iterate_following", iterate_following, iterate_following);
    nh.param("publish_twist", publish_twist, publish_twist);
    // subscriber
    path_sub = nh.subscribe(input_path_topic_name, 1, &PathFollower::path_callback, this);
    // publisher
    cmd_pub = nh.advertise<geometry_msgs::Twist>(output_cmd_topic_name, 10);
    if (publish_twist)
        twist_pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 10);
}

void PathFollower::path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    path = *msg;
    prev_nearest_path_index = -1;
}

void PathFollower::spin(void)
{
    double eo = 0.0;
    ros::Rate loop_rate(cmd_publish_hz);
    while (ros::ok())
    {
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
        // 3d robot pose
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
        // search the nearest path point
        double min_dist;
        int i0, i1;
        if (prev_nearest_path_index < 0)
        {
            i0 = 1;
            i1 = path.poses.size();
        }
        else
        {
            i0 = prev_nearest_path_index - 30;
            i1 = prev_nearest_path_index + 30;
            if (i0 < 1)
                i0 = 1;
            if (i1 >= path.poses.size())
                i1 = path.poses.size();
        }
        for (int i = i0; i < i1; i++)
        {
            double dx = path.poses[i].pose.position.x - x;
            double dy = path.poses[i].pose.position.y - y;
            double dl = sqrt(dx * dx + dy * dy);
            if (i == i0)
            {
                nearest_path_index = i;
                min_dist = dl;
            }
            else if (dl < min_dist)
            {
                nearest_path_index = i;
                min_dist = dl;
            }
        }
        prev_nearest_path_index = nearest_path_index;
        // search the target path point
        int target_path_index = -1;
        for (int i = nearest_path_index; i < path.poses.size(); i++)
        {
            double dx = path.poses[i].pose.position.x - x;
            double dy = path.poses[i].pose.position.y - y;
            double dl = sqrt(dx * dx + dy * dy);
            if (dl >= look_ahead_dist)
            {
                target_path_index = i;
                break;
            }
        }
        // determine twist command
        geometry_msgs::TransformStamped cmd_vel;
        geometry_msgs::Twist twist_cmd;
        bool stop;
        nh.getParam("/path_follower/stop", stop);
        if (target_path_index < 0 || path.poses.size() == 0 || stop)
        {
            // reach at the goal point or no path data
            cmd_vel.header.stamp = ros::Time::now();
            twist_cmd.linear.x = cmd_vel.transform.translation.x = 0.0;
            twist_cmd.linear.y = cmd_vel.transform.translation.y = 0.0;
            twist_cmd.linear.z = cmd_vel.transform.translation.z = 0.0;
            twist_cmd.angular.x = cmd_vel.transform.rotation.x = 0.0;
            twist_cmd.angular.y = cmd_vel.transform.rotation.y = 0.0;
            twist_cmd.angular.z = cmd_vel.transform.rotation.z = 0.0;
            cmd_vel.transform.rotation.w = 0.0; // this is used as priority
            eo = 0.0;
        }
        else
        {
            // compute deviation from the path and determine twist_cmd based on PD control
            double dx = path.poses[target_path_index].pose.position.x - path.poses[target_path_index - 1].pose.position.x;
            double dy = path.poses[target_path_index].pose.position.y - path.poses[target_path_index - 1].pose.position.y;
            double dz = path.poses[target_path_index].pose.position.z - path.poses[target_path_index - 1].pose.position.z;
            double th = atan2(dy, dx);
            double x2p = dx * cos(th) + dy * sin(th);
            dx = path.poses[target_path_index].pose.position.x - x;
            dy = path.poses[target_path_index].pose.position.y - y;
            double xrp = dx * cos(th) + dy * sin(th);
            double yrp = -dx * sin(th) + dy * cos(th);
            double t = atan2(dy, dx);
            double e = t - yaw;
            if (e < -M_PI)
                e += 2.0 * M_PI;
            if (e > M_PI)
                e -= 2.0 * M_PI;
            cmd_vel.header.stamp = ros::Time::now();
            twist_cmd.linear.x = cmd_vel.transform.translation.x = max_vel - kv * fabs(e);
            twist_cmd.linear.y = cmd_vel.transform.translation.y = max_vel - kv * fabs(e);
            twist_cmd.linear.z = cmd_vel.transform.translation.z = max_vel - kv * fabs(e);
            twist_cmd.angular.x = cmd_vel.transform.rotation.x = 0.0;
            twist_cmd.angular.y = cmd_vel.transform.rotation.y = 0.0;
            twist_cmd.angular.z = cmd_vel.transform.rotation.z = 0.2 * e + 0.01 * eo;
            cmd_vel.transform.rotation.w = 5.0; // this is used as priority
            eo = e;
        }
        // publish commands
        cmd_pub.publish(cmd_vel);
        if (publish_twist)
            twist_pub.publish(twist_cmd);
        // print debug message
        if (target_path_index >= 0)
        {
            printf("robot pose: x = %.3lf [m], y = %.3lf [m], z = %.3lf, yaw = %.3lf [deg]\n", x, y, z, yaw * 180.0 / M_PI);
            printf("target path point: x = %.3lf [m], y = %.3lf [m], z = %.3lf [m]\n",
                path.poses[target_path_index].pose.position.x, path.poses[target_path_index].pose.position.y, path.poses[target_path_index].pose.position.z);
            printf("target path index = %d, path point num = %d\n", target_path_index, (int)path.poses.size());
            printf("twist command: x = %.3lf [m/sec], y = %.3lf [m/sec], z = %.3lf [m/sec], ang_vel = %.3lf [rad/sec], priority = %.1f\n",
                twist_cmd.linear.x, twist_cmd.linear.y, twist_cmd.linear.z, twist_cmd.angular.z, cmd_vel.transform.rotation.w);
            printf("param: stop = %d, iterate_following = %d, publish_twist = %d\n", stop, iterate_following, publish_twist);
            printf("\n");
        }
        else
        {
            printf("arrived at the end of the path.\n");
            printf("robot pose: x = %.3lf [m], y = %.3lf [m], z = %.3lf, yaw = %.3lf [deg]\n", x, y, z, yaw * 180.0 / M_PI);
            printf("param: stop = %d, iterate_following = %d, publish_twist = %d\n", stop, iterate_following, publish_twist);
            printf("\n");
        }
        // check iteration for path following
        // nh.getParam("/path_follower/iterate_following", iterate_following);
        if (target_path_index < 0 && (int)path.poses.size() != 0 && iterate_following)
            prev_nearest_path_index = 0;
        // sleep
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_follower");
    PathFollower node;
    node.spin();
    return 0;
}
