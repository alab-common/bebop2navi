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
#include <visualization_msgs/Marker.h>

class PathFollower
{
private:
    ros::NodeHandle nh;
    ros::Subscriber path_sub;
    ros::Publisher cmd_pub, twist_pub, target_point_marker_pub;
    std::string map_frame, base_link_frame;
    std::string input_path_topic_name, output_cmd_topic_name;
    nav_msgs::Path path;
    tf::TransformListener tf_listener;
    double look_ahead_dist, max_linear_vel, max_angular_vel;
    double bvy, bvz, bvyaw, pgy, dgy, pgz, dgz, pgyaw, dgyaw;
    double cmd_publish_hz, displacement_threshold, translation_scale;
    bool iterate_following, publish_twist;
    int nearest_path_index, prev_nearest_path_index;

public:
    PathFollower();
    ~PathFollower() {};
    void path_callback(const nav_msgs::Path::ConstPtr& msg);
    void spin(void);
    double check_velocity(double v, double vmax);
};

PathFollower::PathFollower():
    nh("~"),
    map_frame("/map"),
    base_link_frame("/base_link2"),
    input_path_topic_name("/target_path"),
    output_cmd_topic_name("/cmd_vel_path_follow"),
    look_ahead_dist(0.05),
    max_linear_vel(0.1),
    max_angular_vel(0.1),
    bvy(1.0),
    bvz(1.0),
    bvyaw(1.0),
    pgy(0.4),
    dgy(0.05),
    pgz(0.4),
    dgz(0.05),
    pgyaw(0.25),
    dgyaw(0.05),
    cmd_publish_hz(20.0),
    displacement_threshold(1.0),
    translation_scale(1.0),
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
    nh.param("max_linear_vel", max_linear_vel, max_linear_vel);
    nh.param("max_angular_vel", max_angular_vel, max_angular_vel);
    nh.param("bvy", bvy, bvy);
    nh.param("bvz", bvz, bvz);
    nh.param("bvyaw", bvyaw, bvyaw);
    nh.param("pgy", pgy, pgy);
    nh.param("dgy", dgy, dgy);
    nh.param("pgz", pgz, pgz);
    nh.param("dgz", dgz, dgz);
    nh.param("pgyaw", pgyaw, pgyaw);
    nh.param("dgyaw", dgyaw, dgyaw);
    nh.param("cmd_publish_hz", cmd_publish_hz, cmd_publish_hz);
    nh.param("displacement_threshold", displacement_threshold, displacement_threshold);
    nh.param("translation_scale", translation_scale, translation_scale);
    nh.param("iterate_following", iterate_following, iterate_following);
    nh.param("publish_twist", publish_twist, publish_twist);
    // subscriber
    path_sub = nh.subscribe(input_path_topic_name, 1, &PathFollower::path_callback, this);
    // publisher
    cmd_pub = nh.advertise<geometry_msgs::TransformStamped>(output_cmd_topic_name, 10);
    target_point_marker_pub = nh.advertise<visualization_msgs::Marker>("/path_following_target_point", 10);
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
    prev_nearest_path_index = 0;
    double eyo = 0.0, ezo = 0.0, eyawo = 0.0;
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
            // publish zero velocities
            geometry_msgs::TransformStamped cmd_vel;
            geometry_msgs::Twist twist_cmd;
            cmd_vel.header.stamp = ros::Time::now();
            twist_cmd.linear.x = cmd_vel.transform.translation.x = 0.0;
            twist_cmd.linear.y = cmd_vel.transform.translation.y = 0.0;
            twist_cmd.linear.z = cmd_vel.transform.translation.z = 0.0;
            twist_cmd.angular.x = cmd_vel.transform.rotation.x = 0.0;
            twist_cmd.angular.y = cmd_vel.transform.rotation.y = 0.0;
            twist_cmd.angular.z = cmd_vel.transform.rotation.z = 0.0;
            cmd_vel.transform.rotation.w = 0.0; // this is used as priority
            cmd_pub.publish(cmd_vel);
            if (publish_twist)
                twist_pub.publish(twist_cmd);
            eyo = ezo = eyawo = 0.0;
            ROS_WARN("%s cannot be read from the tf tree. Published zero velocities.", base_link_frame.c_str());
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
            i0 = prev_nearest_path_index + 0;
            i1 = prev_nearest_path_index + 10;
            if (i0 < 1)
                i0 = 1;
            if (i1 >= path.poses.size())
                i1 = path.poses.size();
        }
        for (int i = i0; i < i1; i++)
        {
            double dx = path.poses[i].pose.position.x - x;
            double dy = path.poses[i].pose.position.y - y;
            double dz = path.poses[i].pose.position.z - z;
            double dl = sqrt(dx * dx + dy * dy + dz * dz);
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
        printf("nearest_path_index = %d\n", nearest_path_index);
        for (int i = nearest_path_index; i < path.poses.size(); i++)
        {
            double dx = path.poses[i].pose.position.x - x;
            double dy = path.poses[i].pose.position.y - y;
            double dz = path.poses[i].pose.position.z - z;
            double dl = sqrt(dx * dx + dy * dy + dz * dz);
            if (dl >= look_ahead_dist)
            {
                target_path_index = i;
                break;
            }
        }
        // calculate the control input
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
            eyo = ezo = eyawo = 0.0;
        }
        else
        {
            // compute deviation from the path and determine the control input based on PD control
            tf::Quaternion pq(path.poses[target_path_index].pose.orientation.x,
                path.poses[target_path_index].pose.orientation.y,
                path.poses[target_path_index].pose.orientation.z,
                path.poses[target_path_index].pose.orientation.w);
            double proll, ppitch, pyaw;
            tf::Matrix3x3 pm(pq);
            pm.getRPY(proll, ppitch, pyaw);
            double dx = path.poses[target_path_index].pose.position.x - x;
            double dy = path.poses[target_path_index].pose.position.y - y;
            double dz = path.poses[target_path_index].pose.position.z - z;
            double dl = sqrt(dx * dx + dy * dy + dz * dz);
            double dyaw = atan2(dy, dx);
            double dpitch = atan2(dz, sqrt(dx * dx + dy * dy));
            // double ex = dx * cos(pyaw) + dy * sin(pyaw);
            double ey = -dx * sin(pyaw) + dy * cos(pyaw);
            double ez = dz;
            double eyaw = pyaw - yaw;
            if (eyaw < -M_PI)
                eyaw += 2.0 * M_PI;
            if (eyaw > M_PI)
                eyaw -= 2.0 * M_PI;
            // double v = max_linear_vel - bvy * fabs(ey) - bvz * fabs(ez) - bvyaw * fabs(eyaw);
            double v = max_linear_vel - bvz * fabs(ez) - bvyaw * fabs(eyaw);
            if (v < 0.0)
                v = 0.0;
            double vx = v * cos(dyaw);
            double vy = v * sin(dyaw);
            double vz = pgz * ez + dgz * ezo;
            double wz = pgyaw * eyaw + dgyaw * eyawo;
            vx = check_velocity(vx, max_linear_vel);
            vy = check_velocity(vy, max_linear_vel);
            vz = check_velocity(vz, max_linear_vel);
            wz = check_velocity(wz, max_angular_vel);
            if (dl > displacement_threshold) {
                vx = vy = vz = wz = 0.0;
                ROS_WARN("The displacement for the path following exceeds the threshold (%.5lf [m])",
                    displacement_threshold);
            }
            cmd_vel.header.stamp = ros::Time::now();
            twist_cmd.linear.x = cmd_vel.transform.translation.x = vx;
            twist_cmd.linear.y = cmd_vel.transform.translation.y = vy;
            twist_cmd.linear.z = cmd_vel.transform.translation.z = vz;
            twist_cmd.angular.x = cmd_vel.transform.rotation.x = 0.0;
            twist_cmd.angular.y = cmd_vel.transform.rotation.y = 0.0;
            twist_cmd.angular.z = cmd_vel.transform.rotation.z = wz;
            cmd_vel.transform.rotation.w = 5.0; // this is used as priority
            eyo = ey;
            ezo = ez;
            eyawo = eyaw;
        }
        // publish commands
        cmd_pub.publish(cmd_vel);
        if (publish_twist)
            twist_pub.publish(twist_cmd);
        // publish target point marger
        if (target_path_index >= 0)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = map_frame;
            marker.header.stamp = ros::Time::now();
            marker.ns = "basic_shapes";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();
            marker.scale.x = 0.5 * translation_scale;
            marker.scale.y = 0.5 * translation_scale;
            marker.scale.z = 0.5 * translation_scale;
            marker.pose.position.x = path.poses[target_path_index].pose.position.x;
            marker.pose.position.y = path.poses[target_path_index].pose.position.y;
            marker.pose.position.z = path.poses[target_path_index].pose.position.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
            target_point_marker_pub.publish(marker);
        }
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

double PathFollower::check_velocity(double v, double vmax)
{
    if (std::isnan(v)) {
        ROS_WARN("nan velocity command is detected.");
        return 0.0;
    }
    if (v > vmax)
        v = vmax;
    if (v < -vmax)
        v = -vmax;
    return v;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_follower");
    PathFollower node;
    node.spin();
    return 0;
}
