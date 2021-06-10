/****************************************************************************
 * 経路追従
 * 速度をPID制御でコントロール
 →目標値と現在位置をsubscribe
 →速度をpublish
 ****************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define kp 3
#define kp_yaw 1
#define ki 0
#define ki_yaw 0
#define kd 5
#define kd_yaw 2
#define hz 10
#define time 1/hz
#define vx_const 1
#define vy_const 1
#define vz_const 1
#define vyaw_const 1

class path_follower
{
private:
    ros::NodeHandle nh;
    ros::Subscriber position_sub, path_sub;
    ros::Publisher position_pub, piloting_pub;
    tf::TransformListener tf_listener;

    float drone_p_x, drone_p_y, drone_p_z, drone_p_yaw; 
    int path_len;
    double path_data[100][7];
    double current_position[7];

public:
    path_follower(void);
    ~path_follower(){};
    void reset(void);
    void check_path(void);
    void get_path_CB(const nav_msgs::Path::ConstPtr &msg);
    void set_goal(void);
    void controller(void);
};

path_follower::path_follower():
    nh("~"),
    tf_listener()
{
    // subscriber
    path_sub = nh.subscribe("/target_path", 1, &path_follower::get_path_CB, this); // 経路データを受け取る
    // publisher
    piloting_pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
}

void path_follower::reset(void) //変数の初期化
{   
    current_position[0] = current_position[1] = current_position[2] = current_position[3] = current_position[4] = current_position[5] = current_position[6] = 0;
}

void path_follower::get_path_CB(const nav_msgs::Path::ConstPtr &msg)   //path_serverから経路データの受け取り
{
    std::cout << "---Path---\n";
    nav_msgs::Path path;
    path = *msg;
    path_len = path.poses.size();
    for (int i = 0; i < path_len; i++){
    	path_data[i][0] = path.poses[i].pose.position.x;
    	path_data[i][1] = path.poses[i].pose.position.y; 
    	path_data[i][2] = 0;
    	path_data[i][3] = path.poses[i].pose.orientation.x; 
    	path_data[i][4] = path.poses[i].pose.orientation.y;
    	path_data[i][5] = path.poses[i].pose.orientation.z; 
    	path_data[i][6] = path.poses[i].pose.orientation.w;
    }
}

void path_follower::check_path(void)
{
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ros::spinOnce();

		if (path_len != 0)
		{
            std::cout << "Path OK\n";
			break;
		}
        else
        {
            ROS_ERROR("Failure to get path");
        }
		loop_rate.sleep();
	}
}

int calculate_dis(double current_position[6], double path_data[][7], int path_len)  //一番近い経路上の点を見つける
{
    int minimum_num = 0;
    float minimum_dis = 100000;
    
   for (int i = 0; i < path_len; i++)
    {
        float dis = std::pow(path_data[i][0] - current_position[0], 2.0) + std::pow(path_data[i][1] - current_position[1], 2.0) + std::pow(path_data[i][2] - current_position[2], 2.0);
        
        if (minimum_dis > dis) 
        {
            minimum_dis = dis;
            minimum_num = i;
        }
    }
    std::cout << "Current Point Num : " << minimum_num << "\n";
    
    return minimum_num;
}

int get_candidate(double path_data[][7], int minimum_num, int path_len)    //0.1m先の点をgoalに設定
{
    float dis_goal_max = 0;
    int goal_num = minimum_num;
    int i = minimum_num;
    
    for (int i = minimum_num; i < path_len; i++)
    {
        float dis_goal_squ = std::pow(path_data[i][0] - path_data[minimum_num][0], 2.0) + std::pow(path_data[i][1] - path_data[minimum_num][1], 2.0) + std::pow(path_data[i][2] - path_data[minimum_num][2], 2.0);
        float dis_goal = std::pow(dis_goal_squ, 0.5);
         
        if (dis_goal < 0.1 && dis_goal > dis_goal_max)
        {
            dis_goal_max = dis_goal;
            goal_num = i;
        }
    }
    std::cout << "Goal Point Num : " << goal_num << "\n";
    
    return goal_num;
}

float check_angular(double current_position[7], double path_data[][7], int goal_num)    //yaw方向の差分計算
{   
    float yaw_goal, yaw_now;

    float E_11_g = std::pow(path_data[goal_num][3], 2) - std::pow(path_data[goal_num][4], 2) - std::pow(path_data[goal_num][5], 2) + std::pow(path_data[goal_num][6], 2);
    float E_12_g = 2 * (path_data[goal_num][3] * path_data[goal_num][6] + path_data[goal_num][4] * path_data[goal_num][5]);
    
    if (E_11_g != 0){
        yaw_goal = std::atan(E_12_g / E_11_g);
    } else {
        yaw_goal = 0;
    }
    
    float E_11_n = std::pow(current_position[3], 2) - std::pow(current_position[4], 2) - std::pow(current_position[5], 2) + std::pow(current_position[6], 2);
    float E_12_n = 2 * (current_position[3] * current_position[6] + current_position[4] * current_position[5]);

    if (E_11_n != 0){
        yaw_now = std::atan(E_12_n / E_11_n);
    } else {
        yaw_now = 0;
    }
    
    float diff_yaw = yaw_goal - yaw_now;

    return diff_yaw;
}

void path_follower::controller(void)
{
    int minimum_num, goal_num;
    float diff_yaw, prev_diff_x, prev_diff_y, prev_diff_z, prev_diff_yaw;

    prev_diff_x = prev_diff_y = prev_diff_z = prev_diff_yaw = 0;

    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        ros::spinOnce();
        
        geometry_msgs::PoseWithCovariance cmd_position;
        geometry_msgs::Twist cmd_twist;
        cmd_twist.linear.x =cmd_twist.linear.y =cmd_twist.linear.z = 0.0;
        cmd_twist.angular.x = cmd_twist.angular.y = cmd_twist.angular.z = 0.0;

        tf::StampedTransform map2base_link;
        ros::Time now = ros::Time::now();
        try     //tfの/base_link2を読む
        {
            tf_listener.waitForTransform("/map", "/base_link2", now, ros::Duration(1.0));
            tf_listener.lookupTransform("/map", "/base_link2", now, map2base_link);
        }
        catch (tf::TransformException ex)
        {
            cmd_twist.linear.x = 0.0;
            cmd_twist.linear.y = 0.0;
            cmd_twist.linear.z = 0.0;
            cmd_twist.angular.z = 0.0;
            piloting_pub.publish(cmd_twist);

            ROS_ERROR("%s", ex.what());
            continue;
        }
        current_position[0] = map2base_link.getOrigin().x();
        current_position[1] = map2base_link.getOrigin().y();
		current_position[2] = map2base_link.getOrigin().z();
        current_position[3] = map2base_link.getRotation().x();
        current_position[4] = map2base_link.getRotation().y();
        current_position[5] = map2base_link.getRotation().z();
        current_position[6] = map2base_link.getRotation().w();

        minimum_num = calculate_dis(current_position, path_data, path_len);     //一番近い経路上の点を見つける
        goal_num = get_candidate(path_data, minimum_num, path_len);             //目標の設定
        diff_yaw = check_angular(current_position, path_data, goal_num);        //yaw方向の差分計算

        float diff_x = path_data[goal_num][0] - current_position[0];
        float diff_y = path_data[goal_num][1] - current_position[1];
        float diff_z = path_data[goal_num][2] - current_position[2];
        float integral_x = 0;
        float integral_y = 0;
        float integral_z = 0;
        float integral_yaw = 0;
        integral_x += (diff_x + prev_diff_x) * time / 2;
        integral_y += (diff_y + prev_diff_y) * time / 2;
        integral_z += (diff_z + prev_diff_z) * time / 2;
        integral_yaw += (diff_yaw + prev_diff_yaw) * time / 2;

        float p_x = kp * diff_x;
        float p_y = kp * diff_y;
        float p_z = kp * diff_z;
        float p_yaw = kp_yaw * diff_yaw;
        float i_x = ki * integral_x;
        float i_y = ki * integral_y;
        float i_z = ki * integral_z;
        float i_yaw = ki_yaw * integral_yaw;
        float d_x = kd * (diff_x - prev_diff_x) / time;
        float d_y = kd * (diff_y - prev_diff_y) / time;
        float d_z = kd * (diff_z - prev_diff_z) / time;
        float d_yaw = kd_yaw * (diff_yaw - prev_diff_yaw) / time;
 
        float vx_coef = p_x + i_x + d_x;
        float vy_coef = p_y + i_y + d_y;
        float vz_coef = p_z + i_z + d_z;
        float vyaw_coef = p_yaw + i_yaw + d_yaw;

        float vx = vx_coef * vx_const;
        float vy = vy_coef * vy_const;
        float vz = vz_coef * vz_const;
        float vyaw = vyaw_coef * vyaw_const;

        if (vx > 0.15)
        {
            vx = 0.15;
        }
        if (vy > 0.15)
        {
            vy = 0.15;
        }
        if (vz > 0.15)
        {
            vz = 0.15;
        }
        if (vyaw > 0.15)
        {
            vyaw = 0.15;
        }
        
        if (vx < -0.15)
        {
            vx = -0.15;
        }
        if (vy < -0.15)
        {
            vy = -0.15;
        }
        if (vz < -0.15)
        {
            vz = -0.15;
        }
        if (vyaw < -0.15)
        {
            vyaw = -0.15;
        }

        //現在の値を過去の値として保存
        prev_diff_x = diff_x;
        prev_diff_y = diff_y;
        prev_diff_z = diff_z;
        prev_diff_yaw = diff_yaw;

        //cmd_xxx
        cmd_twist.linear.x = vx;
        cmd_twist.linear.y = vy;
        cmd_twist.linear.z = vz;
        cmd_twist.angular.z = vyaw;

        //publish
        piloting_pub.publish(cmd_twist);

        //sleep
        loop_rate.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");

    printf("Start Bebop-Drone2 Controller\n");

    path_follower node;
    node.reset();
    node.check_path();
    node.controller();

    return 0;
}
