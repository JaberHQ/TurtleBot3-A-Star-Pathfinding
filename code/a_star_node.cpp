#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include "a_star.h"
#include "d_star_lite.h"

/* Implementation of A* / D* algorithm using ROS
   This class integrates the A* / D* algorithm with the ROS library.
   
   Switch AStar to DStarLite and vice versa
*/
class AStarNode 
{
public:
    AStarNode(ros::NodeHandle& nh) 
        : m_aStar(1, 1), m_have_map(false), m_have_odom(false), m_following(false) 
    {
        /* Subscribe */
        m_map_sub = nh.subscribe("/map", 1, &AStarNode::MapCallback, this); // Subscribe to map
        m_odom_sub = nh.subscribe("/odom", 1, &AStarNode::OdomCallback, this);
        m_ball_sub = nh.subscribe("/ball_position", 1, &AStarNode::BallCallback, this);

        /* Publish */
        m_path_pub = nh.advertise<nav_msgs::Path>("a_star_path", 1);
        m_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        m_plan_timer = nh.createTimer(ros::Duration(0.5), &AStarNode::PlanTimerCallback, this);
    }

    void BallCallback(const geometry_msgs::Point::ConstPtr& msg) 
    {
        // Update A* goal with ball position
        SetGoal(msg->x, msg->y);
        ROS_INFO("Updated goal to ball position: (%f, %f)", msg->x, msg->y);
    }


    /* Odometry */
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
    {
        m_current_x = msg->pose.pose.position.x;
        m_current_y = msg->pose.pose.position.y;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        m_current_yaw = yaw;

        m_have_odom = true;
    }

    /* Build A* grid based on map size */
    void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
    {
        m_aStar = DStarLite(msg->info.width, msg->info.height);
        m_map_width = msg->info.width;
        m_map_height = msg->info.height;
        m_map_resolution = msg->info.resolution;
        m_map_origin_x = msg->info.origin.position.x;
        m_map_origin_y = msg->info.origin.position.y;

        for (unsigned int y = 0; y < msg->info.height; y++) 
        {
            for (unsigned int x = 0; x < msg->info.width; x++) 
            {
                int i = y * msg->info.width + x;
                if (msg->data[i] > 50) {
                    m_aStar.SetObstacle(x, y);
                }
            }
        }

        m_have_map = true;
    }


    void PlanTimerCallback(const ros::TimerEvent&) 
    {
       
        if (!m_have_map || !m_have_odom || m_following)
            return;

        geometry_msgs::Point start, goal;
        start.x = (m_current_x - m_map_origin_x) / m_map_resolution;
        start.y = (m_current_y - m_map_origin_y) / m_map_resolution;

        goal.x = (m_goal_x - m_map_origin_x) / m_map_resolution;
        goal.y = (m_goal_y - m_map_origin_y) / m_map_resolution;

        std::vector<geometry_msgs::Point> path = m_aStar.FindPath(start, goal);
        ROS_INFO("Planned path size: %zu", path.size());

        nav_msgs::Path ros_path;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";

        for (size_t i = 0; i < path.size(); ++i) 
        {
            geometry_msgs::PoseStamped pose;
            pose.header = ros_path.header;
            pose.pose.position.x = path[i].x * m_map_resolution + m_map_origin_x;
            pose.pose.position.y = path[i].y * m_map_resolution + m_map_origin_y;
            pose.pose.orientation.w = 1.0;
            ros_path.poses.push_back(pose);
        }

        m_path_pub.publish(ros_path);

        if (!ros_path.poses.empty()) 
        {
            m_following = true;
            FollowPath(ros_path);
            m_following = false;
        }
    }


    void FollowPath(const nav_msgs::Path& ros_path)
    {
        ros::Rate rate(10);
        geometry_msgs::Twist cmd;

        for (size_t i = 0; i < ros_path.poses.size(); i++)
        {
            double target_x = ros_path.poses[i].pose.position.x;
            double target_y = ros_path.poses[i].pose.position.y;

            while (ros::ok())
            {
                double dx = target_x - m_current_x;
                double dy = target_y - m_current_y;
                double distance = std::sqrt(dx*dx + dy*dy);
                double target_yaw = std::atan2(dy, dx);
                double yaw_error = target_yaw - m_current_yaw;

                while (yaw_error > M_PI) yaw_error -= 2*M_PI;
                while (yaw_error < -M_PI) yaw_error += 2*M_PI;

                if (distance < 0.1) // Reached waypoint
                {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    m_cmd_pub.publish(cmd);

                    if (i == ros_path.poses.size() - 1) // Final waypoint
                    {
                        ROS_INFO("Reached ball location, will keep chasing if it moves");
                        return; 
                    }

                    break; 
                }

                float maxSpeed = 1.3f;

                cmd.linear.x = maxSpeed * distance;
                cmd.angular.z = 1.0 * yaw_error;

                if (cmd.linear.x > maxSpeed)
                    cmd.linear.x = maxSpeed;
                if (cmd.angular.z > 1.0) 
                    cmd.angular.z = 1.0;
                if (cmd.angular.z < -1.0) 
                    cmd.angular.z = -1.0;

                m_cmd_pub.publish(cmd);
                ros::spinOnce();
                rate.sleep();
            }
        }
    }



    void SetGoal(const double goal_x, const double goal_y) 
    {
        m_goal_x = goal_x;
        m_goal_y = goal_y;
    }

    double GetGoal_X() const { return m_goal_x; }
    double GetGoal_Y() const { return m_goal_y; }

private:
    ros::Subscriber m_map_sub;
    ros::Subscriber m_odom_sub;
    ros::Subscriber m_ball_sub;
    ros::Publisher m_path_pub;
    ros::Publisher m_cmd_pub;
    ros::Timer m_plan_timer;
    ros::Time m_goal_reached_time;
    bool m_waiting_at_goal = false;

    DStarLite m_aStar;
    int m_map_width, m_map_height;
    double m_map_resolution, m_map_origin_x, m_map_origin_y;
    double m_goal_x = 0;
    double m_goal_y = 0;
    double m_current_x, m_current_y, m_current_yaw;
    bool m_have_map;
    bool m_have_odom;
    bool m_following;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "a_star_node");
    ros::NodeHandle nh;
    AStarNode node(nh);
    ros::spin();
    return 0;
}
