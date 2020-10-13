#ifndef MULTI_TURTLEBOT3_SIMULATION_NAVIGATOR
#define MULTI_TURTLEBOT3_SIMULATION_NAVIGATOR

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <math.h>

namespace multi_turtlebot3_navigation
{

class Navigator 
{ 
    public: 
      Navigator(const std::string s_robot_namespace, 
                          const std::string s_global_frame, 
                          const std::string s_robot_base_frame, 
                          const std::string s_planner)
      {
        robot_namespace  = s_robot_namespace;
        robot_base_frame = s_global_frame;
        global_frame     = s_robot_base_frame;
        planner          = s_planner;

        // calculation robot's location
        this->getRobotPose(&robot_pose);
      }

      double getDistance(const geometry_msgs::PoseStamped poseStamped) 
      { 
        // Get the path from the robot to the pose
        ROS_INFO("Get the path from the robot to the pose...");
        nav_msgs::GetPlan getPlan;
        getPlan.request.start     = robot_pose;
        getPlan.request.goal      = poseStamped;
        getPlan.request.tolerance = .5;

        n.serviceClient<nav_msgs::GetPlan>("/" + robot_namespace + "/" + planner).call(getPlan);
        nav_msgs::Path path = getPlan.response.plan;

        // Calculating path length of plan
        ROS_INFO("Calculating path length of plan...");
        double path_length = 0;

        if (path.poses.size() < 2) { return path_length; } // Prevent segmentation fault
        else {
          for (int i = 0; i < path.poses.size() - 1; i++) {
            double position_a_x = path.poses[i].pose.position.x;
            double position_b_x = path.poses[i+1].pose.position.x;
            double position_a_y = path.poses[i].pose.position.y;
            double position_b_y = path.poses[i+1].pose.position.y;

            path_length += sqrt(pow((position_b_x - position_a_x), 2) + pow((position_b_y - position_a_y), 2));
          }

          return path_length;
        } 
      } 

    protected:
      ros::NodeHandle n;

      std::string robot_namespace  = "";
      std::string global_frame     = "map"; 
      std::string robot_base_frame = "base_footprint";
      std::string planner          = "move_base/NavfnROS/make_plan";

      // the robot's location
      geometry_msgs::PoseStamped robot_pose;
      void getRobotPose(geometry_msgs::PoseStamped* s_robot_pose)
      {
        // Get robot's pose
        ROS_INFO("Get robot's pose from %s/%s to %s", robot_namespace.c_str(), robot_base_frame.c_str(), global_frame.c_str());
        tf::StampedTransform robot_transform = this->getTransform(global_frame, robot_namespace + "/" + robot_base_frame);

        ROS_INFO("Transform received!");
        ROS_INFO("transform.frame_id: %s", robot_transform.frame_id_.c_str());
        ROS_INFO("transform.stamp: %f", robot_transform.stamp_.toSec());

        *s_robot_pose = this->toPoseStamped(robot_transform);
      }

      // get transform data from tf
      tf::StampedTransform getTransform(const std::string target_frame, const std::string source_frame)
      {
        tf::TransformListener listener;
        tf::StampedTransform transform;

        while (n.ok()){
          try{
            listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
          }
          catch (tf::TransformException ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(0.5).sleep();
            continue;
          }
          return transform;
        }
      }

      // get Pose Stamed from Stamped Transform
      geometry_msgs::PoseStamped toPoseStamped(const tf::StampedTransform transform)
      {
        geometry_msgs::PoseStamped poseStamped;

        // Assign header
        poseStamped.header.frame_id = transform.frame_id_;
        poseStamped.header.stamp    = transform.stamp_;

        // Assign position
        poseStamped.pose.position.x = transform.getOrigin().x();
        poseStamped.pose.position.y = transform.getOrigin().y();
        poseStamped.pose.position.z = transform.getOrigin().z();
        
        // Assign orientation
        poseStamped.pose.orientation.x = transform.getRotation().x();
        poseStamped.pose.orientation.y = transform.getRotation().y();
        poseStamped.pose.orientation.z = transform.getRotation().z();
        poseStamped.pose.orientation.w = transform.getRotation().w();

        return poseStamped;
      }
};
}

#endif