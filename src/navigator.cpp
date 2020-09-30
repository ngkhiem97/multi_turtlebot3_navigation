#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

tf::StampedTransform getTransform(const std::string& target_frame, const std::string& source_frame)
{
  ros::NodeHandle n;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  while (n.ok()){
    try{
      listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    return transform;
  }
}

geometry_msgs::PoseStamped tfToPoseStamped(const tf::StampedTransform& transform)
{
  geometry_msgs::PoseStamped poseStamped;

  // Assign header
  poseStamped.header.frame_id = "map";
  poseStamped.header.stamp = ros::Time::now();

  // Assign position
  poseStamped.pose.position.x = transform.getOrigin().x();
  poseStamped.pose.position.y = transform.getOrigin().y();
  poseStamped.pose.position.z = transform.getOrigin().z();
  
  // Assign orientation
  poseStamped.pose.orientation.x = transform.getOrigin().x();
  poseStamped.pose.orientation.y = transform.getOrigin().y();
  poseStamped.pose.orientation.z = transform.getOrigin().z();
  poseStamped.pose.orientation.w = transform.getOrigin().w();

  return poseStamped;
}

double getDistance(const nav_msgs::Path& path)
{
  double path_length = 0;
  double position_a_x, position_b_x, position_a_y, position_b_y;
  if (path.poses.size() >= 2) {
    for (int i = 0; i < path.poses.size() - 1; i++) {
      position_a_x = path.poses[i].pose.position.x;
      position_b_x = path.poses[i+1].pose.position.x;
      position_a_y = path.poses[i].pose.position.y;
      position_b_y = path.poses[i+1].pose.position.y;

      path_length += sqrt(pow((position_b_x - position_a_x), 2) + pow((position_b_y - position_a_y), 2));
    }
  }
  ROS_INFO("Distance returned!");
  printf(std::to_string(path_length).c_str());
  return path_length;
}

void sendGoal(const std::string& actionlib, const geometry_msgs::PoseStamped::ConstPtr& poseStamped)
{
  //tell the action client that we want to spin a thread by default
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(actionlib, true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal move_base_goal;
  move_base_goal.target_pose.header.frame_id = "map";
  move_base_goal.target_pose.header.stamp    = ros::Time::now();

  move_base_goal.target_pose.pose.position.x = poseStamped->pose.position.x;
  move_base_goal.target_pose.pose.position.y = poseStamped->pose.position.y;
  move_base_goal.target_pose.pose.position.z = poseStamped->pose.position.z;
  move_base_goal.target_pose.pose.orientation.x = poseStamped->pose.orientation.x;
  move_base_goal.target_pose.pose.orientation.y = poseStamped->pose.orientation.y;
  move_base_goal.target_pose.pose.orientation.z = poseStamped->pose.orientation.z;
  move_base_goal.target_pose.pose.orientation.w = poseStamped->pose.orientation.w;

  ROS_INFO("Send navigation goal to:");
  ROS_INFO(actionlib.c_str());
  ac.sendGoal(move_base_goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, naviagtion goal send");
  else
    ROS_INFO("Robot failed to move");

}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void setGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& poseStamped)
{
  ROS_INFO("I heard a goal!!!");
  ROS_INFO("header.frame_id: [%s]", poseStamped->header.frame_id.c_str());
  ROS_INFO("pose.position.x: [%f]", poseStamped->pose.position.x);
  ROS_INFO("pose.position.y: [%f]", poseStamped->pose.position.y);
  ROS_INFO("pose.position.z: [%f]", poseStamped->pose.position.z);

  ros::NodeHandle n;

  /**
   * Loop three Turtlebots
   */ 

  // Get navigation plan of turtlebot01
  tf::StampedTransform transform_1 = getTransform("/map", "/turtle_1/base_footprint");
  nav_msgs::GetPlan getPlan01;
  getPlan01.request.start     = tfToPoseStamped(transform_1);
  getPlan01.request.goal      = *poseStamped;
  getPlan01.request.tolerance = .5;
  n.serviceClient<nav_msgs::GetPlan>("/turtle_1/move_base/NavfnROS/make_plan").call(getPlan01);

  ROS_INFO("Getting distance for turtle_1!!!");
  double d1 = getDistance(getPlan01.response.plan);

  // Get navigation plan of turtlebot02
  tf::StampedTransform transform_2 = getTransform("/map", "/turtle_2/base_footprint");
  nav_msgs::GetPlan getPlan02;
  getPlan02.request.start     = tfToPoseStamped(transform_2);
  getPlan02.request.goal      = *poseStamped;
  getPlan02.request.tolerance = .5;
  n.serviceClient<nav_msgs::GetPlan>("/turtle_2/move_base/NavfnROS/make_plan").call(getPlan02);

  ROS_INFO("Getting distance for turtle_2!!!");
  double d2 = getDistance(getPlan02.response.plan);

  /**
   * Send goal to shortest distance
   */ 
  if (d1 <= d2) {
    sendGoal("/turtle_1/move_base", poseStamped);
  } else {
    sendGoal("/turtle_2/move_base", poseStamped);
  }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "intermediate_navigator");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/move_base_simple/goal", 1000, setGoalCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}