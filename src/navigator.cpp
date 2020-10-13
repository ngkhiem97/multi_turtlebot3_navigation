#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <multi_turtlebot3_navigation/navigator.h>
#include <multi_turtlebot3_navigation/task_distributor.h>
#include <map>
#include <thread>

// Store the received goals from /move_base_simple/goal topic
std::vector<geometry_msgs::PoseStamped> goals;
std::vector<std::string> robots;

void sendGoal(const std::string actionlib, const geometry_msgs::PoseStamped poseStamped)
{
  ROS_WARN("Begin sending goal!");
  //tell the action client that we want to spin a thread by default
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(actionlib, true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal move_base_goal;
  move_base_goal.target_pose.header.frame_id = "map";
  move_base_goal.target_pose.header.stamp    = ros::Time::now();

  move_base_goal.target_pose.pose.position.x = poseStamped.pose.position.x;
  move_base_goal.target_pose.pose.position.y = poseStamped.pose.position.y;
  move_base_goal.target_pose.pose.position.z = poseStamped.pose.position.z;
  move_base_goal.target_pose.pose.orientation.x = poseStamped.pose.orientation.x;
  move_base_goal.target_pose.pose.orientation.y = poseStamped.pose.orientation.y;
  move_base_goal.target_pose.pose.orientation.z = poseStamped.pose.orientation.z;
  move_base_goal.target_pose.pose.orientation.w = poseStamped.pose.orientation.w;

  ROS_INFO("Sending goal");
  ac.sendGoal(move_base_goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, naviagtion goal send");
  else
    ROS_INFO("Robot failed to move");
}

// Navigate multiple robots
void executeSeq()
{
  multi_turtlebot3_navigation::NavigationAlgorithmSimple simpleAlgorithm;
  multi_turtlebot3_navigation::TaskDistributor distributor(&simpleAlgorithm);

  ROS_INFO("Getting the plan!");
  std::multimap<std::string, geometry_msgs::PoseStamped> plan = distributor.run(robots, goals);
  ROS_INFO("Plan size: %d", (int) plan.size());

  std::vector<std::thread> threads;
  for (std::multimap<std::string, geometry_msgs::PoseStamped>::iterator it = plan.begin(); it != plan.end(); it++)
  {
    ROS_WARN("For plan: %s",it->first.c_str());
    std::thread thread(sendGoal, it->first + "/move_base", it->second);
    threads.push_back(std::move(thread));
  }

  for (std::vector<std::thread>::iterator it = threads.begin(); it != threads.end(); ++it)
  {
    if (it->joinable()) it->join();
  }
}

void setGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& poseStamped)
{
  goals.push_back(*poseStamped);

  if (goals.size() == robots.size())
  {
    executeSeq();

    ROS_WARN("Clearing the buffer!");
    // Clear the goals buffer
    goals.clear(); 
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

  robots.push_back("turtlebot01");
  robots.push_back("turtlebot02");
  robots.push_back("turtlebot03");

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