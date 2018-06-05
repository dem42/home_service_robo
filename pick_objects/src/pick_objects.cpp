#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <unistd.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_goal;

  // set up the frame parameters
  pickup_goal.target_pose.header.frame_id = "map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickup_goal.target_pose.pose.position.x = 3.5;
  pickup_goal.target_pose.pose.position.y = 4.0;
  pickup_goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(pickup_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("[ROBOT] Reached pickup goal");
  else
    ROS_INFO("[ROBOT] Failed to reach pickup goal");
  
  ac.stopTrackingGoal();
  
  // sleep for 5s
  sleep(5);
  // Drop off
  move_base_msgs::MoveBaseGoal dropoff_goal;
  
  dropoff_goal.target_pose.header.frame_id = "map";
  dropoff_goal.target_pose.header.stamp = ros::Time::now();
  dropoff_goal.target_pose.pose.position.x = -3.5;
  dropoff_goal.target_pose.pose.position.y = 0.0;
  dropoff_goal.target_pose.pose.orientation.w = 1.0;
  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(dropoff_goal);
  // Wait an infinite time for the results
  ac.waitForResult();
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("[ROBOT] Reached dropoff goal");
  else
    ROS_INFO("[ROBOT] Failed to reach dropoff goal");

  // wait for ROS to be shut down
  while (ros::ok()) {
    sleep(1);
  }
  
  return 0;
}
