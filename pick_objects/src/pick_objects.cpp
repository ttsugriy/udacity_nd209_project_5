#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static const float pickup_pose[3] = {3.5, 5.5, 1.0};
static const float dropoff_pose[3] = {-1.5, 0.0, 1.0};

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pickup_pose[0];
  goal.target_pose.pose.position.y = pickup_pose[1];
  goal.target_pose.pose.orientation.w = pickup_pose[2] ;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending Pick up goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     
    {
     ROS_INFO("Hooray, Robot reached PICK-UP......");
     ros::Duration(5.0).sleep();
     //Go to drop off point
     // Define a position and orientation for the robot to reach
     goal.target_pose.pose.position.x = dropoff_pose[0];
     goal.target_pose.pose.position.y = dropoff_pose[1];
     goal.target_pose.pose.orientation.w = dropoff_pose[2];
     // Send the goal position and orientation for the robot to reach
     ROS_INFO("Sending goal sending drop off goal");
     ac.sendGoal(goal);
     // Wait an infinite time for the results
     ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     {ROS_INFO("Hooray, Robot reached DROP OFF......");
     ros::Duration(5.0).sleep();}
  else
     {ROS_INFO("Robot failed to reach Drop off location for some reason");}
    }
  else
    {ROS_INFO("Robot failed to reach pick up location for some reason");}

  return 0;
}