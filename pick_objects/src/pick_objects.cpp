#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static const float pickup[3] = {3.0, 5.0, 1.0};
static const float dropoff[3] = {-1.0, 0.0, 1.0};

static move_base_msgs::MoveBaseGoal createGoal(const float target[3]) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = target[0];
  goal.target_pose.pose.position.y = target[1];
  goal.target_pose.pose.orientation.w = target[2];

  return goal;
}

int main(int argc, char **argv)
{
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending Pick up goal");
  ac.sendGoal(createGoal(pickup));

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)

  {
    ROS_INFO("Robot successfully reached pick up");
    ros::Duration(5.0).sleep();
    //Go to drop off point
    ROS_INFO("Publishing drop-off goal");
    ac.sendGoal(createGoal(dropoff));
    // Wait an infinite time for the results
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Robot successfully reached drop-off goal");
      ros::Duration(5.0).sleep();
    }
    else
    {
      ROS_INFO("Robot failed to reach drop-off");
    }
  }
  else
  {
    ROS_INFO("Robot failed to reach pick up");
  }

  return 0;
}