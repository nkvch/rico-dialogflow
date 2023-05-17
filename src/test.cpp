#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_msgs/SaySentenceAction.h>

typedef actionlib::SimpleActionClient<tiago_msgs::SaySentenceAction> SaySentenceClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "say_sentence_client");
  ros::NodeHandle nh;

  // Create the action client
  SaySentenceClient ac("rico_says", true);

  // Wait for the action server to start
  ROS_INFO("Waiting for action server to start...");
  ac.waitForServer();

  // Send the goal to the action server
  tiago_msgs::SaySentenceGoal goal;
  goal.sentence = "niekorzystne warunki pogodowe mogą wpłynąć na bezpieczeństwo";
  ROS_INFO("Sending goal to server...");
  ac.sendGoal(goal);

  // Wait for the result
  ac.waitForResult();

  // Print the result
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    tiago_msgs::SaySentenceResultConstPtr result = ac.getResult();
    ROS_INFO("Result: %s", result->success ? "true" : "false");
  }
  else
  {
    ROS_ERROR("Failed to execute action");
  }

  return 0;
}
