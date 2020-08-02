#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double x_cor = 0.0;
double y_cor = 0.0;
double z_cor = 0.0;
double w_orientation = 0.0;


int main(int argc, char** argv){
  ros::init(argc, argv, "send_navigation_goal");
  ros::param::get("/linorobot/x_cor", x_cor);
  ros::param::get("/linorobot/y_cor", y_cor);
  ros::param::get("/linorobot/w_orient", w_orientation);


  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  //kasih goal ke move base based on the point yang kita mau
  goal.target_pose.pose.position.x = x_cor; //specify x goal
  goal.target_pose.pose.position.y = y_cor; //specify y goal
  goal.target_pose.pose.position.z =  0.081;
  goal.target_pose.pose.orientation.w = w_orientation;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot moved to point: (%.2f, %.2f)", goal.target_pose.pose.position.x , goal.target_pose.pose.position.y);
  else
    ROS_INFO("The robot failed to achieve it's goal, muehehe");

  return 0;
}
