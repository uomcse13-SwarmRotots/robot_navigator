#include "ros/ros.h"
#include "std_msgs/String.h"
#include "controller/keyboard_teleop.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <controller/cmd_val_controller.h>
#include <planner/navigation_planner.h>
#include <support/path_visualizer.h>



/*
------------------------------------------------
*/
swarm_navigator::CmdValController* controller;
NavigationPlanner* navigation_planner;
Visualizer* visualizer;

/*
----subcribers----------------------------------
*/

ros::Subscriber robot_controller_sub;
ros::Subscriber goal_sub;

/*
----publishers----------------------------------
*/

ros::Publisher vis_pub;

/*
----threads-------------------------------------
*/

boost::thread* controller_thread = NULL;

/*
------------------------------------------------
*/

void autoDrive(){
  // stop = false;
  while(true){
    controller->driveForward(10);
  }

  // while(true){
  //     // get robot pose
  //     tf::Stamped<tf::Pose> global_pose;
  //     controller->getRobotPose(global_pose);
  //     geometry_msgs::PoseStamped current_position;
  //     tf::poseStampedTFToMsg(global_pose, current_position);

  //     //get plan
  //     std::vector<geometry_msgs::PoseStamped> plan = 
  //                     navigation_planner->getNavPlan(current_position);
  //     ROS_INFO("Done planning....");

      
  //     visualizer->showPath(plan);
  //     controller->followPath(plan);
  // }
}

void rotateRobot(){
  //ROS_INFO("%f",M_PI);
  controller->turn(M_PI,true);
  controller->turn(M_PI,true);
  controller->turn(M_PI,true);
  controller->turn(M_PI,true);
  controller->turn(M_PI,true);
  controller->turn(M_PI,true);
}

void stop(){
  if(controller_thread!=NULL)
    controller_thread->interrupt();
  controller_thread = NULL;
  controller->stop();
}



/*
-----callbacks-----------------------------------
*/

void controlCallback(const std_msgs::String::ConstPtr& msg){

  ROS_INFO("I heard: [%s]", msg->data.c_str());
  if(msg->data=="auto"){
    if(controller_thread==NULL)
      controller_thread = new boost::thread(boost::bind(autoDrive));
  }else if(msg->data=="stop"){
    stop();
  }
  else if(msg->data=="rotate"){
    stop();
    if(controller_thread==NULL)
      controller_thread = new boost::thread(boost::bind(rotateRobot));
  }else if(msg->data=="exit"){
    //stop();
  }

}



int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "robot_controller");
  ros::NodeHandle nh;
  ros::NodeHandle simple_nh("move_base_simple");

  /*
    Patameters-----------------------------
  */
  std::string topic_cmd_val;
  std::string topic_odom;
  std::string base_link;
  std::string odom_link;
  std::string topic_octomap;

  ros::NodeHandle private_nh("~");     
  private_nh.param("cmd_vel_topic", topic_cmd_val, std::string("/cmd_vel")); 
  private_nh.param("odom_topic", topic_odom, std::string("/odom")); 
  private_nh.param("base_link", base_link, std::string("base_footprint")); 
  private_nh.param("odom_link", odom_link, std::string("odom")); 
  private_nh.param("topic_octomap",topic_octomap, std::string("/octomap_point_cloud_centers"));

  /*
  ------------------------------------------
  */

  /*
  ----subcribers----------------------------
  */
  robot_controller_sub = nh.subscribe("robot_controller", 1000, controlCallback);  
  //goal_sub = simple_nh.subscribe("goal", 1, callback);

  /*
  ----publishers----------------------------
  */

  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  /*
  ------------------------------------------
  */

  controller = new swarm_navigator::CmdValController(nh,topic_cmd_val,base_link,odom_link);
  navigation_planner = new NavigationPlanner(nh,topic_octomap);
  visualizer = new Visualizer(vis_pub,odom_link);

  ros::spin();

  return 0;
}