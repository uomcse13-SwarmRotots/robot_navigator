#include "ros/ros.h"
#include "std_msgs/String.h"
#include "controller/keyboard_teleop.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <stdlib.h> 

#include <controller/cmd_val_controller.h>
#include <planner/navigation_planner.h>
#include <support/path_visualizer.h>



/*
------------------------------------------------
*/
swarm_navigator::CmdValController* controller;
NavigationPlanner* navigation_planner;
Visualizer* visualizer;
bool isGoalTesting = false;
bool isDirectGoalTesting = false;
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
  
  while(true){
      // get robot pose
      tf::Stamped<tf::Pose> global_pose;
      controller->getRobotPose(global_pose);
      geometry_msgs::PoseStamped current_position;
      tf::poseStampedTFToMsg(global_pose, current_position);

      //get plan
      ROS_INFO("Start planning....");
      std::vector<geometry_msgs::PoseStamped> plan = 
                      navigation_planner->getNavPlan(current_position);
      ROS_INFO("Done planning....");

      if(!plan.empty()){
        visualizer->showPath(plan);
        controller->followPath(plan);
      }
  }
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
  isGoalTesting = false;
  isDirectGoalTesting = false;
  if(controller_thread!=NULL){
    controller_thread->interrupt();
    free(controller_thread);
  }
  controller_thread = NULL;
  controller->stop();
}

void achieveGoal(const geometry_msgs::PoseStamped& goal){
    ROS_INFO("Goal Came....");    

    tf::Stamped<tf::Pose> global_pose;
    controller->getRobotPose(global_pose);
    geometry_msgs::PoseStamped current_position;

    tf::poseStampedTFToMsg(global_pose, current_position);

    //get plan
    ROS_INFO("Start planning....");
    std::vector<geometry_msgs::PoseStamped> plan = 
                    navigation_planner->getNavPlanToTarget(current_position,goal);
    ROS_INFO("Done planning....");
    // std::vector<geometry_msgs::PoseStamped> plan;    
   
    if(!plan.empty()){
      visualizer->showPath(plan);
      ROS_INFO("Done vitualizing....");
      controller->followPath(plan);
    }
   
}

void achieveDirectGoal(const geometry_msgs::PoseStamped& goal){
    ROS_INFO("Direct Goal Came....");
    controller->achieveGoal(goal);   
}


/*
-----callbacks-----------------------------------
*/

void controlCallback(const std_msgs::String::ConstPtr& msg){

  ROS_INFO("I heard: [%s]", msg->data.c_str());
  if(msg->data=="auto"){
    stop();
    if(controller_thread==NULL)
      controller_thread = new boost::thread(boost::bind(autoDrive));
  }
  else if(msg->data=="stop"){
    stop();
  }
  else if(msg->data=="rotate"){
    stop();
    if(controller_thread==NULL)
      controller_thread = new boost::thread(boost::bind(rotateRobot));
  }
  else if(msg->data=="goal"){
    stop();
    isGoalTesting = true;
  }
  else if(msg->data=="dgoal"){
    stop();
    isDirectGoalTesting = true;
  }
  //   else if(msg->data=="exit"){
  //   exit(0);
  // }

}

void goalTestCallback(const geometry_msgs::PoseStamped& goal)
{  
  if(isGoalTesting){  
    if(controller_thread!=NULL){
      controller_thread->interrupt();
      free(controller_thread);
    }
    controller_thread = NULL;
    controller->stop();
    if(controller_thread==NULL)
      controller_thread = new boost::thread(boost::bind(achieveGoal,goal));
  }else if(isDirectGoalTesting){    
    if(controller_thread!=NULL){
      controller_thread->interrupt();
      free(controller_thread);
    }
    controller_thread = NULL;
    controller->stop();
    if(controller_thread==NULL)
      controller_thread = new boost::thread(boost::bind(achieveDirectGoal,goal));
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
  double robot_dimention_width;
  double robot_dimention_height;
  int robot_area_x_min;
  int robot_area_x_max;
  int robot_area_y_min;
  int robot_area_y_max;

  int window_size;
  double min_distance;
  double max_distance;
  double min_slope;

  ros::NodeHandle private_nh("~");     
  private_nh.param("cmd_vel_topic", topic_cmd_val, std::string("/cmd_vel")); 
  private_nh.param("odom_topic", topic_odom, std::string("/odom")); 
  private_nh.param("base_link", base_link, std::string("base_footprint")); 
  private_nh.param("odom_link", odom_link, std::string("odom")); 
  private_nh.param("topic_octomap",topic_octomap, std::string("/octomap_point_cloud_centers"));
  private_nh.param("robot_dimention_width",robot_dimention_width, 0.3);
  private_nh.param("robot_dimention_height",robot_dimention_height, 0.3);
  private_nh.param("robot_area_x_min",robot_area_x_min, -20);
  private_nh.param("robot_area_x_max",robot_area_x_max, 20);
  private_nh.param("robot_area_y_min",robot_area_y_min, -20);
  private_nh.param("robot_area_y_max",robot_area_y_max, 20);

  private_nh.param("window_size",window_size, 20);
  private_nh.param("min_distance",min_distance, 0.1);
  private_nh.param("max_distance",max_distance, 3.0);
  private_nh.param("min_slope",min_slope, 0.1);

  /*
  ------------------------------------------
  */
  
   

  /*
  ----subcribers----------------------------
  */
  robot_controller_sub = nh.subscribe("robot_controller", 1000, controlCallback);  
  goal_sub = simple_nh.subscribe("goal", 1, goalTestCallback);

  /*
  ----publishers----------------------------
  */

  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  /*
  ------------------------------------------
  */


  
  controller = new swarm_navigator::CmdValController(nh,topic_cmd_val,base_link,odom_link);
  navigation_planner = new NavigationPlanner(nh,topic_octomap,robot_dimention_width
        ,robot_dimention_height,robot_area_x_min,robot_area_x_max,robot_area_y_min,robot_area_y_max);
  navigation_planner->setParametersForground(min_slope,min_distance,max_distance,window_size);
  navigation_planner->start();
  visualizer = new Visualizer(vis_pub,odom_link);

  ros::spin();

  return 0;
}