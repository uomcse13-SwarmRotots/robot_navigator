#include "ros/ros.h"
#include "std_msgs/String.h"
#include "controller/keyboard_teleop.h"

#include <sstream>
#include <iostream>

TurtlebotTeleop* turtlebot_teleop; 

void startKeyboardTeleop(std::string cmd_vel_topic){
    ROS_INFO("Keyboard stearing stared...");          
    
    boost::thread my_thread(boost::bind(&TurtlebotTeleop::keyLoop_cmd_vel, turtlebot_teleop));      
   
    my_thread.interrupt() ;
    my_thread.join() ;  
    
    ROS_INFO("Keyboard stearing ended...");
}

void showHelp(){
                   
  ROS_INFO("------Robot contoller-----");
  ROS_INFO("exit - exit");
  ROS_INFO("teleop - keyboard control");
  ROS_INFO("rqt - rqt");
  ROS_INFO("auto - automate robot navigation");  
  ROS_INFO("rotate - rotate the robot");
  ROS_INFO("stop - stop automate robot navigation,rotation");
  ROS_INFO("------Robot contoller-----");

}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_remote_controller");

  ros::NodeHandle n;

  /*
    Patameters-----------------------------
  */
  std::string cmd_vel_topic;
  std::string robot_controller_topic;

  ros::NodeHandle private_nh("~");     
  // private_nh.param("cmd_vel_topic", cmd_vel_topic, std::string("/mobile_base/commands/velocity")); 
  private_nh.param("cmd_vel_topic", cmd_vel_topic, std::string("/cmd_vel"));
  private_nh.param("robot_controller_topic", robot_controller_topic, std::string("robot_controller")); 

  /*
    ----------------------------------------
  */
  turtlebot_teleop = new TurtlebotTeleop(cmd_vel_topic);


  /*
    Publishers------------------------------
  */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>(robot_controller_topic, 1000);


  /*
    ----------------------------------------
  */
  ros::Rate loop_rate(10);
 
  int count = 0;
  while (ros::ok())
  {
   
    std_msgs::String msg;

    std::stringstream ss;
    std::cout << "robot controller: ";
    std::cin >> msg.data;    

    if(msg.data=="exit")
        break;
    else if(msg.data=="teleop")
        startKeyboardTeleop(cmd_vel_topic);
    else if(msg.data=="help")
        showHelp();
    else if(msg.data=="rqt")
        system("rqt");
    else{
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
  }


  return 0;
}