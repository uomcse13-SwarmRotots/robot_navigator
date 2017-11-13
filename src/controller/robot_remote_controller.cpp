#include "ros/ros.h"
#include "std_msgs/String.h"
#include "controller/keyboard_teleop.h"

#include <sstream>
#include <iostream>


void startKeyboardTeleop(){
    ROS_INFO("Teleop stared...");
    TurtlebotTeleop turtlebot_teleop("/mobile_base/commands/velocity");      
    ros::NodeHandle n;
    signal(SIGINT,quit);

    boost::thread my_thread(boost::bind(&TurtlebotTeleop::keyLoop, &turtlebot_teleop));      
    
    ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::watchdog, &turtlebot_teleop));

    my_thread.interrupt() ;
    my_thread.join() ;
    ROS_INFO("Teleop ended...");
}

void showHelp(){
                   
  ROS_INFO("------Robot contoller-----");
  ROS_INFO("exit             - exit");
  ROS_INFO("keyboard control - teleop");
  //ROS_INFO("------Robot contoller-----");

}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_remote_controller");


  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("robot_controller", 1000);

  ros::Rate loop_rate(10);
 
  int count = 0;
  while (ros::ok())
  {
   
    std_msgs::String msg;

    std::stringstream ss;
    std::cout << "robot controller: ";
    std::cin >> msg.data;
    //ss << "hello world " << count;
    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    if(msg.data=="exit")
        break;
    else if(msg.data=="teleop")
        startKeyboardTeleop();
    else if(msg.data=="help")
        showHelp();
    else{
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
  }


  return 0;
}