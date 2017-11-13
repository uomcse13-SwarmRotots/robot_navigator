#ifndef KEYBOARD_TELEOP_INCLUDE
#define KEYBOARD_TELEOP_INCLUDE


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

#define KEYCODE_2 0x32
#define KEYCODE_8 0x38
#define KEYCODE_1 0x31
#define KEYCODE_7 0x37
#define KEYCODE_SPACE 0x20

#define MAX_SPEED 1.0
#define MIN_SPEED 0.1


class TurtlebotTeleop
{
public:
  TurtlebotTeleop(std::string topic);
  void keyLoop();
  void watchdog();

private:

  double speed;
  double speed_anguler;

  void speedup();
  void speeddown();
  void speedupAnguler();
  void speeddownAnguler();

  ros::NodeHandle nh_,ph_;
  double linear_, angular_;
  ros::Time first_publish_;
  ros::Time last_publish_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  void publish(double, double);
  boost::mutex publish_mutex_;

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

#endif
