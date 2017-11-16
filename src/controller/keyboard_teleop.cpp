#include "controller/keyboard_teleop.h"

TurtlebotTeleop::TurtlebotTeleop(std::string topic):
  ph_("~"),
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0),
  speed(0.2),
  speed_anguler(0.5)
{
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(topic,1,true);
  controller_thread = NULL;
  state = "NONE";
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_teleop");

  std::string cmd_vel_topic;

  ros::NodeHandle private_nh("~");     
  private_nh.param("cmd_vel_topic", cmd_vel_topic, std::string("/cmd_vel"));
  TurtlebotTeleop turtlebot_teleop(cmd_vel_topic);
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&TurtlebotTeleop::keyLoop_cmd_vel, &turtlebot_teleop));
  
  
  //ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::watchdog, &turtlebot_teleop));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;
      
  return(0);
}

void TurtlebotTeleop::speeddown(){
  speed-=0.1;
  if(speed<=MIN_SPEED)
    speed+=0.1;
}

void TurtlebotTeleop::speedup(){  
  speed+=0.1;
  if(speed>=MAX_SPEED)
    speed-=0.1;
}
  
void TurtlebotTeleop::speeddownAnguler(){
  speed_anguler-=0.1;
  if(speed<=MIN_SPEED)
    speed_anguler+=0.1;
}

void TurtlebotTeleop::speedupAnguler(){  
  speed_anguler+=0.1;
  if(speed>=MAX_SPEED)
    speed_anguler-=0.1;
}

void TurtlebotTeleop::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0, 0);
}

void TurtlebotTeleop::keyLoop()
{
  char c;
  bool stop = false;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtlebot.");


  while (ros::ok()&&!stop)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = speed_anguler;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -speed_anguler;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = speed;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -speed;
        break;
      case KEYCODE_Q:
        puts("Exit keyboard teleop");
        stop = true;
        break;      
      case KEYCODE_SPACE:        
        ROS_DEBUG("POSE");
        linear_=angular_=0;   
        break;
      case KEYCODE_2:        
        speeddown();
        break;
      case KEYCODE_8:        
        speedup();
        break;
      case KEYCODE_1:        
        speeddownAnguler();
        break;
      case KEYCODE_7:        
        speedupAnguler();
        break;
      
    }
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) { 
      first_publish_ = ros::Time::now();
    }
    last_publish_ = ros::Time::now();
    publish(angular_, linear_);
  }

  return;
}

void TurtlebotTeleop::keyLoop_cmd_vel()
{
  char c;
  bool stop = false;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 8,4,2,6 keys to move the turtlebot.");
  puts("Use key 5 to stop the turtlebot.");
  puts("Use key Q to exit the keyboard stearing.");


  while (ros::ok()&&!stop)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_4:
        ROS_DEBUG("LEFT");        
        if(state!="LEFT"){
          state = "LEFT";
          if(controller_thread!=NULL){
            controller_thread->interrupt();
            free(controller_thread);
            controller_thread = NULL;
          }
          controller_thread = new boost::thread(boost::bind(&TurtlebotTeleop::keyboard_left, this));          
        }        
        break;
      case KEYCODE_6:
        ROS_DEBUG("RIGHT");       
        if(state!="RIGHT"){
          state = "RIGHT";
          if(controller_thread!=NULL){
            controller_thread->interrupt();
            free(controller_thread);
            controller_thread = NULL;
          }
          controller_thread = new boost::thread(boost::bind(&TurtlebotTeleop::keyboard_right, this));          
        }      
        break;
      case KEYCODE_8:
        ROS_DEBUG("UP");        
        if(state!="UP"){
          state = "UP";
          if(controller_thread!=NULL){
            controller_thread->interrupt();
            free(controller_thread);
            controller_thread = NULL;
          }
          controller_thread = new boost::thread(boost::bind(&TurtlebotTeleop::keyboard_foward, this));          
        }     
        break;
      case KEYCODE_2:
        ROS_DEBUG("DOWN");        
        if(state!="DOWN"){
          state = "DOWN";
          if(controller_thread!=NULL){
            controller_thread->interrupt();
            free(controller_thread);
            controller_thread = NULL;
          }
          controller_thread = new boost::thread(boost::bind(&TurtlebotTeleop::keyboard_backward, this));          
        }     
        break;
      case KEYCODE_5:
        ROS_DEBUG("STOP");  
        if(state!="STOP"){         
          state = "STOP";
          if(controller_thread!=NULL){
            controller_thread->interrupt();
            free(controller_thread);
            controller_thread = NULL;
          }         
          keyboard_stop();
        }
        break;
      case KEYCODE_Q:
        puts("Exit keyboard teleop");
        if(controller_thread!=NULL){
          controller_thread->interrupt();
          free(controller_thread);
        }  
        stop = true;
        break;     
      case KEYCODE_3:        
        speeddown();
        break;
      case KEYCODE_9:        
        speedup();
        break;
      case KEYCODE_1:        
        speeddownAnguler();
        break;
      case KEYCODE_7:        
        speedupAnguler();
        break;
      
    }   
  }

  return;
}


void TurtlebotTeleop::publish(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular;
    vel.linear.x = l_scale_*linear;

    vel_pub_.publish(vel);    


  return;
}

void TurtlebotTeleop::keyboard_foward(){
   
    geometry_msgs::Twist base_cmd;        
    
    ros::Rate rate(10.0);
    
    while (nh_.ok())
    {
        base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = speed;
        //send the drive command
        vel_pub_.publish(base_cmd);
        rate.sleep();
        boost::this_thread::interruption_point();
        
    }
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
    vel_pub_.publish(base_cmd);  
}
void TurtlebotTeleop::keyboard_backward(){
   
    geometry_msgs::Twist base_cmd;        
    
    ros::Rate rate(10.0);
    
    while (nh_.ok())
    {
        base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = -speed;
        //send the drive command
        vel_pub_.publish(base_cmd);
        rate.sleep();
        boost::this_thread::interruption_point();
        
    }
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
    vel_pub_.publish(base_cmd);
}

void TurtlebotTeleop::keyboard_left(){
   
    geometry_msgs::Twist base_cmd;        
    
    ros::Rate rate(10.0);
    
    while (nh_.ok())
    {
        base_cmd.linear.x = base_cmd.linear.y = 0.0;
        base_cmd.angular.z = speed_anguler;
        //send the drive command
        vel_pub_.publish(base_cmd);
        rate.sleep();
        boost::this_thread::interruption_point();
        
    }
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
    vel_pub_.publish(base_cmd);
}
void TurtlebotTeleop::keyboard_right(){        
    
    geometry_msgs::Twist base_cmd;   
    ros::Rate rate(10.0);
    
    while (nh_.ok())
    {
        base_cmd.linear.x = base_cmd.linear.y = 0.0;
        base_cmd.angular.z = -speed_anguler;
        //send the drive command
        vel_pub_.publish(base_cmd);
        rate.sleep();
        boost::this_thread::interruption_point();
        
    }
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
    vel_pub_.publish(base_cmd);
}

void TurtlebotTeleop::keyboard_stop(){        
    
    geometry_msgs::Twist base_cmd;   
   
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
    vel_pub_.publish(base_cmd);
}



