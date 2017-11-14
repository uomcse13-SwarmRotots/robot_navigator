#ifndef CMD_VAL_CONTROLLER_INCLUDE
#define CMD_VAL_CONTROLLER_INCLUDE

#include <string> 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>

#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

#define MAX_SPEED 1.0
#define MIN_SPEED 0.1

namespace swarm_navigator {

class CmdValController{

    private:
    
        ros::NodeHandle nh_;  
        ros::Publisher publisher_;
        tf::TransformListener listener_;
        std::string base_link_;
        std::string odom_link_;

        float liner_velocity_;
        float angular_velocity_;
        
        float keyboard_liner_velocity_;
        float keyboard_angular_velocity_;
        bool keyboard_stop_;
        
        

    public:
    
        CmdValController(ros::NodeHandle &nh,std::string topic);       
        CmdValController(ros::NodeHandle &nh,std::string topic,std::string base_link,std::string odom_link);
        void stop();
        void foward();
        void backward();
        void left();
        void right();
       
        bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;
        
        bool driveForward(double distance);
        bool turn(bool clockwise, double radians);
        bool achieveGoal(const geometry_msgs::PoseStamped& goal);
        bool followPath(const std::vector<geometry_msgs::PoseStamped>& plan);

        void setLinerVelocity(float velocity);
        void setAngularVelocity(float velocity);
        bool ready();

        void keyboard_foward();
        void keyboard_backward();
        void keyboard_left();
        void keyboard_right();
        void keyboard_stop();

        void keyboard_speedup();
        void keyboard_speeddown();
        void keyboard_speeddownAnguler();
        void keyboard_speedupAnguler();
        

};


}

#endif