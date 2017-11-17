#ifndef VISUALIZER_INCLUDE
#define VISUALIZER_INCLUDE

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>


class Visualizer{

private:
    ros::Publisher& marker_publisher_;    
    std::string vis_base_frame_;
    std::vector<geometry_msgs::PoseStamped> point_array;

public:    
    Visualizer(ros::Publisher& marker_publisher,std::string vis_base_frame);
    ~Visualizer();
    visualization_msgs::Marker publishMarker(const geometry_msgs::PoseStamped& pose);
    void showPath(std::vector<geometry_msgs::PoseStamped>& plan);
    void showPoints(const geometry_msgs::PoseStamped& pose);
    void showTargetPath(std::vector<geometry_msgs::PoseStamped>& plan);

};
#endif