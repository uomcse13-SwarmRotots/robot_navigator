#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <controller/cmd_val_controller.h>
#include <planner/navigation_planner.h>
#include <support/path_visualizer.h>

using namespace sensor_msgs;
using namespace message_filters;

swarm_navigator::CmdValController* controller;
NavigationPlanner* navigation_planner;
ros::Publisher vis_pub;
std::vector<geometry_msgs::PoseStamped> point_array;
Visualizer* visualizer;

void push(double x,double y,std::vector<geometry_msgs::PoseStamped>& plan){
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = -0.000005;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
}


void callback(const geometry_msgs::PoseStamped& goal)
{
    // if(odom==NULL)
    //     ROS_INFO("pointcloud recieved______________________");
    // else
    //     ROS_INFO("odom recieved ######################");

    ROS_INFO("Came....");
    // ROS_INFO("POINT Seq: [%d]", points->header.seq);

    tf::Stamped<tf::Pose> global_pose;
    controller->getRobotPose(global_pose);
    geometry_msgs::PoseStamped current_position;

    tf::poseStampedTFToMsg(global_pose, current_position);
    std::vector<geometry_msgs::PoseStamped> plan = 
                    navigation_planner->getNavPlan(current_position);

    // std::vector<geometry_msgs::PoseStamped> plan;

    ROS_INFO("Done planning....");
    visualizer->showPath(plan);
    controller->followPath(plan);

    //controller->achieveGoal(goal);
    
}

void testcallback(const geometry_msgs::PoseStamped& goal){
    visualizer->showPoints(goal);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation");

    ros::NodeHandle nh;
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

    controller = new swarm_navigator::CmdValController(nh,topic_cmd_val,base_link,odom_link);
    
    // message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/depth_camera/depth_camera/depth/points", 1);
    // TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2> sync(odom_sub, points_sub, 10);
    // sync.registerCallback(boost::bind(&callback, _1, _2));

    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    visualizer = new Visualizer(vis_pub,odom_link);

    navigation_planner = new NavigationPlanner(nh,topic_octomap);
    navigation_planner->start();

    ros::NodeHandle simple_nh("move_base_simple");
    ros::Subscriber goal_sub_ = simple_nh.subscribe("goal", 1, callback);
    

    // ros::Subscriber goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1);
    ros::spin();

    return 0;
}


