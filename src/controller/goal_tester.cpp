#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <controller/cmd_val_controller.h>
#include <planner/navigation_planner.h>

using namespace sensor_msgs;
using namespace message_filters;

swarm_navigator::CmdValController* controller;
NavigationPlanner* navigation_planner;
ros::Publisher vis_pub;
std::vector<geometry_msgs::PoseStamped> point_array;


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

visualization_msgs::Marker publishMarker(const geometry_msgs::PoseStamped& pose){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.pose.position.x;
    marker.pose.position.y = pose.pose.position.y;
    marker.pose.position.z = pose.pose.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub.publish( marker );

}


void showPath(std::vector<geometry_msgs::PoseStamped>& plan,ros::Publisher& publisher){
    
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/odom";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is red
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    // Line list is blue
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;

    for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan.end ()-1; 
        it != plan.begin (); --it){

        geometry_msgs::Point p;
        p.x = (*it).pose.position.x;
        p.y = (*it).pose.position.y;
        p.z = (*it).pose.position.z;

        points.points.push_back(p);
        line_strip.points.push_back(p);

        // The line list needs two points for each line
        line_list.points.push_back(p);
        p.z += 1.0;
        line_list.points.push_back(p);
        //ros::Duration(0.5).sleep(); 
    }

    publisher.publish(points);
    publisher.publish(line_strip);
    publisher.publish(line_list);

}

void showPoints(const geometry_msgs::PoseStamped& pose,std::vector<geometry_msgs::PoseStamped>& array
    ,ros::Publisher& publisher){

        array.push_back(pose);
        showPath(array,publisher);

}

// void callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& points)
// {
//     // if(odom==NULL)
//     //     ROS_INFO("pointcloud recieved______________________");
//     // else
//     //     ROS_INFO("odom recieved ######################");

//     ROS_INFO("ODOM Seq: [%d]", odom->header.seq);
//     ROS_INFO("POINT Seq: [%d]", points->header.seq);
    
// }

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
    // push(0.204518,3.540746,plan);
    // push(-0.295482,3.540746,plan);
    // push(-0.795482,3.040746,plan);
    // push(-0.795482,2.540746,plan);
    // push(-0.795482,2.040746,plan);
    // push(-0.795482,1.540746,plan);
    // push(-0.295482,1.040746,plan);
    // push(0.204518,0.540746,plan);
    // push(0.704518,0.040746,plan);
    // push(1.204518,-0.459254,plan);
    // push(1.704518,-0.959254,plan);
    // push(1.204518,-1.459254,plan);
    // push(0.704518,-1.959254,plan);
    // push(0.204518,-2.459254,plan);
    // push(-0.295482,-2.959254,plan);   
    
    // push(0204518,3540746,(*plan));
    // push(0204518,3540746,(*plan));

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = current_position.pose.position.x+0.5;
    // pose.pose.position.y = current_position.pose.position.y;
    // pose.pose.position.z = current_position.pose.position.z;
    // pose.pose.orientation.x = 0.0;
    // pose.pose.orientation.y = 0.0;
    // pose.pose.orientation.z = 0.0;
    // pose.pose.orientation.w = 1.0;
    // plan.push_back(pose);

    // pose.pose.position.x = current_position.pose.position.x+1.0;
    // pose.pose.position.y = current_position.pose.position.y;
    // pose.pose.position.z = current_position.pose.position.z;
    // pose.pose.orientation.x = 0.0;
    // pose.pose.orientation.y = 0.0;
    // pose.pose.orientation.z = 0.0;
    // pose.pose.orientation.w = 1.0;
    // plan.push_back(pose);

    // pose.pose.position.x = current_position.pose.position.x+1.0;
    // pose.pose.position.y = current_position.pose.position.y+0.5;
    // pose.pose.position.z = current_position.pose.position.z;
    // pose.pose.orientation.x = 0.0;
    // pose.pose.orientation.y = 0.0;
    // pose.pose.orientation.z = 0.0;
    // pose.pose.orientation.w = 1.0;
    // plan.push_back(pose);

    // pose.pose.position.x = current_position.pose.position.x+0.5;
    // pose.pose.position.y = current_position.pose.position.y+0.5;
    // pose.pose.position.z = current_position.pose.position.z;
    // pose.pose.orientation.x = 0.0;
    // pose.pose.orientation.y = 0.0;
    // pose.pose.orientation.z = 0.0;
    // pose.pose.orientation.w = 1.0;
    // plan.push_back(pose);

    // pose.pose.position.x = current_position.pose.position.x+0.5;
    // pose.pose.position.y = current_position.pose.position.y;
    // pose.pose.position.z = current_position.pose.position.z;
    // pose.pose.orientation.x = 0.0;
    // pose.pose.orientation.y = 0.0;
    // pose.pose.orientation.z = 0.0;
    // pose.pose.orientation.w = 1.0;
    // plan.push_back(pose);

    // pose.pose.position.x = current_position.pose.position.x+0.5;
    // pose.pose.position.y = current_position.pose.position.y-0.5;
    // pose.pose.position.z = current_position.pose.position.z;
    // pose.pose.orientation.x = 0.0;
    // pose.pose.orientation.y = 0.0;
    // pose.pose.orientation.z = 0.0;
    // pose.pose.orientation.w = 1.0;
    // plan.push_back(pose);

    ROS_INFO("Done planning....");
    showPath(plan,vis_pub);
    controller->followPath(plan);

    //controller->achieveGoal(goal);
    
}

void testcallback(const geometry_msgs::PoseStamped& goal){
    showPoints(goal,point_array,vis_pub);
    
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

    navigation_planner = new NavigationPlanner(nh,topic_octomap);
    navigation_planner->start();

    ros::NodeHandle simple_nh("move_base_simple");
    ros::Subscriber goal_sub_ = simple_nh.subscribe("goal", 1, testcallback);


    // ros::Subscriber goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1);
    ros::spin();

    return 0;
}


