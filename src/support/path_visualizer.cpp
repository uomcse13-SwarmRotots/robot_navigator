#include <support/path_visualizer.h>



Visualizer::Visualizer(ros::Publisher& marker_publisher,std::string vis_base_frame)
    :marker_publisher_(marker_publisher),vis_base_frame_(vis_base_frame){
    
}

Visualizer::~Visualizer(){
    free(marker_publisher_);    
    free(&vis_base_frame_);
    free(&point_array);
}


visualization_msgs::Marker Visualizer::publishMarker(const geometry_msgs::PoseStamped& pose){
    visualization_msgs::Marker marker;
    marker.header.frame_id = vis_base_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = "marker";
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
    marker_publisher_.publish( marker );

}


void Visualizer::showPath(std::vector<geometry_msgs::PoseStamped>& plan){
    
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = vis_base_frame_;
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

    marker_publisher_.publish(points);
    marker_publisher_.publish(line_strip);
    marker_publisher_.publish(line_list);

}

void Visualizer::showPoints(const geometry_msgs::PoseStamped& pose){

        point_array.push_back(pose);
        showPath(point_array);

}


