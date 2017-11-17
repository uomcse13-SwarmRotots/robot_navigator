#include "planner/navigation_planner.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
struct Graph_Node *current_node;
float current_x_node;
float current_y_node;
float current_z_node;
float marker_z_cordinate;

void NavigationPlanner::setParametersForground(float slope,float min_height,float max_height,int window_size){
    slope_ = slope;
    min_height_ = min_height;
    max_height_ = max_height;
    window_size_ = window_size;

}


bool NavigationPlanner::isRobotInside(float current_x_cordinate, float current_y_cordinate, float current_z_cordinate, float target_x_cordinate,float target_y_cordinate){
    float box_min_x = current_x_cordinate - box_dimension_;
    float box_max_x = current_x_cordinate + box_dimension_;
    float box_min_y = current_y_cordinate - box_dimension_;
    float box_max_y = current_y_cordinate + box_dimension_;

    if(box_min_x<=target_x_cordinate && box_max_x>=target_x_cordinate && box_min_y<=target_y_cordinate && box_max_y>=target_y_cordinate){
        ROS_INFO("true");
        return true;
    }else{
        return false;
    }
}

bool NavigationPlanner::planeTraversabilityCheck(float a,float b,float c,float e, float x0,float y0,float z0, float d, int type){
	float a2_ = a/(sqrt(a*a+b*b+c*c));
    float b2_ = b/(sqrt(a*a+b*b+c*c));
    float c2_ = c/(sqrt(a*a+b*b+c*c));
    float angle_x = acos(a2_);
    float angle_y = acos(b2_);
    float angle_z = acos(c2_);
    float z;
	//Print angle	
    float acceptedAngle 	= 0.523599; // 30 degree
    float acceptedAngle90  = 1.39626; // 80 degree
    float acceptedDistance_1 = (d/2)*sin(angle_x);
    float acceptedDistance_2 = (d/2)*sin(angle_y);
    float acceptedDistance_3 = (d/2)*sin(angle_z);
	//checking the traversability
    //Angle check
    float distance = (a*x0 + b*y0 + c*z0 + e)/sqrt(a*a+b*b+c*c);
    std::cout<< "a2_ " << a2_ << endl;
    std::cout<< "b2_ " << b2_ << endl;
    std::cout<< "c2_ " << c2_ << endl;
    std::cout<< "angle_x " << angle_x << endl;
    std::cout<< "angle_y " << angle_y << endl;
    std::cout<< "angle_z " << angle_z << endl;
    std::cout<< "acceptedAngle " << acceptedAngle << endl;
    std::cout<< "acceptedDistance_1 " << acceptedDistance_1 << endl;
    std::cout<< "acceptedDistance_2 " << acceptedDistance_2 << endl;
    std::cout<< "acceptedDistance_3 " << acceptedDistance_3 << endl;
    std::cout<< "distance " << distance << endl;

    switch(type){
        case 1:
	    z = (-1*e - a*(x0+d) -b*y0)/c;
	    std::cout << "New Z" << z << "Z diff " << z - z0 << endl;
            if(angle_y>acceptedAngle90){
                if(angle_z<acceptedAngle){ 
                    if(distance>=0 && distance > acceptedDistance_3){
                        std::cout<< "traversable case 1+ " << endl;
                        marker_z_cordinate = (-1*e - a*(x0+d) -b*y0)/c;
                        std::cout<< "New Z " << marker_z_cordinate << endl;
                        return true;
                    }
                    else if (distance<0 && (distance*(-1)) > acceptedDistance_3){
                        std::cout<< "traversable case 1-" << endl;
                        marker_z_cordinate = (-1*e - a*(x0+d) -b*y0)/c;
                        std::cout<< "New Z " << marker_z_cordinate << endl;
                        return true;
                    }
                    else {
                        std::cout<< "un-traversable case 1" << endl;
                        return false;
                    }
                }
                else {
                    std::cout<< "un-traversable case 1o" << endl;
                    return false;
                }
            }
            std::cout<< "un-traversable o" << endl;
            return false;
            break;
        case 3:
	    z = (-1*e - a*x0 -b*(y0+d))/c;
	    std::cout<< "New Z" << z <<  "Z diff " << z - z0 << endl;
            if(angle_x>acceptedAngle90){
                if(angle_z<acceptedAngle){ 
                    if(distance>=0 && distance > acceptedDistance_3){
                        std::cout<< "traversable case 3+" << endl;
                        marker_z_cordinate = (-1*e - a*x0 -b*(y0+d))/c;
                        std::cout<< "New Z " << marker_z_cordinate << endl;
                        return true;
                    }
                    else if (distance<0 && (distance*(-1)) > acceptedDistance_3){
                        std::cout<< "traversable case 3-" << endl;
                        marker_z_cordinate = (-1*e - a*x0 -b*(y0+d))/c;
                        std::cout<< "New Z " << marker_z_cordinate << endl;
                        return true;
                    }
                    else {
                        std::cout<< "un-traversable case 3" << endl;
                        return false;
                    }
                }
                else {
                    std::cout<< "un-traversable case 3o" << endl;
                    return false;
                }
            }
            std::cout<< "un-traversable o" << endl;
            return false;
            break;
        case 5:
	    z = (-1*e - a*(x0-d) -b*y0)/c;
	    std::cout<< "New Z" << z <<  "Z diff " << z - z0 << endl;
            if(angle_y>acceptedAngle90){
                if(angle_z<acceptedAngle){ 
                    if(distance>=0 && distance > acceptedDistance_3){
                        std::cout<< "traversable case 5+" << endl;
                        marker_z_cordinate = (-1*e - a*(x0-d) -b*y0)/c;
                        std::cout<< "New Z " << marker_z_cordinate << endl;
                        return true;
                    }
                    else if (distance<0 && (distance*(-1)) > acceptedDistance_3){
                        std::cout<< "traversable case 5-" << endl;
                        marker_z_cordinate = (-1*e - a*(x0-d) -b*y0)/c;
                        std::cout<< "New Z " << marker_z_cordinate << endl;
                        return true;
                    }
                    else {
                        std::cout<< "un-traversable case 5" << endl;
                        return false;
                    }
                }
                else {
                    std::cout<< "un-traversable case 5o" << endl;
                    return false;
                }
            }
            std::cout<< "un-traversable o" << endl;
            return false;
            break;
        case 7:
	    z = (-1*e - a*x0 -b*(y0-d))/c;
	    std::cout<< "New Z" << z <<  "Z diff " << z - z0 << endl;
            if(angle_x>acceptedAngle90){
                if(angle_z<acceptedAngle){ 
                    if(distance>=0 && distance > acceptedDistance_3){
                        std::cout<< "traversable case 7+" << endl;
                        marker_z_cordinate = (-1*e - a*x0 -b*(y0-d))/c;
                        std::cout<< "New Z " << marker_z_cordinate << endl;
                        return true;
                    }
                    else if (distance<0 && (distance*(-1)) > acceptedDistance_3){
                        std::cout<< "traversable case 7-" << endl;
                        marker_z_cordinate = (-1*e - a*x0 -b*(y0-d))/c;
                        std::cout<< "New Z " << marker_z_cordinate << endl;
                        return true;
                    }
                    else {
                        std::cout<< "un-traversable case 7" << endl;
                        return false;
                    }
                }
                else {
                    std::cout<< "un-traversable case 7o" << endl;
                    return false;
                }
            }
            std::cout<< "un-traversable o" << endl;
            return false;
            break;
        case 2:
	    z = (-1*e - a*(x0+d) -b*(y0+d))/c;
	    std::cout<< "New Z" << z <<  "Z diff " << z - z0 << endl;
	    return false;
        case 4:
	    z = (-1*e - a*(x0-d) -b*(y0+d))/c;
	    std::cout<< "New Z" << z << "Z diff " << z - z0 << endl;
	    return false;
        case 6:
	    z = (-1*e - a*(x0-d) -b*(y0-d))/c;
	    std::cout<< "New Z" << z << "Z diff " << z - z0 << endl;
	    return false;
        case 8:
	    z = (-1*e - a*(x0+d) -b*(y0-d))/c;
	    std::cout<< "New Z" << z << "Z diff " << z - z0 << endl;
	    return false;
        default:
            return false;
    }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr NavigationPlanner::calculateConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr surrounding_cloud, vector<pcl::PointXYZ> point_vector,int point_type){
    pcl::CropHull<pcl::PointXYZ> cropHullFilter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hullCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr hullPoints(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> hullPolygons;
    vector<pcl::PointXYZ>::iterator it;
    for(it = point_vector.begin(); it != point_vector.end(); it++){
        hullCloud->push_back(*it);
    }

    pcl::ConvexHull<pcl::PointXYZ> cHull;
    cHull.setInputCloud(hullCloud);
    cHull.reconstruct(*hullPoints, hullPolygons);

    cropHullFilter.setHullIndices(hullPolygons);
    cropHullFilter.setHullCloud(hullPoints);
    cropHullFilter.setDim(2);
    cropHullFilter.setCropOutside(true);

    cropHullFilter.setInputCloud(surrounding_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cropHullFilter.filter(*filtered_cloud);

    std::string file_name1="filtered";
    file_name1 = file_name1 + boost::lexical_cast<std::string>(point_type);
    file_name1 = file_name1+".pcd";

    std::string file_name2="hullPoints";
    file_name2 = file_name2 + boost::lexical_cast<std::string>(point_type);
    file_name2 = file_name2+".pcd";

    std::string file_name3="cloud";
    file_name3 = file_name3 + boost::lexical_cast<std::string>(point_type);
    file_name3 = file_name3+".pcd";

    if(filtered_cloud->size()>0){
        //pcl::io::savePCDFile(file_name1, *filtered_cloud, true);
    }
    if(hullPoints->size()>0){
        // pcl::io::savePCDFile(file_name2, *hullPoints,true);
    }
    if(cloud->size()>0){
        // pcl::io::savePCDFile(file_name3, *cloud,true);
    }
    return filtered_cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr NavigationPlanner::getConvexHull( pcl::PointCloud<pcl::PointXYZ>::Ptr surrounding_cloud,float x_cordinate, float y_cordinate, float z_cordinate, int point_type, float box_dimension){
    vector<pcl::PointXYZ> point_vector; 
    if(point_type==2){
        pcl::PointXYZ point1;
        point1.x=x_cordinate - 2*box_dimension;
        point1.y=y_cordinate;
        point1.z=z_cordinate;
        point_vector.push_back(point1);
        
        pcl::PointXYZ point2;
        point2.x=x_cordinate - box_dimension;
        point2.y=y_cordinate - box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate + box_dimension;
        point3.y=y_cordinate - box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate + box_dimension;
        point4.y=y_cordinate + box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);

        pcl::PointXYZ point5;
        point5.x=x_cordinate;
        point5.y=y_cordinate + 2*box_dimension;
        point5.z=z_cordinate;
        point_vector.push_back(point5);

        pcl::PointXYZ point6;
        point6.x=x_cordinate - 2*box_dimension;;
        point6.y=y_cordinate + 2*box_dimension;;
        point6.z=z_cordinate;
        point_vector.push_back(point6);
    }else if(point_type==1){
        pcl::PointXYZ point1;
        point1.x=x_cordinate - box_dimension;
        point1.y=y_cordinate - box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);

        pcl::PointXYZ point2;
        point2.x=x_cordinate + box_dimension;
        point2.y=y_cordinate - box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate + box_dimension;
        point3.y=y_cordinate + 2*box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate - box_dimension;
        point4.y=y_cordinate + 2*box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);
    }else if(point_type==3){
        pcl::PointXYZ point1;
        point1.x=x_cordinate + box_dimension;
        point1.y=y_cordinate + box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);

        pcl::PointXYZ point2;
        point2.x=x_cordinate - 2*box_dimension;
        point2.y=y_cordinate + box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate - 2*box_dimension;
        point3.y=y_cordinate - box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate + box_dimension;
        point4.y=y_cordinate - box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);
    }else if(point_type==4){
        pcl::PointXYZ point1;
        point1.x=x_cordinate + box_dimension;
        point1.y=y_cordinate + box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);
        
        pcl::PointXYZ point2;
        point2.x=x_cordinate - box_dimension;
        point2.y=y_cordinate + box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate - 2*box_dimension;
        point3.y=y_cordinate;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate - 2*box_dimension;
        point4.y=y_cordinate - 2*box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);

        pcl::PointXYZ point5;
        point5.x=x_cordinate;
        point5.y=y_cordinate - 2*box_dimension;
        point5.z=z_cordinate;
        point_vector.push_back(point5);

        pcl::PointXYZ point6;
        point6.x=x_cordinate + box_dimension;;
        point6.y=y_cordinate - box_dimension;;
        point6.z=z_cordinate;
        point_vector.push_back(point6);
    }else if(point_type==5){
        pcl::PointXYZ point1;
        point1.x=x_cordinate - box_dimension;
        point1.y=y_cordinate + box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);

        pcl::PointXYZ point2;
        point2.x=x_cordinate - box_dimension;
        point2.y=y_cordinate - 2*box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate + box_dimension;
        point3.y=y_cordinate - 2*box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate + box_dimension;
        point4.y=y_cordinate + box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);
    }else if(point_type==6){
        pcl::PointXYZ point1;
        point1.x=x_cordinate + box_dimension;
        point1.y=y_cordinate + box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);
        
        pcl::PointXYZ point2;
        point2.x=x_cordinate - box_dimension;
        point2.y=y_cordinate + box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate - box_dimension;
        point3.y=y_cordinate - box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate;
        point4.y=y_cordinate - 2*box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);

        pcl::PointXYZ point5;
        point5.x=x_cordinate + 2*box_dimension;
        point5.y=y_cordinate - 2*box_dimension;
        point5.z=z_cordinate;
        point_vector.push_back(point5);

        pcl::PointXYZ point6;
        point6.x=x_cordinate + 2*box_dimension;
        point6.y=y_cordinate;
        point6.z=z_cordinate;
        point_vector.push_back(point6);
    }else if(point_type==7){
        pcl::PointXYZ point1;
        point1.x=x_cordinate - box_dimension;
        point1.y=y_cordinate + box_dimension;
        point1.z=z_cordinate;
        point_vector.push_back(point1);

        pcl::PointXYZ point2;
        point2.x=x_cordinate - box_dimension;
        point2.y=y_cordinate - box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate + 2*box_dimension;
        point3.y=y_cordinate - box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate + 2*box_dimension;
        point4.y=y_cordinate + box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);
    }else if(point_type==8){
        pcl::PointXYZ point1;
        point1.x=x_cordinate + 2*box_dimension;
        point1.y=y_cordinate;
        point1.z=z_cordinate;
        point_vector.push_back(point1);
        
        pcl::PointXYZ point2;
        point2.x=x_cordinate + 2*box_dimension;
        point2.y=y_cordinate + 2*box_dimension;
        point2.z=z_cordinate;
        point_vector.push_back(point2);

        pcl::PointXYZ point3;
        point3.x=x_cordinate;
        point3.y=y_cordinate + 2*box_dimension;
        point3.z=z_cordinate;
        point_vector.push_back(point3);

        pcl::PointXYZ point4;
        point4.x=x_cordinate - box_dimension;
        point4.y=y_cordinate + box_dimension;
        point4.z=z_cordinate;
        point_vector.push_back(point4);

        pcl::PointXYZ point5;
        point5.x=x_cordinate - box_dimension;
        point5.y=y_cordinate - box_dimension;
        point5.z=z_cordinate;
        point_vector.push_back(point5);

        pcl::PointXYZ point6;
        point6.x=x_cordinate - box_dimension;
        point6.y=y_cordinate + box_dimension;
        point6.z=z_cordinate;
        point_vector.push_back(point6);
    }

    return calculateConvexHull(surrounding_cloud,point_vector,point_type);

}

bool NavigationPlanner::planerCoefficientApproximation(pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud, int type){
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (plane_cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0){
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return false;
    }
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;
    return planeTraversabilityCheck(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3], current_x_node,current_y_node,current_z_node, box_dimension_, type);
}


void NavigationPlanner::clusterObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr& object_cloud){
    std::cerr << "cluster objects " << std::endl;
    if(object_cloud->size()>0){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (object_cloud);
        vg.setLeafSize (0.05f, 0.05f, 0.05f);
        vg.filter (*cloud_filtered);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        int i=0, nr_points = (int) cloud_filtered->points.size ();
        while (cloud_filtered->points.size () > 0.3 * nr_points) {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0) {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);

            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);
            std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);
            *cloud_filtered = *cloud_f;
        }

    // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.2); // 2cm
        ec.setMinClusterSize (25);
        ec.setMaxClusterSize (50000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            std::stringstream ss;
            ss << "cloud_cluster_" << j << ".pcd";
            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
            planerCoefficientApproximation(cloud_cluster,0);
            j++;
        }

        if(j==0){
            std::cout << "No points found after filter" << std::endl;
        }
    }else{
        std::cout << "No obstacles found - null input cloud" << std::endl; 
    }
}

int  NavigationPlanner::groundNonGroundExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cube, int type){
    ROS_INFO("Reached");
    if(cloud_cube->size()>20){
        pcl::PointIndicesPtr ground (new pcl::PointIndices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
        pmf.setInputCloud (cloud_cube);
        pmf.setMaxWindowSize (window_size_);
        pmf.setSlope (slope_);
        pmf.setInitialDistance (min_height_);
        pmf.setMaxDistance (max_height_);
        pmf.extract (ground->indices);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_cube);
        extract.setIndices (ground);
        extract.filter (*cloud_filtered);
        bool ground_cloud = false;
        pcl::PCDWriter writer;
        if(cloud_filtered->size()>0){
            ground_cloud = true;
        }

        extract.setNegative (true);
        extract.filter (*cloud_filtered);

        if(cloud_filtered->size()>0){
            //writer.write<pcl::PointXYZ> ("samp11-utm_object.pcd", *cloud_filtered, false);
            //clusterObjects(cloud_filtered);
            bool is_traversable = planerCoefficientApproximation(cloud_filtered,type);
            if(is_traversable){
                return 1;
            }else{
                return 0;
            }
        }else{
            ROS_INFO("End %d",1);
            if(ground_cloud){
                return 1;
            }else{
                return -1;
            }
            
        }
    }else{
        ROS_INFO("End %d",-1);
        return -1;
    }
    
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NavigationPlanner::segmentBoundingCube(pcl::PointCloud<pcl::PointXYZ>::Ptr& object_cloud, float min_z_cordinate, float max_z_cordinate){

   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("Print %f, %f", min_z_cordinate, max_z_cordinate);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (object_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (min_z_cordinate, max_z_cordinate);
    pass.filter (*cloud_filtered);

    return cloud_filtered;
}

void NavigationPlanner::cloudCallback(const PointCloud::ConstPtr& msg){
    cloud->width  = msg->width;
    cloud->height = msg->height;
    cloud->points.resize (cloud->width * cloud->height);

    int i=0;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
        cloud->points[i].x = pt.x;
        cloud->points[i].y = pt.y;
        cloud->points[i].z = pt.z;
        i++;
    }
}


NavigationPlanner::NavigationPlanner(ros::NodeHandle &nh, std::string topic){
    node_handle_ = nh;
    topic_ = topic;
}

NavigationPlanner::NavigationPlanner(ros::NodeHandle &nh, std::string topic, float box_dimension,float robot_height, float min_x_cordinate, float max_x_cordinate, float min_y_cordinate, float max_y_cordinate){
    node_handle_ = nh;
    topic_ = topic;
    box_dimension_ = box_dimension;
    robot_height_ = robot_height;
    min_x_cordinate_ = min_x_cordinate;
    max_x_cordinate_ = max_x_cordinate;
    min_y_cordinate_ = min_y_cordinate;
    max_y_cordinate_ = max_y_cordinate;
    
}

NavigationPlanner::~NavigationPlanner(){}

struct Graph_Node* NavigationPlanner::breadthFirstSearch(float x_cordinate, float y_cordinate, float z_cordinate){

    current_x_node = x_cordinate;
    current_y_node = y_cordinate;
    current_z_node = z_cordinate;

    boost::this_thread::interruption_point();
    
    if(min_x_cordinate_<x_cordinate && max_x_cordinate_>x_cordinate && min_y_cordinate_<y_cordinate && max_y_cordinate_>y_cordinate){
        pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr surrounding_cloud = segmentBoundingCube(cloud,current_z_node-robot_height_,current_z_node+robot_height_);
        ROS_INFO("size is :  %d",surrounding_cloud->size());
        int result;
        ROS_INFO("0 %f %f %f",x_cordinate, y_cordinate, z_cordinate);

        float front_x = x_cordinate + box_dimension_;
        float front_y = y_cordinate;
        if(!found_nodes->hasValue(front_x,front_y,z_cordinate) & !found_obstacles->hasValue(front_x,front_y,z_cordinate)){
            convex_cloud = getConvexHull(surrounding_cloud,front_x,front_y,z_cordinate,1,box_dimension_);
            marker_z_cordinate = z_cordinate;
            result = groundNonGroundExtraction(convex_cloud,1);
            if(result == -1){
                ROS_INFO("Return on 1");
                return current_node;
            }else if(result == 1){

                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = front_x;
                temp_node->y_cordinate = front_y;
                temp_node->z_cordinate = marker_z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                temp_node->marker_z_cordinate = marker_z_cordinate;
                found_nodes->setValue(front_x,front_y,z_cordinate,temp_node);
                node_queue.push(temp_node);
                ROS_INFO("1 %f %f %f",front_x,front_y,z_cordinate);
                
            }else{
                found_obstacles->setValue(front_x,front_y,z_cordinate,0);
            }
        }else if(!found_obstacles->hasValue(front_x,front_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_x;
            temp_node->y_cordinate = front_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1;
            if(found_nodes->getValue(front_x,front_y,z_cordinate)->path_cost>temp_node->path_cost){
                found_nodes->getValue(front_x,front_y,z_cordinate)->predecessor = current_node;
                found_nodes->getValue(front_x,front_y,z_cordinate)->path_cost = temp_node->path_cost;
            }
        }

        float front_left_x = x_cordinate + box_dimension_;
        float front_left_y = y_cordinate + box_dimension_;
    
        
        if(!found_nodes->hasValue(front_left_x,front_left_y,z_cordinate) && !found_obstacles->hasValue(front_left_x,front_left_y,z_cordinate)){
            convex_cloud = getConvexHull(surrounding_cloud,front_left_x,front_left_y,z_cordinate,2,box_dimension_);
            marker_z_cordinate = z_cordinate;
            result = groundNonGroundExtraction(convex_cloud,2); 
            if(result == -1){
                ROS_INFO("Return on 2");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = front_left_x;
                temp_node->y_cordinate = front_left_y;
                temp_node->z_cordinate = marker_z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;
                temp_node->marker_z_cordinate = marker_z_cordinate;
                found_nodes->setValue(front_left_x,front_left_y,z_cordinate,temp_node);
                node_queue.push(temp_node);
                    ROS_INFO("2 %f %f %f",front_left_x,front_left_y,z_cordinate);
                
            }else{
                found_obstacles->setValue(front_left_x,front_left_y,z_cordinate,0);
            }
        }else if(!found_obstacles->hasValue(front_left_x,front_left_y,z_cordinate)){

            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_left_x;
            temp_node->y_cordinate = front_left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1.414;
            if(found_nodes->getValue(front_left_x,front_left_y,z_cordinate)->path_cost>temp_node->path_cost){
                found_nodes->getValue(front_left_x,front_left_y,z_cordinate)->predecessor = current_node;
                found_nodes->getValue(front_left_x,front_left_y,z_cordinate)->path_cost = temp_node->path_cost;
            }
        }      

        float left_x = x_cordinate;
        float left_y = y_cordinate + box_dimension_;
        if(!found_nodes->hasValue(left_x,left_y,z_cordinate) && !found_obstacles->hasValue(left_x,left_y,z_cordinate)){
            convex_cloud = getConvexHull(surrounding_cloud,left_x,left_y,z_cordinate,3,box_dimension_);
            marker_z_cordinate = z_cordinate;
            result = groundNonGroundExtraction(convex_cloud,3);
            if(result == -1){
                ROS_INFO("Return on 3");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = left_x;
                temp_node->y_cordinate = left_y;
                temp_node->z_cordinate = marker_z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                temp_node->marker_z_cordinate = marker_z_cordinate;
                found_nodes->setValue(left_x,left_y,z_cordinate,temp_node);
                node_queue.push(temp_node);
                    ROS_INFO("3 %f %f %f",left_x,left_y,z_cordinate);
                
            }else{
                found_obstacles->setValue(left_x,left_y,z_cordinate,0);
            } 
        }else if(!found_obstacles->hasValue(left_x,left_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = left_x;
            temp_node->y_cordinate = left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1;
            if(found_nodes->getValue(left_x,left_y,z_cordinate)->path_cost>temp_node->path_cost){
                found_nodes->getValue(left_x,left_y,z_cordinate)->predecessor = current_node;
                found_nodes->getValue(left_x,left_y,z_cordinate)->path_cost = temp_node->path_cost;
            }
        }

        float back_left_x = x_cordinate - box_dimension_;
        float back_left_y = y_cordinate + box_dimension_;
        if(!found_nodes->hasValue(back_left_x,back_left_y,z_cordinate) && !found_obstacles->hasValue(back_left_x,back_left_y,z_cordinate)){
            convex_cloud = getConvexHull(surrounding_cloud,back_left_x,back_left_y,z_cordinate,4,box_dimension_);
            marker_z_cordinate = z_cordinate;
            result = groundNonGroundExtraction(convex_cloud,4);
            if(result == -1){
                ROS_INFO("Return on 4");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = back_left_x;
                temp_node->y_cordinate = back_left_y;
                temp_node->z_cordinate = marker_z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;
                temp_node->marker_z_cordinate = marker_z_cordinate;
                found_nodes->setValue(back_left_x,back_left_y,z_cordinate,temp_node);
                node_queue.push(temp_node);
                ROS_INFO("4 %f %f %f",back_left_x,back_left_y,z_cordinate);
                
            }else{
                found_obstacles->setValue(back_left_x,back_left_y,z_cordinate,0);
            }
        }else if(!found_obstacles->hasValue(back_left_x,back_left_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_left_x;
            temp_node->y_cordinate = back_left_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1.414;
            if(found_nodes->getValue(back_left_x,back_left_y,z_cordinate)->path_cost>temp_node->path_cost){
                found_nodes->getValue(back_left_x,back_left_y,z_cordinate)->predecessor = current_node;
                found_nodes->getValue(back_left_x,back_left_y,z_cordinate)->path_cost = temp_node->path_cost;
            }
        }

        float back_x = x_cordinate - box_dimension_;
        float back_y = y_cordinate;
        if(!found_nodes->hasValue(back_x,back_y,z_cordinate) && !found_obstacles->hasValue(back_x,back_y,z_cordinate)){
            convex_cloud = getConvexHull(surrounding_cloud,back_x,back_y,z_cordinate,5,box_dimension_);
            marker_z_cordinate = z_cordinate;
            result = groundNonGroundExtraction(convex_cloud,5);
            if(result == -1){
                ROS_INFO("Return on 5");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = back_x;
                temp_node->y_cordinate = back_y;
                temp_node->z_cordinate = marker_z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                temp_node->marker_z_cordinate = marker_z_cordinate;
                found_nodes->setValue(back_x,back_y,z_cordinate,temp_node);
                node_queue.push(temp_node);
                ROS_INFO("5 %f %f %f",back_x,back_y,z_cordinate);
                
            } else{
                found_obstacles->setValue(back_x,back_y,z_cordinate,0);
            }
        }else if(!found_obstacles->hasValue(back_x,back_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_x;
            temp_node->y_cordinate = back_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1;
            if(found_nodes->getValue(back_x,back_y,z_cordinate)->path_cost>temp_node->path_cost){
                found_nodes->getValue(back_x,back_y,z_cordinate)->predecessor = current_node;
                found_nodes->getValue(back_x,back_y,z_cordinate)->path_cost = temp_node->path_cost;
            }
        }

        float back_right_x = x_cordinate - box_dimension_;
        float back_right_y = y_cordinate - box_dimension_;
        if(!found_nodes->hasValue(back_right_x,back_right_y,z_cordinate) && !found_obstacles->hasValue(back_right_x,back_right_y,z_cordinate)){
            convex_cloud = getConvexHull(surrounding_cloud,back_right_x,back_right_y,z_cordinate,6,box_dimension_);
            marker_z_cordinate = z_cordinate;
            result = groundNonGroundExtraction(convex_cloud,6);
            if(result == -1){
                ROS_INFO("Return on 6");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = back_right_x;
                temp_node->y_cordinate = back_right_y;
                temp_node->z_cordinate = marker_z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;
                temp_node->marker_z_cordinate = marker_z_cordinate;       
                found_nodes->setValue(back_right_x,back_right_y,z_cordinate,temp_node);
                node_queue.push(temp_node);
                ROS_INFO("6 %f %f %f",back_right_x,back_right_y,z_cordinate);
                
            } else{
                found_obstacles->setValue(back_right_x,back_right_y,z_cordinate,0);
            }
        }else if(!found_obstacles->hasValue(back_right_x,back_right_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = back_right_x;
            temp_node->y_cordinate = back_right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1.414;
            if(found_nodes->getValue(back_right_x,back_right_y,z_cordinate)->path_cost>temp_node->path_cost){
                found_nodes->getValue(back_right_x,back_right_y,z_cordinate)->predecessor = current_node;
                found_nodes->getValue(back_right_x,back_right_y,z_cordinate)->path_cost = temp_node->path_cost;
            }
        }

        float right_x = x_cordinate;
        float right_y = y_cordinate - box_dimension_;
        if(!found_nodes->hasValue(right_x,right_y,z_cordinate) && !found_obstacles->hasValue(right_x,right_y,z_cordinate)){
            convex_cloud = getConvexHull(surrounding_cloud,right_x,right_y,z_cordinate,7,box_dimension_);
            marker_z_cordinate = z_cordinate;
            result = groundNonGroundExtraction(convex_cloud,7);
            if(result == -1){
                ROS_INFO("Return on 7");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = right_x;
                temp_node->y_cordinate = right_y;
                temp_node->z_cordinate = marker_z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                temp_node->marker_z_cordinate = marker_z_cordinate;
                found_nodes->setValue(right_x,right_y,z_cordinate,temp_node);
                node_queue.push(temp_node);
                ROS_INFO("7 %f %f %f",right_x,right_y,z_cordinate);
                
            } else{
                found_obstacles->setValue(right_x,right_y,z_cordinate,0);
            }
        }else if(!found_obstacles->hasValue(right_x,right_y,z_cordinate)){
            ROS_INFO("Printed");
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = right_x;
            temp_node->y_cordinate = right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1;
            if(found_nodes->getValue(right_x,right_y,z_cordinate)->path_cost>temp_node->path_cost){
                found_nodes->getValue(right_x,right_y,z_cordinate)->predecessor = current_node;
                found_nodes->getValue(right_x,right_y,z_cordinate)->path_cost = temp_node->path_cost;
            }
        }

        float front_right_x = x_cordinate + box_dimension_;
        float front_right_y = y_cordinate - box_dimension_; 
        if(!found_nodes->hasValue(front_right_x,front_right_y,z_cordinate) && !found_obstacles->hasValue(front_right_x,front_right_y,z_cordinate)){
            convex_cloud = getConvexHull(surrounding_cloud,front_right_x,front_right_y,z_cordinate,8,box_dimension_);
            marker_z_cordinate = z_cordinate;
            result = groundNonGroundExtraction(convex_cloud,8);
            if(result == -1){
                ROS_INFO("Return on 8");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = front_right_x;
                temp_node->y_cordinate = front_right_y;
                temp_node->z_cordinate = marker_z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;
                temp_node->marker_z_cordinate = marker_z_cordinate;
                found_nodes->setValue(front_right_x,front_right_y,z_cordinate,temp_node);
                node_queue.push(temp_node);
                ROS_INFO("8 %f %f %f",front_right_x,front_right_y,z_cordinate);

            }else{
                found_obstacles->setValue(front_right_x,front_right_y,z_cordinate,0);
            } 
        }else if(!found_obstacles->hasValue(front_right_x,front_right_y,z_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = front_right_x;
            temp_node->y_cordinate = front_right_y;
            temp_node->z_cordinate = z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1.414;
            if(found_nodes->getValue(front_right_x,front_right_y,z_cordinate)->path_cost>temp_node->path_cost){
                found_nodes->getValue(front_right_x,front_right_y,z_cordinate)->predecessor = current_node;
                found_nodes->getValue(front_right_x,front_right_y,z_cordinate)->path_cost = temp_node->path_cost;
            }
        }
    }

    if(node_queue.empty()){
        ROS_INFO("No Nodes to Traverse");
        return NULL;
    }else{
        struct Graph_Node *next_node = node_queue.front();
        node_queue.pop();
        //ROS_INFO("Started Braeadth Search");
        current_node=next_node;
        return breadthFirstSearch(next_node->x_cordinate,next_node->y_cordinate,next_node->z_cordinate);
    }    
} 

struct Graph_Node* NavigationPlanner::breadthFirstSearchToGoal(float x_cordinate, float y_cordinate, float z_cordinate, float target_x_cordinate, float target_y_cordinate, float target_z_cordinate){

    current_x_node = x_cordinate;
    current_y_node = y_cordinate;
    current_z_node = z_cordinate;
    
    if(min_x_cordinate_<x_cordinate && max_x_cordinate_>x_cordinate && min_y_cordinate_<y_cordinate && max_y_cordinate_>y_cordinate){
        pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr surrounding_cloud = segmentBoundingCube(cloud,current_z_node-robot_height_,current_z_node+robot_height_);
        ROS_INFO("size is :  %d",surrounding_cloud->size());
        int result;
        ROS_INFO("0 %f %f %f",x_cordinate, y_cordinate, z_cordinate);
        ROS_INFO("0 %f %f %f",target_x_cordinate, target_y_cordinate, target_z_cordinate);
        float front_x = x_cordinate + box_dimension_;
        float front_y = y_cordinate;
        
        if(isRobotInside(front_x,front_y,z_cordinate,target_x_cordinate,target_y_cordinate)){
            ROS_INFO("Returned From 1");
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = target_x_cordinate;
            temp_node->y_cordinate = target_y_cordinate;
            temp_node->z_cordinate = z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1;
            return temp_node;
        }else{
            if(!found_nodes->hasValue(front_x,front_y,z_cordinate) & !found_obstacles->hasValue(front_x,front_y,z_cordinate)){
                convex_cloud = getConvexHull(surrounding_cloud,front_x,front_y,z_cordinate,1,box_dimension_);
                marker_z_cordinate = z_cordinate;
                result = groundNonGroundExtraction(convex_cloud,1);
                
                if(result == -1){
                    //ROS_INFO("Return on 1");
                    //return current_node;
                }else if(result == 1){
                    //ROS_INFO("ok 1");
                    struct Graph_Node *temp_node = new Graph_Node;
                    temp_node->x_cordinate = front_x;
                    temp_node->y_cordinate = front_y;
                    temp_node->z_cordinate = marker_z_cordinate;
                    temp_node->predecessor = current_node;
                    temp_node->path_cost = current_node->path_cost+1;
                    temp_node->marker_z_cordinate = marker_z_cordinate;
                    found_nodes->setValue(front_x,front_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    ROS_INFO("1 %f %f %f",front_x,front_y,z_cordinate);
                    
                }else{
                    found_obstacles->setValue(front_x,front_y,z_cordinate,0);
                }
            }else if(!found_obstacles->hasValue(front_x,front_y,z_cordinate)){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = front_x;
                temp_node->y_cordinate = front_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                if(found_nodes->getValue(front_x,front_y,z_cordinate)->path_cost>temp_node->path_cost){
                    found_nodes->getValue(front_x,front_y,z_cordinate)->predecessor = current_node;
                    found_nodes->getValue(front_x,front_y,z_cordinate)->path_cost = temp_node->path_cost;
                }
            }
        }

        float front_left_x = x_cordinate + box_dimension_;
        float front_left_y = y_cordinate + box_dimension_;
    
        if(isRobotInside(front_left_x,front_left_y,z_cordinate,target_x_cordinate,target_y_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = target_x_cordinate;
            temp_node->y_cordinate = target_y_cordinate;
            temp_node->z_cordinate = marker_z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1.414;
            return temp_node;
        }else{
            if(!found_nodes->hasValue(front_left_x,front_left_y,z_cordinate) && !found_obstacles->hasValue(front_left_x,front_left_y,z_cordinate)){
                convex_cloud = getConvexHull(surrounding_cloud,front_left_x,front_left_y,z_cordinate,2,box_dimension_);
                marker_z_cordinate = z_cordinate;
                result = groundNonGroundExtraction(convex_cloud,2); 
                if(result == -1){
                    // ROS_INFO("Return on 2");
                    // return current_node;
                }else if(result == 1){
                    struct Graph_Node *temp_node = new Graph_Node;
                    temp_node->x_cordinate = front_left_x;
                    temp_node->y_cordinate = front_left_y;
                    temp_node->z_cordinate = z_cordinate;
                    temp_node->predecessor = current_node;
                    temp_node->path_cost = current_node->path_cost+1.414;
                    temp_node->marker_z_cordinate = marker_z_cordinate;
                    found_nodes->setValue(front_left_x,front_left_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                        ROS_INFO("2 %f %f %f",front_left_x,front_left_y,z_cordinate);
                    
                }else{
                    found_obstacles->setValue(front_left_x,front_left_y,z_cordinate,0);
                }
            }else if(!found_obstacles->hasValue(front_left_x,front_left_y,z_cordinate)){

                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = front_left_x;
                temp_node->y_cordinate = front_left_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;
                if(found_nodes->getValue(front_left_x,front_left_y,z_cordinate)->path_cost>temp_node->path_cost){
                    found_nodes->getValue(front_left_x,front_left_y,z_cordinate)->predecessor = current_node;
                    found_nodes->getValue(front_left_x,front_left_y,z_cordinate)->path_cost = temp_node->path_cost;
                }
            }
        }   

        float left_x = x_cordinate;
        float left_y = y_cordinate + box_dimension_;

        if(isRobotInside(left_x,left_y,z_cordinate,target_x_cordinate,target_y_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = target_x_cordinate;
            temp_node->y_cordinate = target_y_cordinate;
            temp_node->z_cordinate = marker_z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1;
            return temp_node;
        }else{
            if(!found_nodes->hasValue(left_x,left_y,z_cordinate) && !found_obstacles->hasValue(left_x,left_y,z_cordinate)){
                convex_cloud = getConvexHull(surrounding_cloud,left_x,left_y,z_cordinate,3,box_dimension_);
                marker_z_cordinate = z_cordinate;
                result = groundNonGroundExtraction(convex_cloud,3);
                if(result == -1){
                    // ROS_INFO("Return on 3");
                    // return current_node;
                }else if(result == 1){
                    struct Graph_Node *temp_node = new Graph_Node;
                    temp_node->x_cordinate = left_x;
                    temp_node->y_cordinate = left_y;
                    temp_node->z_cordinate = z_cordinate;
                    temp_node->predecessor = current_node;
                    temp_node->path_cost = current_node->path_cost+1;
                    temp_node->marker_z_cordinate = marker_z_cordinate;
                    found_nodes->setValue(left_x,left_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                        ROS_INFO("3 %f %f %f",left_x,left_y,z_cordinate);
                    
                }else{
                    found_obstacles->setValue(left_x,left_y,z_cordinate,0);
                } 
            }else if(!found_obstacles->hasValue(left_x,left_y,z_cordinate)){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = left_x;
                temp_node->y_cordinate = left_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                if(found_nodes->getValue(left_x,left_y,z_cordinate)->path_cost>temp_node->path_cost){
                    found_nodes->getValue(left_x,left_y,z_cordinate)->predecessor = current_node;
                    found_nodes->getValue(left_x,left_y,z_cordinate)->path_cost = temp_node->path_cost;
                }
            }
        }
        

        float back_left_x = x_cordinate - box_dimension_;
        float back_left_y = y_cordinate + box_dimension_;

        if(isRobotInside(back_left_x,back_left_y,z_cordinate,target_x_cordinate,target_y_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = target_x_cordinate;
            temp_node->y_cordinate = target_y_cordinate;
            temp_node->z_cordinate = marker_z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1.414;
            return temp_node;
        }else{
            if(!found_nodes->hasValue(back_left_x,back_left_y,z_cordinate) && !found_obstacles->hasValue(back_left_x,back_left_y,z_cordinate)){
                convex_cloud = getConvexHull(surrounding_cloud,back_left_x,back_left_y,z_cordinate,4,box_dimension_);
                marker_z_cordinate = z_cordinate;
                result = groundNonGroundExtraction(convex_cloud,4);
                if(result == -1){
                    // ROS_INFO("Return on 4");
                    // return current_node;
                }else if(result == 1){
                    struct Graph_Node *temp_node = new Graph_Node;
                    temp_node->x_cordinate = back_left_x;
                    temp_node->y_cordinate = back_left_y;
                    temp_node->z_cordinate = z_cordinate;
                    temp_node->predecessor = current_node;
                    temp_node->path_cost = current_node->path_cost+1.414;
                    temp_node->marker_z_cordinate = marker_z_cordinate;
                    found_nodes->setValue(back_left_x,back_left_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    ROS_INFO("4 %f %f %f",back_left_x,back_left_y,z_cordinate);
                    
                }else{
                    found_obstacles->setValue(back_left_x,back_left_y,z_cordinate,0);
                }
            }else if(!found_obstacles->hasValue(back_left_x,back_left_y,z_cordinate)){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = back_left_x;
                temp_node->y_cordinate = back_left_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;
                if(found_nodes->getValue(back_left_x,back_left_y,z_cordinate)->path_cost>temp_node->path_cost){
                    found_nodes->getValue(back_left_x,back_left_y,z_cordinate)->predecessor = current_node;
                    found_nodes->getValue(back_left_x,back_left_y,z_cordinate)->path_cost = temp_node->path_cost;
                }
            }
        }

        float back_x = x_cordinate - box_dimension_;
        float back_y = y_cordinate;

        if(isRobotInside(back_x,back_y,z_cordinate,target_x_cordinate,target_y_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = target_x_cordinate;
            temp_node->y_cordinate = target_y_cordinate;
            temp_node->z_cordinate = marker_z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1;
            return temp_node;
        }else{
            if(!found_nodes->hasValue(back_x,back_y,z_cordinate) && !found_obstacles->hasValue(back_x,back_y,z_cordinate)){
                convex_cloud = getConvexHull(surrounding_cloud,back_x,back_y,z_cordinate,5,box_dimension_);
                marker_z_cordinate = z_cordinate;
                result = groundNonGroundExtraction(convex_cloud,5);
                if(result == -1){
                    // ROS_INFO("Return on 5");
                    // return current_node;
                }else if(result == 1){
                    struct Graph_Node *temp_node = new Graph_Node;
                    temp_node->x_cordinate = back_x;
                    temp_node->y_cordinate = back_y;
                    temp_node->z_cordinate = z_cordinate;
                    temp_node->predecessor = current_node;
                    temp_node->path_cost = current_node->path_cost+1;
                    temp_node->marker_z_cordinate = marker_z_cordinate;
                    found_nodes->setValue(back_x,back_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    ROS_INFO("5 %f %f %f",back_x,back_y,z_cordinate);
                    
                } else{
                    found_obstacles->setValue(back_x,back_y,z_cordinate,0);
                }
            }else if(!found_obstacles->hasValue(back_x,back_y,z_cordinate)){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = back_x;
                temp_node->y_cordinate = back_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                if(found_nodes->getValue(back_x,back_y,z_cordinate)->path_cost>temp_node->path_cost){
                    found_nodes->getValue(back_x,back_y,z_cordinate)->predecessor = current_node;
                    found_nodes->getValue(back_x,back_y,z_cordinate)->path_cost = temp_node->path_cost;
                }
            }
        }

        float back_right_x = x_cordinate - box_dimension_;
        float back_right_y = y_cordinate - box_dimension_;
        
        if(isRobotInside(back_right_x,back_right_y,z_cordinate,target_x_cordinate,target_y_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = target_x_cordinate;
            temp_node->y_cordinate = target_y_cordinate;
            temp_node->z_cordinate = marker_z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1.414;
            return temp_node;
        }else{
            if(!found_nodes->hasValue(back_right_x,back_right_y,z_cordinate) && !found_obstacles->hasValue(back_right_x,back_right_y,z_cordinate)){
                convex_cloud = getConvexHull(surrounding_cloud,back_right_x,back_right_y,z_cordinate,6,box_dimension_);
                marker_z_cordinate = z_cordinate;
                result = groundNonGroundExtraction(convex_cloud,6);
                if(result == -1){
                    // ROS_INFO("Return on 6");
                    // return current_node;
                }else if(result == 1){
                    struct Graph_Node *temp_node = new Graph_Node;
                    temp_node->x_cordinate = back_right_x;
                    temp_node->y_cordinate = back_right_y;
                    temp_node->z_cordinate = z_cordinate;
                    temp_node->predecessor = current_node;
                    temp_node->path_cost = current_node->path_cost+1.414;
                    temp_node->marker_z_cordinate = marker_z_cordinate;       
                    found_nodes->setValue(back_right_x,back_right_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    ROS_INFO("6 %f %f %f",back_right_x,back_right_y,z_cordinate);
                    
                } else{
                    found_obstacles->setValue(back_right_x,back_right_y,z_cordinate,0);
                }
            }else if(!found_obstacles->hasValue(back_right_x,back_right_y,z_cordinate)){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = back_right_x;
                temp_node->y_cordinate = back_right_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;
                if(found_nodes->getValue(back_right_x,back_right_y,z_cordinate)->path_cost>temp_node->path_cost){
                    found_nodes->getValue(back_right_x,back_right_y,z_cordinate)->predecessor = current_node;
                    found_nodes->getValue(back_right_x,back_right_y,z_cordinate)->path_cost = temp_node->path_cost;
                }
            }
        }

        float right_x = x_cordinate;
        float right_y = y_cordinate - box_dimension_;

        if(isRobotInside(right_x,right_y,z_cordinate,target_x_cordinate,target_y_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = target_x_cordinate;
            temp_node->y_cordinate = target_y_cordinate;
            temp_node->z_cordinate = marker_z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1;
            return temp_node;
        }else{
            if(!found_nodes->hasValue(right_x,right_y,z_cordinate) && !found_obstacles->hasValue(right_x,right_y,z_cordinate)){
                convex_cloud = getConvexHull(surrounding_cloud,right_x,right_y,z_cordinate,7,box_dimension_);
                marker_z_cordinate = z_cordinate;
                result = groundNonGroundExtraction(convex_cloud,7);
                if(result == -1){
                    // ROS_INFO("Return on 7");
                    // return current_node;
                }else if(result == 1){
                    struct Graph_Node *temp_node = new Graph_Node;
                    temp_node->x_cordinate = right_x;
                    temp_node->y_cordinate = right_y;
                    temp_node->z_cordinate = z_cordinate;
                    temp_node->predecessor = current_node;
                    temp_node->path_cost = current_node->path_cost+1;
                    temp_node->marker_z_cordinate = marker_z_cordinate;
                    found_nodes->setValue(right_x,right_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    ROS_INFO("7 %f %f %f",right_x,right_y,z_cordinate);
                    
                } else{
                    found_obstacles->setValue(right_x,right_y,z_cordinate,0);
                }
            }else if(!found_obstacles->hasValue(right_x,right_y,z_cordinate)){
                ROS_INFO("Printed");
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = right_x;
                temp_node->y_cordinate = right_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                if(found_nodes->getValue(right_x,right_y,z_cordinate)->path_cost>temp_node->path_cost){
                    found_nodes->getValue(right_x,right_y,z_cordinate)->predecessor = current_node;
                    found_nodes->getValue(right_x,right_y,z_cordinate)->path_cost = temp_node->path_cost;
                }
            }
        }

        float front_right_x = x_cordinate + box_dimension_;
        float front_right_y = y_cordinate - box_dimension_;

        if(isRobotInside(front_right_x,front_right_y,z_cordinate,target_x_cordinate,target_y_cordinate)){
            struct Graph_Node *temp_node = new Graph_Node;
            temp_node->x_cordinate = target_x_cordinate;
            temp_node->y_cordinate = target_y_cordinate;
            temp_node->z_cordinate = marker_z_cordinate;
            temp_node->predecessor = current_node;
            temp_node->path_cost = current_node->path_cost+1.414;
            return temp_node;
        }else{
            if(!found_nodes->hasValue(front_right_x,front_right_y,z_cordinate) && !found_obstacles->hasValue(front_right_x,front_right_y,z_cordinate)){
                convex_cloud = getConvexHull(surrounding_cloud,front_right_x,front_right_y,z_cordinate,8,box_dimension_);
                marker_z_cordinate = z_cordinate;
                result = groundNonGroundExtraction(convex_cloud,8);
                if(result == -1){
                    // ROS_INFO("Return on 8");
                    // return current_node;
                }else if(result == 1){
                    struct Graph_Node *temp_node = new Graph_Node;
                    temp_node->x_cordinate = front_right_x;
                    temp_node->y_cordinate = front_right_y;
                    temp_node->z_cordinate = z_cordinate;
                    temp_node->predecessor = current_node;
                    temp_node->path_cost = current_node->path_cost+1.414;
                    temp_node->marker_z_cordinate = marker_z_cordinate;
                    found_nodes->setValue(front_right_x,front_right_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    ROS_INFO("8 %f %f %f",front_right_x,front_right_y,z_cordinate);

                }else{
                    found_obstacles->setValue(front_right_x,front_right_y,z_cordinate,0);
                } 
            }else if(!found_obstacles->hasValue(front_right_x,front_right_y,z_cordinate)){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = front_right_x;
                temp_node->y_cordinate = front_right_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;
                if(found_nodes->getValue(front_right_x,front_right_y,z_cordinate)->path_cost>temp_node->path_cost){
                    found_nodes->getValue(front_right_x,front_right_y,z_cordinate)->predecessor = current_node;
                    found_nodes->getValue(front_right_x,front_right_y,z_cordinate)->path_cost = temp_node->path_cost;
                }
            }
        }
    }

    if(node_queue.empty()){
        ROS_INFO("No Paths Found");
        return NULL;
    }else{
        struct Graph_Node *next_node = node_queue.front();
        node_queue.pop();
        //ROS_INFO("Started Braeadth Search");
        current_node=next_node;
        return breadthFirstSearchToGoal(next_node->x_cordinate,next_node->y_cordinate,next_node->z_cordinate,target_x_cordinate,target_y_cordinate,target_z_cordinate);
        //ROS_INFO("Returned Braeadth Goal Search");
    }    
} 

std::vector<geometry_msgs::PoseStamped> NavigationPlanner::publishPath(struct Graph_Node *node){
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    std::vector<geometry_msgs::PoseStamped> plan;
    if(node == NULL){
        ROS_INFO("NO PATH PLANNED");
    }else{    
        struct Graph_Node *temp_node = node;
        geometry_msgs::PoseStamped pose;
        struct Graph_Node *temp_node1 = temp_node;
        while(temp_node1!=NULL){
            pose.pose.position.x = temp_node1->x_cordinate;
            pose.pose.position.y = temp_node1->y_cordinate;
            pose.pose.position.z = temp_node1->marker_z_cordinate;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 0.0;
            ROS_INFO("X %f , Y %f , Z %f",temp_node1->x_cordinate, temp_node1->y_cordinate, temp_node1->marker_z_cordinate);
            plan.push_back(pose);    
            temp_node1 = temp_node1->predecessor;
        }
        current_node=NULL;
        ROS_INFO("PATH PLANNED");
    }
    return plan;
  
}

std::vector<geometry_msgs::PoseStamped> NavigationPlanner::publishPathToTarget(struct Graph_Node *node){
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    std::vector<geometry_msgs::PoseStamped> plan;
    if(node == NULL){
        ROS_INFO("NO PATH PLANNED");
    }else{    
        struct Graph_Node *temp_node = node;
        geometry_msgs::PoseStamped pose;
        struct Graph_Node *temp_node1 = temp_node;
        while(temp_node1!=NULL){
            pose.pose.position.x = temp_node1->x_cordinate;
            pose.pose.position.y = temp_node1->y_cordinate;
            pose.pose.position.z = temp_node1->z_cordinate;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 0.0;
            ROS_INFO("X %f , Y %f , Z %f",temp_node1->x_cordinate, temp_node1->y_cordinate, temp_node1->marker_z_cordinate);
            plan.push_back(pose);    
            temp_node1 = temp_node1->predecessor;
        }
        current_node=NULL;
        ROS_INFO("PATH PLANNED");
    }
    return plan;
  
}

void NavigationPlanner::getFrontPlane(const geometry_msgs::PoseStamped& pose){
    std::cout << "----0------" << std::endl;
    float x_cordinate = pose.pose.position.x;
    float y_cordinate = pose.pose.position.y;
    float z_cordinate = pose.pose.position.z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surrounding_cloud = segmentBoundingCube(cloud,current_z_node-robot_height_,current_z_node+robot_height_);
    convex_cloud0 = getConvexHull(surrounding_cloud,x_cordinate,y_cordinate,z_cordinate,8,box_dimension_);        
    int result = groundNonGroundExtraction(convex_cloud0, 0);
    ROS_INFO("%d",result);
    std::cout << "----1------" << std::endl;
    x_cordinate = pose.pose.position.x+1.0;
    y_cordinate = pose.pose.position.y;
    z_cordinate = pose.pose.position.z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud1;
    convex_cloud1 = getConvexHull(surrounding_cloud,x_cordinate,y_cordinate,z_cordinate,8,box_dimension_);        
    result = groundNonGroundExtraction(convex_cloud1, 1);
    ROS_INFO("%d",result);
    std::cout << "----2------" << std::endl;
    x_cordinate = pose.pose.position.x+1.0;
    y_cordinate = pose.pose.position.y+1.0;
    z_cordinate = pose.pose.position.z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud2;
    convex_cloud2 = getConvexHull(surrounding_cloud,x_cordinate,y_cordinate,z_cordinate,8,box_dimension_);        
    result = groundNonGroundExtraction(convex_cloud2, 2);
    ROS_INFO("%d",result);
    std::cout << "----3------" << std::endl;
    x_cordinate = pose.pose.position.x;
    y_cordinate = pose.pose.position.y+1.0;
    z_cordinate = pose.pose.position.z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud3;
    convex_cloud3 = getConvexHull(surrounding_cloud,x_cordinate,y_cordinate,z_cordinate,8,box_dimension_);        
    result = groundNonGroundExtraction(convex_cloud3, 3);
    ROS_INFO("%d",result);
    std::cout << "----4------" << std::endl;
    x_cordinate = pose.pose.position.x-1.0;
    y_cordinate = pose.pose.position.y+1.0;
    z_cordinate = pose.pose.position.z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud4;
    convex_cloud4 = getConvexHull(surrounding_cloud,x_cordinate,y_cordinate,z_cordinate,8,box_dimension_);        
    result = groundNonGroundExtraction(convex_cloud4, 4);
    ROS_INFO("%d",result);
    std::cout << "----5------" << std::endl;
    x_cordinate = pose.pose.position.x-1.0;
    y_cordinate = pose.pose.position.y;
    z_cordinate = pose.pose.position.z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud5;
    convex_cloud4 = getConvexHull(surrounding_cloud,x_cordinate,y_cordinate,z_cordinate,8,box_dimension_);        
    result = groundNonGroundExtraction(convex_cloud5, 5);
    ROS_INFO("%d",result);
    std::cout << "----6------" << std::endl;
    x_cordinate = pose.pose.position.x-1.0;
    y_cordinate = pose.pose.position.y-1.0;
    z_cordinate = pose.pose.position.z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud6;
    convex_cloud4 = getConvexHull(surrounding_cloud,x_cordinate,y_cordinate,z_cordinate,8,box_dimension_);        
    result = groundNonGroundExtraction(convex_cloud6, 6);
    ROS_INFO("%d",result);
    std::cout << "----7------" << std::endl;
    x_cordinate = pose.pose.position.x;
    y_cordinate = pose.pose.position.y-1.0;
    z_cordinate = pose.pose.position.z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud7;
    convex_cloud4 = getConvexHull(surrounding_cloud,x_cordinate,y_cordinate,z_cordinate,8,box_dimension_);        
    result = groundNonGroundExtraction(convex_cloud7, 7);
    ROS_INFO("%d",result);
    std::cout << "----8------" << std::endl;
    x_cordinate = pose.pose.position.x+1.0;
    y_cordinate = pose.pose.position.y-1.0;
    z_cordinate = pose.pose.position.z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud8;
    convex_cloud4 = getConvexHull(surrounding_cloud,x_cordinate,y_cordinate,z_cordinate,8,box_dimension_);        
    result = groundNonGroundExtraction(convex_cloud8, 8);
    ROS_INFO("%d",result);

}

std::vector<geometry_msgs::PoseStamped> NavigationPlanner::getNavPlan(const geometry_msgs::PoseStamped& pose){
    ROS_INFO("1");
    float x_cordinate = pose.pose.position.x;
    float y_cordinate = pose.pose.position.y;
    float z_cordinate = pose.pose.position.z;
    found_nodes = new Array3D<struct Graph_Node*>;
    found_obstacles = new Array3D<struct Graph_Node*>;
    current_node =  new Graph_Node;
    current_node->x_cordinate = x_cordinate;
    current_node->y_cordinate = y_cordinate;
    current_node->z_cordinate = z_cordinate;
    current_node->path_cost = 0;
    current_node->predecessor = NULL;
    found_nodes->setValue(x_cordinate,y_cordinate,z_cordinate,current_node);
    ROS_INFO("2");
    
    struct Graph_Node *node = breadthFirstSearch(x_cordinate,y_cordinate,z_cordinate);
    ROS_INFO("3");

    while(!node_queue.empty()){
        node_queue.pop();
    }
    return publishPath(node);
}

std::vector<geometry_msgs::PoseStamped> NavigationPlanner::getNavPlanToTarget(const geometry_msgs::PoseStamped& pose,const geometry_msgs::PoseStamped& target){
    ROS_INFO("1");
    float x_cordinate = pose.pose.position.x;
    float y_cordinate = pose.pose.position.y;
    float z_cordinate = pose.pose.position.z;

    float target_x_cordinate = target.pose.position.x;
    float target_y_cordinate = target.pose.position.y;
    float target_z_cordinate = target.pose.position.z;

    found_nodes = new Array3D<struct Graph_Node*>;
    found_obstacles = new Array3D<struct Graph_Node*>;
    current_node =  new Graph_Node;
    current_node->x_cordinate = x_cordinate;
    current_node->y_cordinate = y_cordinate;
    current_node->z_cordinate = z_cordinate;
    current_node->path_cost = 0;
    current_node->predecessor = NULL;
    found_nodes->setValue(x_cordinate,y_cordinate,z_cordinate,current_node);
    ROS_INFO("2");
    
    struct Graph_Node *node = breadthFirstSearchToGoal(x_cordinate,y_cordinate,z_cordinate,target_x_cordinate,target_y_cordinate,target_z_cordinate);
    ROS_INFO("3");

    while(!node_queue.empty()){
        node_queue.pop();
    }
    return publishPathToTarget(node);
}

void NavigationPlanner::startTraversal(const geometry_msgs::PoseStamped& pose){
    float x_cordinate = pose.pose.position.x;
    float y_cordinate = pose.pose.position.y;
    float z_cordinate = pose.pose.position.z;
    
    if(found_nodes->hasValue(x_cordinate,y_cordinate,z_cordinate)){
        current_node = found_nodes->getValue(x_cordinate,y_cordinate,z_cordinate);
        current_node->path_cost = 0;
        found_nodes->setValue(x_cordinate,y_cordinate,z_cordinate,current_node);
    }else{
        current_node =  new Graph_Node;
        current_node->x_cordinate = x_cordinate;
        current_node->y_cordinate = y_cordinate;
        current_node->z_cordinate = z_cordinate;
        current_node->path_cost = 0;
        current_node->predecessor = NULL;
        found_nodes->setValue(x_cordinate,y_cordinate,z_cordinate,current_node);
    }
    
    ROS_INFO("Start Node X %f , Y %f , Z %f",x_cordinate, y_cordinate, z_cordinate);
    struct Graph_Node *node = breadthFirstSearch(x_cordinate,y_cordinate,z_cordinate);
    ROS_INFO("FOUND NULL TRAVERSE");
    publishPath(node);
}


void NavigationPlanner::start(){
    subscriber_node = node_handle_.subscribe(topic_, 1000, &NavigationPlanner::cloudCallback,this);
    // subscriber_node = node_handle_.subscribe(topic_, 1000, cloud_call_back);
    // ros::spin();
}

