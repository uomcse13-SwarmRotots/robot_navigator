#include "planner/navigation_planner.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
struct Graph_Node *current_node;
float current_x_node;
float current_y_node;
float current_z_node;

bool NavigationPlanner::planeTraversabilityCheck(double a,double b,double c,double e, double x0,double y0,double z0, double d){
	double c2_ = c/(sqrt(a*a+b*b+c*c));
	double angle = acos(c2_);
	//Print angle	
	double acceptedAngle 	= 0.349066; // 20 degree
	double acceptedDistance = (d/2)*sin(angle);
	//checking the traversability
	//Angle check
	if(angle<acceptedAngle){ 
		//distance calculation and check
		//double upper = a*x0 + b*y0 + c*z0 + d;
		//double lower = sqrt(a*a+b*b+c*c);
		double distance = (a*x0 + b*y0 + c*z0 + e)/sqrt(a*a+b*b+c*c);
		//Print distance
		//take evaluationg decision based on values
		return true;
		
	}
	else return false;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr NavigationPlanner::calculateConvexHull(vector<pcl::PointXYZ> point_vector,int point_type){
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

    cropHullFilter.setInputCloud(cloud);
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

pcl::PointCloud<pcl::PointXYZ>::Ptr NavigationPlanner::getConvexHull(float x_cordinate, float y_cordinate, float z_cordinate, int point_type, float box_dimension){
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

    return calculateConvexHull(point_vector,point_type);

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
    return planeTraversabilityCheck(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3], current_x_node,current_y_node,current_z_node, 0.5);
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
    //ROS_INFO("Reached");
    if(cloud_cube->size()>20){
        pcl::PointIndicesPtr ground (new pcl::PointIndices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
        pmf.setInputCloud (cloud_cube);
        pmf.setMaxWindowSize (20);
        pmf.setSlope (1.0f);
        pmf.setInitialDistance (0.1f);
        pmf.setMaxDistance (3.0f);
        pmf.extract (ground->indices);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_cube);
        extract.setIndices (ground);
        extract.filter (*cloud_filtered);

        pcl::PCDWriter writer;
        if(cloud_filtered->size()>100){
            //writer.write<pcl::PointXYZ> ("samp11-utm_ground.pcd", *cloud_filtered, false);
        }else{
            //ROS_INFO("End %d",-1);
            return -1;
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
            //ROS_INFO("End %d",1);
            return 1;
        }
        //ROS_INFO("END %d",0);
        return 0;
    }else{
        //ROS_INFO("End %d",-1);
        return -1;
    }
    
}

int NavigationPlanner::segmentBoundingCube(float x_cordinate, float y_cordinate, float z_cordinate){

    float resolution = 0.5;
    float x_min = x_cordinate - resolution;
    float x_max = x_cordinate + resolution;
    float y_min = y_cordinate - resolution;
    float y_max = y_cordinate + resolution;
    float z_min = z_cordinate - resolution;
    float z_max = z_cordinate + resolution;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_min, y_max);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_min, x_max);
    pass.filter (*cloud_filtered);

    if(cloud_filtered->size()>0){
        int state = groundNonGroundExtraction(cloud_filtered,0);
        if(state == 1){
            std::cerr << "Can ENTER........" << std::endl;
            return 1;
        }else if(state == 0){
            std::cerr << "Cannot ENTER......." << std::endl;
            return 0;
        }else{
            std::cerr << "No Data Points Near By - Uninitialized:" << std::endl;
            return -1;
        }
    }else{
        std::cerr << "No Data Points Near By - Uninitialized: " << std::endl;
        return -1;
    }
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

NavigationPlanner::~NavigationPlanner(){}



struct Graph_Node* NavigationPlanner::breadthFirstSearch(float x_cordinate, float y_cordinate, float z_cordinate){

    float box_dimension = 0.5; // half of robot length
    current_x_node = x_cordinate;
    current_y_node = y_cordinate;
    current_z_node = z_cordinate;

    if(min_x_cordinate_<x_cordinate && max_x_cordinate_>x_cordinate && min_y_cordinate_<y_cordinate && max_y_cordinate_>y_cordinate){
        pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud;
        //ROS_INFO("size is :  %d",cloud->size());
        int result;
        //ROS_INFO("0 %f %f %f",x_cordinate, y_cordinate, z_cordinate);

        float front_x = x_cordinate + box_dimension;
        float front_y = y_cordinate;
        if(!found_nodes->hasValue(front_x,front_y,z_cordinate) & !found_obstacles->hasValue(front_x,front_y,z_cordinate)){
            convex_cloud = getConvexHull(front_x,front_y,z_cordinate,1,0.50);
            
            result = groundNonGroundExtraction(convex_cloud,1);
            if(result == -1){
                //ROS_INFO("Return on 1");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = front_x;
                temp_node->y_cordinate = front_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                
                    found_nodes->setValue(front_x,front_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    //ROS_INFO("1 %f %f %f",front_x,front_y,z_cordinate);
                
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

        float front_left_x = x_cordinate + box_dimension;
        float front_left_y = y_cordinate + box_dimension;
    
        
        if(!found_nodes->hasValue(front_left_x,front_left_y,z_cordinate) && !found_obstacles->hasValue(front_left_x,front_left_y,z_cordinate)){
            convex_cloud = getConvexHull(front_left_x,front_left_y,z_cordinate,2,0.50);
            
            result = groundNonGroundExtraction(convex_cloud,2); 
            if(result == -1){
                //ROS_INFO("Return on 2");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = front_left_x;
                temp_node->y_cordinate = front_left_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;
                
                    found_nodes->setValue(front_left_x,front_left_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    //ROS_INFO("2 %f %f %f",front_left_x,front_left_y,z_cordinate);
                
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
        float left_y = y_cordinate + box_dimension;
        if(!found_nodes->hasValue(left_x,left_y,z_cordinate) && !found_obstacles->hasValue(left_x,left_y,z_cordinate)){
            convex_cloud = getConvexHull(left_x,left_y,z_cordinate,3,0.50);
            
            result = groundNonGroundExtraction(convex_cloud,3);
            if(result == -1){
                //ROS_INFO("Return on 3");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = left_x;
                temp_node->y_cordinate = left_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                
                    found_nodes->setValue(left_x,left_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    //ROS_INFO("3 %f %f %f",left_x,left_y,z_cordinate);
                
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

        float back_left_x = x_cordinate - box_dimension;
        float back_left_y = y_cordinate + box_dimension;
        if(!found_nodes->hasValue(back_left_x,back_left_y,z_cordinate) && !found_obstacles->hasValue(back_left_x,back_left_y,z_cordinate)){
            convex_cloud = getConvexHull(back_left_x,back_left_y,z_cordinate,4,0.50);
            
            result = groundNonGroundExtraction(convex_cloud,4);
            if(result == -1){
                //ROS_INFO("Return on 4");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = back_left_x;
                temp_node->y_cordinate = back_left_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;
                
                    found_nodes->setValue(back_left_x,back_left_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    //ROS_INFO("4 %f %f %f",back_left_x,back_left_y,z_cordinate);
                
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

        float back_x = x_cordinate - box_dimension;
        float back_y = y_cordinate;
        if(!found_nodes->hasValue(back_x,back_y,z_cordinate) && !found_obstacles->hasValue(back_x,back_y,z_cordinate)){
            convex_cloud = getConvexHull(back_x,back_y,z_cordinate,5,0.50);
            
            result = groundNonGroundExtraction(convex_cloud,5);
            if(result == -1){
                //ROS_INFO("Return on 5");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = back_x;
                temp_node->y_cordinate = back_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                
                
                    found_nodes->setValue(back_x,back_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    //ROS_INFO("5 %f %f %f",back_x,back_y,z_cordinate);
                
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

        float back_right_x = x_cordinate - box_dimension;
        float back_right_y = y_cordinate - box_dimension;
        if(!found_nodes->hasValue(back_right_x,back_right_y,z_cordinate) && !found_obstacles->hasValue(back_right_x,back_right_y,z_cordinate)){
            convex_cloud = getConvexHull(back_right_x,back_right_y,z_cordinate,6,0.50);
            
            result = groundNonGroundExtraction(convex_cloud,6);
            if(result == -1){
                //ROS_INFO("Return on 6");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = back_right_x;
                temp_node->y_cordinate = back_right_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;
                            
                    found_nodes->setValue(back_right_x,back_right_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    //ROS_INFO("6 %f %f %f",back_right_x,back_right_y,z_cordinate);
                
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
        float right_y = y_cordinate - box_dimension;
        if(!found_nodes->hasValue(right_x,right_y,z_cordinate) && !found_obstacles->hasValue(right_x,right_y,z_cordinate)){
            convex_cloud = getConvexHull(right_x,right_y,z_cordinate,7,0.50);
            
            result = groundNonGroundExtraction(convex_cloud,7);
            if(result == -1){
                //ROS_INFO("Return on 7");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = right_x;
                temp_node->y_cordinate = right_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1;
                
                    found_nodes->setValue(right_x,right_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    //ROS_INFO("7 %f %f %f",right_x,right_y,z_cordinate);
                
            } else{
                found_obstacles->setValue(right_x,right_y,z_cordinate,0);
            }
        }else if(!found_obstacles->hasValue(right_x,right_y,z_cordinate)){
            //ROS_INFO("Printed");
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

        float front_right_x = x_cordinate + box_dimension;
        float front_right_y = y_cordinate - box_dimension; 
        if(!found_nodes->hasValue(front_right_x,front_right_y,z_cordinate) && !found_obstacles->hasValue(front_right_x,front_right_y,z_cordinate)){
            convex_cloud = getConvexHull(front_right_x,front_right_y,z_cordinate,8,0.50);
            
            result = groundNonGroundExtraction(convex_cloud,8);
            if(result == -1){
                //ROS_INFO("Return on 8");
                return current_node;
            }else if(result == 1){
                struct Graph_Node *temp_node = new Graph_Node;
                temp_node->x_cordinate = front_right_x;
                temp_node->y_cordinate = front_right_y;
                temp_node->z_cordinate = z_cordinate;
                temp_node->predecessor = current_node;
                temp_node->path_cost = current_node->path_cost+1.414;

                    found_nodes->setValue(front_right_x,front_right_y,z_cordinate,temp_node);
                    node_queue.push(temp_node);
                    //ROS_INFO("8 %f %f %f",front_right_x,front_right_y,z_cordinate);

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
        //ROS_INFO("No Nodes to Traverse");
        return current_node;
    }else{
        struct Graph_Node *next_node = node_queue.front();
        node_queue.pop();
        ////ROS_INFO("Started Braeadth Search");
        current_node=next_node;
        breadthFirstSearch(next_node->x_cordinate,next_node->y_cordinate,next_node->z_cordinate);
        return current_node;
    }    
} 

void NavigationPlanner::setBoundaries(float min_x_cordinate, float max_x_cordinate, float min_y_cordinate,float max_y_cordinate){
    max_x_cordinate_ = max_x_cordinate;
    min_x_cordinate_ = min_x_cordinate;
    max_y_cordinate_ = max_y_cordinate;
    min_y_cordinate_ = min_y_cordinate;
}

std::vector<geometry_msgs::PoseStamped> NavigationPlanner::publishPath(struct Graph_Node *node){
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    struct Graph_Node *temp_node = node;
    geometry_msgs::PoseStamped pose;
    std::vector<geometry_msgs::PoseStamped> plan;
    struct Graph_Node *temp_node1 = temp_node;
    while(temp_node1!=NULL){

        pose.pose.position.x = temp_node1->x_cordinate;
        pose.pose.position.y = temp_node1->y_cordinate;
        pose.pose.position.z = temp_node1->z_cordinate;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 0.0;
        ROS_INFO("X %f , Y %f , Z %f",temp_node1->x_cordinate, temp_node1->y_cordinate, temp_node1->z_cordinate);
        plan.push_back(pose);    
        temp_node1 = temp_node1->predecessor;
    }
    current_node=NULL;
    //ROS_INFO("PATH PLANNED");
    return plan;
  
}

void NavigationPlanner::getFrontPlane(const geometry_msgs::PoseStamped& pose){
    float x_cordinate = pose.pose.position.x+0.5;
    float y_cordinate = pose.pose.position.y;
    float z_cordinate = pose.pose.position.z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_cloud;
    convex_cloud = getConvexHull(x_cordinate,y_cordinate,z_cordinate,8,0.50);        
    int result = groundNonGroundExtraction(convex_cloud,0);
    //ROS_INFO("%d",result);
}

std::vector<geometry_msgs::PoseStamped> NavigationPlanner::getNavPlan(const geometry_msgs::PoseStamped& pose){
    //ROS_INFO("1");
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
    //ROS_INFO("2");
    
    struct Graph_Node *node = breadthFirstSearch(x_cordinate,y_cordinate,z_cordinate);
    //ROS_INFO("3");
    while(!node_queue.empty()){
        node_queue.pop();
    }
    return publishPath(node);
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
    
    //ROS_INFO("Start Node X %f , Y %f , Z %f",x_cordinate, y_cordinate, z_cordinate);
    struct Graph_Node *node = breadthFirstSearch(x_cordinate,y_cordinate,z_cordinate);
    //ROS_INFO("FOUND NULL TRAVERSE");
    publishPath(node);
}


void NavigationPlanner::start(){
    subscriber_node = node_handle_.subscribe(topic_, 1000, &NavigationPlanner::cloudCallback,this);
    // subscriber_node = node_handle_.subscribe(topic_, 1000, cloud_call_back);
    // ros::spin();
}

