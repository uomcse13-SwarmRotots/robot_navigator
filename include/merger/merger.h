#ifndef MERGER_INCLUDE
#define MERGER_INCLUDE

#include <ros/ros.h>
#include <Eigen/SVD>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <octomap/OcTree.h>
#include <fstream> 
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <list>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseStamped.h>

#define MAX_ITER 500

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

/*
    namespace initialization
*/
using namespace octomap;
using namespace octomath;
using namespace octomap_msgs;
using namespace geometry_msgs;
using std::cout;
using std::endl;

class Merger{
    private:
        /*
            variables
        */
        ros::Subscriber subscriber_node;
        ros::NodeHandle node_handle_;
        std::string topic_;
        /*
            methods
        */
        // inline float round(float val);
        int getSign(double value);
        bool pointInBBox(pcl::PointXYZ& point, pcl::PointXYZ& bboxMin, pcl::PointXYZ& bboxMax);
        void expandLevel(std::vector<OcTreeNode *> *nodePtrs);
    public:
        /*
            methods
        */
        void tree2PointCloud(OcTree *tree, pcl::PointCloud<pcl::PointXYZ>& pclCloud);
        void transformTree(OcTree *tree, Eigen::Matrix4f& transform);
        Merger(ros::NodeHandle &nh, std::string topic);
        Merger();
        unsigned expandNodeMultiLevel(OcTree *tree, OcTreeNode *node, unsigned currentDepth, unsigned levels);
        int getNodeDepth(OcTree* tree, point3d& point, OcTreeNode* node);
        Eigen::Matrix4f getICPTransformation(pcl::PointCloud<pcl::PointXYZ>& cloud1,pcl::PointCloud<pcl::PointXYZ>& cloud2, Eigen::Matrix4f& tfEst, double mapRes);
        ~Merger();
};
#endif