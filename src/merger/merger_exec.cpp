#include "merger/merger.h"


octomap::OcTree* octomap_mine;
octomap::OcTree* octomap_other;

int argc_;
char** argv_;

void getTreeFromRobot1(const octomap_msgs::OctomapConstPtr& msg){
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    octomap_mine = dynamic_cast<OcTree*>(tree);
}

void getTreeFromRobot2(const octomap_msgs::OctomapConstPtr& msg){
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    octomap_other = dynamic_cast<OcTree*>(tree);
}

void mergeOctomapsFromTopic(){
    Merger* merger = new Merger();

    std::string outputFileName = "OcTree.ot";

    cout<< "\nReading octree files......\n";

    double roll, pitch, yaw;
    double res = 0.05;

    point3d translation;
    if(argc_ == 7 || argc_ == 10){
        translation = point3d(atof(argv_[4]),atof(argv_[5]),atof(argv_[6]));
    }else{
        translation = point3d(0,0,0);
    }

    if(argc_ == 10){
        roll = atof(argv_[7]);
        pitch = atof(argv_[8]);
        yaw = atof(argv_[9]);
    }else{
        roll = 0;
        pitch = 0;
        yaw = 0;
        // yaw = 90;
    }

    Pose6D pose(translation.x(),translation.y(),translation.z(),roll,pitch,yaw);
    Eigen::Matrix4f transform;
    std::vector<double> coeffs;
    pose.rot().toRotMatrix(coeffs);

    transform<< coeffs[0], coeffs[1], coeffs[2], translation.x(),
                coeffs[3], coeffs[4], coeffs[5], translation.y(),
                coeffs[6], coeffs[7], coeffs[8], translation.z(),
                0,0,0,1;
    
    OcTree* tree1 = octomap_mine;
    OcTree* tree2 = octomap_other;

    std::cout<<transform<<std::endl;

    cout<<"Registering map to improve tf estimate" << endl << endl;

    pcl::PointCloud<pcl::PointXYZ> tree1Points;
    merger->tree2PointCloud(tree1, tree1Points);
    pcl::PointCloud<pcl::PointXYZ> tree2Points;
    merger->tree2PointCloud(tree2,tree2Points);

    transform = merger->getICPTransformation(tree1Points, tree2Points, transform,res);

    //printf("passedICP %d\n",1);
    if(roll != 0 || pitch != 0 || yaw != 0 || translation.x() != 0 || translation.y() != 0 || translation.z() != 0){
        merger->transformTree(tree2,transform);
    }

    for(OcTree::leaf_iterator it = tree2->begin_leafs(); it != tree2->end_leafs(); ++it){
        if(tree2->isNodeOccupied(*it)){
            it->setLogOdds(logodds(0.6));
        }

        OcTreeNode *nodeIn1 = tree1->search(it.getCoordinate());
        OcTreeKey nodeKey = tree1->coordToKey(it.getCoordinate());
        point3d point = it.getCoordinate();
        if(nodeIn1 != NULL){
            int depthIn1 = merger->getNodeDepth(tree1, point, nodeIn1);
            if(depthIn1 != -1){
                int depthDiff = it.getDepth() - depthIn1;
                if(depthDiff == 0){
                    tree1->updateNode(nodeKey, it->getLogOdds());
                }else if(depthDiff > 0){
                    for(int i=0; i<depthDiff; i++){
                        if(depthIn1 == (int)tree1->getTreeDepth()){
                            break;
                        }
                        cout<<"Reached first expand" << endl << endl;
                        nodeIn1->expandNode();
                        nodeKey = tree1->coordToKey(point);
                        depthIn1++;
                    }
                    nodeIn1->setLogOdds(logodds(nodeIn1->getOccupancy()+it->getOccupancy()));
                }else if(depthDiff<0){
                    merger->expandNodeMultiLevel(tree2,tree2->search(point),it.getDepth(),abs(depthDiff));
                }
            }
        }else{
            OcTreeNode* newNode = tree1->updateNode(point,true);
            newNode->setLogOdds(it->getLogOdds());
        }
    }

    std::cout<<"Compressing merged result\n";
    tree1->prune();
    tree1->write(outputFileName);
    delete tree1;
    delete tree2;
}

void mergeOctomapsFromFile(std::string filename1,std::string filename2){
    Merger* merger = new Merger();

    std::string outputFileName = "OcTree.ot";

    cout<< "\nReading octree files......\n";

    double roll, pitch, yaw;
    double res = 0.05;

    point3d translation;
    if(argc_ == 7 || argc_ == 10){
        translation = point3d(atof(argv_[4]),atof(argv_[5]),atof(argv_[6]));
    }else{
        translation = point3d(0,0,0);
    }

    if(argc_ == 10){
        roll = atof(argv_[7]);
        pitch = atof(argv_[8]);
        yaw = atof(argv_[9]);
    }else{
        roll = 0;
        pitch = 0;
        yaw = 0;
        // yaw = 90;
    }

    Pose6D pose(translation.x(),translation.y(),translation.z(),roll,pitch,yaw);
    Eigen::Matrix4f transform;
    std::vector<double> coeffs;
    pose.rot().toRotMatrix(coeffs);

    transform<< coeffs[0], coeffs[1], coeffs[2], translation.x(),
                coeffs[3], coeffs[4], coeffs[5], translation.y(),
                coeffs[6], coeffs[7], coeffs[8], translation.z(),
                0,0,0,1;
    
    OcTree* tree1 = dynamic_cast<OcTree*>(OcTree::read(filename1));
    OcTree* tree2 = dynamic_cast<OcTree*>(OcTree::read(filename2));

    std::cout<<transform<<std::endl;

    cout<<"Registering map to improve tf estimate" << endl << endl;

    pcl::PointCloud<pcl::PointXYZ> tree1Points;
    merger->tree2PointCloud(tree1, tree1Points);
    pcl::PointCloud<pcl::PointXYZ> tree2Points;
    merger->tree2PointCloud(tree2,tree2Points);

    transform = merger->getICPTransformation(tree1Points, tree2Points, transform,res);

    //printf("passedICP %d\n",1);
    if(roll != 0 || pitch != 0 || yaw != 0 || translation.x() != 0 || translation.y() != 0 || translation.z() != 0){
        merger->transformTree(tree2,transform);
    }

    for(OcTree::leaf_iterator it = tree2->begin_leafs(); it != tree2->end_leafs(); ++it){
        if(tree2->isNodeOccupied(*it)){
            it->setLogOdds(logodds(0.6));
        }

        OcTreeNode *nodeIn1 = tree1->search(it.getCoordinate());
        OcTreeKey nodeKey = tree1->coordToKey(it.getCoordinate());
        point3d point = it.getCoordinate();
        if(nodeIn1 != NULL){
            int depthIn1 = merger->getNodeDepth(tree1, point, nodeIn1);
            if(depthIn1 != -1){
                int depthDiff = it.getDepth() - depthIn1;
                if(depthDiff == 0){
                    tree1->updateNode(nodeKey, it->getLogOdds());
                }else if(depthDiff > 0){
                    for(int i=0; i<depthDiff; i++){
                        if(depthIn1 == (int)tree1->getTreeDepth()){
                            break;
                        }
                        cout<<"Reached first expand" << endl << endl;
                        nodeIn1->expandNode();
                        nodeKey = tree1->coordToKey(point);
                        depthIn1++;
                    }
                    nodeIn1->setLogOdds(logodds(nodeIn1->getOccupancy()+it->getOccupancy()));
                }else if(depthDiff<0){
                    merger->expandNodeMultiLevel(tree2,tree2->search(point),it.getDepth(),abs(depthDiff));
                }
            }
        }else{
            OcTreeNode* newNode = tree1->updateNode(point,true);
            newNode->setLogOdds(it->getLogOdds());
        }
    }

    std::cout<<"Compressing merged result\n";
    tree1->prune();
    tree1->write(outputFileName);
    delete tree1;
    delete tree2;
}

void mergeTree(const geometry_msgs::PoseStamped& pose){
    mergeOctomapsFromTopic();
}

int main(int argc, char** argv){
    
    argc_ = argc;
    argv_ = argv;
    ros::init(argc, argv, "merger");
    ros::NodeHandle nh;
    std::string topic_octomap_mine;
    std::string topic_octomap_other;
    ros::NodeHandle private_nh("~"); 

    private_nh.param("topic_octomap_mine", topic_octomap_mine, std::string("/cmd_vel")); 
    private_nh.param("topic_octomap_other", topic_octomap_other, std::string("/odom")); 

    ros::NodeHandle simple_nh("move_base_simple");
    ros::Subscriber goal_sub_ = simple_nh.subscribe("goal", 1, mergeTree);
    ros::Subscriber merger_sub_mine_ = nh.subscribe(topic_octomap_mine, 1, getTreeFromRobot1);
    ros::Subscriber merger_sub_other_ = nh.subscribe(topic_octomap_other, 1, getTreeFromRobot2);

    ros::spin();

    return 0;
}

// int main(int argc, char** argv){
//     argc_ = argc;
//     argv_ = argv;
//     mergeOctomapsFromFile("map1.ot","map2.ot");
// }