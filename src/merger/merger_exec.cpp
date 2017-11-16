#include "merger/merger.h"

OcTree* getTree(const octomap_msgs::OctomapConstPtr& msg){
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    octomap::OcTree* octomap = dynamic_cast<OcTree*>(tree);
    return octomap;
}

int main(int argc, char** argv){
    
    Merger* merger = new Merger();
    std::string filename1 = std::string(argv[1]);
    std::string filename2 = std::string(argv[2]);
    std::string outputFileName = std::string(argv[3]);

    cout<< "\nReading octree files......\n";

    double roll, pitch, yaw;
    double res = 0.05;

    point3d translation;
    if(argc == 7 || argc == 10){
        translation = point3d(atof(argv[4]),atof(argv[5]),atof(argv[6]));
    }else{
        translation = point3d(0,0,0);
    }

    if(argc == 10){
        roll = atof(argv[7]);
        pitch = atof(argv[8]);
        yaw = atof(argv[9]);
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