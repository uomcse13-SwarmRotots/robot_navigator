#include "merger/merger.h"

void Merger::tree2PointCloud(OcTree *tree, pcl::PointCloud<pcl::PointXYZ>& pclCloud){
    cout<<"Acheived tree to point cloud" << endl << endl;
    int count = 0;
    for(OcTree::leaf_iterator it = tree->begin_leafs(),end = tree->end_leafs(); it != end; ++it){
        if(tree->isNodeOccupied(*it)){
            pclCloud.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
            count++;
        }
    }
    //printf("Count is = %d\n",count);
}

int Merger::getSign(double value){
    if(value == 0){
        return 0;
    }else if(value > 0){
        return 1;
    }else{
        return -1;
    }
}

bool Merger::pointInBBox(pcl::PointXYZ& point, pcl::PointXYZ& bboxMin, pcl::PointXYZ& bboxMax){
    return (point.x < bboxMax.x && point.x > bboxMin.x) && (point.y < bboxMax.y && point.y > bboxMin.y) && (point.z < bboxMax.z && point.z > bboxMin.z);
}

Eigen::Matrix4f Merger::getICPTransformation(pcl::PointCloud<pcl::PointXYZ>& cloud1,pcl::PointCloud<pcl::PointXYZ>& cloud2, Eigen::Matrix4f& tfEst, double mapRes){
    cout<<"Started ICPTransformation" << endl << endl;
    pcl::transformPointCloud(cloud2,cloud2,tfEst);
    pcl::PointXYZ minCloud1;
    pcl::PointXYZ maxCloud1;
    pcl::getMinMax3D(cloud1,minCloud1,maxCloud1);
    cout<<"Passed first step ICPTransformation" << endl << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2filtered(new pcl::PointCloud<pcl::PointXYZ>);

    for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud2.begin(); it!=cloud2.end();it++){
        if(pointInBBox(*it,minCloud1,maxCloud1)){
            cloud2filtered->push_back(*it);
        }
    }

    cout<<"Passed Second step ICPTransformation" << endl << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1filtered(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointXYZ minCloud2filtered;
    pcl::PointXYZ maxCloud2filtered;
    pcl::getMinMax3D(*cloud2filtered,minCloud2filtered,maxCloud2filtered);

    minCloud2filtered = pcl::PointXYZ(minCloud2filtered.x-1,minCloud2filtered.y-1,minCloud2filtered.z-1);
    maxCloud2filtered = pcl::PointXYZ(maxCloud2filtered.x+1,maxCloud2filtered.y+1,maxCloud2filtered.z+1);

    for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud1.begin(); it!=cloud1.end(); it++){
        if(pointInBBox(*it,minCloud2filtered,maxCloud2filtered)){
            cloud1filtered->push_back(*it);
        }
    }

    cout<<"Passed Third step ICPTransformation" << endl << endl;

    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(mapRes*10,mapRes*10,mapRes*10);
    grid.setInputCloud(cloud1filtered);
    grid.filter(*tgt);
    
    grid.setInputCloud(cloud2filtered);
    grid.filter(*src);

    pcl::IterativeClosestPointNonLinear<PointT,PointT> reg;
    reg.setTransformationEpsilon(mapRes/60);
    reg.setMaxCorrespondenceDistance(10*mapRes);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(),prev;
    PointCloud::Ptr reg_result;

    cout<<"Passed Fourth step ICPTransformation" << endl << endl;

    //printf("Size of : %lu\n", src->size());
    //printf("Size of : %lu\n", tgt->size());

    if(src->size() < tgt->size()){
        cout<<"Inside main if of ICPTransformation" << endl << endl;
        reg.setInputSource(src);
        reg.setInputTarget(tgt);
        reg_result = src;
        reg.setMaximumIterations(2);
        for(int i=0l; i<MAX_ITER; ++i){
            src = reg_result;
            reg.setInputSource(src);
            reg.align(*reg_result);
            Ti = reg.getFinalTransformation()* Ti;

            if(reg.getMaxCorrespondenceDistance() > 0.2){
                if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 1);
            }else if(reg.getMaxCorrespondenceDistance()>0.002){
                if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
            }
            prev = reg.getLastIncrementalTransformation();
        }
    }else{

        cout<<"Inside main else of ICPTransformation" << endl << endl;

        reg.setInputSource(tgt);
        reg.setInputTarget(src);

        reg_result = tgt;
        reg.setMaximumIterations(2);
        for(int i=0; i<MAX_ITER; ++i){
            tgt = reg_result;
            reg.setInputSource(tgt);
            reg.align(*reg_result);
            Ti = reg.getFinalTransformation()*Ti;

            if(reg.getMaxCorrespondenceDistance() > 0.2){
                if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 1);
            }else if(reg.getMaxCorrespondenceDistance()>0.002){
                if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
            }
            prev = reg.getLastIncrementalTransformation();
        }
        Ti = Ti.inverse();    
    }
    return Ti*tfEst;
}

void Merger::transformTree(OcTree *tree, Eigen::Matrix4f& transform){
    double treeRes = tree->getResolution();
    OcTree* transformed = new OcTree(treeRes);

    Eigen::Matrix3f rotation;
    Eigen::Matrix3f invRotation;
    Eigen::Matrix4f invTransform;
    rotation << transform(0,0),transform(0,1),transform(0,2),
                transform(1,0),transform(1,1),transform(1,2),
                transform(2,0),transform(2,1),transform(2,2);
    invRotation = rotation.transpose();
    invTransform << transform(0,0),transform(0,1),transform(0,2), -transform(0,3),
                transform(1,0),transform(1,1),transform(1,2), -transform(1,3),
                transform(2,0),transform(2,1),transform(2,2), -transform(2,3),
                0,0,0,1;

    double minX, maxX, minY, maxY, minZ, maxZ;

    tree->getMetricMin(minX,minY,minZ);
    tree->getMetricMax(maxX,maxY,maxZ);

    //OcTreeLUT ocTreeLUT(treeRes);
    std::vector<point3d> points;

    points.push_back(point3d(maxX,minY,minZ));
    points.push_back(point3d(minX,minY,minZ));
    points.push_back(point3d(minX,maxY,minZ));
    points.push_back(point3d(maxX,maxY,minZ));
    points.push_back(point3d(maxX,minY,maxZ));
    points.push_back(point3d(minX,minY,maxZ));
    points.push_back(point3d(minX,maxY,maxZ));
    points.push_back(point3d(maxX,maxY,maxZ));

    for(unsigned i=0; i<points.size(); i++){
        Eigen::Vector4f point(points[i].x(),points[i].y(),points[i].z(),1);
        point = transform * point;
        points[i] = point3d(point(0),point(1),point(2));
    }

    minX = points[0].x();
    maxX = points[0].x();
    minY = points[0].y();
    maxY = points[0].y();
    minZ = points[0].z();
    maxZ = points[0].z();

    for(unsigned i=0; i<points.size(); i++){
        minX = (points[i].x()<minX)?points[i].x():minX;
        maxX = (points[i].y()<minY)?points[i].y():minY;
        minY = (points[i].z()<minZ)?points[i].z():minZ;
        maxY = (points[i].x()<maxX)?points[i].x():maxX;
        minZ = (points[i].y()<maxY)?points[i].y():maxY;
        maxZ = (points[i].z()<maxZ)?points[i].z():maxZ;
    }

    for(double z=minZ-treeRes/2; z<(maxZ+treeRes/2);z+=treeRes){
        for(double y=minY-treeRes/2; y<(maxY+treeRes/2);y+=treeRes){
            for(double x=minX-treeRes/2; x<(maxX+treeRes/2);x+=treeRes){
                OcTreeKey destVoxel = transformed->coordToKey(point3d(x,y,z));
                Eigen::Vector4f point(x,y,z,1);
                point = invTransform*point;
                point3d sourcePoint = point3d(point(0),point(1),point(2));
                OcTreeKey sourceVoxel = tree->coordToKey(sourcePoint);
                point3d nn=tree->keyToCoord(sourceVoxel);

                OcTreeNode *oldNode = tree->search(sourceVoxel);

                double c000,c001,c010,c011,c100,c101,c110,c111,c00,c01,c10,c11,c0,c1;
                double xd,yd,zd;

                xd = (sourcePoint.x()-nn.x())/treeRes;
                yd = (sourcePoint.y()-nn.y())/treeRes;
                zd = (sourcePoint.z()-nn.z())/treeRes;

                if(oldNode!=NULL){
                    c000 = oldNode->getOccupancy();
                    OcTreeNode *node;

                    if((node = tree->search(point3d(nn.x(),nn.y(),nn.z()+getSign(zd)*treeRes)))!=NULL){
                        c001 = node->getOccupancy();                        
                    }else{
                        c001 = 0;
                    }

                    if((node = tree->search(point3d(nn.x(),nn.y()+getSign(yd)*treeRes,nn.z())))!=NULL){
                        c010 = node->getOccupancy();                        
                    }else{
                        c010 = 0;
                    }

                    if((node = tree->search(point3d(nn.x(),nn.y()+getSign(yd)*treeRes,nn.z()+getSign(zd)*treeRes)))!=NULL){
                        c011 = node->getOccupancy();                        
                    }else{
                        c011 = 0;
                    }

                    if((node = tree->search(point3d(nn.x()+getSign(xd)*treeRes,nn.y(),nn.z())))!=NULL){
                        c100 = node->getOccupancy();                        
                    }else{
                        c100 = 0;
                    }

                    if((node = tree->search(point3d(nn.x()+getSign(xd)*treeRes,nn.y(),nn.z()+getSign(zd)*treeRes)))!=NULL){
                        c101 = node->getOccupancy();                        
                    }else{
                        c101 = 0;
                    }

                    if((node = tree->search(point3d(nn.x()+getSign(xd)*treeRes,nn.y()+getSign(yd)*treeRes,nn.z())))!=NULL){
                        c110 = node->getOccupancy();                        
                    }else{
                        c110 = 0;
                    }

                    if((node = tree->search(point3d(nn.x()+getSign(xd)*treeRes,nn.y()+getSign(yd)*treeRes,nn.z()+getSign(zd)*treeRes)))!=NULL){
                        c111 = node->getOccupancy();                        
                    }else{
                        c111 = 0;
                    }

                    c00 = (1-fabs(xd))*c000 + fabs(xd)*c100;
                    c01 = (1-fabs(xd))*c001 + fabs(xd)*c101;
                    c10 = (1-fabs(xd))*c010 + fabs(xd)*c110;
                    c11 = (1-fabs(xd))*c011 + fabs(xd)*c111;

                    c0 = (1-fabs(yd))*c00 + fabs(yd)*c10;
                    c1 = (1-fabs(yd))*c01 + fabs(yd)*c11;

                    OcTreeNode *newNode = transformed->updateNode(destVoxel,true);
                    newNode->setLogOdds(logodds((-fabs(zd))*c0 + fabs(zd)*c1));
                }
            }
        }

    }

    tree->swapContent(*transformed);
    delete transformed;
}

void Merger::expandLevel(std::vector<OcTreeNode *> *nodePtrs){
    unsigned size = nodePtrs->size();
    cout<<"Reached Second Expand" << endl << endl;
    for(unsigned i=0; i<size; i++){
        OcTreeNode *parent = nodePtrs->front();
        
        parent->expandNode();
        nodePtrs->erase(nodePtrs->begin());
        for(unsigned j=0; j<8; j++){
            nodePtrs->push_back(parent->getChild(j));
        }
    }
}

unsigned Merger::expandNodeMultiLevel(OcTree *tree, OcTreeNode *node, unsigned currentDepth, unsigned levels){
    if(currentDepth == (int)tree->getTreeDepth()){
        return 0;
    }

    int levelsCounter = 0;
    std::vector<OcTreeNode *> nodePtrs;
    nodePtrs.push_back(node);

    for(unsigned i=0; i<levels; i++){
        if(currentDepth == (int)tree->getTreeDepth()){
            return levelsCounter;
        }
        expandLevel(&nodePtrs);
        levelsCounter++;
        currentDepth++;
    }
    return levelsCounter;
}

int Merger::getNodeDepth(OcTree* tree, point3d& point, OcTreeNode* node){
    for(int depth=tree->getTreeDepth(); depth>1; depth--){
        if(tree->search(point,depth)==node){
            return depth;
        }
    }
    return -1;
}

Merger::Merger(ros::NodeHandle &nh, std::string topic){
    node_handle_ = nh;
    topic_ = topic;
}

Merger::Merger(){}

Merger::~Merger(){}





