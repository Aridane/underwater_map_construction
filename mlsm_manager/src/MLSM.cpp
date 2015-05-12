#include "MLSM.h"

MLSM::MLSM() {
    kdtree_ = kd_create(2);
    resolution_ = 0.3;
    spanX_ = DEFAULTSIZEXMETERS / resolution_;
    spanY_ = DEFAULTSIZEYMETERS / resolution_;
    sizeXMeters_ = DEFAULTSIZEXMETERS;
    sizeYMeters_ = DEFAULTSIZEYMETERS;
    grid_ = boost::make_shared<QuadGrid>(
                QuadGrid(spanX_, spanY_, spanX_, spanY_));
    markersId = 0;
}

MLSM::~MLSM() {

}

MLSM::MLSM(double resolution, double sizeXMeters, double sizeYMeters, double verticalElasticity) {
    kdtree_ = kd_create(2);
    resolution_ = resolution;
    sizeXMeters_ = sizeXMeters;
    sizeYMeters_ = sizeYMeters;
    spanX_ = sizeXMeters / resolution;
    spanY_ = sizeYMeters / resolution;
    grid_ = boost::make_shared<QuadGrid>(
                QuadGrid(spanX_, spanY_, spanX_, spanY_));
    verticalElasticity_ = verticalElasticity;
}

void MLSM::init(double resolution, double sizeXMeters, double sizeYMeters, double verticalElasticity) {
    kdtree_ = kd_create(2);
    resolution_ = resolution;
    sizeXMeters_ = sizeXMeters;
    sizeYMeters_ = sizeYMeters;
    spanX_ = sizeXMeters / resolution;
    spanY_ = sizeYMeters / resolution;
    grid_ = boost::make_shared<QuadGrid>(
                QuadGrid(spanX_, spanY_, spanX_, spanY_));
    verticalElasticity_ = verticalElasticity;
}

double MLSM::getResolution(){
    return resolution_;
}

int MLSM::getSpanX(){
    return spanX_;
}

int MLSM::getSpanY(){
    return spanY_;
}

bool MLSM::checkCell(int i, int j){
    return ((*grid_)(i,j) ? true : false);
}

Block* MLSM::findClosestBlock(pcl::PointXYZI point){
    Block* blockPtr = NULL;
    struct kdres *closestCellRes;
    double pos[2];

    pos[0] = floor(point.x / resolution_);
    pos[1] = floor(point.y / resolution_);

    //ROS_INFO("Point %g %g",pos[0],pos[1]);

    // Find correct cell
    //ROS_INFO("GET NEAREST POINT");
    closestCellRes = kd_nearest(kdtree_, pos);
    //ROS_INFO("Find suitable block %d",closestCellRes->size);
    kd_res_item(closestCellRes, pos);

    //int i = (int)(round(pos[0]));
    //int j = (int)(round(pos[1]));
    //cellPos = (int*)kd_res_item_data(closestCellRes);
    //ROS_INFO("iCell %d %d",i,j);
    //ROS_INFO("fCell %f %f",pos[0],pos[1]);
    //if (cellP != NULL) ROS_INFO("NOT NULL CELL");
    //else ROS_INFO("NULL CELL");
    // Find suitable block in cell

    blockPtr = findSuitableBlock(round(pos[0]),round(pos[1]),point);
    //ROS_INFO("Suitable block found?");
    return blockPtr;

}

Block* MLSM::findSuitableBlock(cellPtr cellP, pcl::PointXYZI point){
    Block* blockPtr = NULL;
    double closestZ = 99999;
    bool candidateFound = false;
    if ((cellP != NULL)
            && (cellP->size() != 0)) {
        ROS_INFO("Getting the block");
        //Find block in height Z in cellP
        cell::iterator iterator, end, olditerator;
        bool updated = false;
        cell candidateList;
        // We follow exactly the same procedure as we did for inserting the block
        for (iterator = cellP->begin(); iterator != cellP->end();) {

            if (fabs(point.z - iterator->get()->mean_.x) < closestZ){
                closestZ = fabs(point.z - iterator->get()->mean_.x);
                blockPtr = (cellP->at(iterator - cellP->begin() + 0)).get();

            }

                if ((fabs(point.z - iterator->get()->height_) < resolution_) ||
                    (fabs(iterator->get()->height_ - iterator->get()->depth_ - point.z) < resolution_)){
                if (!candidateFound){
                    blockPtr = (cellP->at(iterator - cellP->begin() + 0)).get();

                    //addObservationToBlock(blockPtr,cloudIterator);
                    //candidateFound = true;
                    return blockPtr;
                }
                /*else {
                    ROS_DEBUG("FUSING BLOCKS!");
                    fuseBlocks(blockPtr,cellP->at(iterator - cellP->begin() + 0));
                    ROS_DEBUG("Deleting BLOCK");
                    iterator = cellP->erase(iterator);
                    ROS_DEBUG("Blocks fused");
                    end = cellP->end();
                    continue;
                }*/
                //candidateList.push_back(cellP->at(iterator - cellP->begin() + 0));
            }
            iterator++;

        }
    }
    return blockPtr;
}

Block* MLSM::findSuitableBlock(int i, int j, pcl::PointXYZI point){
    Block* blockPtr = NULL;
    cell* cellP;
    double closestZ = 99999;
    bool candidateFound = false;
    if (((cellP = (*grid_)(i, j)) != NULL)
            && (cellP->size() != 0)) {
        //Find block in height Z in cellP
        cell::iterator iterator, end, olditerator;
        bool updated = false;
        cell candidateList;
        // We follow exactly the same procedure as we did for inserting the block
        for (iterator = cellP->begin(); iterator != cellP->end();) {

            if (fabs(point.z - iterator->get()->mean_.x) < closestZ){
                closestZ = fabs(point.z - iterator->get()->mean_.x);
                blockPtr = (cellP->at(iterator - cellP->begin() + 0)).get();

            }

                if ((fabs(point.z - iterator->get()->height_) < resolution_) ||
                    (fabs(iterator->get()->height_ - iterator->get()->depth_ - point.z) < resolution_)){
                if (!candidateFound){
                    blockPtr = (cellP->at(iterator - cellP->begin() + 0)).get();

                    //addObservationToBlock(blockPtr,cloudIterator);
                    //candidateFound = true;
                    return blockPtr;
                }
                /*else {
                    ROS_DEBUG("FUSING BLOCKS!");
                    fuseBlocks(blockPtr,cellP->at(iterator - cellP->begin() + 0));
                    ROS_DEBUG("Deleting BLOCK");
                    iterator = cellP->erase(iterator);
                    ROS_DEBUG("Blocks fused");
                    end = cellP->end();
                    continue;
                }*/
                //candidateList.push_back(cellP->at(iterator - cellP->begin() + 0));
            }
            iterator++;

        }
    }
    return blockPtr;
}

void addObservationToBlock(boost::shared_ptr<Block> blockPtr, intensityCloud::iterator cloudIterator){
    //Update height and depth
    double heightDifference = fabs(blockPtr->height_ - cloudIterator->z);
    if (blockPtr->height_ < cloudIterator->z) {
        blockPtr->depth_ += heightDifference;
        blockPtr->height_ = cloudIterator->z;
    }
    else if (heightDifference > blockPtr->depth_) blockPtr->depth_ = heightDifference;

    //Update nPoints
    blockPtr->nPoints_ = blockPtr->nPoints_ + 1;

    //Update mean
    blockPtr->mean_.x = (blockPtr->nPoints_ - 1.0)
            * blockPtr->mean_.x / blockPtr->nPoints_
            + cloudIterator->x
            / blockPtr->nPoints_;
    blockPtr->mean_.y = (blockPtr->nPoints_ - 1.0)
            * blockPtr->mean_.y / blockPtr->nPoints_
            + cloudIterator->y / blockPtr->nPoints_;
    blockPtr->mean_.z = (blockPtr->nPoints_ - 1.0)
            * blockPtr->mean_.z / blockPtr->nPoints_
            + cloudIterator->z / blockPtr->nPoints_;
    blockPtr->mean_.intensity = (blockPtr->nPoints_ - 1.0)
            * blockPtr->mean_.intensity / blockPtr->nPoints_
            + cloudIterator->intensity / blockPtr->nPoints_;


    //Update variance
    blockPtr->variance_.x = (blockPtr->nPoints_ - 2.0)
            * blockPtr->variance_.x / (blockPtr->nPoints_-1.0)
            + (cloudIterator->x - blockPtr->mean_.x) / (blockPtr->nPoints_ - 1.0);
    blockPtr->variance_.y = (blockPtr->nPoints_ - 2.0)
            * blockPtr->variance_.y / (blockPtr->nPoints_-1.0)
            + (cloudIterator->y - blockPtr->mean_.y) / (blockPtr->nPoints_ - 1.0);
    blockPtr->variance_.z = (blockPtr->nPoints_ - 2.0)
            * blockPtr->variance_.z / (blockPtr->nPoints_-1.0)
            + (cloudIterator->z - blockPtr->mean_.z) / (blockPtr->nPoints_ - 1.0);
    blockPtr->variance_.intensity = (blockPtr->nPoints_ - 2.0)
            * blockPtr->variance_.intensity / (blockPtr->nPoints_-1.0)
            + (cloudIterator->intensity - blockPtr->mean_.intensity) / (blockPtr->nPoints_ - 1.0);
}

void fuseBlocks(boost::shared_ptr<Block> target, boost::shared_ptr<Block> newBlock){
    pcl::PointXYZI combinedMean;
    combinedMean.x = (target->mean_.x + newBlock->mean_.x) * 0.5;
    combinedMean.y = (target->mean_.y + newBlock->mean_.y) * 0.5;
    combinedMean.z = (target->mean_.z + newBlock->mean_.z) * 0.5;
    combinedMean.intensity = (target->mean_.intensity + newBlock->mean_.intensity) * 0.5;

    target->variance_.x = (target->nPoints_*(target->variance_.x
                                             +((target->mean_.x - combinedMean.x)
                                               *(target->mean_.x - combinedMean.x))))
            +(newBlock->nPoints_*(newBlock->variance_.x
                                  +((newBlock->mean_.x - combinedMean.x)
                                    *(newBlock->mean_.x - combinedMean.x))));

    target->variance_.y = (target->nPoints_*(target->variance_.y
                                             +((target->mean_.y - combinedMean.y)
                                               *(target->mean_.y - combinedMean.y))))
            +(newBlock->nPoints_*(newBlock->variance_.y
                                  +((newBlock->mean_.y - combinedMean.y)
                                    *(newBlock->mean_.y - combinedMean.y))));

    target->variance_.z = (target->nPoints_ *
                           (target->variance_.z + ((target->mean_.z -
                                                    combinedMean.z) * (target->mean_.z - combinedMean.z)))) +
            (newBlock->nPoints_
             * (newBlock->variance_.z
                + ((newBlock->mean_.z - combinedMean.z)
                   * (newBlock->mean_.z - combinedMean.z))));

    target->variance_.intensity = (target->nPoints_ *
                                   (target->variance_.intensity + ((target->mean_.z -
                                                                    combinedMean.z) * (target->mean_.z - combinedMean.z))))
            + (newBlock->nPoints_
               * (newBlock->variance_.intensity
                  + ((newBlock->mean_.z - combinedMean.z)
                     * (newBlock->mean_.z - combinedMean.z))));

    target->mean_.x = combinedMean.x;
    target->mean_.y = combinedMean.y;
    target->mean_.z = combinedMean.z;
    target->mean_.intensity = combinedMean.intensity;

    //Update depth and height
    /*double lowestZ = 0;
    if ((newBlock->height_ - newBlock->depth_) < (target->height_ - target->depth_)) lowestZ = newBlock->height_ - newBlock->depth_;
    else lowestZ = target->height_ - target->depth_;
    if (newBlock->height_ > target->height_) target->height_ = newBlock->height_;
    target->depth_ = fabs(target->height_ - lowestZ);*/

    /*Might be worth testing*/
    double heightDifference = fabs(newBlock->height_ - target->height_);
    if (newBlock->height_ > target->height_){
        target->depth_ += heightDifference;
        target->height_ = newBlock->height_;
    }
    else {
        target->depth_ = newBlock->depth_ + heightDifference;
    }
}

//
// Centre of grid is origin of the world
//
//
//
int MLSM::addPointCloud(intensityCloud::Ptr cloud) {
    //ROS_INFO("\nNEW CLOUD");
    intensityCloud::iterator cloudIterator = cloud->begin();
    boost::shared_ptr<Block> blockPtr;
    bool candidateFound = false;
    pcl::PointXYZ index;
    pcl::PointXYZI newMean;
    pcl::PointXYZI newVariance;

    bool expand = false;
    cell* cellP;
    for (; cloudIterator != cloud->end(); cloudIterator++) {
        //Get the indexes
        index.x = floor(cloudIterator->x / resolution_);
        index.y = floor(cloudIterator->y / resolution_);
        index.z = floor(cloudIterator->z / resolution_);

        //Model empty spaces?
        if (isnan(cloudIterator->x) || isnan(cloudIterator->y) || isnan(cloudIterator->z)) continue;
        if (cloudIterator->z > 4.7) continue;
        ROS_DEBUG("Adding point x = %.2f y = %.2f z = %.2f", cloudIterator->x, cloudIterator->y ,cloudIterator->z);
        ROS_DEBUG("Index x = %d y = %d z=%d",(int)index.x,(int)index.y,(int)index.z);
        //Insert/Update
        //Read position, if exists, update. If doesn't, create.


        if (((cellP = (*grid_)((int) index.x, (int) index.y)) != NULL)
                && (cellP->size() != 0)) {
            //Find block in height Z in cellP
            cell::iterator iterator, end, olditerator;
            bool updated = false;
            cell candidateList;
            /****************************************************************************/
            /*First we obtain all the blocks that are candidates to hold the measurement*/
            /****************************************************************************/
            for (iterator = cellP->begin(); iterator != cellP->end();) {
                if ((fabs(cloudIterator->z - iterator->get()->height_) < resolution_)
                            ||(fabs(iterator->get()->height_ - iterator->get()->depth_ - cloudIterator->z) < resolution_)
                            || ((cloudIterator->z < iterator->get()->height_)
                                &&(cloudIterator->z > (iterator->get()->height_ - iterator->get()->depth_))))
                {
                    if (!candidateFound){
                        ROS_DEBUG("CANDIDATE FOUND!");
                        blockPtr = cellP->at(iterator - cellP->begin() + 0);
                        addObservationToBlock(blockPtr,cloudIterator);
                        candidateFound = true;
                    }
                    else {
                        ROS_DEBUG("FUSING BLOCKS!");
                        fuseBlocks(blockPtr,cellP->at(iterator - cellP->begin() + 0));
                        ROS_DEBUG("Deleting BLOCK");
                        iterator = cellP->erase(iterator);
                        ROS_DEBUG("Blocks fused");
                        end = cellP->end();
                        continue;
                    }
                    //candidateList.push_back(cellP->at(iterator - cellP->begin() + 0));
                }
                iterator++;

            }
            /****************************************************************************/
            /*If there are more than 1 candidates, we fuse them, if there's only one, we*/
            /*update it, otherwise, we create it                                        */
            /****************************************************************************/
            if (!candidateFound){
                //If the measure is not collected we create a new horizontal block with
                // height = pz and variance = block variance

                newMean.x = cloudIterator->x;
                newMean.y = cloudIterator->y;
                newMean.z = cloudIterator->z;
                newMean.intensity = cloudIterator->intensity;


                newVariance.x = 0;
                newVariance.y = 0;
                newVariance.z = 0;
                newVariance.intensity = 0;

                blockPtr = boost::make_shared<Block>(newMean,newVariance, 1,
                                                     cloudIterator->z, 0.0, FLOOR);
                ROS_DEBUG("NEW BLOCK CREATED");
                cellP->push_back(blockPtr);
                occupiedBlocks_.push_back(blockPtr);
            }
            /*else if (candidateList.size() == 1){
                //If there is only one block collecting the measurement we update it
                iterator = candidateList.begin();
                addObservationToBlock(iterator, cloudIterator);
            } else if (candidateList.size() > 1){
                //if there are several candidates we fuse them and then add the new observation
                iterator = candidateList.begin();
                blockPtr = boost::make_shared<Block>(iterator->get()->mean_, iterator->get()->variance_,
                                                     iterator->get()->nPoints_,
                                                     iterator->get()->height_, iterator->get()->depth_,
                                                     FLOOR);
                iterator++;
                pcl::PointXYZI combinedMean;
                for (end = candidateList.end(); iterator != end;++iterator) {
                    combinedMean.x = (blockPtr->mean_.x + iterator->get()->mean_.x) * 0.5;
                    combinedMean.y = (blockPtr->mean_.y + iterator->get()->mean_.y) * 0.5;
                    combinedMean.z = (blockPtr->mean_.z + iterator->get()->mean_.z) * 0.5;
                    combinedMean.intensity = (blockPtr->mean_.intensity + iterator->get()->mean_.intensity) * 0.5;

                    blockPtr->variance_.x = (blockPtr->nPoints_*(blockPtr->variance_.x
                                                                 +((blockPtr->mean_.x - combinedMean.x)
                                                                   *(blockPtr->mean_.x - combinedMean.x))))
                            +(iterator->get()->nPoints_*(iterator->get()->variance_.x
                                                         +((iterator->get()->mean_.x - combinedMean.x)
                                                           *(iterator->get()->mean_.x - combinedMean.x))));

                    blockPtr->variance_.y = (blockPtr->nPoints_*(blockPtr->variance_.y
                                                                 +((blockPtr->mean_.y - combinedMean.y)
                                                                   *(blockPtr->mean_.y - combinedMean.y))))
                            +(iterator->get()->nPoints_*(iterator->get()->variance_.y
                                                         +((iterator->get()->mean_.y - combinedMean.y)
                                                           *(iterator->get()->mean_.y - combinedMean.y))));

                    blockPtr->variance_.z = (blockPtr->nPoints_ *
                                             (blockPtr->variance_.z + ((blockPtr->mean_.z -
                                                                        combinedMean.z) * (blockPtr->mean_.z - combinedMean.z)))) +
                            (iterator->get()->nPoints_
                             * (iterator->get()->variance_.z
                                + ((iterator->get()->mean_.z - combinedMean.z)
                                   * (iterator->get()->mean_.z - combinedMean.z))));

                    blockPtr->variance_.intensity = (blockPtr->nPoints_ *
                                                     (blockPtr->variance_.intensity + ((blockPtr->mean_.z -
                                                                                        combinedMean.z) * (blockPtr->mean_.z - combinedMean.z))))
                            + (iterator->get()->nPoints_
                               * (iterator->get()->variance_.intensity
                                  + ((iterator->get()->mean_.z - combinedMean.z)
                                     * (iterator->get()->mean_.z - combinedMean.z))));

                    blockPtr->mean_.x = combinedMean.x;
                    blockPtr->mean_.y = combinedMean.y;
                    blockPtr->mean_.z = combinedMean.z;
                    blockPtr->mean_.intensity = combinedMean.intensity;

                }
                // After fused we add the current point
                addObservationToBlock(iterator, cloudIterator);
            }*/
        } else if (cellP == NULL) {
            ROS_ERROR("[MLSM] GOT NULL cellP");
        } else if (cellP->size() == 0) {
            //ROS_INFO("Cell empty: NEW BLOCK CREATED");
            //Empty cell
            newMean.x = cloudIterator->x;
            newMean.y = cloudIterator->y;
            newMean.z = cloudIterator->z;
            newMean.intensity = cloudIterator->intensity;


            newVariance.x = 0;
            newVariance.y = 0;
            newVariance.z = 0;
            newVariance.intensity = 0;

            blockPtr = boost::make_shared<Block>(newMean,newVariance, 1,
                                                 cloudIterator->z, 0.0, FLOOR);

            cellP->push_back(blockPtr);
            occupiedBlocks_.push_back(blockPtr);
            double pos[2] = { round(index.x), round(index.y)};
            assert(kd_insert( kdtree_, pos,0) == 0);
        }
        candidateFound = false;
    }
    return 0;
}



visualization_msgs::MarkerArray MLSM::getROSMarkers(string frameId) {
    visualization_msgs::MarkerArray result;

    cell::const_iterator iterator, end;
    cell* cellP = NULL;
    Block* blockPtr = NULL;
    int id = 0;
    for (int i=-spanX_+1;i<spanX_;i++) {
        for (int j=-spanY_+1;j<spanY_;j++){
            if((cellP = (*grid_)(i,j)) == NULL) continue;

            for (iterator = cellP->begin(), end = cellP->end(); iterator != end; ++iterator){
                visualization_msgs::Marker marker;
                char str[20];
                marker.header.frame_id = frameId;
                marker.header.stamp = ros::Time::now();
                marker.ns = "basic_shapes";
                marker.id = markersId;
                markersId++;

                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker.scale.x = resolution_ - 0.1;
                marker.scale.y = resolution_ - 0.1;
                if (iterator->get()->depth_ == 0)marker.scale.z = 0.1;
                else marker.scale.z = iterator->get()->depth_;

                marker.pose.position.x = (floor(iterator->get()->mean_.x / resolution_)) * resolution_ + resolution_/2.0;
                marker.pose.position.y = (floor(iterator->get()->mean_.y / resolution_)) * resolution_ + resolution_/2.0;
                marker.pose.position.z = (iterator->get()->height_ - iterator->get()->depth_) + marker.scale.z /2.0;

                //ROS_DEBUG("Mean X %f, Y %f, Z %f",iterator->get()->mean_.x,iterator->get()->mean_.y,iterator->get()->mean_.z);
                //ROS_DEBUG("MARKER X %f, Y %f, Z %f",marker.pose.position.x,marker.pose.position.y,marker.pose.position.z);
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.lifetime = ros::Duration(5);

                // Set the color -- be sure to set alpha to something non-zero!
                marker.color.r = (float)(iterator->get()->depth_ / 0.5)/5.0;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 0.5;

                result.markers.push_back(marker);
            }
        }
    }
    markersId = 0;
    return result;
}
