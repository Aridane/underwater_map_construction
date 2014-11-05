#include "MLSM.h"

MLSM::MLSM() {
    resolution_ = 0.3;
    spanX_ = DEFAULTSIZEXMETERS / resolution_;
    spanY_ = DEFAULTSIZEYMETERS / resolution_;
    sizeXMeters_ = DEFAULTSIZEXMETERS;
    sizeYMeters_ = DEFAULTSIZEYMETERS;
    grid_ = boost::make_shared<QuadGrid>(
                QuadGrid(spanX_, spanY_, spanX_, spanY_));
}

MLSM::~MLSM() {

}

MLSM::MLSM(double resolution, double sizeXMeters, double sizeYMeters) {
    resolution_ = resolution;
    sizeXMeters_ = sizeXMeters;
    sizeYMeters_ = sizeYMeters;
    spanX_ = sizeXMeters / resolution;
    spanY_ = sizeYMeters / resolution;
    grid_ = boost::make_shared<QuadGrid>(
                QuadGrid(spanX_, spanY_, spanX_, spanY_));
}

void MLSM::init(double resolution, double sizeXMeters, double sizeYMeters) {
    resolution_ = resolution;
    sizeXMeters_ = sizeXMeters;
    sizeYMeters_ = sizeYMeters;
    spanX_ = sizeXMeters / resolution;
    spanY_ = sizeYMeters / resolution;
    grid_ = boost::make_shared<QuadGrid>(
                QuadGrid(spanX_, spanY_, spanX_, spanY_));
}
//
// Centre of grid is origin of the world
//
//
//
int MLSM::addPointCloud(intensityCloud cloud) {
    intensityCloud::iterator cloudIterator = cloud.begin();
    boost::shared_ptr<Block> blockPtr;
    pcl::PointXYZ index;
    pcl::PointXYZI newMean;
    pcl::PointXYZI newVariance;

    bool expand = false;
    cell* cellPtr;
    for (; cloudIterator != cloud.end(); cloudIterator++) {
        //Get the indexes
        index.x = (int) (cloudIterator->x / resolution_);
        index.y = (int) (cloudIterator->y / resolution_);
        index.z = (int) (cloudIterator->z / resolution_);
        //Insert/Update
        //Read position, if exists, update. If doesn't, create.


        if (((cellPtr = (*grid_)((int) index.x, (int) index.y)) != NULL)
                && (cellPtr->size() != 0)) {
            //Find block in height Z in cellPtr
            cell::iterator iterator, end;
            bool updated = false;
            cell candidateList;
            /****************************************************************************/
            /*First we obtain all the blocks that are candidates to hold the measurement*/
            /****************************************************************************/
            for (iterator = cellPtr->begin(), end = cellPtr->end(); iterator != end; ++iterator) {
                //Candidates require |z-height| < resolution
                // and |height - d - z| < resolution
                if ((fabs(cloudIterator->z - iterator->get()->height_) < resolution_) &&
                        (fabs(iterator->get()->height_ - iterator->get()->depth_ - cloudIterator->z) < resolution_)){
                    candidateList.push_back(cellPtr->at(iterator- cellPtr->begin() + 0));
                }
            }
            /****************************************************************************/
            /*If there are more than 1 candidates, we fuse them, if there's only one, we*/
            /*update it, otherwise, we create it                                        */
            /****************************************************************************/
            if (candidateList.size() == 0){
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

                cellPtr->push_back(blockPtr);
                occupiedBlocks_.push_back(blockPtr);
            } else if (candidateList.size() == 1){
                //If there is only one block collecting the measurement we update it

                iterator = candidateList.begin();
                //Update nPoints
                iterator->get()->nPoints_ = iterator->get()->nPoints_ + 1;

                //Update mean
                iterator->get()->mean_.x = (iterator->get()->nPoints_ - 1.0)
                        * iterator->get()->mean_.x / iterator->get()->nPoints_
                        + cloudIterator->x
                        / iterator->get()->nPoints_;
                iterator->get()->mean_.y = (iterator->get()->nPoints_ - 1.0)
                        * iterator->get()->mean_.y / iterator->get()->nPoints_
                        + cloudIterator->y / iterator->get()->nPoints_;
                iterator->get()->mean_.z = (iterator->get()->nPoints_ - 1.0)
                        * iterator->get()->mean_.z / iterator->get()->nPoints_
                        + cloudIterator->z / iterator->get()->nPoints_;
                iterator->get()->mean_.intensity = (iterator->get()->nPoints_ - 1.0)
                        * iterator->get()->mean_.intensity / iterator->get()->nPoints_
                        + cloudIterator->intensity / iterator->get()->nPoints_;


                //Update variance
                iterator->get()->variance_.x = (iterator->get()->nPoints_ - 2.0)
                        * iterator->get()->variance_.x / (iterator->get()->nPoints_-1.0)
                        + (cloudIterator->x - iterator->get()->mean_.x) / (iterator->get()->nPoints_ - 1.0);
                iterator->get()->variance_.y = (iterator->get()->nPoints_ - 2.0)
                        * iterator->get()->variance_.y / (iterator->get()->nPoints_-1.0)
                        + (cloudIterator->y - iterator->get()->mean_.y) / (iterator->get()->nPoints_ - 1.0);
                iterator->get()->variance_.z = (iterator->get()->nPoints_ - 2.0)
                        * iterator->get()->variance_.z / (iterator->get()->nPoints_-1.0)
                        + (cloudIterator->z - iterator->get()->mean_.z) / (iterator->get()->nPoints_ - 1.0);
                iterator->get()->variance_.intensity = (iterator->get()->nPoints_ - 2.0)
                        * iterator->get()->variance_.intensity / (iterator->get()->nPoints_-1.0)
                        + (cloudIterator->intensity - iterator->get()->mean_.intensity) / (iterator->get()->nPoints_ - 1.0);

            } else if (candidateList.size() > 1){
                //if there are several candidates we fuse them
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
            }
        } else if (cellPtr == NULL) {
            ROS_ERROR("[MLSM] GOT NULL cellPtr");
        } else if (cellPtr->size() == 0) {
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

            cellPtr->push_back(blockPtr);
            occupiedBlocks_.push_back(blockPtr);
        }
    }
    return 0;
}

visualization_msgs::MarkerArray MLSM::getROSMarkers() {
    visualization_msgs::MarkerArray result;

    cell::const_iterator iterator, end;
    int i=0;
    for (iterator = occupiedBlocks_.begin(), end = occupiedBlocks_.end();
         iterator != end; ++iterator) {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = i;
        i++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = iterator->get()->mean_.x;
        marker.pose.position.y = iterator->get()->mean_.y;
        marker.pose.position.z = iterator->get()->mean_.z;
        ROS_INFO("MARKER X %f, Y %f, Z %f",marker.pose.position.x,marker.pose.position.y,marker.pose.position.z);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        marker.lifetime = ros::Duration();

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 0.1f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        result.markers.push_back(marker);
    }
    return result;
}
