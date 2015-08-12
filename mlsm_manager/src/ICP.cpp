#include "ICP.h"

ICP::ICP(){

}

ICP::ICP(int maxIterations, double errorThreshold){
    maxIterations_ = maxIterations;
    errorThreshold_ = errorThreshold;
}
ICP::ICP(int maxIterations, double errorThreshold, ros::Publisher* debugPublisher){
    maxIterations_ = maxIterations;
    errorThreshold_ = errorThreshold;
    debugPublisher_ = debugPublisher;
}

ICP::~ICP(){

}


void ICP::setDebugPublisher(ros::Publisher* debugPublisher){
    debugPublisher_ = debugPublisher;
}

void ICP::setErrorThreshold(double errorThreshold){
    errorThreshold_ = errorThreshold;
}

void ICP::setMaxIterations(int maxIterations){
    maxIterations_ = maxIterations;
}

void ICP::setWidth(double width){
    width_ = width;
}

void ICP::setSampleStep(double sampleStep){
    sampleStep_ = sampleStep;
}

void ICP::setNSamples(int nSamples){
    nSamples_ = nSamples;
    ROS_INFO("ICP: Nsamples set to %d", nSamples_);
}

// Get the closest points in the direction of estimated movement using a triangle
std::vector<BlockInfo> ICP::closestPoints(intensityCloud::Ptr P, MLSM *X,std::vector<double> timeStamps, Vector3d eV){

    std::vector<BlockInfo> result;
    BlockInfo blockInfo;
    mlsm::Block* blockPtr, *bestBlockPtr;
   //intensityCloud::iterator cloudIterator = P->begin();
    cell::iterator cellIterator;
    int i, j;
    int ci, cj;
    cell cellPtr;
    bool found = false;
    double minError = DBL_MAX, d0;
    Vector3d unitSpeed;
    Vector3d unitDir;
    double angle, closestAngle = M_PI;
    //ROS_INFO("Vel %f, %f %f", eV[0], eV[1],eV[2]);

    //  if ((eV[0] == 0.0) && (eV[1] == 0.0)){
    //ROS_INFO("No estimation");
    pcl::PointXYZI p0,p[nSamples_];
    double movingTime = 0;
    double firstStamp = *(timeStamps.begin());

    for(int i=0;i<P->size();i++){
        if (isnan(P->at(i).x) || isnan(P->at(i).y) || isnan(P->at(i).z)) continue;

        movingTime = fabs(timeStamps[i]) - fabs(timeStamps.front());//lastStamp - timeStamps[i];

        closestAngle = M_PI;
        p0 = P->at(i);
        //ROS_INFO("Point %d X %f Y %f Z %f",i, p0.x, p0.y, p0.z);
        bestBlockPtr = X->findClosestBlock(p0);
        //ROS_INFO("Candt %d X %f Y %f Z %f\n",i,bestBlockPtr->mean_.x,bestBlockPtr->mean_.y,bestBlockPtr->mean_.z);
        //ROS_INFO("ICP: Nsamples is %d", nSamples_);

        if ((((eV[0] != 0) || (eV[1] != 0) || (eV[2] != 0)) || (timeStamps.size() != P->size())) && (nSamples_ > 0)){
            unitSpeed[0] = eV[0];
            unitSpeed[1] = eV[1];
            unitSpeed[2] = eV[2];
            unitSpeed.normalize();
            for (int j=0;j<nSamples_;j++){
                p[j].x = p0.x + movingTime* eV[0] * j * sampleStep_;
                p[j].y = p0.y + movingTime* eV[1] * j * sampleStep_;
                p[j].z = p0.z + movingTime* eV[2] * j * sampleStep_;
                //ROS_INFO("Point Sample %d X %f Y %f Z %f",j, p[j].x, p[j].y, p[j].z);
                blockPtr = X->findClosestBlock(p[j]);
                if (blockPtr == NULL) break;

                // Find angle between the block and p0
                unitDir[0] = blockPtr->mean_.x - p0.x;
                unitDir[1] = blockPtr->mean_.y - p0.y;
                unitDir[2] = blockPtr->mean_.z - p0.z;
                unitDir.normalize();
                angle = acos(unitSpeed.dot(unitDir));
                if (fabs(angle) < closestAngle){
                    bestBlockPtr = blockPtr;
                    closestAngle = angle;
                }
            }
        }
        blockInfo.blockPtr = bestBlockPtr;
        blockInfo.i = i;
        blockInfo.j = j;
        result.push_back(blockInfo);
        minError = DBL_MAX;
    }
    ////ROS_INFO("Finished finding closest points");
    return result;
    /*}
    //ROS_INFO("With estimation");

    if (eV[0] > eV[1]){
        eV[1] = eV[1] / eV[0];
        eV[0] = 1;
    }
    else{
        eV[0] = eV[0] / eV[1];
        eV[1] = 1;
    }

    double m = 100.0;
    int stepY = 1;
    int stepX = 1;
    int error;
    int errorprev;
    int incY;
    int incX;
    for (;cloudIterator != P->end();cloudIterator++){
        if (isnan(cloudIterator->x) || isnan(cloudIterator->y) || isnan(cloudIterator->z)) continue;
        ////ROS_INFO("Point x = %f, resolution =  %f", cloudIterator->x, X->getResolution());
        i = cloudIterator->x / X->getResolution();
        j = cloudIterator->y / X->getResolution();
        ////ROS_INFO("%d, %d", i, j);
        incY = abs(m * (eV[1] - j));
        incX = abs(m * (eV[0] - i));
        stepY = 1;
        stepX = 1;
        if (incY < 0){
            stepY = -1;
        }
        if (incX < 0){
            stepX = -1;
        }
        if (incX >= incY){
            error = incY - incX;
            errorprev = error;
            while ((i < X->getSpanX())&&(j < X->getSpanY())){
                ////ROS_INFO("%d, %d", i, j);
                if (X->checkCell(i,j)){
                    ////ROS_INFO("CHECK %d, %d", i, j);
                    blockInfo.blockPtr = X->findSuitableBlock(i,j,*cloudIterator);
                    blockInfo.i = i;
                    blockInfo.j = j;
                    result.push_back(blockInfo);
                    break;
                }
                if (error >= 0){
                    j += stepY;
                    errorprev = error;
                    error -= incX;

                    if (error + errorprev < incX){
                        if (X->checkCell(i,j - stepY)){
                            blockInfo.blockPtr = X->findSuitableBlock(i,j,*cloudIterator);
                            blockInfo.i = i;
                            blockInfo.j = j;
                            result.push_back(blockInfo);
                            break;
                        }
                    }
                    if (error + errorprev > incX){
                        if (X->checkCell(i - stepX,j)){
                            blockInfo.blockPtr = X->findSuitableBlock(i,j,*cloudIterator);
                            blockInfo.i = i;
                            blockInfo.j = j;
                            result.push_back(blockInfo);
                            break;
                        }
                    }
                    else {
                        if (X->checkCell(i,j - stepY)){
                            blockInfo.blockPtr = X->findSuitableBlock(i,j,*cloudIterator);
                            blockInfo.i = i;
                            blockInfo.j = j;
                            result.push_back(blockInfo);
                            break;
                        }
                        if (X->checkCell(i - stepX,j)){
                            blockInfo.blockPtr = X->findSuitableBlock(i,j,*cloudIterator);
                            blockInfo.i = i;
                            blockInfo.j = j;
                            result.push_back(blockInfo);
                            break;
                        }
                    }
                }
                i += stepX;
                error += incY;
            }
        }
        else {
            error = incX - incY;
            errorprev = error;
            while ((i < X->getSpanX())&&(j < X->getSpanY())){
                if (X->checkCell(i,j)){
                    blockInfo.blockPtr = X->findSuitableBlock(i,j,*cloudIterator);
                    blockInfo.i = i;
                    blockInfo.j = j;
                    result.push_back(blockInfo);
                    break;
                }
                if (error >= 0){
                    i += stepX;
                    errorprev = error;
                    error -= incY;

                    if (error + errorprev < incY){
                        if (X->checkCell(i - stepX,j)){
                            blockInfo.blockPtr = X->findSuitableBlock(i,j,*cloudIterator);
                            blockInfo.i = i;
                            blockInfo.j = j;
                            result.push_back(blockInfo);
                            break;
                        }
                    }
                    if (error + errorprev > incY){
                        if (X->checkCell(i,j - stepY)){
                            blockInfo.blockPtr = X->findSuitableBlock(i,j,*cloudIterator);
                            blockInfo.i = i;
                            blockInfo.j = j;
                            result.push_back(blockInfo);
                            break;
                        }
                    }
                    else {
                        if (X->checkCell(i,j - stepY)){
                            blockInfo.blockPtr = X->findSuitableBlock(i,j,*cloudIterator);
                            blockInfo.i = i;
                            blockInfo.j = j;
                            result.push_back(blockInfo);
                            break;
                        }
                        if (X->checkCell(i - stepX,j)){
                            blockInfo.blockPtr = X->findSuitableBlock(i,j,*cloudIterator);
                            blockInfo.i = i;
                            blockInfo.j = j;
                            result.push_back(blockInfo);
                            break;
                        }
                    }
                }
                j += stepY;
                error += incX;
            }
        }
    }
    return result;*/
}

std::vector<Vector3d> ICP::applyTransformation(intensityCloud::Ptr P, std::vector<double> timeStamps, Matrix4f R, std::vector<Vector3d> transforms, Vector3d T){
    std::vector<Vector3d> result;
    intensityCloud::iterator cloudIterator = P->begin();
    //R(0,3) = T[0];
    //R(1,3) = T[1];
    //R(2,3) = T[2];
    //pcl::transformPointCloud(*P,*P,R);
    //ROS_INFO("Speed %f %f %f Stamps %d Points %d",V[0],V[1],V[2], timeStamps.size(), P->size());

    /*if (((V[0] == 0) && (V[1] == 0) && (V[2] == 0)) || (timeStamps.size() != P->size())){
        R(0,3) = T[0];
        R(1,3) = T[1];
        R(2,3) = T[2];

        pcl::transformPointCloud(*P,*P,R);
    }
    else {*/
        //double prevStamp = cloudIterator->data_c[3];
        /*double lastStamp;
        double firstStamp = *(timeStamps.begin());
        std::vector<double>::reverse_iterator rit = timeStamps.rbegin();
        for (; rit!= timeStamps.rend(); ++rit)
        {
            if (*rit != 0){
                lastStamp = *rit;
                break;
            }
        }

        double movingTime = 0;
        Vector3d transformation;*/
        for(int i=0;i<P->size();i++){
            if (isnan(P->at(i).x) || isnan(P->at(i).y) || isnan(P->at(i).z)) continue;
            // Possitive stamp means movement. Accumulate translation time

            //if (timeStamps[i] != 0) movingTime = timeStamps[i] - firstStamp;//lastStamp - timeStamps[i];
            //ROS_INFO("MovingT %f tn = %f ti = %f",movingTime,lastStamp,timeStamps[i]);

            //transformation[0] = //V[0] * movingTime;
            //transformation[1] = //V[1] * movingTime;
            //transformation[2] = //V[2] * movingTime;
            //ROS_INFO("Transformation x = %f y = %f z = %f",transformation[0],transformation[1],transformation[2]);
            // Rotation? (Assuming only linear movement, orientation known
            //result.push_back(transformation);
            P->at(i).x = P->at(i).x + transforms.at(i)[0];// + T[0];
            P->at(i).y = P->at(i).y + transforms.at(i)[1];// + T[1];
            P->at(i).z = P->at(i).z + transforms.at(i)[2];// + T[2];
            //prevStamp = fabs(cloudIterator->data_c[3]);

        }
    //}
    return result;
}

double slope(const vector<double>& x, const vector<double>& y){
    double n = x.size();
    if (x.size() != y.size()){
        ROS_ERROR("Different sizes");
    }
    double avgX = accumulate(x.begin(), x.end(), 0.0) / n;
    double avgY = accumulate(y.begin(), y.end(), 0.0) / n;

    double numerator = 0.0;
    double denominator = 0.0;

    for(int i=0; i<n; ++i){
        numerator += (x[i] - avgX) * (y[i] - avgY);
        denominator += (x[i] - avgX) * (x[i] - avgX);
    }
    if (denominator == 0){
        return 0;
    }
    return numerator / denominator;
}

double ICP::registration(intensityCloud::Ptr P, std::vector<BlockInfo>* Y, std::vector<double> timeStamps, std::vector<Vector3d>* transforms, Vector3d *T, Matrix4f *R, Vector3d direction){
    double error = 0;
    pcl::PointXYZI centroidP = 0, centroidY = 0, centroid2 = 0;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI> svd;
    pcl::registration::TransformationEstimationLM<pcl::PointXYZI, pcl::PointXYZI> lm;
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZI, pcl::PointXYZI> correspondenceEstimator;
    int i = 0;


    intensityCloud::iterator cloudIterator = P->begin();
    unsigned int validPoints = 0;
    intensityCloud candidatePointCloud, pointCloud;
    pcl::PointXYZI p;
    int ySize = Y->size();
    //Centroid calculation
    for (;cloudIterator != P->end();cloudIterator++){
        if (isnan(cloudIterator->x) || isnan(cloudIterator->y) || isnan(cloudIterator->z)) continue;
        validPoints++;
        centroidP.x += cloudIterator->x;
        centroidP.y += cloudIterator->y;
        centroidP.z += cloudIterator->z;
    }
    centroidP.x /= validPoints;
    centroidP.y /= validPoints;
    centroidP.z /= validPoints;

    for(i=0;i<ySize;i++){
        centroidY.x += Y->at(i).blockPtr->mean_.x;
        centroidY.y += Y->at(i).blockPtr->mean_.y;
        centroidY.z += Y->at(i).blockPtr->mean_.z;
    }
    centroidY.x /= Y->size();
    centroidY.y /= Y->size();
    centroidY.z /= Y->size();
    //ROS_INFO("Candidates centroid %f %f %f",centroidY.x,centroidY.y,centroidY.z);
    //ROS_INFO("Points centroid %f %f %f",centroidP.x,centroidP.y,centroidP.z);

    //Move both to same origin
    for(i=0;i<ySize;i++){
        p.x = Y->at(i).blockPtr->mean_.x - centroidY.x;
        p.y = Y->at(i).blockPtr->mean_.y - centroidY.y;
        p.z = Y->at(i).blockPtr->mean_.z - centroidY.z;
        candidatePointCloud.push_back(p);
    }
    for (cloudIterator = P->begin();cloudIterator != P->end();cloudIterator++){
        if (isnan(cloudIterator->x) || isnan(cloudIterator->y) || isnan(cloudIterator->z)) continue;
        p.x = cloudIterator->x - centroidP.x;
        p.y = cloudIterator->y - centroidP.y;
        p.z = cloudIterator->z - centroidP.z;
        pointCloud.push_back(p);
    }
    // Calculate SVD
    //pcl::ConstCloudIterator<pcl::PointXYZI> p1(pointCloud);
    //pcl::ConstCloudIterator<pcl::PointXYZI> p2(candidatePointCloud);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>::Matrix4 rotation;
    rotation = Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZI>::Ptr source (new pcl::PointCloud<pcl::PointXYZI>(pointCloud));
    pcl::PointCloud<pcl::PointXYZI>::Ptr target (new pcl::PointCloud<pcl::PointXYZI>(candidatePointCloud));
    boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
    correspondenceEstimator.setInputSource(source);
    correspondenceEstimator.setInputTarget(target);
    correspondenceEstimator.determineCorrespondences(*correspondences);


    //svd.estimateRigidTransformation(pointCloud,candidatePointCloud, *correspondences,rotation);
    //ROS_INFO("\n \tSVD Rotation \n%f %f %f\n%f %f %f\n%f %f %f",rotation(0,0),rotation(0,1),rotation(0,2),
    //rotation(1,0),rotation(1,1),rotation(1,2),
    //rotation(2,0),rotation(2,1),rotation(2,2));


    //lm.estimateRigidTransformation(pointCloud,candidatePointCloud, *correspondences,rotation);
    ////ROS_INFO("\n \tLM Rotation \n%f %f %f\n%f %f %f\n%f %f %f",rotation(0,0),rotation(0,1),rotation(0,2),
    //                                                                   rotation(1,0),rotation(1,1),rotation(1,2),
    //         rotation(2,0),rotation(2,1),rotation(2,2));
    /*pcl::PointCloud<pcl::PointXYZI>::Ptr source (new pcl::PointCloud<pcl::PointXYZI>(pointCloud));
    pcl::PointCloud<pcl::PointXYZI>::Ptr target (new pcl::PointCloud<pcl::PointXYZI>(candidatePointCloud));
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

    icp.setInputSource(source);
    icp.setInputTarget(target);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (10);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);
    pcl::PointCloud<pcl::PointXYZI> Final;
    icp.align(Final);
    //std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    //icp.getFitnessScore() << std::endl;
    //std::cout << icp.getFinalTransformation() << std::endl;
    Eigen::Matrix4f rotation = icp.getFinalTransformation ();*/

    //ROS_INFO("TransformPointCloud");
    pcl::transformPointCloud(pointCloud,pointCloud,rotation);

    (*R) = rotation;



    /*

    for (cloudIterator = pointCloud.begin();cloudIterator != pointCloud.end();cloudIterator++){
        if (isnan(cloudIterator->x) || isnan(cloudIterator->y) || isnan(cloudIterator->z)) continue;
        cloudIterator->x += centroidP.x;
        cloudIterator->y += centroidP.y;
        cloudIterator->z += centroidP.z;
        centroid2.x += cloudIterator->x;
        centroid2.y += cloudIterator->y;
        centroid2.z += cloudIterator->z;
    }
    centroid2.x /= validPoints;
    centroid2.y /= validPoints;
    centroid2.z /= validPoints;
    //ROS_INFO("\n\tCentroid2 x = %f y = %f z = %f",centroid2.x, centroid2.y, centroid2.z);

    (*T)[0] = centroidY.x - centroidP.x;
    (*T)[1] = centroidY.y - centroidP.y;
    (*T)[2] = centroidY.z - centroidP.z;*/


    /*Vector3d v, v0,v1;
    Vector3d diffs;
    v[0] = v[1] = v[2] = 0;
    v0[0] = v0[1] = v0[2] = 0;
    v1[0] = v1[1] = v1[2] = 0;
    v0[0] = ((Y->at(0).blockPtr->mean_.x - P->at(0).x) < errorThreshold_) ? 0 : Y->at(0).blockPtr->mean_.x - P->at(0).x;
    v0[1] = ((Y->at(0).blockPtr->mean_.y - P->at(0).y) < errorThreshold_) ? 0 : Y->at(0).blockPtr->mean_.y - P->at(0).y;
    v0[2] = ((Y->at(0).blockPtr->mean_.z - P->at(0).z) < errorThreshold_) ? 0 : Y->at(0).blockPtr->mean_.z - P->at(0).z;

    for(int i=1;i<ySize;i++){
        v1[0] = ((Y->at(i).blockPtr->mean_.x - P->at(i).x) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.x - P->at(i).x;
        v1[1] = ((Y->at(i).blockPtr->mean_.y - P->at(i).y) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.y - P->at(i).y;
        v1[2] = ((Y->at(i).blockPtr->mean_.z - P->at(i).z) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.z - P->at(i).z;
        if ((timeStamps[i] - timeStamps[i-1]) != 0 ){
            diffs[0] = ((v1[0] - v0[0])/(timeStamps[i] - timeStamps[i-1]))/i;
            diffs[1] = ((v1[1] - v0[1])/(timeStamps[i] - timeStamps[i-1]))/i;
            diffs[2] = ((v1[2] - v0[2])/(timeStamps[i] - timeStamps[i-1]))/i;
        }
        else {
            diffs[0] = 0;
            diffs[1] = 0;
            diffs[2] = 0;
        }
        v[0] = (i-1) * (v[0]/i) + diffs[0];
        v[1] = (i-1) * (v[1]/i) + diffs[1];
        v[2] = (i-1) * (v[2]/i) + diffs[2];
        v0[0] = v1[0];
        v0[1] = v1[1];
        v0[2] = v1[2];
    }


    (*Tv)[0] = v[0];
    (*Tv)[1] = v[1];
    (*Tv)[2] = v[2];*/
    double angle, closestAngle = M_PI, longest;
    (*T)[0] = 0;
    (*T)[1] = 0;
    (*T)[2] = 0;
    int bestI = 0;
    std::vector<double> errorsX, errorsY, errorsZ;
    double slopeX, slopeY, slopeZ;
    std::vector<double> positiveStamps;
    //for (i=0;i<ySize;i++){
    //    errorsX.push_back(transforms->at(i)[0]);
    //    errorsY.push_back(transforms->at(i)[1]);
    //    errorsZ.push_back(transforms->at(i)[2]);
    //    positiveStamps.push_back(fabs(fabs(timeStamps.at(i))-fabs(timeStamps.front())));
    //}
    i = 0;
    bool moving = false;
    double tX = 0,tY = 0,tZ = 0;
    int samples = 0;
    int accountedSamples = 0;
    Vector3d linearTranslation,totalLinearTranslation;
    linearTranslation[0] = 0;
    linearTranslation[1] = 0;
    linearTranslation[2] = 0;
    totalLinearTranslation[0] = 0;
    totalLinearTranslation[1] = 0;
    totalLinearTranslation[2] = 0;
    int linearSegments = 0;
    int dynamicSamples = 0;
    int slopeBegin = 0;
    int lineBegin = 0;

    /*while (i < ySize){
        if (timeStamps.at(i) < 0){
            if ((moving == true) && (errorsX.size() > 0)){
                            // Calculate movement with dinamic data
                            slopeX = slope(positiveStamps,errorsX);
                            slopeY = slope(positiveStamps,errorsY);
                            slopeZ = slope(positiveStamps,errorsZ);
                            for (int j = slopeBegin;j<i-1;j++){
                                transforms->at(j)[0] = fabs(positiveStamps.at(j-slopeBegin)) * slopeX;
                                transforms->at(j)[1] = fabs(positiveStamps.at(j-slopeBegin)) * slopeY;
                                transforms->at(j)[2] = fabs(positiveStamps.at(j-slopeBegin)) * slopeZ;
                            }
                            lineBegin = i;
                            ROS_INFO("SlopeX %f SlopeY %f Slopez %f", slopeX, slopeY, slopeZ);
                            (*T)[0] += slopeX * positiveStamps.back();
                            (*T)[1] += slopeY * positiveStamps.back();
                            (*T)[2] += slopeZ * positiveStamps.back();
                            positiveStamps.clear();
                            moving = false;
                            errorsX.clear();
                            errorsY.clear();
                            errorsZ.clear();
                            tX = 0;
                            tY = 0;
                            tZ = 0;
                            samples = 0;
                            linearTranslation[0] = 0;
                            linearTranslation[1] = 0;
                            linearTranslation[2] = 0;
                            ROS_INFO("==============STATIC=============");

            }
            moving = false;
            tX = (fabs(Y->at(i).blockPtr->mean_.x - P->at(i).x) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.x - P->at(i).x;
            tY = (fabs(Y->at(i).blockPtr->mean_.y - P->at(i).y) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.y - P->at(i).y;
            tZ = (fabs(Y->at(i).blockPtr->mean_.z - P->at(i).z) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.z - P->at(i).z;

            ROS_INFO("");
            ROS_INFO("SPoint %d X %f Y %f Z %f",i , P->at(i).x, P->at(i).y,P->at(i).z);
            ROS_INFO("SCandt %d X %f Y %f Z %f",i, Y->at(i).blockPtr->mean_.x, Y->at(i).blockPtr->mean_.y, Y->at(i).blockPtr->mean_.z);
            ROS_INFO("SError %d X %f Y %f Z %f Time %f",i , tX, tY, tZ,fabs(fabs(timeStamps.at(i))-fabs(timeStamps.at(slopeBegin))));
            linearTranslation[0] = ((samples * linearTranslation[0]) / (samples+1)) + (tX/(samples+1));
            linearTranslation[1] = ((samples * linearTranslation[1]) / (samples+1)) + (tY/(samples+1));
            linearTranslation[2] = ((samples * linearTranslation[2]) / (samples+1)) + (tZ/(samples+1));

            samples++;
            i++;
        }
        else {
            if ((moving == false) && (samples > 0)){
                            // Calculate movement with static data
                            slopeBegin = i;
                            for (int j = lineBegin;j<i-1;j++){
                                transforms->at(j)[0] =  linearTranslation[0];
                                transforms->at(j)[1] =  linearTranslation[1];
                                transforms->at(j)[2] =  linearTranslation[2];
                            }
                            ROS_INFO("Linear X %f Y %f Z %f", tX/samples, tY/samples, tZ/samples);

                            totalLinearTranslation[0] = (accountedSamples * totalLinearTranslation[0] + linearTranslation[0] * samples) / (accountedSamples + samples);
                            totalLinearTranslation[1] = (accountedSamples * totalLinearTranslation[1] + linearTranslation[1] * samples) / (accountedSamples + samples);
                            totalLinearTranslation[2] = (accountedSamples * totalLinearTranslation[2] + linearTranslation[2] * samples) / (accountedSamples + samples);
                            linearSegments++;
                            linearTranslation[0] = 0;
                            linearTranslation[1] = 0;
                            linearTranslation[2] = 0;
                            //(*T)[0] = (accountedSamples*(*T)[0])/(accountedSamples + samples) + tX/(accountedSamples + samples);
                            //(*T)[1] = (accountedSamples*(*T)[1])/(accountedSamples + samples) + tY/(accountedSamples + samples);
                            //(*T)[2] = (accountedSamples*(*T)[2])/(accountedSamples + samples) + tZ/(accountedSamples + samples);
                            accountedSamples += samples;
                            samples = 0;
                            tX = 0;
                            tY = 0;
                            tZ = 0;
                            ROS_INFO("==============MOVING=============");
            }
            tX = (fabs(Y->at(i).blockPtr->mean_.x - P->at(i).x) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.x - P->at(i).x;
            tY = (fabs(Y->at(i).blockPtr->mean_.y - P->at(i).y) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.y - P->at(i).y;
            tZ = (fabs(Y->at(i).blockPtr->mean_.z - P->at(i).z) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.z - P->at(i).z;
            dynamicSamples++;
            ROS_INFO("");
            ROS_INFO("DPoint %d X %f Y %f Z %f",i , P->at(i).x, P->at(i).y,P->at(i).z);
            ROS_INFO("DCandt %d X %f Y %f Z %f",i, Y->at(i).blockPtr->mean_.x, Y->at(i).blockPtr->mean_.y, Y->at(i).blockPtr->mean_.z);
            ROS_INFO("DError %d X %f Y %f Z %f Time %f",i , tX, tY, tZ,fabs(fabs(timeStamps.at(i))-fabs(timeStamps.at(slopeBegin))));

            errorsX.push_back(tX);
            errorsY.push_back(tY);
            errorsZ.push_back(tZ);
            moving = true;
            positiveStamps.push_back(fabs(fabs(timeStamps.at(i))-fabs(timeStamps.at(slopeBegin))));
            i++;
        }
    }
    if ((moving == false) && (samples > 0)){
                    // Calculate movement with static data
                    slopeBegin = i;
                    for (int j = lineBegin;j<i-1;j++){
                        transforms->at(j)[0] =  linearTranslation[0];
                        transforms->at(j)[1] =  linearTranslation[1];
                        transforms->at(j)[2] =  linearTranslation[2];
                    }
                    ROS_INFO("Linear X %f Y %f Z %f", linearTranslation[0], linearTranslation[1], linearTranslation[2]);

                    totalLinearTranslation[0] = (accountedSamples * totalLinearTranslation[0] + linearTranslation[0] * samples) / (accountedSamples + samples);
                    totalLinearTranslation[1] = (accountedSamples * totalLinearTranslation[1] + linearTranslation[1] * samples) / (accountedSamples + samples);
                    totalLinearTranslation[2] = (accountedSamples * totalLinearTranslation[2] + linearTranslation[2] * samples) / (accountedSamples + samples);
                    samples = 0;
                    tX = 0;
                    tY = 0;
                    tZ = 0;
    }
    if ((moving == true) && (errorsX.size() > 0)){
                    // Calculate movement with dinamic data
                    slopeX = slope(positiveStamps,errorsX);
                    slopeY = slope(positiveStamps,errorsY);
                    slopeZ = slope(positiveStamps,errorsZ);
                    ROS_INFO("SlopeX %f SlopeY %f Slopez %f", slopeX, slopeY, slopeZ);
                    for (int j = slopeBegin;j<i;j++){
                        transforms->at(j)[0] = fabs(positiveStamps.at(j-slopeBegin)) * slopeX;
                        transforms->at(j)[1] = fabs(positiveStamps.at(j-slopeBegin)) * slopeY;
                        transforms->at(j)[2] = fabs(positiveStamps.at(j-slopeBegin)) * slopeZ;
                    }
                    (*T)[0] += slopeX * positiveStamps.back();
                    (*T)[1] += slopeY * positiveStamps.back();
                    (*T)[2] += slopeZ * positiveStamps.back();
                    positiveStamps.clear();
                    errorsX.clear();
                    errorsY.clear();
                    errorsZ.clear();
    }*/
    //(*T)[0] += totalLinearTranslation[0];
    //(*T)[1] += totalLinearTranslation[1];
    //(*T)[2] += totalLinearTranslation[2];





    /*slopeX = slope(positiveStamps,errorsX);
    slopeY = slope(positiveStamps,errorsY);
    slopeZ = slope(positiveStamps,errorsZ);

    transforms->at(0)[0] = (fabs(Y->at(0).blockPtr->mean_.x - P->at(0).x) < errorThreshold_) ? 0 : Y->at(0).blockPtr->mean_.x - P->at(0).x;
    transforms->at(0)[1] = (fabs(Y->at(0).blockPtr->mean_.y - P->at(0).y) < errorThreshold_) ? 0 : Y->at(0).blockPtr->mean_.y - P->at(0).y;
    transforms->at(0)[2] = (fabs(Y->at(0).blockPtr->mean_.z - P->at(0).z) < errorThreshold_) ? 0 : Y->at(0).blockPtr->mean_.z - P->at(0).z;
    for (int i=1;i<ySize;i++){
        if (timeStamps.at(i) < 0){

            tX = (fabs(Y->at(i).blockPtr->mean_.x - P->at(i).x) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.x - P->at(i).x;
            tY = (fabs(Y->at(i).blockPtr->mean_.y - P->at(i).y) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.y - P->at(i).y;
            tZ = (fabs(Y->at(i).blockPtr->mean_.z - P->at(i).z) < errorThreshold_) ? 0 : Y->at(i).blockPtr->mean_.z - P->at(i).z;
        }
        else{
            tX = slopeX * positiveStamps.at(i);
            tY = slopeY * positiveStamps.at(i);
            tZ = slopeZ * positiveStamps.at(i);
        }

        transforms->at(i)[0] = (fabs(tX) < errorThreshold_) ? 0 : tX;
        transforms->at(i)[1] = (fabs(tY) < errorThreshold_) ? 0 : tY;
        transforms->at(i)[2] = (fabs(tZ) < errorThreshold_) ? 0 : tZ;
        (*T) += transforms->at(i);

    }*/

    // 1st Independent, 2nd Independent

    //ROS_INFO("Time: %f", positiveStamps.back());

    //(*T)[0] = slopeX * positiveStamps.back();
    //(*T)[1] = slopeY * positiveStamps.back();
    //(*T)[2] = slopeZ * positiveStamps.back();

    (*T)[0] = centroidY.x - centroidP.x;
    (*T)[1] = centroidY.y - centroidP.y;
    (*T)[2] = centroidY.z - centroidP.z;

    //(*T)[0] /= ySize;
    //(*T)[1] /= ySize;
    //(*T)[2] /= ySize;



    ROS_INFO("Translation x = %f y = %f z = %f",(*T)[0], (*T)[1], (*T)[2]);

    return error;
}

double calculateError(intensityCloud::Ptr P, std::vector<BlockInfo>* Y){
    intensityCloud::iterator cloudIterator = P->begin();
    //ROS_INFO("Cloud size %d, Candidates size %d", P->size(), Y->size());
    int i = 0;
    double error = 0;
    double distance;
    Vector3d distanceV;
    for (;cloudIterator != P->end();cloudIterator++){
        if (isnan(cloudIterator->x) || isnan(cloudIterator->y) || isnan(cloudIterator->z)) continue;
        distanceV[0] = fabs(Y->at(i).blockPtr->mean_.x - cloudIterator->x);
        distanceV[1] = fabs(Y->at(i).blockPtr->mean_.y - cloudIterator->y);
        distanceV[2] = fabs(Y->at(i).blockPtr->mean_.z - cloudIterator->z);


        if ((distanceV[0] < sqrt(Y->at(i).blockPtr->variance_.x)) && (distanceV[1] < sqrt(Y->at(i).blockPtr->variance_.y)) && (distanceV[2] < sqrt(Y->at(i).blockPtr->variance_.z))){
            error += 0;
        }
        else {
            error += sqrt((Y->at(i).blockPtr->mean_.x - cloudIterator->x) * (Y->at(i).blockPtr->mean_.x - cloudIterator->x)
                                     + (Y->at(i).blockPtr->mean_.y - cloudIterator->y) * (Y->at(i).blockPtr->mean_.y - cloudIterator->y)
                                     + (Y->at(i).blockPtr->mean_.z - cloudIterator->z) * (Y->at(i).blockPtr->mean_.z - cloudIterator->z));
        }
        i++;
    }
    error /= (double)i;
    return error;
}

intensityCloud::Ptr ICP::getTransformation(avora_msgs::StampedIntensityCloudPtr P0, MLSM *X, Vector3d eV, Vector3d eT, Matrix4f eR, Vector3d *Tv, Vector3d *T, Matrix4f *R){
    double error = DBL_MAX;
    unsigned int iterations = 0;
    std::vector<BlockInfo> Y;
    intensityCloud cloud, cloud0;
    pcl::fromROSMsg(P0->cloud,cloud);
    pcl::fromROSMsg(P0->cloud,cloud0);
    std::vector<Vector3d> transforms(cloud.size());
    intensityCloud::Ptr P = boost::make_shared<intensityCloud>(cloud);
    // Initial transform
    ROS_INFO("Applying initial transformation");
    Vector3d accumulatedT;
    Vector3d accumulatedTv;
    Matrix4f accumulatedR = Matrix<float, 4, 4>::Identity();
    accumulatedT[0] = 0;
    accumulatedT[1] = 0;
    accumulatedT[2] = 0;
    accumulatedTv[0] = 0;//eV[0];
    accumulatedTv[1] = 0;//eV[1];
    accumulatedTv[2] = 0;//eV[2];
    (*Tv)[0] = 0;//eV[0];
    (*Tv)[1] = 0;//eV[1];
    (*Tv)[2] = 0;//eV[2];
    //std::vector<Vector3d> transforms = applyTransformation(P, P0->timeStamps, eR, eV, eT);
    ROS_INFO("Initial transformation applied");
    //intensityCloud sum;
    sensor_msgs::PointCloud2 debugCloud;
    //sum = cloud0 + *P;
    //pcl::toROSMsg(*P,debugCloud);
    //debugCloud.header.frame_id = "map";
    //debugPublisher_->publish(debugCloud);
    //sleep(5);
    while ((iterations < maxIterations_) && (error > errorThreshold_)){
        (*R) = Matrix<float, 4, 4>::Identity();
        // Calculate closest points
        ROS_INFO("Get closest points");
        Y = closestPoints(P,X, P0->timeStamps,eV);
        // Calculate transformation
        ROS_INFO("Get transformation");
        error = registration(P,&Y,P0->timeStamps,&transforms, T, R,eV);

        // Iterate through P applying the correct transformation depending on Tv and the stamp
        ROS_INFO("Applying transformation");
        //transforms = applyTransformation(P,P0->timeStamps, *R, *Tv, *T);
        applyTransformation(P,P0->timeStamps, *R, transforms, *T);
        // Calculate error
        ROS_INFO("Calculate error");
        error = calculateError(P, &Y);
        //ROS_INFO("%d error = %f",iterations, error);

        // Iterate?
        iterations++;
        accumulatedT += *T;
        //accumulatedT += transforms.back();
        accumulatedTv += *Tv;
        //ROS_INFO("\n%d \tAcc Vel x = %f y = %f z = %f error = %f",iterations,accumulatedTv[0], accumulatedTv[1], accumulatedTv[2], error);


        accumulatedR = accumulatedR * (*R);
        //ROS_INFO("\n%d \tTrl x = %f y = %f z = %f error = %f \n\tAcc  x = %f y = %f z = %f",iterations,(*T)[0], (*T)[1], (*T)[2], error,
        //accumulatedT[0], accumulatedT[1], accumulatedT[2]);

        //ROS_INFO("\n%d \tRotation \n%f %f %f\n%f %f %f\n%f %f %f",iterations,accumulatedR(0,0),accumulatedR(0,1),accumulatedR(0,2),
        //                                                        accumulatedR(1,0),accumulatedR(1,1),accumulatedR(1,2),
        //                                                        accumulatedR(2,0),accumulatedR(2,1),accumulatedR(2,2));

        //pcl::toROSMsg(*P,debugCloud);
        //debugCloud.header.frame_id = "map";
        //debugPublisher_->publish(debugCloud);
        //sleep(1);
    }
    pcl::toROSMsg(*P,debugCloud);
    debugCloud.header.frame_id = "map";
    debugPublisher_->publish(debugCloud);
    //(*T)[0] = //(*Tv)[0] * (P0->timeStamps.back() - P0->timeStamps.front());
    //(*T)[1] = //(*Tv)[1] * (P0->timeStamps.back() - P0->timeStamps.front());
    //(*T)[2] = //(*Tv)[2] * (P0->timeStamps.back() - P0->timeStamps.front());
    (*T) = accumulatedT;
    (*R) = accumulatedR;
    ROS_INFO("Scan time: %f",fabs(P0->timeStamps.back()) - fabs(P0->timeStamps.front()));
    ROS_INFO("\n%d \tMoved x = %f y = %f z = %f error = %f ",iterations,(*T)[0], (*T)[1], (*T)[2], error);
    //ROS_INFO("\n%d \tRotation \n%f %f %f\n%f %f %f\n%f %f %f",iterations,accumulatedR(0,0),accumulatedR(0,1),accumulatedR(0,2),
    //         accumulatedR(1,0),accumulatedR(1,1),accumulatedR(1,2),
    //         accumulatedR(2,0),accumulatedR(2,1),accumulatedR(2,2));

    //(*T) = accumulatedT;
    return P;
    //return error < errorThreshold_;


}
