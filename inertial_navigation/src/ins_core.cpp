#include "ins_core.h"
#define GRAVITY 9.81

std::string turtle_name;

ros::Publisher odom_pub;

/*void poseCallback(const turtlesim::PoseConstPtr& msg){
    ros::Time current_time;
    current_time = ros::Time::now();

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, msg->theta) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //set the position
    odom.pose.pose.position.x = msg->x;
    odom.pose.pose.position.y = msg->y;              // y measurement GPS.
    odom.pose.pose.position.z = 0;              // z measurement GPS.
    odom.pose.pose.orientation.x = 1;               // identity quaternion
    odom.pose.pose.orientation.y = 0;               // identity quaternion
    odom.pose.pose.orientation.z = 0;               // identity quaternion
    odom.pose.pose.orientation.w = 0;               // identity quaternion
    odom.pose.covariance[0]  = 10;
    odom.pose.covariance[7]  = 10;
    odom.pose.covariance[14] = 10;
    odom.pose.covariance[21] = 99999;
    odom.pose.covariance[28] = 99999;
    odom.pose.covariance[35] = 99999;

    //publish the message
    odom_pub.publish(odom);
}*/
ins_core::ins_core(ros::NodeHandle *n){
    for (int i=0;i<3;i++){
        oldVelocity[i] = 0;
        oldAcceleration[i] = 0;
        oldPosition[i] = 0;
    }
    last_time_ = ros::Time::now();
    imuSubscriber_ = n->subscribe("/imu/data",10,&ins_core::imuCallback,this);
    debugImuPub_ = n->advertise<sensor_msgs::Imu>("Debug/ImuGrav",10);
    odomPublisher_ = n->advertise<nav_msgs::Odometry>("INS/odom",10);
    iterator_ = 0;
    maxIterations_ = 15;
}
ins_core::~ins_core(){

}

int ins_core::getIterator(){
    return iterator_;
}
int ins_core::getMaxIterations(){
    return maxIterations_;
}
int ins_core::setIterator(int value){
    return iterator_ = value;

}

/*# compensate the accelerometer readings from gravity.
# @param q the quaternion representing the orientation of a 9DOM MARG sensor array
# @param acc the readings coming from an accelerometer expressed in g
#
# @return a 3d vector representing dinamic acceleration expressed in g
def gravity_compensate(q, acc):
  g = [0.0, 0.0, 0.0]

  # get expected direction of gravity
  g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
  g[1] = 2 * (q[0] * q[1] + q[2] * q[3])
  g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]

  # compensate accelerometer readings with the expected direction of gravity
  return [acc[0] - g[0], acc[1] - g[1], acc[2] - g[2]]*/
void ins_core::updateOdom(){
    double newVelocity[3];
    double newPosition[3];

    current_time_ = ros::Time::now();

    double dt = current_time_.toSec() - last_time_.toSec();

    newVelocity[0] = oldVelocity[0] + imuMsg.linear_acceleration.x;
    newVelocity[1] = oldVelocity[1] + imuMsg.linear_acceleration.y;
    newVelocity[2] = oldVelocity[2] + imuMsg.linear_acceleration.z;

    newPosition[0] = oldPosition[0] + newVelocity[0]*(dt) + 0.5*imuMsg.linear_acceleration.x*dt;
    newPosition[1] = oldPosition[1] + newVelocity[1]*(dt) + 0.5*imuMsg.linear_acceleration.y*dt;;
    newPosition[2] = oldPosition[2] + newVelocity[2]*(dt) + 0.5*imuMsg.linear_acceleration.z*dt;;

    //ROS_INFO("diffX %f diffY %f diffZ %f, oldX %f oldY %f oldZ %f",diff1,diff2,diff3, oldAcceleration[0], oldAcceleration[1],oldAcceleration[2]);




    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //set the position
    odom.pose.pose.position.x = newPosition[0];
    odom.pose.pose.position.y = newPosition[1];              // y measurement GPS.
    odom.pose.pose.position.z = newPosition[2];              // z measurement GPS.
    odom.pose.pose.orientation.x = imuMsg.orientation.x;               // identity quaternion
    odom.pose.pose.orientation.y = imuMsg.orientation.y;               // identity quaternion
    odom.pose.pose.orientation.z = imuMsg.orientation.z;               // identity quaternion
    odom.pose.pose.orientation.w = imuMsg.orientation.w;               // identity quaternion
    odom.pose.covariance[0]  = 5;
    odom.pose.covariance[7]  = 5;
    odom.pose.covariance[14] = 5;
    odom.pose.covariance[21] = 1;
    odom.pose.covariance[28] = 1;
    odom.pose.covariance[35] = 1;

    odomPublisher_.publish(odom);
    oldVelocity[0] = newVelocity[0];
    oldVelocity[1] = newVelocity[1];
    oldVelocity[2] = newVelocity[2];

    oldPosition[0] = newPosition[0];
    oldPosition[1] = newPosition[1];
    oldPosition[2] = newPosition[2];
}

void ins_core::imuCallback(sensor_msgs::Imu::Ptr msg){
    double currentAcceleration[3];
    double gravity[3];

    //Compensate gravity

    gravity[0] = 2.0* (msg->orientation.x * msg->orientation.z - msg->orientation.w*msg->orientation.y) * GRAVITY;
    gravity[1] = 2.0* (msg->orientation.w * msg->orientation.x + msg->orientation.y*msg->orientation.z) * GRAVITY;
    gravity[2] = (msg->orientation.w*msg->orientation.w
                  - msg->orientation.x*msg->orientation.x
                  - msg->orientation.y*msg->orientation.y
                  + msg->orientation.z*msg->orientation.z) * GRAVITY;



    currentAcceleration[0] = msg->linear_acceleration.x - gravity[0];
    currentAcceleration[1] = msg->linear_acceleration.y - gravity[1];
    currentAcceleration[2] = msg->linear_acceleration.z - gravity[2];

    imuMsg.header = msg->header;


    if (fabs(oldAcceleration[0] - currentAcceleration[0]) < 0.058) currentAcceleration[0] = oldAcceleration[0];
    if (fabs(oldAcceleration[1] - currentAcceleration[1]) < 0.058) currentAcceleration[1] = oldAcceleration[1];
    if (fabs(oldAcceleration[2] - currentAcceleration[2]) < 0.058) currentAcceleration[2] = oldAcceleration[2];

    if (fabs(currentAcceleration[0]) < 0.05) currentAcceleration[0] = 0;
    if (fabs(currentAcceleration[1]) < 0.05) currentAcceleration[1] = 0;
    if (fabs(currentAcceleration[2]) < 0.05) currentAcceleration[2] = 0;


    if (iterator_ == 0){
        last_time_ = ros::Time::now();
        imuMsg.linear_acceleration.x = currentAcceleration[0];
        imuMsg.linear_acceleration.y = currentAcceleration[1];
        imuMsg.linear_acceleration.z = currentAcceleration[2];
        imuMsg.orientation = msg->orientation;

    }
    else{
        imuMsg.linear_acceleration.x = iterator_*imuMsg.linear_acceleration.x/(iterator_+1.0) + currentAcceleration[0]/(iterator_+1.0);
        imuMsg.linear_acceleration.y = iterator_*imuMsg.linear_acceleration.y/(iterator_+1.0) + currentAcceleration[1]/(iterator_+1.0);
        imuMsg.linear_acceleration.z = iterator_*imuMsg.linear_acceleration.z/(iterator_+1.0) + currentAcceleration[2]/(iterator_+1.0);
        imuMsg.orientation = msg->orientation;
    }
    debugImuPub_.publish(imuMsg);



    oldAcceleration[0] = currentAcceleration[0];
    oldAcceleration[1] = currentAcceleration[1];
    oldAcceleration[2] = currentAcceleration[2];

    iterator_++;


}





int main(int argc, char** argv){
    ros::init(argc, argv, "ins_core");
    ros::NodeHandle n;

    ins_core ins(&n);

    while (ros::ok()){
        ros::spinOnce();
        if (ins.getIterator() > ins.getMaxIterations()){
            ins.setIterator(0);
            ins.updateOdom();
        }
    }
    return 0;
};
