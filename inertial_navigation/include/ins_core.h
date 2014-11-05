#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

class ins_core{


private:
    double rate_;
    int maxIterations_;
    int iterator_;

    nav_msgs::Odometry odomMsg;
    sensor_msgs::Imu imuMsg;

    ros::Subscriber imuSubscriber_;
    ros::Publisher odomPublisher_;
    ros::Publisher debugImuPub_;

    double oldVelocity[3];
    double oldAcceleration[3];
    double oldPosition[3];

    bool first;

    ros::Time current_time_, last_time_;
public:
    ins_core(ros::NodeHandle *n);
    ~ins_core();

    void imuCallback(sensor_msgs::Imu::Ptr imuMsg);
    void updateOdom();
    int getIterator();
    int getMaxIterations();
    int setIterator(int value);



};
