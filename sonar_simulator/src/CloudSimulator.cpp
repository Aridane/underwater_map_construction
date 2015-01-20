#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;



int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("/sonar/Cloud", 1);

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "odom";
  msg->height = 1;

  pcl::PointXYZI p;

  /**************Three Blocks****************************/
  p.x = 5; p.y = 0.5; p.z = 2.9; p.intensity = 128;
  msg->points.push_back (p);
  p.x = 5; p.y = 1.5; p.z = 2.9; p.intensity = 128;
  msg->points.push_back (p);
  p.x = 5; p.y = -0.5; p.z = 2.9; p.intensity = 128;
  msg->points.push_back (p);

  p.x = 5; p.y = 0.5; p.z = 2.1; p.intensity = 128;
  msg->points.push_back (p);
  p.x = 5; p.y = 1.5; p.z = 2.1; p.intensity = 128;
  msg->points.push_back (p);
  p.x = 5; p.y = -0.5; p.z = 2.1; p.intensity = 128;
  msg->points.push_back (p);

  /***********************Two new blocks below************/
  p.x = 5; p.y = 1.5; p.z = 1.6; p.intensity = 128;
  msg->points.push_back (p);
  p.x = 5; p.y = -0.5; p.z = 1.6; p.intensity = 128;
  msg->points.push_back (p);
  /********************Force Fuse*************************/
  p.x = 5; p.y = 1.5; p.z = 2; p.intensity = 128;
  msg->points.push_back (p);
  p.x = 5; p.y = -0.5; p.z = 2; p.intensity = 128;
  msg->points.push_back (p);

  msg->width = 10;

  ros::Rate loop_rate(1);
  sensor_msgs::PointCloud2 pc2;

  pcl::toROSMsg(*msg,pc2);
  while (nh.ok())
  {
    pc2.header.stamp = ros::Time::now();
    pub.publish (pc2);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}