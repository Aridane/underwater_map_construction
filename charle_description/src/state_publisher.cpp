#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#include <iostream>

using namespace std;

float servo_pan = 1.54;
float servo_tilt = 0;
float servo_sonar = 0;

void updateServo_pan(const std_msgs::Int16 &servo_pan_aux)
{
	servo_pan = servo_pan_aux.data*M_PI/180;
	cout << "updateServo_pan = " << servo_pan << endl;
}


void updateServo_tilt(const std_msgs::Int16 &servo_tilt_aux)
{
	servo_tilt = (-1) *  servo_tilt_aux.data*M_PI/180;
	cout << "updateServo_tilt= " << servo_tilt << endl;
}

void updateServo_sonar(const std_msgs::Int16 &servo_sonar_aux)
{
	servo_sonar = servo_sonar_aux.data*M_PI/180;
	cout << "updateServo_sonar= " << servo_sonar << endl;
}

int main(int argc, char** argv) {
ros::init(argc, argv, "state_publisher");
    
    ros::NodeHandle n;
    ros::Subscriber servo_pan_sub = n.subscribe("/servo_cam_pan", 10,  updateServo_pan);

    ros::Subscriber servo_tilt_sub = n.subscribe("/servo_cam_tilt", 10,  updateServo_tilt);

    ros::Subscriber servo_sonar_sub = n.subscribe("/servo_sonar", 10,  updateServo_sonar);
    
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

  // robot state
  double inc= 0.005, servo_sonar = 0;
  double angle= 0;
  // message declarations
  geometry_msgs::TransformStamped odom_trans;
  sensor_msgs::JointState joint_state;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  while (ros::ok()) {
	ros::spinOnce();
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] ="servo_head_to_base_link";
        //cout << "servo_pan = " << servo_pan << endl;
	joint_state.position[0] = servo_tilt;
      joint_state.name[1] ="head_to_servo_head";
      joint_state.position[1] = servo_pan;
      joint_state.name[2] ="sonar_to_base_link";
      joint_state.position[2] = servo_sonar;

      // update transform
      // (moving in a circle with radius 1)
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.transform.translation.x = cos(angle)*1;
      odom_trans.transform.translation.y = sin(angle)*1;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw (0, 0, angle+(degree*90.0));
//tf::createQuaternionMsgFromYaw();

       //send the joint state and transform
      joint_pub.publish(joint_state);
      //broadcaster.sendTransform(odom_trans);

       // Create new robot state
       angle += degree/4;

      // This will adjust as needed per iteration
      loop_rate.sleep();
  }
  return 0;
}
