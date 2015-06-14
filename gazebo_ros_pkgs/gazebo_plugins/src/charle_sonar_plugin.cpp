/*
* Copyright 2013 Open Source Robotics Foundation
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
*/
/*
Desc: GazeboRosGpuLaser plugin for simulating ray sensors in Gazebo
Author: Mihai Emanuel Dolha
Date: 29 March 2012
*/

#include <algorithm>
#include <string>
#include <assert.h>

#include <sdf/sdf.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>


#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo_plugins/charle_sonar_plugin.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(CharleSonarPlugin)
////////////////////////////////////////////////////////////////////////////////
// Constructor
CharleSonarPlugin::CharleSonarPlugin()
{
    this->seed = 0;
    this->currentStep_ = 0;
}
////////////////////////////////////////////////////////////////////////////////
// Destructor
CharleSonarPlugin::~CharleSonarPlugin()
{
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void CharleSonarPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
   // load plugin
  GpuRayPlugin::Load(_parent, this->sdf);
  // Get the world name.
  std::string worldName = _parent->GetWorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  this->parent_ray_sensor_ =
    boost::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("CharleSonarPlugin controller requires a Ray Sensor as its parent");

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO("Laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");


  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO("Laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  this->laser_connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  if (!this->sdf->HasElement("step"))
  {
    ROS_INFO("Laser plugin missing <step>, defaults to 3");
    this->step_ = 3;
  }
  else
    this->step_ = atoi((this->sdf->Get<std::string>("step")).c_str());
    
  if (!this->sdf->HasElement("decay"))
  {
    ROS_INFO("Laser plugin missing <decay>, defaults to 32");
    this->decay_ = 32;
  }
  else
    this->decay_ = atoi((this->sdf->Get<std::string>("decay")).c_str());

  if (!this->sdf->HasElement("binCount"))
  {
    ROS_INFO("Laser plugin missing <binCount>, defaults to 500");
    this->binCount_ = 500;
  }
  else
    this->binCount_ = atoi((this->sdf->Get<std::string>("binCount")).c_str());

  ROS_INFO ( "Starting Laser Plugin (ns = %s)!", this->robot_namespace_.c_str() );
  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&CharleSonarPlugin::LoadThread, this));

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void CharleSonarPlugin::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name_);

  this->pmq.startServiceThread();

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
  if(this->tf_prefix_.empty()) {
      this->tf_prefix_ = this->robot_namespace_;
      boost::trim_right_if(this->tf_prefix_,boost::is_any_of("/"));
  }
  ROS_INFO("Laser Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  // resolve tf prefix
  this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<avora_msgs::SonarScanLine>(
      this->topic_name_, 1,
      boost::bind(&CharleSonarPlugin::LaserConnect, this),
      boost::bind(&CharleSonarPlugin::LaserDisconnect, this),
      ros::VoidPtr(), NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<avora_msgs::SonarScanLine>();
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void CharleSonarPlugin::LaserConnect()
{
  this->laser_connect_count_++;
  if (this->laser_connect_count_ == 1)
    this->laser_scan_sub_ =
      this->gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(),
                                    &CharleSonarPlugin::OnScan, this);
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void CharleSonarPlugin::LaserDisconnect()
{
  this->laser_connect_count_--;
  if (this->laser_connect_count_ == 0)
    this->laser_scan_sub_.reset();
}
////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void CharleSonarPlugin::OnScan(ConstLaserScanStampedPtr &_msg)
{
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  /*sensor_msgs::LaserScan laser_msg;
  /*laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  laser_msg.header.frame_id = this->frame_name_;
  laser_msg.angle_min = _msg->scan().angle_min();
  laser_msg.angle_max = _msg->scan().angle_max();
  laser_msg.angle_increment = _msg->scan().angle_step();
  laser_msg.time_increment = 0;  // instantaneous simulator scan
  laser_msg.scan_time = 0;  // not sure whether this is correct
  laser_msg.range_min = _msg->scan().range_min();
  laser_msg.range_max = _msg->scan().range_max();
  laser_msg.ranges.resize(_msg->scan().ranges_size());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().end(),
            laser_msg.ranges.begin());
  laser_msg.intensities.resize(_msg->scan().intensities_size());
  std::copy(_msg->scan().intensities().begin(),
            _msg->scan().intensities().end(),
            laser_msg.intensities.begin());*/
  //this->pub_queue_->push(laser_msg, this->pub_);
  avora_msgs::SonarScanLine sonar_msg;
  sonar_msg.intensities.resize(binCount_);
  sonar_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  sonar_msg.header.frame_id = this->frame_name_;
  sonar_msg.maxrange_meters = _msg->scan().range_max();
  sonar_msg.range_resolution = _msg->scan().range_max() / binCount_;
  sonar_msg.angle = ((this->currentStep_+1) * this->step_)*(M_PI / 180.0);

  int pos = (_msg->scan().ranges().Get(currentStep_) * ((double)binCount_)) / _msg->scan().range_max();
  int intensity = 128;
  if (_msg->scan().ranges().Get(currentStep_)!= _msg->scan().range_max())
  {
    if (pos != 0){
    	sonar_msg.intensities[pos] = intensity;
  	  int i = 1;
  	  while(intensity != 0){
  	  	if ((intensity = intensity - this->decay_) < 0) intensity = 0;
  	  	if ((pos + i) < binCount_) sonar_msg.intensities[pos+i] = intensity;
  	  	if ((pos - i) >= 0) sonar_msg.intensities[pos-i] = intensity;
  	  	i++;
  	  }
    }
  }
  currentStep_ = (currentStep_ + 1) % _msg->scan().ranges_size();

  this->pub_queue_->push(sonar_msg, this->pub_);
  //this->pub_queue_->push(laser_msg, this->pub_);
}
}
