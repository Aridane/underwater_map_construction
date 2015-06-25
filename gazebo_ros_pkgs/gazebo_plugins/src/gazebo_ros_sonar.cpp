/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: GazeboRosSonar plugin for simulating Imaging sonar sensors in Gazebo
   Author: Aridane J. Sarrionandia de León
   Date: 24 June 2015
 */

#include <algorithm>
#include <string>
#include <assert.h>

#include <sdf/sdf.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SonarSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "gazebo_plugins/gazebo_ros_sonar.h"
#include <gazebo_plugins/gazebo_ros_utils.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosSonar)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosSonar::GazeboRosSonar()
{
  this->seed = 0;
  this->current_head_angle_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosSonar::~GazeboRosSonar()
{
  ROS_DEBUG_STREAM_NAMED("sonar","Shutting down Sonar");
  this->rosnode_->shutdown();
  delete this->rosnode_;
  ROS_DEBUG_STREAM_NAMED("sonar","Unloaded");
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSonar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  SonarPlugin::Load(_parent, this->sdf);
  // Get the world name.
  std::string worldName = _parent->GetWorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  this->parent_sonar_sensor_ =
    boost::dynamic_pointer_cast<sensors::SonarSensor>(_parent);

  if (!this->parent_sonar_sensor_)
    gzthrow("GazeboRosSonar controller requires a Ray Sensor as its parent");

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO("GazeboRosSonar plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");

  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO("GazeboRosSonar plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  if (!this->sdf->HasElement("step"))
  {
    ROS_INFO("GazeboRosSonar plugin missing <step>, defaults to 3");
    this->step_ = 3;
  }
  else
    this->step_  = this->sdf->Get<double>("step");

  if (!this->sdf->HasElement("binCount"))
  {
    ROS_INFO("Sonar plugin missing <binCount>, defaults to 500");
    this->bin_count_ = 500;
  }
  else
      this->bin_count_ = this->sdf->Get<unsigned int>("binCount");

  if (!this->sdf->HasElement("coneAngle"))
  {
    ROS_INFO("Sonar plugin missing <coneAngle>, defaults to 500");
    this->cone_angle_ = 3;
  }
  else
      this->cone_angle_ = this->sdf->Get<double>("coneAngle");

  this->sonar_connect_count_ = 0;

  //this->parentSensor->radius = tan(cone_angle_ * M_PI/180.0) * this->parentSensor->GetRangeMax();


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO ( "Starting GazeboRosSonar Plugin (ns = %s)!", this->robot_namespace_.c_str() );
  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosSonar::LoadThread, this));

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSonar::LoadThread()
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
  ROS_INFO("Sonar Plugin (ns = %s) <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  // resolve tf prefix
  this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<avora_msgs::SonarScanLine>(
      this->topic_name_, 1,
      boost::bind(&GazeboRosSonar::SonarConnect, this),
      boost::bind(&GazeboRosSonar::SonarDisconnect, this),
      ros::VoidPtr(), NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<avora_msgs::SonarScanLine>();
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_sonar_sensor_->SetActive(false);

  ROS_INFO_STREAM_NAMED("sonar","LoadThread function completed");
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosSonar::SonarConnect()
{
  this->sonar_connect_count_++;
  if (this->sonar_connect_count_ == 1)
    this->sonar_beam_sub_ =
      this->gazebo_node_->Subscribe(this->parent_sonar_sensor_->GetTopic(),
                                    &GazeboRosSonar::OnBeam, this);
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosSonar::SonarDisconnect()
{
  this->sonar_connect_count_--;
  if (this->sonar_connect_count_ == 0)
    this->sonar_beam_sub_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void GazeboRosSonar::OnBeam(ConstSonarStampedPtr &_msg)
{
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  /*sensor_msgs::LaserScan laser_msg;
  laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
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
  avora_msgs::SonarScanLine sonar_msg;
  sonar_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  sonar_msg.header.frame_id = this->frame_name_;
  sonar_msg.angle = this->current_head_angle_;
  //sonar_msg.gain;
  //ROS_INFO_STREAM_NAMED("sonar","Range %f Contact X %f Y %f Z %f", _msg->mutable_sonar()->range(), _msg->mutable_sonar()->contact().x(), _msg->mutable_sonar()->contact().y());

  sonar_msg.maxrange_meters = _msg->mutable_sonar()->range_max();
  sonar_msg.range_resolution = (_msg->mutable_sonar()->range_max() - _msg->mutable_sonar()->range_min()) / this->bin_count_;
  _msg->mutable_sonar()->contact();
  sonar_msg.intensities;

  //this->pub_queue_->push(laser_msg, this->pub_);
}
}
