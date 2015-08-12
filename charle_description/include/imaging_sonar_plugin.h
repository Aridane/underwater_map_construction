/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_ROS_IMAGING_SONAR_HH
#define GAZEBO_ROS_IMAGING_SONAR_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <avora_msgs/SonarScanLine.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/ImagingSonarPlugin.hh>
 
#include <sdf/sdf.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class AvoraImagingSonarPlugin : public ImagingSonarPlugin
  {
    /// \brief Constructor
    public: AvoraImagingSonarPlugin();

    /// \brief Destructor
    public: ~AvoraImagingSonarPlugin();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Keep track of number of connctions
    private: int imaging_sonar_connect_count_;
    private: void ImagingSonarConnect();
    private: void ImagingSonarDisconnect();

    // Pointer to the model
    private: std::string world_name_;
    private: physics::WorldPtr world_;
    /// \brief The parent sensor
    private: sensors::ImagingSonarSensorPtr parent_imaging_sonar_sensor_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<avora_msgs::SonarScanLine>::Ptr pub_queue_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;
    
    /// \brief tf prefix
    private: std::string tf_prefix_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: unsigned int seed;

    // Current sonar echo emitting head position
    private: double current_head_angle_;
    // Step between head positions in degrees
    private: unsigned int bin_count_;                   
    private: double cone_angle_;                                    



    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr imaging_sonar_beam_sub_;
    private: void OnBeam(ConstSonarScanLineStampedPtr &_msg);

    /// \brief prevents blocking
    private: PubMultiQueue pmq;
  };
}
#endif
