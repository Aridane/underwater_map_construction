//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <laser_imaging_sonar_plugin.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <limits>

namespace gazebo {

AvoraLaserSonar::AvoraLaserSonar()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
AvoraLaserSonar::~AvoraLaserSonar()
{
  updateTimer.Disconnect(updateConnection);
  sensor_->SetActive(false);

  dynamic_reconfigure_server_.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void AvoraLaserSonar::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboRosSonar requires a Ray Sensor as its parent");
    return;
  }
  gazebo::math::Rand::SetSeed(time(NULL));  

  // Get the world name.
  std::string worldName = sensor_->GetWorldName();
  world = physics::get_world(worldName);

  // default parameters
  namespace_.clear();
  topic_ = "sonar";
  frame_id_ = "/sonar_link";
  binCount_ = 500;
  step_ = 0.05235987755;
  angle_ = 0;
  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("binCount"))
    binCount_ = _sdf->GetElement("binCount")->GetValueInt();

  if (_sdf->HasElement("step"))
    step_ = _sdf->GetElement("step")->GetValueDouble();


  sensor_model_.Load(_sdf);

  scanLine_.header.frame_id = frame_id_;
  scanLine_.maxrange_meters = sensor_->GetRangeMax();
  scanLine_.range_resolution = sensor_->GetRangeMax() / binCount_;
  scanLine_.intensities.reserve(binCount_);
  scanLine_.intensities.resize(binCount_);


  range_.header.frame_id = frame_id_;
  range_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_.field_of_view = std::min(fabs((sensor_->GetAngleMax() - sensor_->GetAngleMin()).Radian()), fabs((sensor_->GetVerticalAngleMax() - sensor_->GetVerticalAngleMin()).Radian()));
  range_.max_range = sensor_->GetRangeMax();
  range_.min_range = sensor_->GetRangeMin();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  publisher_ = node_handle_->advertise<sensor_msgs::Range>(topic_ + "/range", 1);
  scanLinePublisher_ = node_handle_->advertise<avora_msgs::SonarScanLine>(topic_, 2);
  // setup dynamic_reconfigure server
  dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_)));
  dynamic_reconfigure_server_->setCallback(boost::bind(&SensorModel::dynamicReconfigureCallback, &sensor_model_, _1, _2));

  Reset();

  // connect Update function
  updateTimer.setUpdateRate(10.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&AvoraLaserSonar::Update, this));

  // activate RaySensor
  sensor_->SetActive(true);
}

void AvoraLaserSonar::Reset()
{
  updateTimer.Reset();
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void AvoraLaserSonar::Update()
{
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  scanLine_.header.stamp.sec = (world->GetSimTime()).sec;
  scanLine_.header.stamp.nsec = (world->GetSimTime()).nsec;
  scanLine_.angle = angle_;
  scanLine_.intensities.clear();
  scanLine_.intensities.reserve(binCount_);
  scanLine_.intensities.resize(binCount_);
  int contactCount[binCount_];
  double intensities[binCount_];
  for (int i=0;i<binCount_;i++){
      contactCount[i] = 0;
      intensities[i] = 0;
      scanLine_.intensities[i] = 0;
  }
  range_.header.stamp.sec  = (world->GetSimTime()).sec;
  range_.header.stamp.nsec = (world->GetSimTime()).nsec;
  
  // find ray with minimal range
  range_.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();
  int num_ranges = sensor_->GetLaserShape()->GetSampleCount() * sensor_->GetLaserShape()->GetVerticalSampleCount();
  int maxContactCount = 1;
  int index;
  int maxContactIndexes[3] = {-1,-1,-1};

  for(int i = 0; i < num_ranges; ++i) {
    double ray = sensor_->GetLaserShape()->GetRange(i);
    int index = ceil((ray * ((double)binCount_/(double)sensor_->GetRangeMax()))) - 1;
    if (ray == sensor_->GetRangeMax()) continue;
    if (index < 0) index = 0;
    contactCount[index] += 1;
    if (contactCount[index] > maxContactCount){
        maxContactCount = contactCount[index];
    }
    /*
    for (int k=0;k<10;k++){
     double auxRay = sensor_model_(ray, dt);
      index = ceil((auxRay * (binCount_/sensor_->GetRangeMax()))) - 1;
      if (index < 0) index = 0;
      if (index > (binCount_-1)) continue;

      contactCount[index] += 1;

      if (contactCount[index] > maxContactCount){
          maxContactCount = contactCount[index];
      }
    }*/
    if (ray < range_.range) range_.range = ray;

  }


  //std::cout << "Max Contact count: " << maxContactCount << "\n";
  
  /*for (int k=0;k<10;k++){
      int auxIndex = gazebo::math::Rand::GetDblNormal (maxContactCount[0], 5);
      index = ceil((auxRay * (binCount_/sensor_->GetRangeMax()))) - 1;
      if (index < 0) index = 0;
      if (index > (binCount_-1)) continue;

      contactCount[index] += 1;

      
  }*/

maxContactCount = num_ranges;
  for (int i=0;i<binCount_;i++){
      //std::cout << "contacts["<< i << "]: " << contactCount[i] << " ";
      if (contactCount[i] != 0){
          for (int k=0;k<contactCount[i];k++){
              double ray = i * sensor_->GetRangeMax() / binCount_;
              double auxRay= gazebo::math::Rand::GetDblNormal(ray, 0.5);
              index = ceil((auxRay * (binCount_/sensor_->GetRangeMax()))) - 1;
              if (index < 0) index = 0;
              if (index > (binCount_-1)) continue;
              intensities[index] += 15;
              //scanLine_.intensities[index] += ;
              if (intensities[index] > 127) intensities[index] = 127;
//              if (contactCount[index] > maxContactCount){
//                maxContactCount = contactCount[index];
//              }
          }
          //scanLine_.intensities[i] = 127;
          //if (contactCount[i] > 0.75 * maxContactCount) scanLine_.intensities[i] = 127;
          //else scanLine_.intensities[i] = 127.0*((double)contactCount[i] / (0.75*maxContactCount == 0 ? 1.0 : (double)0.75*maxContactCount));
          //scanLine_.intensities[i] = 127.0*((double)contactCount[i] / (maxContactCount == 0 ? 1.0 : (double)maxContactCount));
      }
  }
    for (int i=0;i<binCount_;i++){
        scanLine_.intensities[i] = floor(intensities[i]);
    }
  for (int k=0;k<15;k++){
      double auxRay = gazebo::math::Rand::GetDblNormal (range_.range, 4);
      index = ceil((auxRay * (binCount_/sensor_->GetRangeMax()))) - 1;
      if (index < 0) index = 0;
      if (index > (binCount_-1)) continue;
      scanLine_.intensities[index] = gazebo::math::Rand::GetDblNormal (53, 20);
  }

  //std::cout << "\n";
  // add Gaussian noise (and limit to min/max range)
  if (range_.range < range_.max_range) {
    range_.range = sensor_model_(range_.range, dt);
    if (range_.range < range_.min_range) range_.range = range_.min_range;
    if (range_.range > range_.max_range) range_.range = range_.max_range;
  }

  publisher_.publish(range_);
  scanLinePublisher_.publish(scanLine_);

  angle_ = fmod(angle_ + step_ ,2*M_PI);


  physics::ModelPtr model = world->GetModel("robot1");
  physics::LinkPtr link = model->GetLink("sonarDato");
  physics::JointPtr joint = model->GetJoint("sonarDato_to_sonar");

  //math:Pose pose = sensor_->GetPose();
  //pose.rot = pose.rot.
  //link->SetWorldPose();
  joint->SetPosition(0,angle_);
  joint->Update();

}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(AvoraLaserSonar)

} // namespace gazebo
