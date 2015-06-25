/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/MeshShape.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/Collision.hh"

#include "gazebo/common/Assert.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Rand.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "ImagingSonar.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("imaging_sonar", ImagingSonarSensor)

//////////////////////////////////////////////////
ImagingSonarSensor::ImagingSonarSensor()
: Sensor(sensors::OTHER)
{
}

//////////////////////////////////////////////////
ImagingSonarSensor::~ImagingSonarSensor()
{
  this->sonarCollision->Fini();
  this->sonarCollision.reset();

  this->sonarShape->Fini();
  this->sonarShape.reset();
}

//////////////////////////////////////////////////
std::string ImagingSonarSensor::GetTopic() const
{
  std::string topicName = "~/";
  topicName += this->parentName + "/" + this->GetName() + "/sonar";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void ImagingSonarSensor::Load(const std::string &_worldName)
{
  sdf::ElementPtr sonarElem = this->sdf->GetElement("sonar");
  if (!sonarElem)
  {
    gzerr << "Sonar sensor is missing <sonar> SDF element";
    return;
  }

  this->rangeMin = sonarElem->Get<double>("min");
  this->rangeMax = sonarElem->Get<double>("max");

  this->angle = 0;
  this->step = sonarElem->Get<double>("step");

  this->openingAngle = sonarElem->Get<double>("opening");
  this->binCount = sonarElem->Get<int>("binCount");


  this->radius = tan(this->openingAngle) * this->rangeMax;

  double range = this->rangeMax - this->rangeMin;

  if (this->radius < 0)
  {
    gzerr << "Sonar radius must be > 0. Current value is["
      << this->radius << "]\n";
    return;
  }

  if (this->rangeMin < 0)
  {
    gzerr << "Min sonar range must be >= 0. Current value is["
      << this->rangeMin << "]\n";
    return;
  }

  if (this->rangeMin > this->rangeMax)
  {
    gzerr << "Min sonar range of [" << this->rangeMin << "] must be less than"
      << "the max sonar range of[" << this->rangeMax << "]\n";
    return;
  }

  if (this->step < 0)
  {
    gzerr << "Sonar step must be > 0. Current value is["
      << this->step << "]\n";
    return;
  }

  if (this->openingAngle < 0)
  {
    gzerr << "Sonar openingAngle must be > 0. Current value is["
      << this->openingAngle << "]\n";
    return;
  }

  if (this->binCount < 0)
  {
    gzerr << "Sonar binCount must be > 0. Current value is["
      << this->binCount << "]\n";
    return;
  }

  Sensor::Load(_worldName);
  GZ_ASSERT(this->world != NULL,
      "ImagingSonarSensor did not get a valid World pointer");

  this->parentEntity = this->world->GetEntity(this->parentName);

  GZ_ASSERT(this->parentEntity != NULL,
      "Unable to get the parent entity.");

  physics::PhysicsEnginePtr physicsEngine = this->world->GetPhysicsEngine();

  GZ_ASSERT(physicsEngine != NULL,
      "Unable to get a pointer to the physics engine");

  /// \todo: Change the collision shape to a cone. Needs a collision shape
  /// within ODE. Or, switch out the collision engine.
  this->sonarCollision = physicsEngine->CreateCollision("mesh",
      this->parentName);

  GZ_ASSERT(this->sonarCollision != NULL,
      "Unable to create a cylinder collision using the physics engine.");

  this->sonarCollision->SetName(this->GetScopedName() + "sensor_collision");

  // We need a few contacts in order to get the closest collision. This is
  // not guaranteed to return the closest contact.
  this->sonarCollision->SetMaxContacts(2);
  this->sonarCollision->AddType(physics::Base::SENSOR_COLLISION);
  this->parentEntity->AddChild(this->sonarCollision);

  this->sonarShape = boost::dynamic_pointer_cast<physics::MeshShape>(
      this->sonarCollision->GetShape());

  GZ_ASSERT(this->sonarShape != NULL,
      "Unable to get the sonar shape from the sonar collision.");

  // Use a scaled cone mesh for the sonar collision shape.
  this->sonarShape->SetMesh("unit_cone");
  this->sonarShape->SetScale(math::Vector3(this->radius*2.0,
        this->radius*2.0, range));

  // Position the collision shape properly. Without this, the shape will be
  // centered at the start of the sonar.
  math::Vector3 offset(0, 0, range * 0.5);
  offset = this->pose.rot.RotateVector(offset);
  this->sonarMidPose.Set(this->pose.pos - offset, this->pose.rot);

  this->sonarCollision->SetRelativePose(this->sonarMidPose);
  this->sonarCollision->SetInitialRelativePose(this->sonarMidPose);

  // Don't create contacts when objects collide with the sonar shape
  this->sonarCollision->GetSurface()->collideWithoutContact = true;
  this->sonarCollision->GetSurface()->collideWithoutContactBitmask = 1;
  this->sonarCollision->SetCollideBits(~GZ_SENSOR_COLLIDE);
  this->sonarCollision->SetCategoryBits(GZ_SENSOR_COLLIDE);

  /*std::vector<std::string> collisions;
  collisions.push_back(this->sonarCollision->GetScopedName());

  physics::ContactManager *contactMgr =
    this->world->GetPhysicsEngine()->GetContactManager();
    */

  // Create a contact topic for the collision shape
  std::string topic =
    this->world->GetPhysicsEngine()->GetContactManager()->CreateFilter(
        this->sonarCollision->GetScopedName(),
        this->sonarCollision->GetScopedName());

  // Subscribe to the contact topic
  this->contactSub = this->node->Subscribe(topic,
      &ImagingSonarSensor::OnContacts, this);

  // Advertise the sensor's topic on which we will output range data.
  this->sonarPub = this->node->Advertise<gazebo_imaging_sonar::msgs::SonarScanLineStamped>(this->GetTopic());

  // Initialize the message that will be published on this->sonarPub.
  this->scanLineMsg.mutable_scan()->set_range_min(this->rangeMin);
  this->scanLineMsg.mutable_scan()->set_range_max(this->rangeMax);
  this->scanLineMsg.mutable_scan()->set_radius(this->radius);

  msgs::Set(this->scanLineMsg.mutable_scan()->mutable_world_pose(), this->sonarMidPose);
  this->scanLineMsg.mutable_scan()->set_range(0);

  this->angle = fmod((this->angle + (this->step * M_PI/180.0)),2*M_PI);

  gazebo::math::Quaternion q;
  q.SetFromEuler(0,0,this->step * M_PI/180.0);
  this->pose.CoordRotationAdd(q);
}

//////////////////////////////////////////////////
void ImagingSonarSensor::Init()
{
  Sensor::Init();
  this->scanLineMsg.mutable_scan()->set_frame(this->parentName);
  msgs::Set(this->scanLineMsg.mutable_time(), this->world->GetSimTime());

  if (this->sonarPub)
    this->sonarPub->Publish(this->scanLineMsg);
}

//////////////////////////////////////////////////
void ImagingSonarSensor::Fini()
{
  this->sonarPub.reset();
  this->contactSub.reset();
  Sensor::Fini();
}

//////////////////////////////////////////////////
double ImagingSonarSensor::GetRangeMin() const
{
  return this->rangeMin;
}

//////////////////////////////////////////////////
double ImagingSonarSensor::GetRangeMax() const
{
  return this->rangeMax;
}

//////////////////////////////////////////////////
double ImagingSonarSensor::GetRadius() const
{
  return this->radius;
}

//////////////////////////////////////////////////
double ImagingSonarSensor::GetRange()
{
  boost::mutex::scoped_lock lock(this->mutex);
  return this->scanLineMsg.scan().range();
}

//////////////////////////////////////////////////
void ImagingSonarSensor::UpdateImpl(bool /*_force*/)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->lastMeasurementTime = this->world->GetSimTime();
  msgs::Set(this->scanLineMsg.mutable_time(), this->lastMeasurementTime);
  this->scanLineMsg.mutable_scan()->set_range(0);

  math::Pose referencePose = this->pose + this->parentEntity->GetWorldPose();
  math::Vector3 pos;

  this->scanLineMsg.mutable_scan()->set_range(this->rangeMax);
  std::vector<int> intensities(this->binCount);
  int maxContactCount = 0;
  // Iterate over all the contact messages
  for (ContactMsgs_L::iterator iter = this->incomingContacts.begin();
      iter != this->incomingContacts.end(); ++iter)
  {
    // Iterate over all the contacts in the message
    for (int i = 0; i < (*iter)->contact_size(); ++i)
    {
      // Debug output:
      // std::cout << "Collision1[" << (*iter)->contact(i).collision1() << "]"
      //  << "C2[" << (*iter)->contact(i).collision2() << "]\n";

      for (int j = 0; j < (*iter)->contact(i).position_size(); ++j)
      {
        pos = msgs::Convert((*iter)->contact(i).position(j));
        math::Vector3 relPos = pos - referencePose.pos;
        double len = pos.Distance(referencePose.pos);

        intensities[(int)(len * (this->rangeMax / this->binCount))] += 1;
        if (intensities[(int)(len * (this->rangeMax / this->binCount))] > maxContactCount){
            maxContactCount = intensities[(int)(len * (this->rangeMax / this->binCount))];
        }
        // Debug output:
        // std::cout << "  SP[" << this->pose.pos << "]  P[" << pos
        //  << "] Len[" << len << "]D["
        //  << (*iter)->contact(i).depth(j) << "]\n";

        // Copy the contact message.
        //if (len < this->sonarMsg.sonar().range())
        //{
        //  this->sonarMsg.mutable_sonar()->set_range(len);
        //}
      }
    }
  }

  for (int i = 0; i < this->binCount; ++i){
    intensities[i] = (intensities[i] / maxContactCount) * 127;
    this->scanLineMsg.mutable_scan()->set_intensities(i,intensities[i]);
  }
  this->scanLineMsg.mutable_scan()->set_angle(this->angle);
  this->scanLineMsg.mutable_scan()->set_range_max(this->rangeMax);
  this->scanLineMsg.mutable_scan()->set_range_min(this->rangeMin);


  // Clear the incoming contact list.
  this->incomingContacts.clear();

  this->update(this->scanLineMsg);

  if (this->sonarPub)
    this->sonarPub->Publish(this->scanLineMsg);
}

//////////////////////////////////////////////////
bool ImagingSonarSensor::IsActive()
{
  return Sensor::IsActive() || this->sonarPub->HasConnections();
}

//////////////////////////////////////////////////
void ImagingSonarSensor::OnContacts(ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Only store information if the sensor is active
  if (this->IsActive() && _msg->contact_size() > 0)
  {
    // Store the contacts message for processing in UpdateImpl
    this->incomingContacts.push_back(_msg);

    // Prevent the incomingContacts list to grow indefinitely.
    if (this->incomingContacts.size() > 100)
      this->incomingContacts.pop_front();
  }
}
