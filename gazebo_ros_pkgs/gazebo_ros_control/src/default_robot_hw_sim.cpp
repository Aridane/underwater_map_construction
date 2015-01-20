/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Johnathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#ifndef _GAZEBO_ROS_CONTROL___DEFAULT_ROBOT_HW_SIM_H_
#define _GAZEBO_ROS_CONTROL___DEFAULT_ROBOT_HW_SIM_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>

namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace gazebo_ros_control
{

class DefaultRobotHWSim : public gazebo_ros_control::RobotHWSim
{
public:

  bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
    // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
    const ros::NodeHandle joint_limit_nh(model_nh, robot_namespace);

    // Resize vectors to our DOF
    n_dof_ = transmissions.size();
    joint_names_.resize(n_dof_);
    joint_types_.resize(n_dof_);
    joint_lower_limits_.resize(n_dof_);
    joint_upper_limits_.resize(n_dof_);
    joint_effort_limits_.resize(n_dof_);
    joint_control_methods_.resize(n_dof_);
    pid_controllers_.resize(n_dof_);
    joint_position_.resize(n_dof_);
    joint_velocity_.resize(n_dof_);
    joint_effort_.resize(n_dof_);
    joint_effort_command_.resize(n_dof_);
    joint_position_command_.resize(n_dof_);
    joint_velocity_command_.resize(n_dof_);

    // Initialize values
    for(unsigned int j=0; j < n_dof_; j++)
    {
      // Check that this transmission has one joint
      if(transmissions[j].joints_.size() == 0)
      {
        ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
          << " has no associated joints.");
        continue;
      }
      else if(transmissions[j].joints_.size() > 1)
      {
        ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
          << " has more than one joint. Currently the default robot hardware simulation "
          << " interface only supports one.");
        continue;
      }

      // Check that this transmission has one actuator
      if(transmissions[j].actuators_.size() == 0)
      {
        ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
          << " has no associated actuators.");
        continue;
      }
      else if(transmissions[j].actuators_.size() > 1)
      {
        ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
          << " has more than one actuator. Currently the default robot hardware simulation "
          << " interface only supports one.");
        continue;
      }

      // Add data from transmission
      joint_names_[j] = transmissions[j].joints_[0].name_;
      joint_position_[j] = 1.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j] = 1.0;  // N/m for continuous joints
      joint_effort_command_[j] = 0.0;
      joint_position_command_[j] = 0.0;
      joint_velocity_command_[j] = 0.0;

#if ROS_VERSION_MINOR > 10 || ROS_VERSION_MAJOR > 1
      const std::string &hardware_interface =
        transmissions[j].actuators_[0].hardware_interfaces_[0];
#else
      const std::string &hardware_interface =
        transmissions[j].actuators_[0].hardware_interface_;
#endif

      // Debug
      ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim","Loading joint '" << joint_names_[j]
        << "' of type '" << hardware_interface << "'");

      // Create joint state interface for all joints
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

      // Decide what kind of command interface this actuator/joint has
      hardware_interface::JointHandle joint_handle;
      if(hardware_interface == "EffortJointInterface")
      {
        // Create effort joint interface
        joint_control_methods_[j] = EFFORT;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_effort_command_[j]);
        ej_interface_.registerHandle(joint_handle);
      }
      else if(hardware_interface == "PositionJointInterface")
      {
        // Create position joint interface
        joint_control_methods_[j] = POSITION;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_position_command_[j]);
        pj_interface_.registerHandle(joint_handle);
      }
      else if(hardware_interface == "VelocityJointInterface")
      {
        // Create velocity joint interface
        joint_control_methods_[j] = VELOCITY;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_velocity_command_[j]);
        vj_interface_.registerHandle(joint_handle);
      }
      else
      {
        ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
          << hardware_interface );
        return false;
      }

      // Get the gazebo joint that corresponds to the robot joint.
      //ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "Getting pointer to gazebo joint: "
      //  << joint_names_[j]);
      gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
      if (!joint)
      {
        ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_[j]
          << "\" which is not in the gazebo model.");
        return false;
      }
      sim_joints_.push_back(joint);

      registerJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j],
                          joint_limit_nh, urdf_model,
                          &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                          &joint_effort_limits_[j]);
      if (joint_control_methods_[j] != EFFORT)
      {
        // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
        // joint->SetVelocity() to control the joint.
        const ros::NodeHandle nh(model_nh, robot_namespace + "/gazebo_ros_control/pid_gains/" +
                                 joint_names_[j]);
        if (pid_controllers_[j].init(nh))
        {
          switch (joint_control_methods_[j])
          {
            case POSITION:
              joint_control_methods_[j] = POSITION_PID;
              break;
            case VELOCITY:
              joint_control_methods_[j] = VELOCITY_PID;
              break;
          }
        }
        else
        {
          // joint->SetMaxForce() must be called if joint->SetAngle() or joint->SetVelocity() are
          // going to be called. joint->SetMaxForce() must *not* be called if joint->SetForce() is
          // going to be called.
          joint->SetMaxForce(0, joint_effort_limits_[j]);
        }
      }
    }

    // Register interfaces
    registerInterface(&js_interface_);
    registerInterface(&ej_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&vj_interface_);

    return true;
  }

  void readSim(ros::Time time, ros::Duration period)
  {
    for(unsigned int j=0; j < n_dof_; j++)
    {
      // Gazebo has an interesting API...
      if (joint_types_[j] == urdf::Joint::PRISMATIC)
      {
        joint_position_[j] = sim_joints_[j]->GetAngle(0).Radian();
      }
      else
      {
        joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                              sim_joints_[j]->GetAngle(0).Radian());
      }
      joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
      joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
    }
  }

  void writeSim(ros::Time time, ros::Duration period)
  {
    ej_sat_interface_.enforceLimits(period);
    ej_limits_interface_.enforceLimits(period);
    pj_sat_interface_.enforceLimits(period);
    pj_limits_interface_.enforceLimits(period);
    vj_sat_interface_.enforceLimits(period);
    vj_limits_interface_.enforceLimits(period);

    for(unsigned int j=0; j < n_dof_; j++)
    {
      switch (joint_control_methods_[j])
      {
        case EFFORT:
          {
            const double effort = joint_effort_command_[j];
            sim_joints_[j]->SetForce(0, effort);
          }
          break;

        case POSITION:
#if GAZEBO_MAJOR_VERSION >= 4
          sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
#else
          sim_joints_[j]->SetAngle(0, joint_position_command_[j]);
#endif
          break;

        case POSITION_PID:
          {
            double error;
            switch (joint_types_[j])
            {
              case urdf::Joint::REVOLUTE:
                angles::shortest_angular_distance_with_limits(joint_position_[j],
                                                              joint_position_command_[j],
                                                              joint_lower_limits_[j],
                                                              joint_upper_limits_[j],
                                                              error);
                break;
              case urdf::Joint::CONTINUOUS:
                error = angles::shortest_angular_distance(joint_position_[j],
                                                          joint_position_command_[j]);
                break;
              default:
                error = joint_position_command_[j] - joint_position_[j];
            }

            const double effort_limit = joint_effort_limits_[j];
            const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                        -effort_limit, effort_limit);
            sim_joints_[j]->SetForce(0, effort);
          }
          break;

        case VELOCITY:
          sim_joints_[j]->SetVelocity(0, joint_velocity_command_[j]);
          break;

        case VELOCITY_PID:
          const double error = joint_velocity_command_[j] - joint_velocity_[j];
          const double effort_limit = joint_effort_limits_[j];
          const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                      -effort_limit, effort_limit);
          sim_joints_[j]->SetForce(0, effort);
          break;
      }
    }
  }

private:
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

  // Register the limits of the joint specified by joint_name and joint_handle. The limits are
  // retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const ControlMethod ctrl_method,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit)
  {
    *joint_type = urdf::Joint::UNKNOWN;
    *lower_limit = -std::numeric_limits<double>::max();
    *upper_limit = std::numeric_limits<double>::max();
    *effort_limit = std::numeric_limits<double>::max();

    joint_limits_interface::JointLimits limits;
    bool has_limits = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;

    if (urdf_model != NULL)
    {
      const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
      if (urdf_joint != NULL)
      {
        *joint_type = urdf_joint->type;
        // Get limits from the URDF file.
        if (joint_limits_interface::getJointLimits(urdf_joint, limits))
          has_limits = true;
        if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
          has_soft_limits = true;
      }
    }
    // Get limits from the parameter server.
    if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
      has_limits = true;

    if (!has_limits)
      return;

    if (*joint_type == urdf::Joint::UNKNOWN)
    {
      // Infer the joint type.

      if (limits.has_position_limits)
      {
        *joint_type = urdf::Joint::REVOLUTE;
      }
      else
      {
        if (limits.angle_wraparound)
          *joint_type = urdf::Joint::CONTINUOUS;
        else
          *joint_type = urdf::Joint::PRISMATIC;
      }
    }

    if (limits.has_position_limits)
    {
      *lower_limit = limits.min_position;
      *upper_limit = limits.max_position;
    }
    if (limits.has_effort_limits)
      *effort_limit = limits.max_effort;

    if (has_soft_limits)
    {
      switch (ctrl_method)
      {
        case EFFORT:
          {
            const joint_limits_interface::EffortJointSoftLimitsHandle
              limits_handle(joint_handle, limits, soft_limits);
            ej_limits_interface_.registerHandle(limits_handle);
          }
          break;
        case POSITION:
          {
            const joint_limits_interface::PositionJointSoftLimitsHandle
              limits_handle(joint_handle, limits, soft_limits);
            pj_limits_interface_.registerHandle(limits_handle);
          }
          break;
        case VELOCITY:
          {
            const joint_limits_interface::VelocityJointSoftLimitsHandle
              limits_handle(joint_handle, limits, soft_limits);
            vj_limits_interface_.registerHandle(limits_handle);
          }
          break;
      }
    }
    else
    {
      switch (ctrl_method)
      {
        case EFFORT:
          {
            const joint_limits_interface::EffortJointSaturationHandle
              sat_handle(joint_handle, limits);
            ej_sat_interface_.registerHandle(sat_handle);
          }
          break;
        case POSITION:
          {
            const joint_limits_interface::PositionJointSaturationHandle
              sat_handle(joint_handle, limits);
            pj_sat_interface_.registerHandle(sat_handle);
          }
          break;
        case VELOCITY:
          {
            const joint_limits_interface::VelocityJointSaturationHandle
              sat_handle(joint_handle, limits);
            vj_sat_interface_.registerHandle(sat_handle);
          }
          break;
      }
    }
  }

  unsigned int n_dof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;

  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<ControlMethod> joint_control_methods_;
  std::vector<control_toolbox::Pid> pid_controllers_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;
};

typedef boost::shared_ptr<DefaultRobotHWSim> DefaultRobotHWSimPtr;

}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::DefaultRobotHWSim, gazebo_ros_control::RobotHWSim)

#endif // #ifndef __GAZEBO_ROS_CONTROL_PLUGIN_DEFAULT_ROBOT_HW_SIM_H_
