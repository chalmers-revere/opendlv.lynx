/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>

#include "vehicle.hpp"

namespace opendlv {
namespace sim {
namespace lynx {

Vehicle::Vehicle(const int &argc, char **argv)
  : TimeTriggeredConferenceClientModule(argc, argv, "sim-lynx-vehicle")
  , m_requestMutex()
  , m_accelerationRequest(1.0f)
  , m_decelerationRequest(0.0f)
  , m_steeringRequest(100.0f)
  , m_vehicleLength(2.65f)
  , m_momentOfInertia(1675.0f)
  , m_cf(49600.0f)
  , m_cr(43300.0f)
  , m_a(1.0865f)
  , m_b(1.5635f)
  , m_lateralVelocity(0.0f)
  , m_yaw(0.0f)
  , m_longitudinalVelocity(0.0f)
  , m_vehicleMass(1290.0f)
{
}

Vehicle::~Vehicle()
{
}

void Vehicle::nextContainer(odcore::data::Container &a_container)
{
  
  if (a_container.getDataType() == opendlv::proxy::GroundAccelerationRequest::ID()) {
    odcore::base::Lock l(m_requestMutex);
    auto groundAccelerationRequest = 
      a_container.getData<opendlv::proxy::GroundAccelerationRequest>();
    m_accelerationRequest = groundAccelerationRequest.getGroundAcceleration();
  } else if (a_container.getDataType() == opendlv::proxy::GroundDecelerationRequest::ID()) {
    odcore::base::Lock l(m_requestMutex);
    auto groundDecelerationRequest = 
      a_container.getData<opendlv::proxy::GroundDecelerationRequest>();
    m_decelerationRequest = groundDecelerationRequest.getGroundDeceleration();
  } else if (a_container.getDataType() == opendlv::proxy::GroundSteeringRequest::ID()) {
    odcore::base::Lock l(m_requestMutex);
    auto groundSteeringRequest = 
      a_container.getData<opendlv::proxy::GroundSteeringRequest>();
    m_steeringRequest = groundSteeringRequest.getGroundSteering();
  }
}

void Vehicle::setUp()
{
}

void Vehicle::tearDown()
{
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Vehicle::body()
{
  float const deltaTime = 1.0f / getFrequency();

  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == 
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    odcore::base::Lock l(m_requestMutex);
    
    std::cout << "Delta time: " << deltaTime << std::endl; // Remove later.

    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    float rollRate = 0.0f;
    float pitchRate = 0.0f;
    float yawRate = 0.1f;
    float theta = 0.0f, alphaF = 0.0f, alphaR = 0.0f, velocityRate = 0.0f;

    // Add vehicle model equations
    std::cout << "Steering Request " << m_steeringRequest << std::endl;
    std::cout << "AccelerationRequest : " << m_accelerationRequest << std::endl;
    std::cout << "DecelerationRequest: " << m_decelerationRequest << std::endl;

    theta = m_steeringRequest/static_cast<float>(14.7);
    m_longitudinalVelocity = m_longitudinalVelocity + (m_accelerationRequest + m_decelerationRequest) * deltaTime;
    alphaF = (((-m_a * m_yaw) - m_lateralVelocity) / m_longitudinalVelocity) + theta;
    alphaR = ((m_b * m_yaw) - m_lateralVelocity) / m_longitudinalVelocity;
    velocityRate = ((m_cf * alphaF) + (m_cr  * alphaR)) / (m_vehicleMass - (m_longitudinalVelocity * m_yaw));
    yawRate = ((m_a * m_cf * alphaF) - (m_b * m_cr * alphaR)) / m_momentOfInertia;
    
    m_yaw = m_yaw + (yawRate * deltaTime);
    m_lateralVelocity = m_lateralVelocity + (velocityRate * deltaTime);
    vx = m_lateralVelocity * static_cast<float>(sin(m_yaw));
    vy = m_lateralVelocity * static_cast<float>(cos(m_yaw));

    std::cout << "Vx: " << vx << std::endl;
    std::cout << "Vy: " << vy << std::endl;
    std::cout << "Yaw Rate: " << yawRate << std::endl;

    opendlv::coord::KinematicState kinematicState(vx, vy, vz, rollRate, 
        pitchRate, yawRate);
    odcore::data::Container container(kinematicState);
    getConference().send(container);
  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

}
}
} 
