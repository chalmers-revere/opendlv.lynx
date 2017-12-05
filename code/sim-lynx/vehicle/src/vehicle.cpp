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
  , m_accelerationRequest(0.0f)
  , m_decelerationRequest(0.0f)
  , m_steeringRequest(0.0f)
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
  odcore::data::TimeStamp lastUpdate;
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == 
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    odcore::base::Lock l(m_requestMutex);
    
    odcore::data::TimeStamp now;
    float deltaTime = static_cast<float>((now.getMicroseconds() - lastUpdate.getMicroseconds()) / 1000000.0);
    lastUpdate = now;

    std::cout << "Delta time: " << deltaTime << std::endl; // Remove later.

    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    float rollRate = 0.0f;
    float pitchRate = 0.0f;
    float yawRate = 0.0f;

    // Add vehicle model equations.
    


    //

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
