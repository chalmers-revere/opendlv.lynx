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
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include <iostream>

#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>

#include "can.hpp"

namespace opendlv {
namespace proxy {
namespace mantis {

Can::Can(int32_t const &a_argc, char **a_argv) :
  TimeTriggeredConferenceClientModule(a_argc, a_argv, "proxy-mantis-can")
{
}

Can::~Can()
{
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Can::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    opendlv::proxy::GroundSpeedReading o1();
    Container c1(o1);
    m_conference.send(c1);

    opendlv::proxy::GlobalPositionWgs84Reading o2();
    Container c2(o2);
    m_conference.send(c2);

    opendlv::proxy::GyroscopeReading o3();
    Container c3(o3);
    m_conference.send(c3);

    opendlv::proxy::AccelerometerReading o4();
    Container c4(o4);
    m_conference.send(c4);

    opendlv::proxy::MagnetometerReading o5();
    Container c5(o5);
    m_conference.send(c5);
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void Can::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::proxy::GroundAccelerationRequest::ID()) {
    // auto kinematicState = a_container.getData<opendlv::coord::KinematicState>();
  }
}

void Can::setUp()
{
  // std::string const exampleConfig = 
  //   getKeyValueConfiguration().getValue<std::string>(
  //     "proxy-mantis-can.example-config");

  // if (isVerbose()) {
  //   std::cout << "Example config is " << exampleConfig << std::endl;
  // }
}

void Can::tearDown()
{
}

}
}
}
