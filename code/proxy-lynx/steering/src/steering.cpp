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
#include <vector>

#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>

#include "steering.hpp"

namespace opendlv {
namespace proxy {
namespace lynx {

Steering::Steering(int32_t const &a_argc, char **a_argv) :
  TimeTriggeredConferenceClientModule(a_argc, a_argv, "proxy-lynx-steering")
  , m_classVar()
{
  std::cout << "[" << getName() << "] I got created!" << std::endl;
  m_classVar = 1;
}

Steering::~Steering()
{
  std::cout << "[" << getName() << "] I got destroyed..." << std::endl;
}




void Steering::setUp()
{
  std::cout << "[" << getName() << "] I'll set up myself." << std::endl;
  auto kv = getKeyValueConfiguration();
  std::string const exampleConfig = kv.getValue<std::string>(
      "proxy-lynx-steering.example-config");
  // proxy-lynx-steering.double = 0.0001
  double const exampleDouble = kv.getValue<double>("proxy-lynx-steering.double");
  // proxy-lynx-steering.int = 3
  int32_t const exampleInt = kv.getValue<int32_t>("proxy-lynx-steering.int");
  // proxy-lynx-steering.boolean = 1
  bool const exampleBool = (kv.getValue<int32_t>("proxy-lynx-steering.boolean") == 1);
  // proxy-lynx-steering.string-seperator = 10,11,22,33
  std::string const exampleSeperator = kv.getValue<std::string>(
      "proxy-lynx-steering.string-seperator");
  std::vector<int32_t> valVector;
  std::vector<std::string> exampleSepSplitted = 
      odcore::strings::StringToolbox::split(exampleSeperator, ','); 
  for (auto str : exampleSepSplitted) {
    valVector.push_back(std::stod(str));
  }

  (void) exampleDouble;
  (void) exampleInt;
  (void) exampleBool;

  if (isVerbose()) {
    std::cout << "Example config is " << exampleConfig << std::endl;
  }
}

void Steering::tearDown()
{
  std::cout << "[" << getName() << "] I'll wrap things up before I get destroyed." << std::endl;  
}

void Steering::nextContainer(odcore::data::Container &a_container)
{
  if (isVerbose()) {
    std::cout << "[" << getName() << "] I got a container... hmm.." << std::endl;
  }
  if (a_container.getDataType() == opendlv::proxy::GroundSteeringRequest::ID()) {
    auto groundSteeringRequest = a_container.getData<opendlv::proxy::GroundSteeringRequest>();
    if (isVerbose()) {
      std::cout << "[" << getName() << "] I got a steering request! " 
          << std::endl << groundSteeringRequest.toString() << std::endl;
      
    } 
  }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Steering::body()
{
  // Todo: actual steering reading
  double steeringReading = 0;
  double increment = 0.01;
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    if (steeringReading < -1.0 || steeringReading > 1.0) {
      increment = -increment;
    }
    steeringReading += increment;
    sendGroundSteeringReading(steeringReading);
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void Steering::sendGroundSteeringReading(double a_groundsteering)
{
  //Send groundsteering
  // std::cout << "Argument" << a_argument << std::endl;

  opendlv::proxy::GroundSteeringReading gsr(a_groundsteering);
  odcore::data::Container c(gsr);
  getConference().send(c);
  if (isVerbose()) {
    std::cout << "[" << getName() << "] Sending: " << gsr.toString() << std::endl;
  }
}

}
}
}
