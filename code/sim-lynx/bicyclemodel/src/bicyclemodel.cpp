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
#include <fstream>

#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>

#include "bicyclemodel.hpp"

namespace opendlv {
namespace sim {
namespace lynx {

BicycleModel::BicycleModel(int32_t const &a_argc, char **a_argv) :
  TimeTriggeredConferenceClientModule(a_argc, a_argv, "sim-lynx-bicyclemodel"),
  m_groundAccelerationMutex{},
  m_groundSteeringAngleMutex{},
  m_groundAcceleration{0.0},
  m_groundSteeringAngle{0.0}
  //,m_newAcc{false}
{
}


BicycleModel::~BicycleModel()
{
}

void BicycleModel::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::proxy::GroundDecelerationRequest::ID()) {
    odcore::base::Lock l(m_groundAccelerationMutex);
    auto groundDeceleration = a_container.getData<opendlv::proxy::GroundDecelerationRequest>();
    m_groundAcceleration = -groundDeceleration.getGroundDeceleration();
    //std::cout<<"GroundAcceleration recieved = "<<m_groundAcceleration<<"\n";
    //m_newAcc = true;
  } else if (a_container.getDataType() == opendlv::proxy::GroundAccelerationRequest::ID()) {
    odcore::base::Lock l(m_groundAccelerationMutex);
    auto groundAcceleration = a_container.getData<opendlv::proxy::GroundAccelerationRequest>();
    m_groundAcceleration = groundAcceleration.getGroundAcceleration();
    //std::cout<<"GroundAcceleration recieved = "<<m_groundAcceleration<<"\n";
    //m_newAcc = true;
  } else if (a_container.getDataType() == opendlv::proxy::GroundSteeringRequest::ID()) {
    odcore::base::Lock m(m_groundSteeringAngleMutex);
    auto groundSteeringAngle = a_container.getData<opendlv::proxy::GroundSteeringRequest>();
    m_groundSteeringAngle = groundSteeringAngle.getGroundSteering();
    //std::cout<<"groundSteeringAngle recieved = "<<m_groundSteeringAngle<<"\n";
  }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode BicycleModel::body()
{
  double const g = 9.82f;

  auto kv = getKeyValueConfiguration();
  double const mass = kv.getValue<double>("sim-lynx-bicyclemodel.mass");
  double const momentOfInertiaZ = kv.getValue<double>("sim-lynx-bicyclemodel.momentOfInertiaZ");
  double const length = kv.getValue<double>("sim-lynx-bicyclemodel.length");
  double const frontToCog = kv.getValue<double>("sim-lynx-bicyclemodel.frontToCog");
  double const rearToCog = length - frontToCog;
  double const frictionCoefficient = kv.getValue<double>("sim-lynx-bicyclemodel.frictionCoefficient");

  double const magicFormulaCAlpha = kv.getValue<double>("sim-lynx-bicyclemodel.magicFormulaCAlpha");
  double const magicFormulaC = kv.getValue<double>("sim-lynx-bicyclemodel.magicFormulaC");
  double const magicFormulaE = kv.getValue<double>("sim-lynx-bicyclemodel.magicFormulaE");
  std::string const filename = kv.getValue<std::string>("sim-lynx-bicyclemodel.filename");
  double const dt = 1.0 / static_cast<double>(getFrequency());

  double longitudinalSpeed{0.01};
  double longitudinalSpeedDot{0.0};
  double lateralSpeed{0.0};
  double yawRate{0.0};
  /*double const kp = 2.0;
  double const kd = 0.0;
  double const ki = 0.01;
  double e, ed, ei;
  double ePrev = 0.0;*/
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    double groundAccelerationCopy;
    double groundSteeringAngleCopy;
    {
      odcore::base::Lock l(m_groundAccelerationMutex);
      odcore::base::Lock m(m_groundSteeringAngleMutex);
      groundAccelerationCopy = m_groundAcceleration;
      groundSteeringAngleCopy = m_groundSteeringAngle;
    }
    /*if (//m_newAcc) {
      ei = 0.0;
      ePrev = 0.0;
      //m_newAcc = false;
    }
    e = groundAccelerationCopy-longitudinalSpeedDot;
    ei += e*dt;
    ed = (e-ePrev)/dt;
    double acceleration = kp*e+kd*ed+ki*ei;
    std::cout<<"groundAccelerationCopy: "<<groundAccelerationCopy<<endl;
    std::cout<<"longitudinalSpeedDot: "<<longitudinalSpeedDot<<endl;
    std::cout<<"ACCELERATION: "<<acceleration<<endl;
    std::cout<<"e: "<<e<<endl;
    std::cout<<"ePrev: "<<ePrev<<endl;
    std::cout<<"dt: "<<dt<<endl;
    std::cout<<"ed: "<<ed<<endl;
    std::cout<<"ei: "<<ei<<endl;
    std::cout<<"kp*e: "<<kp*e<<endl;
    std::cout<<"kd*ed: "<<kd*ed<<endl;
    std::cout<<"ki*ei: "<<ki*ei<<endl;
    if (acceleration > 5.0) {
      acceleration = 5.0;
    }else if(acceleration < -5.0){
      acceleration = -5.0;
    }*/

    double slipAngleFront = groundSteeringAngleCopy - std::atan(
        (lateralSpeed + frontToCog * yawRate) / std::abs(longitudinalSpeed));
    double slipAngleRear = -std::atan((lateralSpeed - rearToCog * yawRate) /
        std::abs(longitudinalSpeed));

    double forceFrontZ = mass * g * (frontToCog / (frontToCog + length));
    double forceRearZ = mass * g * (length / (frontToCog + length));

    double forceFrontY = magicFormula(slipAngleFront, forceFrontZ,
        frictionCoefficient, magicFormulaCAlpha, magicFormulaC, magicFormulaE);
    double forceRearY = magicFormula(slipAngleRear, forceRearZ,
        frictionCoefficient, magicFormulaCAlpha, magicFormulaC, magicFormulaE);

    double rollResistance;
    if (longitudinalSpeed>0) {rollResistance = -9.81*0.02;}
    else if (longitudinalSpeed<0){rollResistance = 9.81*0.02;}
    else {rollResistance = 0.0;}

    longitudinalSpeedDot = groundAccelerationCopy - std::sin(groundSteeringAngleCopy)*forceFrontY/mass + yawRate * lateralSpeed + rollResistance;

    double lateralSpeedDot =
      (forceFrontY * std::cos(groundSteeringAngleCopy) + forceRearY) / mass -
      yawRate * longitudinalSpeed;

    double yawRateDot = (frontToCog * forceFrontY *
        std::cos(groundSteeringAngleCopy) - rearToCog * forceRearY) /
      momentOfInertiaZ;

    longitudinalSpeed += longitudinalSpeedDot * dt;
    if (longitudinalSpeed<0) {
      longitudinalSpeed = 0.01;
    }
    lateralSpeed += lateralSpeedDot * dt;
    yawRate += yawRateDot * dt;

    //ePrev = e;

    opendlv::sim::KinematicState kinematicState;
    kinematicState.setVx(longitudinalSpeed);
    kinematicState.setVy(lateralSpeed);
    kinematicState.setYawRate(yawRate);
    kinematicState.setVz(0.0);
    kinematicState.setRollRate(0.0);
    kinematicState.setPitchRate(0.0);

    odcore::data::Container c(kinematicState);
    getConference().send(c);

    float groundSpeed = static_cast<float>(sqrt(pow(longitudinalSpeed,2)+pow(lateralSpeed,2)));
    opendlv::proxy::GroundSpeedReading groundSpeedReading;
    groundSpeedReading.setGroundSpeed(groundSpeed);
    odcore::data::Container c1(groundSpeedReading);
    getConference().send(c1);


    std::ofstream speedFile;
    speedFile.open("/opt/opendlv.data/"+filename,std::ios_base::app);
    speedFile<<longitudinalSpeed<<","<<lateralSpeed<<","<<yawRate<<","<<groundAccelerationCopy<<","<<longitudinalSpeedDot<<","<<lateralSpeedDot<<","<<yawRateDot<<std::endl;
    speedFile.close();

  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


void BicycleModel::setUp()
{
}

void BicycleModel::tearDown()
{
}

double BicycleModel::magicFormula(double const &a_slipAngle, double const &a_forceZ,
    double const &a_frictionCoefficient, double const &a_cAlpha, double const &a_c,
    double const &a_e) const
{
  double const b = a_cAlpha / (a_c * a_frictionCoefficient * a_forceZ);
  double const forceY = a_frictionCoefficient * a_forceZ * std::sin(a_c *
     std::atan(b * a_slipAngle - a_e * (b * a_slipAngle - std::atan(b * a_slipAngle))));
  return forceY;
}

}
}
}
