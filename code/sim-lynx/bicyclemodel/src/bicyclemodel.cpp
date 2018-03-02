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

#include "bicyclemodel.hpp"

namespace opendlv {
namespace sim {
namespace lynx {

BicycleModel::BicycleModel(int32_t const &a_argc, char **a_argv) :
  TimeTriggeredConferenceClientModule(a_argc, a_argv, "sim-lynx-bicyclemodel"),
  m_vehicleModelParameters(),
  m_sampleTime(),
  m_groundAcceleration(),
  m_states(),
  m_delta(),
  m_accelerationMutex(),
  m_aimPointMutex(),
  m_kp()
{
m_vehicleModelParameters = Eigen::MatrixXf::Zero(7,1);
m_states = Eigen::MatrixXf::Zero(5,1);
m_delta = 0.0f;
}


BicycleModel::~BicycleModel()
{
}

void BicycleModel::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::proxy::GroundDecelerationRequest::ID()) {
    odcore::base::Lock l(m_accelerationMutex);
    auto groundDeceleration = a_container.getData<opendlv::proxy::GroundDecelerationRequest>();
    m_groundAcceleration = groundDeceleration.getGroundDeceleration();
  }
  if (a_container.getDataType() == opendlv::proxy::GroundAccelerationRequest::ID()) {
    odcore::base::Lock l(m_accelerationMutex);
    auto groundAcceleration = a_container.getData<opendlv::proxy::GroundAccelerationRequest>();
    m_groundAcceleration = groundAcceleration.getGroundAcceleration();

  }
  if (a_container.getDataType() == opendlv::logic::action::AimPoint::ID()) {
    odcore::base::Lock m(m_aimPointMutex);
    auto aimPoint = a_container.getData<opendlv::logic::action::AimPoint>();
    m_delta = m_kp*aimPoint.getAzimuthAngle();
  }


}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode BicycleModel::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    /*m_states << xPosition,
      yPosition,
      groundSpeedx,
      groundSpeedy,
      yawRate,
      yaw;
      */
    m_states << vehicleModel(m_states);

    opendlv::sim::KinematicState kinematicState;
    kinematicState.setVx(m_states(2));
    kinematicState.setVy(m_states(3));
    kinematicState.setYawRate(m_states(4));
    odcore::data::Container c1(kinematicState);
    getConference().send(c1);

  /*message opendlv.sim.KinematicState [id = 1002]
  float vx [id = 1];
  float vy [id = 2];
  float vz [id = 3];
  float rollRate [id = 4];
  float pitchRate [id = 5];
  float yawRate [id = 6];
  */
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


void BicycleModel::setUp()
{
  //auto kv = getKeyValueConfiguration();
  //%%%%%%%%%%%%VEHICLE MODEL PARAMETERS%%%%%%%%%%%%%%%%%%%
	float const vM = 188.0f; //kg     //kv.getValue<float>("sim-lynx-bicyclemodel.vehicle-parameter.m");
	float const vIz = 105.0f; //kgm²  //kv.getValue<float>("sim-lynx-bicyclemodel.vehicle-parameter.Iz");
	float const vG = 9.82f; //m/s²  //kv.getValue<float>("sim-lynx-bicyclemodel.vehicle-parameter.g");
	float const vL = 1.53f; //m     //kv.getValue<float>("sim-lynx-bicyclemodel.vehicle-parameter.l");
	float const vLf = 0.765f; //m (l*0.5) //kv.getValue<float>("sim-lynx-bicyclemodel.vehicle-parameter.lf");
	float const vLr = 0.765f; //m (l*0.5) //kv.getValue<float>("sim-lynx-bicyclemodel.vehicle-parameter.lr");
	float const vMu = 0.9f;         //kv.getValue<float>("sim-lynx-bicyclemodel.vehicle-parameter.mu");
	m_vehicleModelParameters << vM,vIz,vG,vL,vLf,vLr,vMu;

  //%%%%%%%%%%%%System Params%%%%%%%%%%%%%%%%%%%%%%
  m_sampleTime = 0.1f; //kv.getValue<float>("sim-lynx-bicyclemodel.T");
  //%%%%%%%%%%%%Steering Parameters%%%%%%%%%%%%%%%%%%%%%%
   m_kp = 2.0f; //kv.getValue<float>("sim-lynx-bicyclemodel.steering-parameter.kp");
}

void BicycleModel::tearDown()
{
}

Eigen::MatrixXf BicycleModel::vehicleModel(Eigen::MatrixXf &x)
{

	if(x(2) < 0.0001f){

		x(2) = 0.01f;
	}
  odcore::base::Lock l(m_accelerationMutex);
  odcore::base::Lock m(m_aimPointMutex);
	float alphaF = std::atan((m_vehicleModelParameters(3)*x(4)) + x(3)/x(2)) - m_delta;
	float alphaR = std::atan((x(3)-m_vehicleModelParameters(4)*x(4))/x(2));

	//Non linear Tire Model

	float Fzf = m_vehicleModelParameters(0)*m_vehicleModelParameters(2)*(m_vehicleModelParameters(4)/(m_vehicleModelParameters(4)+m_vehicleModelParameters(3)));
	float Fzr = m_vehicleModelParameters(0)*m_vehicleModelParameters(2)*(m_vehicleModelParameters(3)/(m_vehicleModelParameters(4)+m_vehicleModelParameters(3)));

	float Fyf = magicFormula(alphaF,Fzf,m_vehicleModelParameters(5));
	float Fyr = magicFormula(alphaR,Fzr,m_vehicleModelParameters(5));

	Eigen::MatrixXf xdot = Eigen::MatrixXf::Zero(6,1);

	xdot << x(2),
			x(3),
			m_groundAcceleration-Fyf*std::sin(m_delta)/m_vehicleModelParameters(0) + x(4)*x(3),
			(Fyf*std::cos(m_delta)+Fyr)/m_vehicleModelParameters(0) - x(4)*x(3),
			(m_vehicleModelParameters(3)*Fyf*std::cos(m_delta)-m_vehicleModelParameters(4)*Fyr)/m_vehicleModelParameters(1),
			x(4);

	Eigen::MatrixXf fx = x + xdot*m_sampleTime;

	return fx;
}

float BicycleModel::magicFormula(float &alpha, float &Fz, float const &mu)
{

	float const C = 1.0f;
	float const c_alpha = 25229.0f;
	float const B = c_alpha/C/mu/Fz;
	float const E = -2.0f;
	float Fy = mu*Fz*std::sin(C*std::atan(B*alpha - E*(B*alpha - std::atan(B*alpha))));

	return Fy;
}

}
}
}
