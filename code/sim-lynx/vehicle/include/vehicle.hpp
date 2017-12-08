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

#ifndef SIM_LYNX_VEHICLE_HPP
#define SIM_LYNX_VEHICLE_HPP

#include <memory>
#include <string>

#include <opendavinci/odcore/base/Mutex.h>
#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>

#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>

namespace opendlv {
namespace sim {
namespace lynx {

class Vehicle : public odcore::base::module::TimeTriggeredConferenceClientModule {
   public:
    Vehicle(int32_t const &, char **);
    Vehicle(Vehicle const &) = delete;
    Vehicle &operator=(Vehicle const &) = delete;
    virtual ~Vehicle();

   private:
    void nextContainer(odcore::data::Container &);
    virtual void setUp();
    virtual void tearDown();
    virtual odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

    odcore::base::Mutex m_requestMutex;
    float m_accelerationRequest;
    float m_decelerationRequest;
    float m_steeringRequest;
    float m_vehicleLength;
    float m_momentOfInertia;
    float m_cf;
    float m_cr;	
    float m_a;
    float m_b;
    float m_lateralVelocity;
    float m_yaw;
    float m_longitudinalVelocity;
    float m_vehicleMass;
};

}
}
}

#endif
