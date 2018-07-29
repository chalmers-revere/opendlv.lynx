/**
 * Copyright (C) 2016 Christian Berger
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

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <odcantools/SocketCANDevice.h>

#include <odcantools/CANDevice.h>
#include <odcantools/CANMessage.h>

#include <odcantools/GenericCANMessageListener.h>

#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/reflection/Message.h>
#include <opendavinci/odcore/reflection/MessageFromVisitableVisitor.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odtools/recorder/Recorder.h>

#include <cfsd18resgw/GeneratedHeaders_cfsd18resgw.h>
#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>
//#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>  


#include "can-res.hpp"

namespace opendlv {
namespace proxy {
namespace lynx {

using namespace std;
using namespace odcore::base;
using namespace odcore::data;
using namespace odcore::reflection;
using namespace odtools::recorder;
using namespace automotive::odcantools;

CanRes::Requests::Requests()
    : m_mutex()
    , m_enableActuationBrake(false)
    , m_enableActuationSteering(false)
    , m_enableActuationThrottle(false)
    , m_acceleration(0.0f)
    , m_steering(0.0f)
    , m_groundspeed(0.0f)
    , m_pressureEbsAct()
    , m_pressureEbsLine()
    , m_pressureEbsServ()
    , m_pressureEbsReg()
    , m_positionRack()
    , m_positionAct()
    , m_asRtd()
    , m_asState()
    , m_ebsFault()
    , m_asTorqueSetPointRight()
    , m_asTorqueSetPointLeft()
    , m_asPrService()
    , m_asPrRegulator()
    , m_asPrEbsLine()
    , m_asPrEbsAct()
    , m_asDoEbsHeartbeat()
    //
    , m_steeringState()
    , m_serviceBrakeState()
    , m_lapCounter(0)
    , m_ebsState()
    , m_conesCountAll()
    , m_conesCountActual()
    , m_assiState()
    , m_amiState()
    //
    , m_yawRate()
    , m_accLong()
    , m_accLat()
    //
    , m_steeringAngleTarget()
    , m_steeringAngleActual()
    , m_speedTarget()
    , m_speedActual()
    , m_motorMomentTarget()
    , m_motorMomentActual()
    , m_brakeHydrTarget()
    , m_brakeHydrActual()
    , m_lastUpdate()
{}

CanRes::Requests::~Requests() {}

CanRes::CanRes(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, "proxy-lynx-can-res")
    , GenericCANMessageListener()
    , m_requests()
    , m_device()
    , m_cfsdCanMessageMapping() 
    {}

CanRes::~CanRes() {}

void CanRes::setUp() {
    const string DEVICE_NODE = getKeyValueConfiguration().getValue< string >("proxy-lynx-can-res.devicenode");

    // Try to open CAN device and register this instance as receiver for GenericCANMessages.
    m_device = shared_ptr< CANDevice >(new SocketCANDevice(DEVICE_NODE, *this));

    // If the device could be successfully opened, create a recording file to dump of the data.
    if (m_device.get() && m_device->isOpen()) {
        cout << "[" << getName() << "]: " << "Successfully opened CAN device '" << DEVICE_NODE << "'." << endl;

        // Start the wrapped CAN device to receive CAN messages concurrently.
        m_device->start();
    } else {
        cerr << "[" << getName() << "]: " << "Failed to open CAN device '" << DEVICE_NODE << "'." << endl;
    }
}

void CanRes::tearDown() {
    // Stop the wrapper CAN device.
    if (m_device.get()) {
        m_device->stop();
    }
}

void CanRes::nextContainer(Container &a_container) {

  //Get groundspeed request from path planning
    if(a_container.getDataType() == opendlv::proxy::GroundSpeedReading::ID()){

        auto gsReading = a_container.getData<opendlv::proxy::GroundSpeedReading>();
        if(a_container.getSenderStamp() == 112){
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp();
            m_requests.m_speedTarget = static_cast<uint8_t>(gsReading.getGroundSpeed()*3.6f);
        }

    }    
    if(a_container.getDataType() == opendlv::proxy::GroundSpeedRequest::ID()){

        auto gsRequest = a_container.getData<opendlv::proxy::GroundSpeedRequest>();
        if(a_container.getSenderStamp() == 112){
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp();
            m_requests.m_speedActual = static_cast<uint8_t>(gsRequest.getGroundSpeed()*3.6f);
        }

    }
    if(a_container.getDataType() == opendlv::proxy::AccelerationReading::ID()){

        auto accReading = a_container.getData<opendlv::proxy::AccelerationReading>();
        if(a_container.getSenderStamp() == 112){
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp();
            m_requests.m_accLong = accReading.getAccelerationX();
            m_requests.m_accLat = accReading.getAccelerationY();
        }

    }
    if(a_container.getDataType() == opendlv::proxy::AngularVelocityReading::ID()){

        auto yawReading = a_container.getData<opendlv::proxy::AngularVelocityReading>();
        if(a_container.getSenderStamp() == 112){
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp();
            m_requests.m_yawRate = yawReading.getAngularVelocityZ()*180/3.14f;
        }

    }

    if (a_container.getDataType() == opendlv::proxy::GroundSteeringReading::ID()) {
        auto groundsteerReading = a_container.getData<opendlv::proxy::GroundSteeringReading>();
        if(a_container.getSenderStamp() == 1206){ // Steering rack reading
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_steeringAngleTarget = groundsteerReading.getGroundSteering();
        }
    }
    if(a_container.getDataType() == opendlv::proxy::GroundSteeringRequest::ID()){

        auto gsRequest = a_container.getData<opendlv::proxy::GroundSteeringRequest>();
        odcore::base::Lock l(m_requests.m_mutex);
        m_requests.m_lastUpdate = odcore::data::TimeStamp();
        m_requests.m_steeringAngleActual = gsRequest.getGroundSteering();

    }

    if (a_container.getDataType() == opendlv::proxy::PressureReading::ID()) {
        auto pressureReading = a_container.getData<opendlv::proxy::PressureReading>();
        if (a_container.getSenderStamp() == 1201){ // EBS Line
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_asPrEbsLine = (uint8_t) (pressureReading.getPressure()*20);
        }else if (a_container.getSenderStamp() == 1202){ // Service tank
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_asPrService = (uint8_t) (pressureReading.getPressure()*20);
        }else if (a_container.getSenderStamp() == 1203){ // EBS Act
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_asPrEbsAct = (uint8_t) (pressureReading.getPressure()*20);
        }else if (a_container.getSenderStamp() == 1205){ // Service regulator
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_asPrRegulator = (uint8_t) (pressureReading.getPressure()*20);
        }
    }

    if (a_container.getDataType() == opendlv::proxy::SwitchStateReading::ID()) {
        auto stateReading = a_container.getData<opendlv::proxy::SwitchStateReading>();
        if (a_container.getSenderStamp() == 1401){ // Current state
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_assiState = (uint8_t) stateReading.getState();
        }else if (a_container.getSenderStamp() == 1406){ // Mission
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_amiState = (uint8_t) stateReading.getState();
        }else if (a_container.getSenderStamp() == 1405){ // EBS Fault
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_ebsFault = (uint8_t) stateReading.getState();
        }else if(a_container.getSenderStamp() == 1406){
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp();
            m_requests.m_amiState = stateReading.getState();
        }else if(a_container.getSenderStamp() == 1411){
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp();
            m_requests.m_conesSeen += stateReading.getState();
        }else if(a_container.getSenderStamp() == 1412){
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp();
            m_requests.m_totalConesInMap = stateReading.getState();
        }else if(a_container.getSenderStamp() == 666){
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp();
            m_requests.m_lapCounter = static_cast<uint8_t>(stateReading.getState());
        }
    }

    if (a_container.getDataType() == opendlv::proxy::TorqueRequest::ID()) {
        auto torqueReq = a_container.getData<opendlv::proxy::TorqueRequest>();
        if (a_container.getSenderStamp() == 1501){ // Right motor
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_asTorqueSetPointRight = (int16_t) torqueReq.getTorque();
        }else if (a_container.getSenderStamp() == 1500){ // Left motor
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_asTorqueSetPointLeft = (int16_t) torqueReq.getTorque();
        }
    }
}

void CanRes::nextGenericCANMessage(const automotive::GenericCANMessage &gcm)
{
    // Log raw CAN data in ASC format.
    //dumpASCData(gcm);

    // Map CAN message to high-level data structures as defined in the ODVD file.
    vector< Container > result = m_cfsdCanMessageMapping.mapNext(gcm);

    for (auto c : result) {
        // Add CAN device driver time stamp to message.
        c.setSampleTimeStamp(gcm.getDriverTimeStamp());
        c.setSentTimeStamp(gcm.getDriverTimeStamp());
        c.setReceivedTimeStamp(gcm.getDriverTimeStamp());

        if(c.getDataType() == opendlv::proxy::PdoResStatus::ID()){
            auto ResStatus = c.getData<opendlv::proxy::PdoResStatus>();
            const uint8_t resStatus = ResStatus.getResStatus();
            const uint8_t resEStop = ResStatus.getResEStop();
            const uint8_t resQuality = ResStatus.getResQuality();
            const uint8_t resButtons = ResStatus.getResButtons();
        {
	        opendlv::proxy::SwitchStateReading switchStateReading;
            switchStateReading.setState(resStatus);
            Container switchStateReadingContainer = Container(switchStateReading);
            switchStateReadingContainer.setSenderStamp(1407);
            getConference().send(switchStateReadingContainer);
        }{
	        opendlv::proxy::SwitchStateReading switchStateReading;
            switchStateReading.setState(resEStop);
            Container switchStateReadingContainer = Container(switchStateReading);
            switchStateReadingContainer.setSenderStamp(1408);
            getConference().send(switchStateReadingContainer);
        }{
	        opendlv::proxy::SwitchStateReading switchStateReading;
            switchStateReading.setState(resQuality);
            Container switchStateReadingContainer = Container(switchStateReading);
            switchStateReadingContainer.setSenderStamp(1409);
            getConference().send(switchStateReadingContainer);
        }{
	        opendlv::proxy::SwitchStateReading switchStateReading;
            switchStateReading.setState(resButtons);
            Container switchStateReadingContainer = Container(switchStateReading);
            switchStateReadingContainer.setSenderStamp(1410);
            getConference().send(switchStateReadingContainer);
        }


          //std::cout << "SoC: " << static_cast<int>(Acc_SoC) << std::endl;
          //std::cout << "Brake_Pressure: " << static_cast<int>(Brake_Front) << " : " << static_cast<int>(Brake_Rear) << std::endl;
          //std::cout << "DL Status: " << static_cast<int>(DL_Status) << std::endl; 
          //std::cout << "AS Mission: " << static_cast<int>(AS_Mission) << std::endl; 

        }
    }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode CanRes::body() {
    while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

        // Write values to CAN.
        {
            odcore::base::Lock l(m_requests.m_mutex);
            odcore::data::TimeStamp now;

            {   // nmtNodeControl
                opendlv::proxy::NmtNodeControl nmtNodeControl;
                nmtNodeControl.setNodeState(1);
                nmtNodeControl.setNodeId(0);
    
                odcore::data::Container nmtNodeControlContainer(nmtNodeControl);
                canmapping::opendlv::proxy::NmtNodeControl nmtNodeControlMapping;
                automotive::GenericCANMessage genericCANmessageNmtNodeControl = nmtNodeControlMapping.encode(nmtNodeControlContainer);
                m_device->write(genericCANmessageNmtNodeControl);

            }

            {   // asEbsSupervision
                opendlv::proxy::AsEbsSupervision asEbsSupervision;
                asEbsSupervision.setAsPrService(m_requests.m_asPrService);
                asEbsSupervision.setAsPrRegulator(m_requests.m_asPrRegulator);
                asEbsSupervision.setAsPrEbsLine(m_requests.m_asPrEbsLine);
                asEbsSupervision.setAsPrEbsAct(m_requests.m_asPrEbsAct);
                asEbsSupervision.setAsDoEbsHeartbeat(m_requests.m_asDoEbsHeartbeat);
    
                odcore::data::Container asEbsSupervisionContainer(asEbsSupervision);
                canmapping::opendlv::proxy::AsEbsSupervision asEbsSupervisionMapping;
                automotive::GenericCANMessage genericCANmessageAsEbsSupervision = asEbsSupervisionMapping.encode(asEbsSupervisionContainer);
                m_device->write(genericCANmessageAsEbsSupervision);

            }

            {   // dvSystemStatus
                opendlv::proxy::DvSystemStatus dvSystemStatus;
                dvSystemStatus.setSteeringState(m_requests.m_steeringState);
                dvSystemStatus.setServiceBrakeState(m_requests.m_serviceBrakeState);
                dvSystemStatus.setLapCounter(m_requests.m_lapCounter);
                dvSystemStatus.setEbsState(m_requests.m_ebsState);
                dvSystemStatus.setConesCountAll(m_requests.m_conesSeen);
                dvSystemStatus.setConesCountActual(m_requests.m_totalConesInMap);
                dvSystemStatus.setAssiState(m_requests.m_assiState);
                dvSystemStatus.setAmiState(m_requests.m_amiState);
    
                odcore::data::Container dvSystemStatusContainer(dvSystemStatus);
                canmapping::opendlv::proxy::DvSystemStatus dvSystemStatusMapping;
                automotive::GenericCANMessage genericCANmessageDvSystemStatus = dvSystemStatusMapping.encode(dvSystemStatusContainer);
                m_device->write(genericCANmessageDvSystemStatus);
            }

            {   // dvDrivingDynamics2
                opendlv::proxy::DvDrivingDynamics2 dvDrivingDynamics2;
                dvDrivingDynamics2.setYawRate(m_requests.m_yawRate);
                dvDrivingDynamics2.setAccLong(m_requests.m_accLong);
                dvDrivingDynamics2.setAccLat(m_requests.m_accLat);
    
                odcore::data::Container dvDrivingDynamics2Container(dvDrivingDynamics2);
                canmapping::opendlv::proxy::DvDrivingDynamics2 dvDrivingDynamics2Mapping;
                automotive::GenericCANMessage genericCANmessageDvDrivingDynamics2 = dvDrivingDynamics2Mapping.encode(dvDrivingDynamics2Container);
                m_device->write(genericCANmessageDvDrivingDynamics2);
            }

            {   // dvDrivingDynamics1
                opendlv::proxy::DvDrivingDynamics1 dvDrivingDynamics1;
                dvDrivingDynamics1.setSteeringAngleTarget(m_requests.m_steeringAngleTarget);
                dvDrivingDynamics1.setSteeringAngleActual(m_requests.m_steeringAngleActual);
                dvDrivingDynamics1.setSpeedTarget(m_requests.m_speedTarget);
                dvDrivingDynamics1.setSpeedActual(m_requests.m_speedActual);
                dvDrivingDynamics1.setMotorMomentTarget(m_requests.m_motorMomentTarget);
                dvDrivingDynamics1.setMotorMomentActual(m_requests.m_motorMomentActual);
                dvDrivingDynamics1.setBrakeHydrTarget(m_requests.m_brakeHydrTarget);
                dvDrivingDynamics1.setBrakeHydrActual(m_requests.m_brakeHydrActual);
    
                odcore::data::Container dvDrivingDynamics1Container(dvDrivingDynamics1);
                canmapping::opendlv::proxy::DvDrivingDynamics1 dvDrivingDynamics1Mapping;
                automotive::GenericCANMessage genericCANmessageDvDrivingDynamics1 = dvDrivingDynamics1Mapping.encode(dvDrivingDynamics1Container);
                m_device->write(genericCANmessageDvDrivingDynamics1);
            }
             
        }
    }

    return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

}
}
}
