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

#include <cfsd18gw/GeneratedHeaders_cfsd18gw.h>
#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>
//#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>  


#include "can.hpp"

namespace opendlv {
namespace proxy {
namespace lynx {

using namespace std;
using namespace odcore::base;
using namespace odcore::data;
using namespace odcore::reflection;
using namespace odtools::recorder;
using namespace automotive::odcantools;

Can::Requests::Requests()
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
    , m_lastUpdate()
{}

Can::Requests::~Requests() {}

Can::Can(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, "proxy-lynx-can")
    , GenericCANMessageListener()
    , m_requests()
    //, m_fifoGenericCanMessages()
    //, m_recorderGenericCanMessages()
    //, m_fifoMappedCanMessages()
    //, m_recorderMappedCanMessages()
    , m_device()
    , m_cfsdCanMessageMapping()
    //, m_startOfRecording()
    //, m_ASCfile()
    //, m_mapOfCSVFiles()
    //, m_mapOfCSVVisitors() 
    {}

Can::~Can() {}

void Can::setUp() {
    const string DEVICE_NODE = getKeyValueConfiguration().getValue< string >("proxy-lynx-can.devicenode");

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

void Can::tearDown() {
    // Stop the wrapper CAN device.
    if (m_device.get()) {
        m_device->stop();
    }
}

void Can::nextContainer(Container &a_container) {

  //Get groundspeed request from path planning

    if (a_container.getDataType() == opendlv::proxy::GroundSteeringReading::ID()) {
        auto groundspeedReading = a_container.getData<opendlv::proxy::GroundSteeringReading>();
        if (a_container.getSenderStamp() == 1200){ // Steering actuator reading
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_positionAct = (uint8_t) ((groundspeedReading.getGroundSteering()+25)*5);
        }else if (a_container.getSenderStamp() == 1206){ // Steering rack reading
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_positionRack = (uint8_t) ((groundspeedReading.getGroundSteering()+25)*5);
        }
    }

    if (a_container.getDataType() == opendlv::proxy::PressureReading::ID()) {
        auto pressureReading = a_container.getData<opendlv::proxy::PressureReading>();
        if (a_container.getSenderStamp() == 1201){ // EBS Line
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_pressureEbsLine = (uint8_t) (pressureReading.getPressure()*20);
        }else if (a_container.getSenderStamp() == 1202){ // Service tank
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_pressureEbsServ = (uint8_t) (pressureReading.getPressure()*20);
        }else if (a_container.getSenderStamp() == 1203){ // EBS Act
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_pressureEbsAct = (uint8_t) (pressureReading.getPressure()*20);
        }else if (a_container.getSenderStamp() == 1205){ // Service regulator
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_pressureEbsReg = (uint8_t) (pressureReading.getPressure()*20);
        }
    }

    if (a_container.getDataType() == opendlv::proxy::SwitchStateReading::ID()) {
        auto stateReading = a_container.getData<opendlv::proxy::SwitchStateReading>();
        if (a_container.getSenderStamp() == 1401){ // Current state
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_asState = (uint8_t) stateReading.getState();
        }else if (a_container.getSenderStamp() == 1404){ // RTD
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_asRtd = (uint8_t) stateReading.getState();
        }else if (a_container.getSenderStamp() == 1405){ // EBS Fault
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_ebsFault = (uint8_t) stateReading.getState();
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

void Can::nextGenericCANMessage(const automotive::GenericCANMessage &gcm)
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

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Can::body() {
    while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

        // Write values to CAN.
        {
            odcore::base::Lock l(m_requests.m_mutex);
            odcore::data::TimeStamp now;

            {   //Set and send sensors readings to car
                opendlv::proxy::NmtNodeControl nmtNodeControl;
                nmtNodeControl.setNodeState(1);
                nmtNodeControl.setNodeId(0);
    
                odcore::data::Container nmtNodeControlContainer(nmtNodeControl);
                canmapping::opendlv::proxy::NmtNodeControl nmtNodeControlMapping;
                automotive::GenericCANMessage genericCANmessageNmtNodeControl = nmtNodeControlMapping.encode(nmtNodeControlContainer);
                m_device->write(genericCANmessageNmtNodeControl);

            }
             
        }
    }

    return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

}
}
}
