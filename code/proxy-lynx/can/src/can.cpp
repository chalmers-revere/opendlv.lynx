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

        // Automatically record all received raw CAN messages.
       /* m_startOfRecording = TimeStamp();
        const string TIMESTAMP = m_startOfRecording.getYYYYMMDD_HHMMSS_noBlankNoColons();

        const bool RECORD_GCM = (getKeyValueConfiguration().getValue< int >("proxy-lynx-can.record_gcm") == 1);
        if (RECORD_GCM) {
            setUpRecordingGenericCANMessage(TIMESTAMP);
        }

        const bool RECORD_MAPPED = (getKeyValueConfiguration().getValue< int >("proxy-lynx-can.record_mapped_data") == 1);
        if (RECORD_MAPPED) {
            setUpRecordingMappedGenericCANMessage(TIMESTAMP);
        }

        bool valueFound = false;
        m_requests.m_enableActuationBrake = getKeyValueConfiguration().getOptionalValue<bool>("proxy-lynx-can.enableActuationBrake", valueFound);
        if (!valueFound) {
          m_requests.m_enableActuationBrake = false;
        }
        if (!m_requests.m_enableActuationBrake) {
          std::cout << "The brakes are not enabled for control." << std::endl;
        }

        m_requests.m_enableActuationSteering = getKeyValueConfiguration().getOptionalValue<bool>("proxy-lynx-can.enableActuationSteering", valueFound);
        if (!valueFound) {
          m_requests.m_enableActuationSteering = false;
        }
        if (!m_requests.m_enableActuationSteering) {
          std::cout << "The steering is not enabled for control." << std::endl;
        }

        m_requests.m_enableActuationThrottle = getKeyValueConfiguration().getOptionalValue<bool>("proxy-lynx-can.enableActuationThrottle", valueFound);
        if (!valueFound) {
          m_requests.m_enableActuationThrottle = false;
        }
        if (!m_requests.m_enableActuationThrottle) {
          std::cout << "The throttle is not enabled for control." << std::endl;
        }*/

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

    // Flush output to CSV files.
    /*for (auto it = m_mapOfCSVFiles.begin(); it != m_mapOfCSVFiles.end(); it++) {
        it->second->flush();
        it->second->close();
    }

    // Flush output to ASC file.
    if (m_ASCfile.get() != NULL) {
        m_ASCfile->flush();
        m_ASCfile->close();
    }*/
}

/*void Can::setUpRecordingMappedGenericCANMessage(const string &timeStampForFileName) {
    // URL for storing containers containing GenericCANMessages.
    stringstream recordingUrl;
    recordingUrl << "file://"
                 << "CID-" << getCID() << "_"
                 << "can_mapped_data_" << timeStampForFileName << ".rec";

    // Size of memory segments (not needed for recording GenericCANMessages).
    const uint32_t MEMORY_SEGMENT_SIZE = 0;

    // Number of memory segments (not needed for recording GenericCANMessages).
    const uint32_t NUMBER_OF_SEGMENTS = 0;

    // Run recorder in asynchronous mode to allow real-time recording in background.
    const bool THREADING = true;

    // Dump shared images and shared data (not needed for recording mapped containers)?
    const bool DUMP_SHARED_DATA = false;

    // Create a recorder instance.
    m_recorderMappedCanMessages = unique_ptr< Recorder >(new Recorder(recordingUrl.str(), MEMORY_SEGMENT_SIZE, NUMBER_OF_SEGMENTS, THREADING, DUMP_SHARED_DATA));

    {

        {
            stringstream fileName;
            fileName << "CID-" << getCID() << "_"
                     << "can_mapped_data_id-" << opendlv::proxy::GroundSteeringReading::ID() << "_" << timeStampForFileName << ".csv";

            // Create map of CSV transformers.
            fstream *f = new fstream(fileName.str(), ios::out);

            // Log Steering.
            m_mapOfCSVFiles[opendlv::proxy::GroundSteeringReading::ID()] = shared_ptr< fstream >(f);
            m_mapOfCSVVisitors[opendlv::proxy::GroundSteeringReading::ID()] = shared_ptr< CSVFromVisitableVisitor >(new CSVFromVisitableVisitor(*f));
        }

        {
            stringstream fileName;
            fileName << "CID-" << getCID() << "_"
                     << "can_mapped_data_id-" << opendlv::proxy::GroundSpeedReading::ID() << "_" << timeStampForFileName << ".csv";

            // Create map of CSV transformers.
            fstream *f = new fstream(fileName.str(), ios::out);

            // Log Propulsion.
            m_mapOfCSVFiles[opendlv::proxy::GroundSpeedReading::ID()] = shared_ptr< fstream >(f);
            m_mapOfCSVVisitors[opendlv::proxy::GroundSpeedReading::ID()] = shared_ptr< CSVFromVisitableVisitor >(new CSVFromVisitableVisitor(*f));
        }
    }
}

void Can::setUpRecordingGenericCANMessage(const string &timeStampForFileName) {
    // URL for storing containers containing GenericCANMessages.
    stringstream recordingUrl;
    recordingUrl << "file://"
                 << "CID-" << getCID() << "_"
                 << "can_gcm_" << timeStampForFileName << ".rec";

    // Size of memory segments (not needed for recording GenericCANMessages).
    const uint32_t MEMORY_SEGMENT_SIZE = 0;

    // Number of memory segments (not needed for recording GenericCANMessages).
    const uint32_t NUMBER_OF_SEGMENTS = 0;

    // Run recorder in asynchronous mode to allow real-time recording in background.
    const bool THREADING = true;

    // Dump shared images and shared data (not needed for recording GenericCANMessages)?
    const bool DUMP_SHARED_DATA = false;

    // Create a recorder instance.
    m_recorderGenericCanMessages = unique_ptr< Recorder >(new Recorder(recordingUrl.str(),
    MEMORY_SEGMENT_SIZE, NUMBER_OF_SEGMENTS, THREADING, DUMP_SHARED_DATA));

    // Create a file to dump CAN data in ASC format.
    stringstream fileName;
    fileName << "CID-" << getCID() << "_"
             << "can_data_" << timeStampForFileName << ".asc";
    m_ASCfile = shared_ptr< fstream >(new fstream(fileName.str(), ios::out));
    (*m_ASCfile) << "Time (s) Channel ID RX/TX d Length Byte 1 Byte 2 Byte 3 Byte 4 Byte 5 Byte 6 Byte 7 Byte 8" << endl;
}*/

void Can::nextContainer(Container &a_container) {

  //praktiskt exempel

    /*if (a_container.getDataType() == opendlv::proxy::GroundspeedReadníng::ID()) {
        auto groundspeedReading = a_container.getData<opendlv::proxy::GroundspeedReadníng>();
        if (actuationRequest.getIsValid()) {
            odcore::base::Lock l(m_requests.m_mutex);
            m_requests.m_lastUpdate = odcore::data::TimeStamp(); // Set time point of last update for these values to now.
            m_requests.m_groundspeed = groundSpeedReading.getGroundspeed();
            
            }
        }
    }*/
  a_container = a_container;

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

        // Enqueue mapped container for direct recording.
        /*if (m_recorderMappedCanMessages.get()) {
            m_fifoMappedCanMessages.add(c);
        }*/

        getConference().send(c);

        // Generate GroundSpeedReading message
        /*if (c.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) {
          auto propulsion = c.getData<opendlv::proxy::GroundSpeedReading>();
          const double groundSpeedKph = static_cast<double>(propulsion.getPropulsionShaftVehicleSpeed());
          const double groundSpeed = groundSpeedKph / 3.6;

          opendlv::proxy::GroundSpeedReading groundSpeedReading;
          groundSpeedReading.setGroundSpeed(groundSpeed);

          Container groundSpeedReadingContainer = Container(groundSpeedReading);
          getConference().send(groundSpeedReadingContainer);
        }*/

      if(c.getDataType() == opendlv::coord::ConeShape::ID()){
        auto knobbers = c.getData<opendlv::coord::ConeShape>();
        const uint32_t knob1 = knobbers.getRadius();
        const uint32_t knob2 = knobbers.getHeight();

        std::cout << "knobber 1: " << knob1 << " knobber 2: " << knob2 << std::endl;

      }
    }

    // Enqueue CAN message wrapped as Container to be recorded if we have a valid recorder.
    /*if (m_recorderGenericCanMessages.get()) {
        Container c(gcm);
        c.setSampleTimeStamp(gcm.getDriverTimeStamp());
        c.setSentTimeStamp(gcm.getDriverTimeStamp());
        c.setReceivedTimeStamp(gcm.getDriverTimeStamp());
        m_fifoGenericCanMessages.add(c);
    }*/
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Can::body() {
    while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
        // Record GenericCANMessages.
        /*if (m_recorderGenericCanMessages.get()) {
            const uint32_t ENTRIES = m_fifoGenericCanMessages.getSize();
            for (uint32_t i = 0; i < ENTRIES; i++) {
                Container c = m_fifoGenericCanMessages.leave();

                // Store container to dump file.
                m_recorderGenericCanMessages->store(c);
            }
        }

        // Record mapped messages from GenericCANMessages.
        if (m_recorderMappedCanMessages.get()) {
            const uint32_t ENTRIES = m_fifoMappedCanMessages.getSize();
            for (uint32_t i = 0; i < ENTRIES; i++) {
                Container c = m_fifoMappedCanMessages.leave();

                // Store container to dump file.
                m_recorderMappedCanMessages->store(c);

                // Transform container to CSV file.
                dumpCSVData(c);
            }
        }*/

        // Write values to CAN.
        {
            odcore::base::Lock l(m_requests.m_mutex);
            odcore::data::TimeStamp now;

            //

            //KNOB MESSAGE TEMPORARILY CONESHAPE FOR TESTING

            opendlv::system::SignalStatusMessage signalStatus;
            signalStatus.setCode(2); //or something..

            odcore::data::Container signalContainer(signalStatus);
            canmapping::opendlv::system::SignalStatusMessage signalMapping;
            automotive::GenericCANMessage genericCanMessage = signalMapping.encode(signalContainer);
            m_device->write(genericCanMessage);
            /*

            //Update values depending on timestamps
            const int64_t ONE_SECOND = 1 * 1000 * 1000;
            const bool TIMEOUT = !(abs((now - m_requests.m_lastUpdate).toMicroseconds()) < ONE_SECOND);
            if (!TIMEOUT) {
                // Updates for actuation values received, prepare values to be sent.
                brakeRequestValue = (m_requests.m_acceleration < 0.0f) ? m_requests.m_acceleration : 0.0f;
                throttleRequestValue = (!(m_requests.m_acceleration < 0.0f)) ? m_requests.m_acceleration : 0.0f;
                steeringRequestValue = m_requests.m_steering;
            }*/



            /*

            //exempler forts
              
             {
                opendlv::proxy::GroundSpeedRequest groundspeedRequest;
                groundSpeedRequest.setGroundspeed(m_request.m_groundspeed);
                odcore::data::Containter groundspeedContainer(groundspeedRequest);
                canmapping::opendlv::proxy::GroundSpeedRequest groundspeedMapping;
                automotive::GenericCANMessage genericCANmessageGroundspeed = groundspeedMapping.encode(groundspeedContainer);
                m_device->write(genericCANmessageGroundspeed);


             } */
        }
    }

    return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

/*void Can::dumpCSVData(Container &c) {
    // Add time stamps for CSV output.
    const uint64_t receivedFromCAN = c.getReceivedTimeStamp().toMicroseconds();

    shared_ptr< Field< uint64_t > > m_receivedTS_ptr = shared_ptr< Field< uint64_t > >(new Field< uint64_t >(receivedFromCAN));
    m_receivedTS_ptr->setFieldIdentifier(0);
    m_receivedTS_ptr->setLongFieldName("Received_TimeStamp");
    m_receivedTS_ptr->setShortFieldName("Received_TimeStamp");
    m_receivedTS_ptr->setFieldDataType(reflection::AbstractField::UINT64_T);
    m_receivedTS_ptr->setSize(sizeof(uint64_t));

    if ((m_mapOfCSVFiles.count(c.getDataType()) == 1) &&
        (m_mapOfCSVVisitors.count(c.getDataType()) == 1)) {
        // We have a CSV file and a transformation available.
        if (c.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) {
            opendlv::proxy::GroundSpeedReading temp = c.getData< opendlv::proxy::GroundSpeedReading >();
            MessageFromVisitableVisitor mfvv;
            temp.accept(mfvv);
            Message m = mfvv.getMessage();
            m.addField(m_receivedTS_ptr);
            m.accept(*m_mapOfCSVVisitors[c.getDataType()]);
        }
        if (c.getDataType() == opendlv::proxy::GroundSteeringReading::ID()) {
            opendlv::proxy::rhino::Propulsion temp = c.getData< opendlv::proxy::rhino::Propulsion >();
            MessageFromVisitableVisitor mfvv;
            temp.accept(mfvv);
            Message m = mfvv.getMessage();
            m.addField(m_receivedTS_ptr);
            m.accept(*m_mapOfCSVVisitors[c.getDataType()]);
        }
    }
}

void Can::dumpASCData(const automotive::GenericCANMessage &gcm) {
    if (m_ASCfile.get() != NULL) {
        TimeStamp now;
        TimeStamp ts = (now - m_startOfRecording);
        (*m_ASCfile) << (ts.getSeconds() + (static_cast< double >(ts.getFractionalMicroseconds()) / (1000.0 * 1000.0)))
                     << " 1"
                     << " " << gcm.getIdentifier()
                     << " Rx"
                     << " d"
                     << " " << static_cast< uint32_t >(gcm.getLength());
        uint64_t data = gcm.getData();
        for (uint8_t i = 0; i < gcm.getLength(); i++) {
            (*m_ASCfile) << " " << hex << (data & 0xFF);
            data = data >> 8;
        }
        (*m_ASCfile) << endl;
    }
}*/

}
}
}