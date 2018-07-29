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

#ifndef OPENDLV_PROXY_LYNX_CAN_RES_H
#define OPENDLV_PROXY_LYNX_CAN_RES_H

#include <fstream>
#include <map>
#include <memory>
#include <string>


#include <odcantools/SocketCANDevice.h>

#include <odcantools/CANDevice.h>
#include <odcantools/CANMessage.h>

#include <odcantools/GenericCANMessageListener.h>

#include <cfsd18resgw/GeneratedHeaders_cfsd18resgw.h>
#include <opendavinci/odcore/base/FIFOQueue.h>
#include <opendavinci/odcore/base/Mutex.h>
#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/reflection/CSVFromVisitableVisitor.h>

namespace automotive {
class GenericCANMessage;
}
namespace automotive {
namespace odcantools {
class CANDevice;
}
}
namespace odtools {
namespace recorder {
class Recorder;
}
}

namespace opendlv {
namespace proxy {
namespace lynx {

using namespace std;
/**
 * Interface to FH16 truck.
 */
class CanRes : public odcore::base::module::TimeTriggeredConferenceClientModule,
                  public automotive::odcantools::GenericCANMessageListener {
   private:
    class Requests {
        public:
            Requests();
            Requests(const Requests&) = delete;
            Requests& operator=(const Requests&) = delete;
            virtual ~Requests();

            odcore::base::Mutex m_mutex;
            bool m_enableActuationBrake;
            bool m_enableActuationSteering;
            bool m_enableActuationThrottle;
            float m_acceleration;
            float m_steering;
            float m_groundspeed;
            uint8_t m_pressureEbsAct;
            uint8_t m_pressureEbsLine;
            uint8_t m_pressureEbsServ;
            uint8_t m_pressureEbsReg;
            uint8_t m_positionRack;
            uint8_t m_positionAct;
            uint8_t m_asRtd;
            uint8_t m_asState;
            uint8_t m_ebsFault;
            int16_t m_asTorqueSetPointRight;
            int16_t m_asTorqueSetPointLeft;
            //
            uint8_t m_asPrService;
            uint8_t m_asPrRegulator;
            uint8_t m_asPrEbsLine;
            uint8_t m_asPrEbsAct;
            uint8_t m_asDoEbsHeartbeat;
            //
            bool m_steeringState;
            uint8_t m_serviceBrakeState;
            uint8_t m_lapCounter;
            uint8_t m_ebsState;
            uint32_t m_conesCountAll;
            uint8_t m_conesCountActual;
            uint8_t m_assiState;
            uint8_t m_amiState;
            //
            float m_yawRate;
            float m_accLong;
            float m_accLat;
            //
            float m_steeringAngleTarget;
            float m_steeringAngleActual;
            uint8_t m_speedTarget;
            uint8_t m_speedActual;
            int8_t m_motorMomentTarget;
            int8_t m_motorMomentActual;
            uint8_t m_brakeHydrTarget;
            uint8_t m_brakeHydrActual;
            uint16_t m_totalConesInMap = 0;
            uint16_t m_conesSeen = 0;
            //
            odcore::data::TimeStamp m_lastUpdate;
    };

   public:
    CanRes(int32_t const &, char **);
    CanRes(CanRes const &) = delete;
    CanRes &operator=(CanRes const &) = delete;
    virtual ~CanRes();

    virtual void nextGenericCANMessage(const automotive::GenericCANMessage &gcm);
    virtual void nextContainer(odcore::data::Container &c);

   private:
    virtual void setUp();
    virtual void tearDown();
    virtual odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

   private:
    //void setUpRecordingGenericCANMessage(const std::string &timeStampForFileName);
    //void setUpRecordingMappedGenericCANMessage(const std::string &timeStampForFileName);

   private:
    //void dumpASCData(const automotive::GenericCANMessage &gcm);
    //void dumpCSVData(odcore::data::Container &c);

   private:
    Requests m_requests;

    //odcore::base::FIFOQueue m_fifoGenericCanMessages;
    //std::unique_ptr< odtools::recorder::Recorder > m_recorderGenericCanMessages;

    //odcore::base::FIFOQueue m_fifoMappedCanMessages;
    //std::unique_ptr< odtools::recorder::Recorder > m_recorderMappedCanMessages;

    std::shared_ptr< automotive::odcantools::CANDevice > m_device;

    canmapping::CanMapping m_cfsdCanMessageMapping;

    //odcore::data::TimeStamp m_startOfRecording;
    //std::shared_ptr< std::fstream > m_ASCfile;
    //std::map< uint32_t, std::shared_ptr< std::fstream > > m_mapOfCSVFiles;
    //std::map< uint32_t, std::shared_ptr< odcore::reflection::CSVFromVisitableVisitor > > m_mapOfCSVVisitors;
};

}
}
}

#endif
