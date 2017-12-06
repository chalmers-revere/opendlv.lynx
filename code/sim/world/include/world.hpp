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

#ifndef SIM_WORLD_WORLD_HPP
#define SIM_WORLD_WORLD_HPP

#include <map>

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/base/Mutex.h>
#include <opendavinci/odcore/data/Container.h>

#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>

namespace opendlv {
namespace sim {

class World : public odcore::base::module::TimeTriggeredConferenceClientModule {
 public:
  World(int32_t const &, char **);
  World(World const &) = delete;
  World &operator=(World const &) = delete;
  virtual ~World();
  virtual void nextContainer(odcore::data::Container &);

 private:
  odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
  void setUp();
  void tearDown();

  std::map<uint32_t, opendlv::coord::Frame> m_clientRootFrames;
  std::map<uint32_t, opendlv::coord::KinematicState> m_clientKinematicStates;
  odcore::base::Mutex m_kinematicsMutex;
};

}
}

#endif
