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

#ifndef OPENDLV_PROXY_LYNX_BRAKE_HPP
#define OPENDLV_PROXY_LYNX_BRAKE_HPP

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>

#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>

namespace opendlv {
namespace proxy {
namespace lynx {

class Brake : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  Brake(int32_t const &, char **);
  Brake(Brake const &) = delete;
  Brake &operator=(Brake const &) = delete;
  virtual ~Brake();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();
};

}
}
}

#endif
