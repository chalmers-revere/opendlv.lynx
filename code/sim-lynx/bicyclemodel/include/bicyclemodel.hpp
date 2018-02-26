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

#ifndef OPENDLV_SIM_LYNX_BICYCLEMODEL_HPP
#define OPENDLV_SIM_LYNX_BICYCLEMODEL_HPP

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>

#include <odvdcfsd18/GeneratedHeaders_ODVDcfsd18.h>

namespace opendlv {
namespace sim {
namespace lynx {

class BicycleModel : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  BicycleModel(int32_t const &, char **);
  BicycleModel(Bicyclemodel const &) = delete;
  BicycleModel &operator=(BicycleModel const &) = delete;
  virtual ~BicycleModel();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();
};

}
}
}

#endif
