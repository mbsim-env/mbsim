/* Copyright (C) 2004-2009 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "mbsimControl/signal_.h"
#include "mbsimControl/extern_signal_source.h"
#include "mbsim/utils/utils.h"
#include "mbsim/dynamic_system.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimControl {

  const PlotFeatureEnum signal;

  MBSIM_OBJECTFACTORY_REGISTERENUM(MBSim::PlotFeatureEnum, MBSIMCONTROL, signal)

  void Signal::init(InitStage stage, const InitConfigSet &config) {
    if (stage==plotting) {
      if(plotFeature[plotRecursive] and plotFeature[signal])
	addToPlot("signal",getSignalSize());
    }
    else if (stage==unknownStage) {
      s.resize(getSignalSize(),NONINIT);
    }
    Link::init(stage, config);
  }

  void Signal::plot() {
    if(plotFeature[plotRecursive] and plotFeature[signal])
      Element::plot(evalSignal());
    Link::plot();
  }

}
