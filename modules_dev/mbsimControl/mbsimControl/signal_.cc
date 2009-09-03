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
 * Contact: schneidm@users.berlios.de
 */

#include "mbsimControl/signal_.h"
#include "mbsim/utils/utils.h"
#include "mbsim/dynamic_system.h"

using namespace fmatvec;

namespace MBSim {

  Signal * getSignalByPath(DynamicSystem * ds,  const std::string& cpath) {
    std::string path=cpath;
    int pos=path.find("Signal");
    path.erase(pos, 6);
    path.insert(pos, "Link");
    Link * s = ds->getLinkByPath(path);
    if (dynamic_cast<Signal *>(s))
      return static_cast<Signal *>(s);
    else {
      std::cerr << "ERROR! \"" << path << "\" is not of Signal-Type." << std::endl; 
      _exit(1);
    }
  }

  void Signal::init(InitStage stage) {
    if (stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
        Vec y=getSignal();
        for (int i=0; i<y.size(); i++)
          plotColumns.push_back("Signal (" + numtostr(i) + ")");
      }
      Link::init(stage);
    }
    else
      Link::init(stage);
  }


  void Signal::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      Vec y=getSignal();
      for (int i=0; i<y.size(); i++)
        plotVector.push_back(y(i));
    }
    Link::plot(t, dt);
  }

}
