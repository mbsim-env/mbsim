/* Copyright (C) 2004-2013 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#include <config.h>
#include "mbsim/observer.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Observer::Observer(const std::string &name) : Element(name) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVGrp = 0;
#endif
  }

  void Observer::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          openMBVGrp=new OpenMBV::Group();
          openMBVGrp->setName(name+"_Group");
          openMBVGrp->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVGrp);
        }
#endif
      }
      Element::init(stage);
    }
    else
      Element::init(stage);
  }

  void Observer::plot(double t, double dt) {
    Element::plot(t,dt);
  }

  Element * Observer::getByPathSearch(string path) {
    if (path.substr(0, 3)=="../") // relative path
      return parent->getByPathSearch(path.substr(3));
    else // absolut path
      if(parent)
        return parent->getByPathSearch(path);
      else
        return getByPathSearch(path.substr(1));
  }

}
