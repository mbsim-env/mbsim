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
#include "mbsim/observers/observer.h"
#include <openmbvcppinterface/group.h>
#include "hdf5serie/simpleattribute.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Observer::Observer(const std::string &name) : Element(name) {
  }

  void Observer::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[openMBV]) {
        openMBVGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
        openMBVGrp->setName(name);
        openMBVGrp->setExpand(false);
        parent->getObserversOpenMBVGrp()->addObject(openMBVGrp);
      }
    }
    Element::init(stage, config);
  }

  void Observer::createPlotGroup() {
    plotGroup=parent->getObserversPlotGroup()->createChildObject<H5::Group>(name)();
    plotGroup->createChildAttribute<H5::SimpleAttribute<string>>("Description")()->write("Object of class: "+boost::core::demangle(typeid(*this).name()));
    plotColumns.insert(plotColumns.begin(), "time");
  }

  void Observer::updateInternalStateRef(Vec& curisParent, Vec& nextisParent) {
    curis.ref(curisParent, RangeV(isInd,isInd+isSize-1));
    nextis.ref(nextisParent, RangeV(isInd,isInd+isSize-1));
  }

}
