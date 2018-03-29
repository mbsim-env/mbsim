/* Copyright (C) 2004-2016 MBSim Development Team
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
#include "mbsim/observers/maxwell_contact_observer.h"
#include "mbsim/links/maxwell_contact.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/contour.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, MaxwellContactObserver)

  MaxwellContactObserver::MaxwellContactObserver(const std::string &name) : Observer(name) {
  }

  void MaxwellContactObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(saved_link!="")
        setMaxwellContact(getByPath<MaxwellContact>(saved_link));
      Observer::init(stage, config);
    }
    else if(stage==preInit) {
      Observer::init(stage, config);
      contactObserver.resize(static_cast<MaxwellContact*>(link)->getSubcontacts().size(),vector<SingleContactObserver>(static_cast<MaxwellContact*>(link)->getSubcontacts()[0].size()));
      for (unsigned int i=0; i<contactObserver.size(); i++) {
        for (unsigned int j=0; j<contactObserver[i].size(); j++) {
          contactObserver[i][j].setParent(this);
          contactObserver[i][j].setMechanicalLink(&static_cast<MaxwellContact*>(link)->getSingleContact(i,j));
          stringstream contactName;
          contactName << "MaxwellContact_" << i << "_" << j;
          contactObserver[i][j].setName(contactName.str());
          contactObserver[i][j].plotFeature = plotFeature;
          contactObserver[i][j].plotFeatureForChildren = plotFeatureForChildren;
          //Set OpenMBV-Properties to single contacts
          if(openMBVForce)
            contactObserver[i][j].setOpenMBVForce((i==0 and j==0)?openMBVForce:OpenMBV::ObjectFactory::create(openMBVForce));
          if(openMBVMoment)
            contactObserver[i][j].setOpenMBVMoment((i==0 and j==0)?openMBVMoment:OpenMBV::ObjectFactory::create(openMBVMoment));
          if(openMBVContactFrame)
            contactObserver[i][j].setOpenMBVContactPoints((i==0 and j==0)?openMBVContactFrame:OpenMBV::ObjectFactory::create(openMBVContactFrame));
          if(contactArrow)
            contactObserver[i][j].setOpenMBVNormalForce((i==0 and j==0)?contactArrow:OpenMBV::ObjectFactory::create(contactArrow));
          if(frictionArrow)
            contactObserver[i][j].setOpenMBVTangentialForce((i==0 and j==0)?frictionArrow:OpenMBV::ObjectFactory::create(frictionArrow));
          contactObserver[i][j].init(stage, config);
        }
      }
    }
    else if (stage == plotting) {
      Observer::init(stage, config);
      for (auto & iter : contactObserver) {
        for (auto jter = iter.begin(); jter != iter.end(); ++jter)
          jter->init(stage, config);
      }
    }
    else if (stage == unknownStage) {
      for (auto & iter : contactObserver) {
        for (auto jter = iter.begin(); jter != iter.end(); ++jter)
          jter->init(stage, config);
      }
      Observer::init(stage, config);
    }
    else
      Observer::init(stage, config);
  }

  void MaxwellContactObserver::plot() {
    Observer::plot();
    for (auto & iter : contactObserver) {
      for (auto jter = iter.begin(); jter != iter.end(); ++jter)
        jter->plot();
    }
  }

  void MaxwellContactObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"maxwellContact");
    saved_link=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(e) {
        OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      ombv.initializeUsingXML(e);
        openMBVForce=ombv.createOpenMBV();
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(e) {
        OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      ombv.initializeUsingXML(e);
        openMBVMoment=ombv.createOpenMBV();
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVContactPoints");
    if (e) {
      OpenMBVFrame ombv;
      ombv.initializeUsingXML(e);
      openMBVContactFrame=ombv.createOpenMBV();
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVNormalForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toHead, OpenMBV::Arrow::toPoint, 1, 1);
      ombv.initializeUsingXML(e);
      contactArrow = ombv.createOpenMBV();
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVTangentialForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toHead, OpenMBV::Arrow::toPoint, 1, 1);
      ombv.initializeUsingXML(e);
      frictionArrow = ombv.createOpenMBV();
    }
  }

  void MaxwellContactObserver::setDynamicSystemSolver(DynamicSystemSolver* sys) {
    Observer::setDynamicSystemSolver(sys);

    for (auto & iter : contactObserver) {
      for (auto jter = iter.begin(); jter != iter.end(); ++jter)
        jter->setDynamicSystemSolver(ds);
    }
  }


}
