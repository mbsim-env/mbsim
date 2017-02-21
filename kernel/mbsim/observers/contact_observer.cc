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
#include "mbsim/observers/contact_observer.h"
#include "mbsim/links/contact.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/contour.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, ContactObserver)

  ContactObserver::ContactObserver(const std::string &name) : Observer(name) {
  }

  void ContactObserver::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_link!="")
        setContact(getByPath<Contact>(saved_link));
      Observer::init(stage);
    }
    else if(stage==preInit) {
      contactObserver.resize(static_cast<Contact*>(link)->getSubcontacts().size(),vector<SingleContactObserver>(static_cast<Contact*>(link)->getSubcontacts()[0].size()));
      for (unsigned int i=0; i<contactObserver.size(); i++) {
        for (unsigned int j=0; j<contactObserver[i].size(); j++) {
          contactObserver[i][j].setParent(this);
          contactObserver[i][j].setDynamicSystemSolver(ds);
          contactObserver[i][j].setMechanicalLink(&static_cast<Contact*>(link)->getSingleContact(i,j));
          stringstream contactName;
          contactName << "Contact_" << i << "_" << j;
          contactObserver[i][j].setName(contactName.str());
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
          contactObserver[i][j].init(stage);
        }
      }
      Observer::init(stage);
    }
    else if (stage == plotting) {
      Observer::init(stage);
      updatePlotFeatures();
      if (getPlotFeature(plotRecursive) == enabled) {
        for (std::vector<std::vector<SingleContactObserver> >::iterator iter = contactObserver.begin(); iter != contactObserver.end(); ++iter) {
          for (std::vector<SingleContactObserver>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
            jter->init(stage);
        }
      }
    }
    else
      Observer::init(stage);
  }

  void ContactObserver::plot() {
    Observer::plot();
    for (std::vector<std::vector<SingleContactObserver> >::iterator iter = contactObserver.begin(); iter != contactObserver.end(); ++iter) {
      for (std::vector<SingleContactObserver>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->plot();
    }
  }

  void ContactObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"contact");
    saved_link=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(e) {
        OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
        openMBVForce=ombv.createOpenMBV(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(e) {
        OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
        openMBVMoment=ombv.createOpenMBV(e);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVContactPoints");
    if (e) {
      OpenMBVFrame ombv;
      openMBVContactFrame=ombv.createOpenMBV(e);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVNormalForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toHead, OpenMBV::Arrow::toPoint, 1, 1);
      contactArrow = ombv.createOpenMBV(e);
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVTangentialForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toHead, OpenMBV::Arrow::toPoint, 1, 1);
      frictionArrow = ombv.createOpenMBV(e);
    }
  }

}
