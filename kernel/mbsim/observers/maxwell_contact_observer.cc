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
      if(sideOfForceInteraction==unknown)
        throwError("(ContactObserver::init): side of force interaction unknown");
      if(sideOfMomentInteraction==unknown)
        throwError("(ContactObserver::init): side of moment interaction unknown");
      if(sideOfContactInteraction==unknown)
        throwError("(ContactObserver::init): side of normal force interaction unknown");
      if(sideOfFrictionInteraction==unknown)
        throwError("(ContactObserver::init): side of tangential force interaction unknown");
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
          if(ombvForce) {
            contactObserver[i][j].setOMBVForce(ombvForce);
            contactObserver[i][j].sideOfForceInteraction=SingleContactObserver::SideOfInteraction(sideOfForceInteraction);
          }
          if(ombvMoment) {
            contactObserver[i][j].setOMBVMoment(ombvMoment);
            contactObserver[i][j].sideOfMomentInteraction=SingleContactObserver::SideOfInteraction(sideOfMomentInteraction);
          }
          if(openMBVContactFrame)
            contactObserver[i][j].setOpenMBVContactPoints((i==0)?openMBVContactFrame:OpenMBV::ObjectFactory::create(openMBVContactFrame));
          if(ombvContact) {
            contactObserver[i][j].setOMBVNormalForce(ombvContact);
            contactObserver[i][j].sideOfContactInteraction=SingleContactObserver::SideOfInteraction(sideOfContactInteraction);
          }
          if(ombvFriction) {
            contactObserver[i][j].setOMBVTangentialForce(ombvFriction);
            contactObserver[i][j].sideOfFrictionInteraction=SingleContactObserver::SideOfInteraction(sideOfFrictionInteraction);
          }
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
      DOMElement* ee=E(e)->getFirstElementChildNamed(MBSIM%"sideOfInteraction");
      if(ee) {
        string sideOfInteractionStr=string(X()%E(ee)->getFirstTextChild()->getData()).substr(1,string(X()%E(ee)->getFirstTextChild()->getData()).length()-2);
        if(sideOfInteractionStr=="action") sideOfForceInteraction=action;
        else if(sideOfInteractionStr=="reaction") sideOfForceInteraction=reaction;
        else if(sideOfInteractionStr=="both") sideOfForceInteraction=both;
        else sideOfForceInteraction=unknown;
      }
      ombvForce = shared_ptr<OpenMBVArrow>(new OpenMBVArrow("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1));
      ombvForce->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(e) {
      DOMElement* ee=E(e)->getFirstElementChildNamed(MBSIM%"sideOfInteraction");
      if(ee) {
        string sideOfInteractionStr=string(X()%E(ee)->getFirstTextChild()->getData()).substr(1,string(X()%E(ee)->getFirstTextChild()->getData()).length()-2);
        if(sideOfInteractionStr=="action") sideOfMomentInteraction=action;
        else if(sideOfInteractionStr=="reaction") sideOfMomentInteraction=reaction;
        else if(sideOfInteractionStr=="both") sideOfMomentInteraction=both;
        else sideOfMomentInteraction=unknown;
      }
      ombvMoment = shared_ptr<OpenMBVArrow>(new OpenMBVArrow("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1));
      ombvMoment->initializeUsingXML(e);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVContactPoints");
    if (e) {
      OpenMBVFrame ombv;
      ombv.initializeUsingXML(e);
      openMBVContactFrame=ombv.createOpenMBV();
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVNormalForce");
    if (e) {
      DOMElement* ee=E(e)->getFirstElementChildNamed(MBSIM%"sideOfInteraction");
      if(ee) {
        string sideOfInteractionStr=string(X()%E(ee)->getFirstTextChild()->getData()).substr(1,string(X()%E(ee)->getFirstTextChild()->getData()).length()-2);
        if(sideOfInteractionStr=="action") sideOfForceInteraction=action;
        else if(sideOfInteractionStr=="reaction") sideOfForceInteraction=reaction;
        else if(sideOfInteractionStr=="both") sideOfForceInteraction=both;
        else sideOfForceInteraction=unknown;
      }
      ombvContact = shared_ptr<OpenMBVArrow>(new OpenMBVArrow("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1));
      ombvContact->initializeUsingXML(e);
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVTangentialForce");
    if (e) {
      DOMElement* ee=E(e)->getFirstElementChildNamed(MBSIM%"sideOfInteraction");
      if(ee) {
        string sideOfInteractionStr=string(X()%E(ee)->getFirstTextChild()->getData()).substr(1,string(X()%E(ee)->getFirstTextChild()->getData()).length()-2);
        if(sideOfInteractionStr=="action") sideOfForceInteraction=action;
        else if(sideOfInteractionStr=="reaction") sideOfForceInteraction=reaction;
        else if(sideOfInteractionStr=="both") sideOfForceInteraction=both;
        else sideOfForceInteraction=unknown;
      }
      ombvFriction = shared_ptr<OpenMBVArrow>(new OpenMBVArrow("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1));
      ombvFriction->initializeUsingXML(e);
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
