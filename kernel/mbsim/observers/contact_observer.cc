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
#include "mbsim/dynamic_system_solver.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, ContactObserver)

  ContactObserver::ContactObserver(const std::string &name) : Observer(name) {
  }

  void ContactObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_link.empty())
        setContact(getByPath<Contact>(saved_link));
      if(not link)
        throwError("Contact is not given!");
      if(not saved_outputFrame.empty())
        setOutputFrame(getByPath<Frame>(saved_outputFrame));
      if(not outputFrame)
        setOutputFrame(ds->getFrameI());
      Observer::init(stage, config);
    }
    else if(stage==preInit) {
      Observer::init(stage, config);
      contactObserver.resize(static_cast<Contact*>(link)->getSubcontacts().size());
      for (unsigned int i=0; i<contactObserver.size(); i++) {
        contactObserver[i].setParent(this);
        contactObserver[i].setOutputFrame(outputFrame);
        contactObserver[i].setMechanicalLink(&static_cast<Contact*>(link)->getSingleContact(i));
        stringstream contactName;
        contactName << "Contact_" << 0 << "_" << i;
        contactObserver[i].setName(contactName.str());
        contactObserver[i].plotFeature = plotFeature;
        contactObserver[i].plotFeatureForChildren = plotFeatureForChildren;
        //Set OpenMBV-Properties to single contacts
        if(ombvForce)
          contactObserver[i].setOMBVForce(ombvForce);
        if(ombvMoment)
          contactObserver[i].setOMBVMoment(ombvMoment);
        if(openMBVContactFrame)
          contactObserver[i].setOpenMBVContactPoints((i==0)?openMBVContactFrame:OpenMBV::ObjectFactory::create(openMBVContactFrame));
        if(ombvContact)
          contactObserver[i].setOMBVNormalForce(ombvContact);
        if(ombvFriction)
          contactObserver[i].setOMBVTangentialForce(ombvFriction);
        contactObserver[i].init(stage, config);
      }
    }
    else if (stage == plotting) {
      Observer::init(stage, config);
      for (auto & iter : contactObserver)
        iter.init(stage, config);
    }
    else if (stage == unknownStage) {
      for (auto & iter : contactObserver)
        iter.init(stage, config);
      Observer::init(stage, config);
    }
    else
      Observer::init(stage, config);
  }

  void ContactObserver::plot() {
    Observer::plot();
    for (auto & iter : contactObserver)
      iter.plot();
  }

  void ContactObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"contact");
    saved_link=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"outputFrame");
    if(e) saved_outputFrame=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(e) {
      ombvForce = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvForce->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(e) {
      ombvMoment = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toDoubleHead,OpenMBVArrow::toPoint));
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
      ombvContact = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvContact->initializeUsingXML(e);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVTangentialForce");
    if (e) {
      ombvFriction = shared_ptr<OpenMBVFrictionArrow>(new OpenMBVFrictionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvFriction->initializeUsingXML(e);
    }
  }

  void ContactObserver::setDynamicSystemSolver(DynamicSystemSolver* sys) {
    Observer::setDynamicSystemSolver(sys);

    for (auto & iter : contactObserver)
      iter.setDynamicSystemSolver(ds);
  }


}
