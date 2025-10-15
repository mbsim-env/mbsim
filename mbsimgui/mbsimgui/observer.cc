/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
   */

#include <config.h>
#include "observer.h"
#include "element_property_dialog.h"
#include "utils.h"
#include "mainwindow.h"
#include "parameter.h"
#include <xercesc/dom/DOMDocument.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  MBSIMGUI_REGOBJECTFACTORY(ContactObserver);
  MBSIMGUI_REGOBJECTFACTORY(FrameObserver);
  MBSIMGUI_REGOBJECTFACTORY(InverseKinematicsConstraintObserver);
  MBSIMGUI_REGOBJECTFACTORY(MechanicalConstraintObserver);
  MBSIMGUI_REGOBJECTFACTORY(MechanicalLinkObserver);
  MBSIMGUI_REGOBJECTFACTORY(RigidBodyObserver);
  MBSIMGUI_REGOBJECTFACTORY(SignalObserver);
  MBSIMGUI_REGOBJECTFACTORY(TyreContactObserver);
  MBSIMGUI_REGOBJECTFACTORY(FrameChaserObserver);
  MBSIMGUI_REGOBJECTFACTORY(MotionObserver);
  MBSIMGUI_REGOBJECTFACTORY(UnknownObserver);

  Observer::Observer() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"observer.svg").string()));
    parameters->setIcon(icon);
  }

  DOMElement* MechanicalLinkObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    return element;
  }

  DOMElement* MechanicalConstraintObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    return element;
  }

  DOMElement* ContactObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVContactPoints");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVNormalForce");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVTangentialForce");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    return element;
  }

  DOMElement* TyreContactObserver::processIDAndHref(DOMElement *element) {
    MechanicalLinkObserver::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVContactPoints");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVNormalForce");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVLongitudinalForce");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVLateralForce");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());
    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVOverturningMoment");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVRollingResistanceMoment");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAligningMoment");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());
    return element;
  }

  DOMElement* FrameObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVPosition");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVVelocity");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularVelocity");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAcceleration");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularAcceleration");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    return element;
  }

  DOMElement* RigidBodyObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVWeight");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointForce");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointMoment");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAxisOfRotation");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    return element;
  }

  DOMElement* InverseKinematicsConstraintObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    return element;
  }

  DOMElement* SignalObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"enableOpenMBV");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    return element;
  }

  DOMElement* FrameChaserObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"enableOpenMBV");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    return element;
  }

  DOMElement* MotionObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"openMBVRigidBody");
    if(ELE) {
      ELE = ELE->getFirstElementChild();
      if(ELE)
        E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());
    }

    return element;
  }

  UnknownObserver::UnknownObserver() {
    icon = QIcon(new OverlayIconEngine((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"observer.svg").string(),
                                       (MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"unknownelement.svg").string()));
  }

}
