/* Copyright (C) 2004-2017 MBSim Development Team
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
#include "mbsim/observers/frame_observer.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/eps.h"
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, FrameObserver)

  FrameObserver::FrameObserver(const std::string &name) : Observer(name), frame(0) {
    evalOMBVPositionColorRepresentation[0] = &FrameObserver::evalNone;
    evalOMBVPositionColorRepresentation[1] = &FrameObserver::evalAbsolutePosition;
    evalOMBVVelocityColorRepresentation[0] = &FrameObserver::evalNone;
    evalOMBVVelocityColorRepresentation[1] = &FrameObserver::evalAbsoluteVelocitiy;
    evalOMBVAngularVelocityColorRepresentation[0] = &FrameObserver::evalNone;
    evalOMBVAngularVelocityColorRepresentation[1] = &FrameObserver::evalAbsoluteAngularVelocitiy;
    evalOMBVAccelerationColorRepresentation[0] = &FrameObserver::evalNone;
    evalOMBVAccelerationColorRepresentation[1] = &FrameObserver::evalAbsoluteAcceleration;
    evalOMBVAngularAccelerationColorRepresentation[0] = &FrameObserver::evalNone;
    evalOMBVAngularAccelerationColorRepresentation[1] = &FrameObserver::evalAbsoluteAngularAcceleration;
  }

  void FrameObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_frame.empty())
        setFrame(getByPath<Frame>(saved_frame));
      if(not frame)
        throwError("Frame is not given!");
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
//        if(plotFeature[position]) plotColumns.push_back("AbsolutePosition");
//        if(plotFeature[velocity]) plotColumns.push_back("AbsoluteVelocity");
//        if(plotFeature[velocity]) plotColumns.push_back("AbsoluteAngularVelocity");
//        if(plotFeature[acceleration]) plotColumns.push_back("AbsoluteAcceleration");
//        if(plotFeature[acceleration]) plotColumns.push_back("AbsoluteAngularAcceleration");
      Observer::init(stage, config);
      if(plotFeature[openMBV]) {
        if(ombvPositionArrow) {
          openMBVPosition=ombvPositionArrow->createOpenMBV();
          openMBVPosition->setName("AbsolutePosition");
          getOpenMBVGrp()->addObject(openMBVPosition);
        }
        if(ombvVelocityArrow) {
          openMBVVelocity=ombvVelocityArrow->createOpenMBV();
          openMBVVelocity->setName("AbsoluteVelocity");
          getOpenMBVGrp()->addObject(openMBVVelocity);
        }
        if(ombvAngularVelocityArrow) {
          openMBVAngularVelocity=ombvAngularVelocityArrow->createOpenMBV();
          openMBVAngularVelocity->setName("AbsoluteAngularVelocity");
          getOpenMBVGrp()->addObject(openMBVAngularVelocity);
        }
        if(ombvAccelerationArrow) {
          openMBVAcceleration=ombvAccelerationArrow->createOpenMBV();
          openMBVAcceleration->setName("AbsoluteAcceleration");
          getOpenMBVGrp()->addObject(openMBVAcceleration);
        }
        if(ombvAngularAccelerationArrow) {
          openMBVAngularAcceleration=ombvAngularAccelerationArrow->createOpenMBV();
          openMBVAngularAcceleration->setName("AbsoluteAngularAcceleration");
          getOpenMBVGrp()->addObject(openMBVAngularAcceleration);
        }
      }
    }
    else
      Observer::init(stage, config);
  }

  void FrameObserver::plot() {
    if(plotFeature[openMBV]) {
      if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
        vector<double> data;
        data.push_back(getTime());
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        Vec3 r = frame->evalPosition();
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back((this->*evalOMBVPositionColorRepresentation[ombvPositionArrow->getColorRepresentation()])());
        openMBVPosition->append(data);
        //          plotVector.push_back(nrm2(r));
      }
      if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 r = frame->evalPosition();
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        Vec3 v = frame->evalVelocity();
        data.push_back(v(0));
        data.push_back(v(1));
        data.push_back(v(2));
        data.push_back((this->*evalOMBVVelocityColorRepresentation[ombvVelocityArrow->getColorRepresentation()])());
        openMBVVelocity->append(data);
        //          plotVector.push_back(nrm2(v));
      }
      if(openMBVAngularVelocity && !openMBVAngularVelocity->isHDF5Link()) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 r = frame->evalPosition();
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        Vec3 om = frame->evalAngularVelocity();
        data.push_back(om(0));
        data.push_back(om(1));
        data.push_back(om(2));
        data.push_back((this->*evalOMBVAngularVelocityColorRepresentation[ombvAngularVelocityArrow->getColorRepresentation()])());
        openMBVAngularVelocity->append(data);
        //          plotVector.push_back(nrm2(om));
      }
      if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 r = frame->evalPosition();
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        Vec3 a = frame->evalAcceleration();
        data.push_back(a(0));
        data.push_back(a(1));
        data.push_back(a(2));
        data.push_back((this->*evalOMBVAccelerationColorRepresentation[ombvAccelerationArrow->getColorRepresentation()])());
        openMBVAcceleration->append(data);
        //          plotVector.push_back(nrm2(a));
      }
      if(openMBVAngularAcceleration && !openMBVAngularAcceleration->isHDF5Link()) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 r = frame->evalPosition();
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        Vec3 psi = frame->evalAngularAcceleration();
        data.push_back(psi(0));
        data.push_back(psi(1));
        data.push_back(psi(2));
        data.push_back((this->*evalOMBVAngularAccelerationColorRepresentation[ombvAngularAccelerationArrow->getColorRepresentation()])());
        openMBVAngularAcceleration->append(data);
        //          plotVector.push_back(nrm2(psi));
      }
    }
    Observer::plot();
  }

  void FrameObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"frame");
    saved_frame=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVPosition");
    if(e) {
      ombvPositionArrow = shared_ptr<OpenMBVArrow>(new OpenMBVArrow);
      ombvPositionArrow->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVVelocity");
    if(e) {
      ombvVelocityArrow = shared_ptr<OpenMBVArrow>(new OpenMBVArrow);
      ombvVelocityArrow->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularVelocity");
    if(e) {
      ombvAngularVelocityArrow = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBVArrow::toDoubleHead,OpenMBVArrow::fromPoint));
      ombvAngularVelocityArrow->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAcceleration");
    if(e) {
      ombvAccelerationArrow = shared_ptr<OpenMBVArrow>(new OpenMBVArrow);
      ombvAccelerationArrow->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularAcceleration");
    if(e) {
      ombvAngularAccelerationArrow = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBVArrow::toDoubleHead,OpenMBVArrow::fromPoint));
      ombvAngularAccelerationArrow->initializeUsingXML(e);
    }
  }

  double FrameObserver::evalAbsolutePosition() {
    return nrm2(frame->evalPosition());
  }

  double FrameObserver::evalAbsoluteVelocitiy() {
    return nrm2(frame->evalVelocity());
  }

  double FrameObserver::evalAbsoluteAngularVelocitiy() {
    return nrm2(frame->evalAngularVelocity());
  }

  double FrameObserver::evalAbsoluteAcceleration() {
    return nrm2(frame->evalAcceleration());
  }

  double FrameObserver::evalAbsoluteAngularAcceleration() {
    return nrm2(frame->evalAngularAcceleration());
  }

}
