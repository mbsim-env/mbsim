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
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/dynamic_system_solver.h"
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
      if(not saved_outputFrame.empty())
        setOutputFrame(getByPath<Frame>(saved_outputFrame));
      if(not outputFrame)
        setOutputFrame(ds->getFrameI());
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        if(plotFeature[position])
          addToPlot("position",{"x","y","z"});
        if(plotFeature[angle])
          addToPlot("angle",{"alpha","beta","gamma"});
        if(plotFeature[velocity])
          addToPlot("velocity",{"x","y","z"});
        if(plotFeature[angularVelocity])
          addToPlot("angular velocity",{"x","y","z"});
        if(plotFeature[acceleration])
          addToPlot("acceleration",{"x","y","z"});
        if(plotFeature[angularAcceleration])
          addToPlot("angular acceleration",{"x","y","z"});
      }
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
    if(plotFeature[plotRecursive]) {
      auto TWOut = outputFrame->evalOrientation();
      if(plotFeature[position])
        Element::plot(TWOut.T()*frame->evalPosition());
      if(plotFeature[angle])
        Element::plot(AIK2Cardan(frame->evalOrientation()));
      if(plotFeature[velocity])
        Element::plot(TWOut.T()*frame->evalVelocity());
      if(plotFeature[angularVelocity])
        Element::plot(TWOut.T()*frame->evalAngularVelocity());
      if(plotFeature[acceleration])
        Element::plot(TWOut.T()*frame->evalAcceleration());
      if(plotFeature[angularAcceleration])
        Element::plot(TWOut.T()*frame->evalAngularAcceleration());
    }
    if(plotFeature[openMBV]) {
      if(openMBVPosition) {
        array<double,8> data;
        data[0] = getTime();
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        Vec3 r = frame->evalPosition();
        data[4] = r(0);
        data[5] = r(1);
        data[6] = r(2);
        data[7] = (this->*evalOMBVPositionColorRepresentation[ombvPositionArrow->getColorRepresentation()])();
        openMBVPosition->append(data);
      }
      if(openMBVVelocity) {
        array<double,8> data;
        data[0] = getTime();
        Vec3 r = frame->evalPosition();
        data[1] = r(0);
        data[2] = r(1);
        data[3] = r(2);
        Vec3 v = frame->evalVelocity();
        data[4] = v(0);
        data[5] = v(1);
        data[6] = v(2);
        data[7] = (this->*evalOMBVVelocityColorRepresentation[ombvVelocityArrow->getColorRepresentation()])();
        openMBVVelocity->append(data);
      }
      if(openMBVAngularVelocity) {
        array<double,8> data;
        data[0] = getTime();
        Vec3 r = frame->evalPosition();
        data[1] = r(0);
        data[2] = r(1);
        data[3] = r(2);
        Vec3 om = frame->evalAngularVelocity();
        data[4] = om(0);
        data[5] = om(1);
        data[6] = om(2);
        data[7] = (this->*evalOMBVAngularVelocityColorRepresentation[ombvAngularVelocityArrow->getColorRepresentation()])();
        openMBVAngularVelocity->append(data);
      }
      if(openMBVAcceleration) {
        array<double,8> data;
        data[0] = getTime();
        Vec3 r = frame->evalPosition();
        data[1] = r(0);
        data[2] = r(1);
        data[3] = r(2);
        Vec3 a = frame->evalAcceleration();
        data[4] = a(0);
        data[5] = a(1);
        data[6] = a(2);
        data[7] = (this->*evalOMBVAccelerationColorRepresentation[ombvAccelerationArrow->getColorRepresentation()])();
        openMBVAcceleration->append(data);
      }
      if(openMBVAngularAcceleration) {
        array<double,8> data;
        data[0] = getTime();
        Vec3 r = frame->evalPosition();
        data[1] = r(0);
        data[2] = r(1);
        data[3] = r(2);
        Vec3 psi = frame->evalAngularAcceleration();
        data[4] = psi(0);
        data[5] = psi(1);
        data[6] = psi(2);
        data[7] = (this->*evalOMBVAngularAccelerationColorRepresentation[ombvAngularAccelerationArrow->getColorRepresentation()])();
        openMBVAngularAcceleration->append(data);
      }
    }
    Observer::plot();
  }

  void FrameObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"frame");
    saved_frame=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"outputFrame");
    if(e) saved_outputFrame=E(e)->getAttribute("ref");

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
