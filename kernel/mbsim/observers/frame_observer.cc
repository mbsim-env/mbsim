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
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  FrameObserver::FrameObserver(const std::string &name) : Observer(name), frame(0) {
  }

  void FrameObserver::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frame!="")
        setFrame(getByPath<Frame>(saved_frame));
      Observer::init(stage);
    }
    else if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
        if(openMBVPosition) plotColumns.push_back("AbsolutePosition");
        if(openMBVVelocity) plotColumns.push_back("AbsoluteVelocity");
        if(openMBVAngularVelocity) plotColumns.push_back("AbsoluteAngularVelocity");
        if(openMBVAcceleration) plotColumns.push_back("AbsoluteAcceleration");
        if(openMBVAngularAcceleration) plotColumns.push_back("AbsoluteAngularAcceleration");
        Observer::init(stage);
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVPosition) {
            openMBVPosGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
            openMBVPosGrp->setName("Position_Group");
            openMBVPosGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVPosGrp);
            openMBVPosition->setName("AbsolutePosition");
            openMBVPosGrp->addObject(openMBVPosition);
          }
          if(openMBVVelocity) {
            openMBVVelGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
            openMBVVelGrp->setName("Velocity_Group");
            openMBVVelGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVVelGrp);
            openMBVVelocity->setName("AbsoluteVelocity");
            openMBVVelGrp->addObject(openMBVVelocity);
          }
          if(openMBVAngularVelocity) {
            openMBVAngVelGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
            openMBVAngVelGrp->setName("Angular_Velocity_Group");
            openMBVAngVelGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVAngVelGrp);
            openMBVAngularVelocity->setName("AbsoluteAngularVelocity");
            openMBVAngVelGrp->addObject(openMBVAngularVelocity);
          }
          if(openMBVAcceleration) {
            openMBVAccGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
            openMBVAccGrp->setName("Acceleration_Group");
            openMBVAccGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVAccGrp);
            openMBVAcceleration->setName("AbsoluteAcceleration");
            openMBVAccGrp->addObject(openMBVAcceleration);
          }
          if(openMBVAngularAcceleration) {
            openMBVAngAccGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
            openMBVAngAccGrp->setName("Angular_Acceleration_Group");
            openMBVAngAccGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVAngAccGrp);
            openMBVAngularAcceleration->setName("AbsoluteAngularAcceleration");
            openMBVAngAccGrp->addObject(openMBVAngularAcceleration);
          }
        }
      }
    }
    else
      Observer::init(stage);
  }

  void FrameObserver::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVPosition&& !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          data.push_back(getTime());
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          Vec3 r = frame->evalPosition();
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(0.5);
          openMBVPosition->append(data);
          plotVector.push_back(nrm2(r));
        }
        if(openMBVVelocity&& !openMBVVelocity->isHDF5Link()) {
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
          data.push_back(0.5);
          openMBVVelocity->append(data);
          plotVector.push_back(nrm2(v));
        }
        if(openMBVAngularVelocity&& !openMBVAngularVelocity->isHDF5Link()) {
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
          data.push_back(0.5);
          openMBVAngularVelocity->append(data);
          plotVector.push_back(nrm2(om));
        }
        if(openMBVAcceleration&& !openMBVAcceleration->isHDF5Link()) {
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
          data.push_back(0.5);
          openMBVAcceleration->append(data);
          plotVector.push_back(nrm2(a));
        }
        if(openMBVAngularAcceleration&& !openMBVAngularAcceleration->isHDF5Link()) {
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
          data.push_back(0.5);
          openMBVAngularAcceleration->append(data);
          plotVector.push_back(nrm2(psi));
        }
      }

      Observer::plot();
    }
  }

  void FrameObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"frame");
    if(e) saved_frame=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVPosition");
    if(e) {
        OpenMBVArrow ombv;
        openMBVPosition=ombv.createOpenMBV(e); 
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVVelocity");
    if(e) {
        OpenMBVArrow ombv;
        openMBVVelocity=ombv.createOpenMBV(e); 
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularVelocity");
    if(e) {
        OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::fromPoint,1,1);
        openMBVAngularVelocity=ombv.createOpenMBV(e); 
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAcceleration");
    if(e) {
        OpenMBVArrow ombv;
        openMBVAcceleration=ombv.createOpenMBV(e); 
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularAcceleration");
    if(e) {
        OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::fromPoint,1,1);
        openMBVAngularAcceleration=ombv.createOpenMBV(e); 
    }
  }

}
