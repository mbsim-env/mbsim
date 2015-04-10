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
#include "mbsim/observers/frame_observer.h"
#include "mbsim/frame.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  FrameObserver::FrameObserver(const std::string &name) : Observer(name), frame(0) {
  }

  void FrameObserver::init(InitStage stage) {
    if(stage==plotting) {
      updatePlotFeatures();

      Observer::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVPosition) {
            openMBVPosition->setName("Position");
            getOpenMBVGrp()->addObject(openMBVPosition);
          }
          if(openMBVVelocity) {
            openMBVVelocity->setName("Velocity");
            getOpenMBVGrp()->addObject(openMBVVelocity);
          }
          if(openMBVAngularVelocity) {
            openMBVAngularVelocity->setName("AngularVelocity");
            getOpenMBVGrp()->addObject(openMBVAngularVelocity);
          }
          if(openMBVAcceleration) {
            openMBVAcceleration->setName("Acceleration");
            getOpenMBVGrp()->addObject(openMBVAcceleration);
          }
          if(openMBVAngularAcceleration) {
            openMBVAngularAcceleration->setName("AngularAcceleration");
            getOpenMBVGrp()->addObject(openMBVAngularAcceleration);
          }
        }
#endif
      }
    }
    else
      Observer::init(stage);
  }

  void FrameObserver::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(0.5);
          openMBVPosition->append(data);
        }
        if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getVelocity()(0));
          data.push_back(frame->getVelocity()(1));
          data.push_back(frame->getVelocity()(2));
          data.push_back(0.5);
          openMBVVelocity->append(data);
        }
        if(openMBVAngularVelocity && !openMBVAngularVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getAngularVelocity()(0));
          data.push_back(frame->getAngularVelocity()(1));
          data.push_back(frame->getAngularVelocity()(2));
          data.push_back(0.5);
          openMBVAngularVelocity->append(data);
        }
        if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getAcceleration()(0));
          data.push_back(frame->getAcceleration()(1));
          data.push_back(frame->getAcceleration()(2));
          data.push_back(0.5);
          openMBVAcceleration->append(data);
        }
        if(openMBVAngularAcceleration && !openMBVAngularAcceleration->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getAngularAcceleration()(0));
          data.push_back(frame->getAngularAcceleration()(1));
          data.push_back(frame->getAngularAcceleration()(2));
          data.push_back(0.5);
          openMBVAngularAcceleration->append(data);
        }
      }
#endif

      Observer::plot(t,dt);
    }
  }

  void FrameObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"frame");
    if(e) saved_frame=E(e)->getAttribute("ref");
//    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVPosition");
//    if(e) openMBVPosition=enableOpenMBVArrow("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::fromPoint,1,1,e);
//    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVVelocity");
//    if(e) openMBVVelocity=enableOpenMBVArrow("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::fromPoint,1,1,e);
//    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularVelocity");
//    if(e) openMBVAngularVelocity=enableOpenMBVArrow("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::fromPoint,1,1,e);
//    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAcceleration");
//    if(e) openMBVAcceleration=enableOpenMBVArrow("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::fromPoint,1,1,e);
//    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularAcceleration");
//    if(e) openMBVAngularAcceleration=enableOpenMBVArrow("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::fromPoint,1,1,e);
  }

}
