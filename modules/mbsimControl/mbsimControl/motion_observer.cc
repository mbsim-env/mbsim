/* Copyright (C) 2004-2024 MBSim Development Team
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
#include "mbsimControl/motion_observer.h"
#include "mbsimControl/signal_.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/rigidbody.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, MotionObserver)

  MotionObserver::MotionObserver(const std::string &name) : Observer(name), rOQ(NONINIT), AIK(NONINIT), AIB(Eye()), ABK(Eye())  {
  }

  void MotionObserver::init(InitStage stage, const MBSim::InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_frame.empty())
	setFrameOfReference(getByPath<MBSim::Frame>(saved_frame));
      if(not saved_position_signal.empty())
	setPositionSignal(getByPath<Signal>(saved_position_signal));
      if(not saved_orientation_signal.empty())
	setOrientationSignal(getByPath<Signal>(saved_orientation_signal));
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
      Observer::init(stage, config);
      if(plotFeature[MBSim::openMBV]) {
	if(openMBVBody) {
          openMBVBody->setName("MotionObserver");
          getOpenMBVGrp()->addObject(openMBVBody);
        }
      }
    }
    else if(stage==preInit) {
      Observer::init(stage, config);
      if(position and (position->getSignalSize()!=3))
	throwError("(MotionObserver::init): size of position signal does not match, must be 3, but is " + to_string(position->getSignalSize()) + ".");
      if(orientation and (orientation->getSignalSize()!=3 and orientation->getSignalSize()!=9))
	throwError("(MotionObserver::init): size of orientation signal does not match, must be 3 or 9, but is " + to_string(orientation->getSignalSize()) + ".");
    }
    else
      Observer::init(stage, config);
  }

  void MotionObserver::plot() {
    if(plotFeature[MBSim::openMBV]) {
      if(openMBVBody) {
	if(frame) {
	  rOP = frame->evalPosition();
	  AIB = frame->evalOrientation();
	}
	if(position)
	  rPQ = AIB*position->evalSignal();
	if(orientation) { 
	  VecV a = orientation->evalSignal();
	  if(a.size()==9) {
	    ABK(0,0) = a(0); ABK(0,1) = a(1); ABK(0,2) = a(2);
	    ABK(1,0) = a(3); ABK(1,1) = a(4); ABK(1,2) = a(5);
	    ABK(2,0) = a(6); ABK(2,1) = a(7); ABK(2,2) = a(8);
	  }
	  else if(a.size()==3)
	    ABK = MBSim::Cardan2AIK(a(0),a(1),a(2));
	}
	rOQ = rOP + rPQ;
	AIK = AIB*ABK;
	cardan = MBSim::AIK2Cardan(AIK);

        array<double,8> data;
        data[0] = getTime();
        data[1] = rOQ(0);
        data[2] = rOQ(1);
        data[3] = rOQ(2);
        data[4] = cardan(0);
        data[5] = cardan(1);
        data[6] = cardan(2);
        data[7] = 0;
	static_pointer_cast<OpenMBV::RigidBody>(openMBVBody)->append(data);
      }
    }
    Observer::plot();
  }

  void MotionObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);

    auto *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"frameOfReference");
    if(e) saved_frame=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"positionSignal");
    if(e) saved_position_signal=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"orientationSignal");
    if(e) saved_orientation_signal=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"openMBVRigidBody");
    if(e) {
      openMBVBody=OpenMBV::ObjectFactory::create<OpenMBV::RigidBody>(e->getFirstElementChild());
      openMBVBody->initializeUsingXML(e->getFirstElementChild());
    }
  }

}
