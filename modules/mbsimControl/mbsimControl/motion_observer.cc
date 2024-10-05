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
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/rigidbody.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, MotionObserver)

  MotionObserver::MotionObserver(const std::string &name) : Observer(name) {
  }

  void MotionObserver::init(InitStage stage, const MBSim::InitConfigSet &config) {
    if(stage==resolveStringRef) {
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
	if(position) r = position->evalSignal();
	if(orientation) { 
	  VecV a = orientation->evalSignal();
	  if(a.size()==9) {
	    SqrMat3 A(NONINIT);
	    A(0,0) = a(0); A(0,1) = a(1); A(0,2) = a(2);
	    A(1,0) = a(3); A(1,1) = a(4); A(1,2) = a(5);
	    A(2,0) = a(6); A(2,1) = a(7); A(2,2) = a(8);
	    cardan = MBSim::AIK2Cardan(A);
	  }
	  else if(a.size()==3)
	    cardan = a;
	}
        vector<double> data;
        data.push_back(getTime());
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        data.push_back(cardan(0));
        data.push_back(cardan(1));
        data.push_back(cardan(2));
        data.push_back(0);
	static_pointer_cast<OpenMBV::RigidBody>(openMBVBody)->append(data);
      }
    }
    Observer::plot();
  }

  void MotionObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);

    auto *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"positionSignal");
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
