/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "mbsimHydraulics/leakage_line.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/rigid_line_pressureloss.h"
#include "mbsimControl/signal_.h"
#include "mbsim/dynamic_system.h"
#include "mbsimHydraulics/defines.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimHydraulics {

  double LeakageLine::getGapLength() const {
    return ((glSignal)?glSignal->getSignal()(0):length);
  }

  double LeakageLine::getSurface1Velocity() const {
    return ((s1vSignal)?s1vSignal->getSignal()(0):0);
  }

  double LeakageLine::getSurface2Velocity() const {
    return ((s2vSignal)?s2vSignal->getSignal()(0):0);
  }

  void LeakageLine::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      RigidHLine::init(stage);
      if (s1vPath!="")
        setSurface1VelocitySignal(getByPath<Signal>(s1vPath));
      if (s2vPath!="")
        setSurface2VelocitySignal(getByPath<Signal>(s2vPath));
      if (glPath!="")
        setGapLengthSignal(getByPath<Signal>(glPath));
    }
    else if (stage==MBSim::modelBuildup) {
      ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"/LeakagePressureLoss", this, lpl, false, false));
      RigidHLine::init(stage);
    }
    else
      RigidHLine::init(stage);
  }

  void LeakageLine::initializeUsingXML(DOMElement * element) {
    RigidHLine::initializeUsingXML(element);
    DOMElement * e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"firstSurfaceVelocity");
    if (e)
      s1vPath=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"secondSurfaceVelocity");
    if (e)
      s2vPath=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"gapLength");
    if (e)
      glPath=E(e)->getAttribute("ref");
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, PlaneLeakageLine,  MBSIMHYDRAULICS%"PlaneLeakageLine")

  void PlaneLeakageLine::setPlaneLeakagePressureLoss(PlaneLeakagePressureLoss * plpl) {
    lpl=plpl;
  }

  void PlaneLeakageLine::init(InitStage stage) {
    if (stage==MBSim::preInit) {  
      LeakageLine::init(stage);
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      Mlocal.resize(1, INIT, rho*length/hGap/wGap);
    }
    else
      LeakageLine::init(stage);
  }

  void PlaneLeakageLine::initializeUsingXML(DOMElement * element) {
    LeakageLine::initializeUsingXML(element);
    DOMElement * e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"width");
    setGapWidth(getDouble(e));
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"height");
    setGapHeight(getDouble(e));
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"planeLeakagePressureLoss");
    PlaneLeakagePressureLoss *p=MBSim::ObjectFactory<FunctionBase>::createAndInit<PlaneLeakagePressureLoss>(e->getFirstElementChild());
    setPlaneLeakagePressureLoss(p);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, CircularLeakageLine,  MBSIMHYDRAULICS%"CircularLeakageLine")

  void CircularLeakageLine::setCircularLeakagePressureLoss(CircularLeakagePressureLoss * clpl) {
    lpl=clpl;
  }

  void CircularLeakageLine::init(InitStage stage) {
    if (stage==MBSim::preInit) {  
      rO=rI+hGap;
      LeakageLine::init(stage);
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      Mlocal.resize(1, INIT, rho*length/(M_PI*(rO*rO-rI*rI)));
    }
    else
      LeakageLine::init(stage);
  }

  void CircularLeakageLine::initializeUsingXML(DOMElement * element) {
    LeakageLine::initializeUsingXML(element);
    DOMElement * e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"innerRadius");
    setInnerRadius(getDouble(e));
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"height");
    setGapHeight(getDouble(e));
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"circularLeakagePressureLoss");
    CircularLeakagePressureLoss *p=MBSim::ObjectFactory<FunctionBase>::createAndInit<CircularLeakagePressureLoss>(e->getFirstElementChild());
    setCircularLeakagePressureLoss(p);
  }

}

