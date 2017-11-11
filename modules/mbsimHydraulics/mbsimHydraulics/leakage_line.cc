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
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimHydraulics {

  LeakageLine::~LeakageLine() {
    delete lpl;
    delete s1vFunction;
    delete s2vFunction;
    delete glFunction;
  }

  double LeakageLine::evalGapLength() const {
    return (glFunction)?(*glFunction)(getTime()):length;
  }

  double LeakageLine::evalSurface1Velocity() const {
    return (s1vFunction)?(*s1vFunction)(getTime()):0;
  }

  double LeakageLine::evalSurface2Velocity() const {
    return (s2vFunction)?(*s2vFunction)(getTime()):0;
  }

  void LeakageLine::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"_LeakagePressureLoss", this, lpl, false, false));
      RigidHLine::init(stage, config);
    }
    else
      RigidHLine::init(stage, config);
    lpl->init(stage, config);
    if(s1vFunction) s1vFunction->init(stage, config);
    if(s2vFunction) s2vFunction->init(stage, config);
    if(glFunction) glFunction->init(stage, config);
  }

  void LeakageLine::initializeUsingXML(DOMElement * element) {
    RigidHLine::initializeUsingXML(element);
    DOMElement * e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"firstSurfaceVelocity");
    if (e) setSurface1VelocityFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"secondSurfaceVelocity");
    if (e) setSurface2VelocityFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"gapLength");
    if (e) setGapLengthFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, PlaneLeakageLine)

  void PlaneLeakageLine::setPlaneLeakagePressureLoss(PlaneLeakagePressureLoss * plpl) {
    lpl=plpl;
  }

  void PlaneLeakageLine::init(InitStage stage, const InitConfigSet &config) {
    if (stage==preInit) {  
      LeakageLine::init(stage, config);
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      Mlocal.resize(1, INIT, rho*length/hGap/wGap);
    }
    else
      LeakageLine::init(stage, config);
  }

  void PlaneLeakageLine::initializeUsingXML(DOMElement * element) {
    LeakageLine::initializeUsingXML(element);
    DOMElement * e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"width");
    setGapWidth(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"height");
    setGapHeight(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"planeLeakagePressureLoss");
    PlaneLeakagePressureLoss *p=MBSim::ObjectFactory::createAndInit<PlaneLeakagePressureLoss>(e->getFirstElementChild());
    setPlaneLeakagePressureLoss(p);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, CircularLeakageLine)

  void CircularLeakageLine::setCircularLeakagePressureLoss(CircularLeakagePressureLoss * clpl) {
    lpl=clpl;
    lpl->setParent(this);
    lpl->setName("lpl");
  }

  void CircularLeakageLine::init(InitStage stage, const InitConfigSet &config) {
    if (stage==preInit) {  
      rO=rI+hGap;
      LeakageLine::init(stage, config);
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      Mlocal.resize(1, INIT, rho*length/(M_PI*(rO*rO-rI*rI)));
    }
    else
      LeakageLine::init(stage, config);
    lpl->init(stage, config);
  }

  void CircularLeakageLine::initializeUsingXML(DOMElement * element) {
    LeakageLine::initializeUsingXML(element);
    DOMElement * e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"innerRadius");
    setInnerRadius(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"height");
    setGapHeight(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"circularLeakagePressureLoss");
    CircularLeakagePressureLoss *p=MBSim::ObjectFactory::createAndInit<CircularLeakagePressureLoss>(e->getFirstElementChild());
    setCircularLeakagePressureLoss(p);
  }

}

