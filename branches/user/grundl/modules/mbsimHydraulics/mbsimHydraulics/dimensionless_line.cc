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
#include "mbsimHydraulics/dimensionless_line.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/hnode_mec.h"
#include "mbsimHydraulics/environment.h"
#include "mbsim/dynamic_system.h"
#include "mbsimControl/signal_.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimHydraulics {

  void DimensionlessLine::init(InitStage stage) {
    if(stage==preInit) {
      if (dynamic_cast<RigidNode*>(nFrom) || dynamic_cast<RigidNodeMec*>(nFrom))
        throw MBSimError("pFrom is of setValued type. not valid for dimensionless lines.");
      if (dynamic_cast<RigidNode*>(nTo) || dynamic_cast<RigidNodeMec*>(nTo))
        throw MBSimError("pTo is of setValued type. not valid for dimensionless lines.");
      HLine::init(stage);
    }
    else if(stage==plotting) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Volume flow [l/min]");
        plotColumns.push_back("Mass flow [kg/min]");
        HLine::init(stage);
      }
    }
    else
      HLine::init(stage);
  }

  void DimensionlessLine::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(Q(0)*6e4);
      plotVector.push_back(Q(0)*HydraulicEnvironment::getInstance()->getSpecificMass()*60.);
      HLine::plot(t, dt);
    }
  }

  void DimensionlessLine::initializeUsingXML(DOMElement * element) {
    DOMElement * e;
    HLine::initializeUsingXML(element);
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"length");
    setLength(getDouble(e));
  }

  Leakage0DOF::~Leakage0DOF() {
    delete lpl;
  }

  double Leakage0DOF::getGapLength() const {
    return ((glSignal)?glSignal->getSignal()(0):length);
  }

  double Leakage0DOF::getSurface1Velocity() const {
    return ((s1vSignal)?s1vSignal->getSignal()(0):0);
  }

  double Leakage0DOF::getSurface2Velocity() const {
    return ((s2vSignal)?s2vSignal->getSignal()(0):0);
  }

  void Leakage0DOF::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      DimensionlessLine::init(stage);
      if (s1vPath!="")
        setSurface1VelocitySignal(getByPath<Signal>(s1vPath));
      if (s2vPath!="")
        setSurface2VelocitySignal(getByPath<Signal>(s2vPath));
      if (glPath!="")
        setGapLengthSignal(getByPath<Signal>(glPath));
    }
    else
      DimensionlessLine::init(stage);
  }

  void Leakage0DOF::updateStateDependentVariables(double t) {
    const double dp=nTo->getla()(0)-nFrom->getla()(0);
    Q(0)=(*lpl)(dp);
  }

  void Leakage0DOF::initializeUsingXML(DOMElement * element) {
    DimensionlessLine::initializeUsingXML(element);
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

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PlaneLeakage0DOF,  MBSIMHYDRAULICS%"PlaneLeakage0DOF")

  void PlaneLeakage0DOF::setPlaneLeakagePressureLoss(PlaneLeakagePressureLoss * plpl) {
    lpl=plpl;
    lpl->setLine(this);
  }

  void PlaneLeakage0DOF::initializeUsingXML(DOMElement * element) {
    Leakage0DOF::initializeUsingXML(element);
    DOMElement * e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"width");
    setGapWidth(getDouble(e));
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"height");
    setGapHeight(getDouble(e));
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"planeLeakagePressureLoss");
    PlaneLeakagePressureLoss *p=MBSim::ObjectFactory::createAndInit<PlaneLeakagePressureLoss>(e->getFirstElementChild());
    setPlaneLeakagePressureLoss(p);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(CircularLeakage0DOF,  MBSIMHYDRAULICS%"CircularLeakage0DOF")

  void CircularLeakage0DOF::setCircularLeakagePressureLoss(CircularLeakagePressureLoss * clpl) {
    lpl=clpl;
    lpl->setLine(this);
  }

  void CircularLeakage0DOF::init(InitStage stage) {
    if (stage==preInit) {  
      rO=rI+hGap;
      Leakage0DOF::init(stage);
    }
    else
      Leakage0DOF::init(stage);
  }

  void CircularLeakage0DOF::initializeUsingXML(DOMElement * element) {
    Leakage0DOF::initializeUsingXML(element);
    DOMElement * e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"innerRadius");
    setInnerRadius(getDouble(e));
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"height");
    setGapHeight(getDouble(e));
    e = E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"circularLeakagePressureLoss");
    CircularLeakagePressureLoss *p=MBSim::ObjectFactory::createAndInit<CircularLeakagePressureLoss>(e->getFirstElementChild());
    setCircularLeakagePressureLoss(p);
  }

}
