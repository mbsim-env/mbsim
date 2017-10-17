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

  void DimensionlessLine::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if (dynamic_cast<RigidNode*>(nFrom) || dynamic_cast<RigidNodeMec*>(nFrom))
        THROW_MBSIMERROR("pFrom is of setValued type. not valid for dimensionless lines.");
      if (dynamic_cast<RigidNode*>(nTo) || dynamic_cast<RigidNodeMec*>(nTo))
        THROW_MBSIMERROR("pTo is of setValued type. not valid for dimensionless lines.");
    }
    else if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        plotColumns.push_back("Volume flow [l/min]");
        plotColumns.push_back("Mass flow [kg/min]");
      }
    }
    HLine::init(stage, config);
  }

  void DimensionlessLine::plot() {
    if(plotFeature[plotRecursive]) {
      plotVector.push_back(evalQIn()(0)*6e4);
      plotVector.push_back(getQIn()(0)*HydraulicEnvironment::getInstance()->getSpecificMass()*60.);
    }
    HLine::plot();
  }

  void DimensionlessLine::initializeUsingXML(DOMElement * element) {
    DOMElement * e;
    HLine::initializeUsingXML(element);
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"length");
    setLength(getDouble(e));
  }

  Leakage0DOF::~Leakage0DOF() {
    delete lpl;
    delete s1vFunction;
    delete s2vFunction;
    delete glFunction;
  }

  double Leakage0DOF::evalGapLength() const {
    return (glFunction)?(*glFunction)(getTime()):length;
  }

  double Leakage0DOF::evalSurface1Velocity() const {
    return (s1vFunction)?(*s1vFunction)(getTime()):0;
  }

  double Leakage0DOF::evalSurface2Velocity() const {
    return (s2vFunction)?(*s2vFunction)(getTime()):0;
  }

  void Leakage0DOF::init(InitStage stage, const InitConfigSet &config) {
    DimensionlessLine::init(stage, config);
    lpl->init(stage, config);
    if(s1vFunction) s1vFunction->init(stage, config);
    if(s2vFunction) s2vFunction->init(stage, config);
    if(glFunction) glFunction->init(stage, config);
  }

  void Leakage0DOF::updateQ() {
    QIn(0)=(*lpl)(nTo->evalGeneralizedForce()(0)-nFrom->evalGeneralizedForce()(0));
    QOut = -QIn;
    updQ = false;
  }

  void Leakage0DOF::initializeUsingXML(DOMElement * element) {
    DimensionlessLine::initializeUsingXML(element);
    DOMElement * e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"firstSurfaceVelocity");
    if (e) setSurface1VelocityFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"secondSurfaceVelocity");
    if (e) setSurface2VelocityFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"gapLength");
    if (e) setGapLengthFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, PlaneLeakage0DOF)

  void PlaneLeakage0DOF::setPlaneLeakagePressureLoss(PlaneLeakagePressureLoss * plpl) {
    lpl=plpl;
    lpl->setParent(this);
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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, CircularLeakage0DOF)

  void CircularLeakage0DOF::setCircularLeakagePressureLoss(CircularLeakagePressureLoss * clpl) {
    lpl=clpl;
    lpl->setParent(this);
    lpl->setLine(this);
  }

  void CircularLeakage0DOF::init(InitStage stage, const InitConfigSet &config) {
    if (stage==preInit) {  
      rO=rI+hGap;
      Leakage0DOF::init(stage, config);
    }
    else
      Leakage0DOF::init(stage, config);
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
