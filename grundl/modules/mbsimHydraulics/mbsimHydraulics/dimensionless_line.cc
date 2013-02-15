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

#include "mbsimHydraulics/dimensionless_line.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/hnode_mec.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsim/dynamic_system.h"
#include "mbsimControl/signal_.h"
#include "mbsimHydraulics/obsolet_hint.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;

namespace MBSimHydraulics {

  void DimensionlessLine::init(InitStage stage) {
    if(stage==MBSim::preInit) {
      if (dynamic_cast<RigidNode*>(nFrom) || dynamic_cast<RigidNodeMec*>(nFrom))
        throw MBSimError("pFrom is of setValued type. not valid for dimensionless lines.");
      if (dynamic_cast<RigidNode*>(nTo) || dynamic_cast<RigidNodeMec*>(nTo))
        throw MBSimError("pTo is of setValued type. not valid for dimensionless lines.");
      HLine::init(stage);
    }
    else if(stage==MBSim::plot) {
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

  void DimensionlessLine::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e;
    HLine::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"length");
    setLength(getDouble(e));
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
    if (stage==MBSim::resolveXMLPath) {
      DimensionlessLine::init(stage);
      if (s1vPath!="")
        setSurface1VelocitySignal(getByPath<Signal>(process_signal_string(s1vPath)));
      if (s2vPath!="")
        setSurface2VelocitySignal(getByPath<Signal>(process_signal_string(s2vPath)));
      if (glPath!="")
        setGapLengthSignal(getByPath<Signal>(process_signal_string(glPath)));
    }
    else
      DimensionlessLine::init(stage);
  }

  void Leakage0DOF::updateStateDependentVariables(double t) {
    const double dp=nTo->getla()(0)-nFrom->getla()(0);
    Q(0)=(*lpl)(dp, this);
  }

  void Leakage0DOF::initializeUsingXML(TiXmlElement * element) {
    DimensionlessLine::initializeUsingXML(element);
    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"firstSurfaceVelocity");
    if (e)
      s1vPath=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"secondSurfaceVelocity");
    if (e)
      s2vPath=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"gapLength");
    if (e)
      glPath=e->Attribute("ref");
  }


  void PlaneLeakage0DOF::setPlaneLeakagePressureLoss(PlaneLeakagePressureLoss * plpl) {
    lpl=plpl;
  }

  void PlaneLeakage0DOF::initializeUsingXML(TiXmlElement * element) {
    Leakage0DOF::initializeUsingXML(element);
    TiXmlElement * e = element->FirstChildElement(MBSIMHYDRAULICSNS"width");
    setGapWidth(getDouble(e));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"height");
    setGapHeight(getDouble(e));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"planeLeakagePressureLoss");
    PlaneLeakagePressureLoss *p=(PlaneLeakagePressureLoss*)(MBSim::ObjectFactory::getInstance()->createFunction1_SS(e->FirstChildElement()));
    setPlaneLeakagePressureLoss(p);
    p->initializeUsingXML(e->FirstChildElement());
  }


  void CircularLeakage0DOF::setCircularLeakagePressureLoss(CircularLeakagePressureLoss * clpl) {
    lpl=clpl;
  }

  void CircularLeakage0DOF::init(InitStage stage) {
    if (stage==MBSim::preInit) {  
      rO=rI+hGap;
      Leakage0DOF::init(stage);
    }
    else
      Leakage0DOF::init(stage);
  }

  void CircularLeakage0DOF::initializeUsingXML(TiXmlElement * element) {
    Leakage0DOF::initializeUsingXML(element);
    TiXmlElement * e = element->FirstChildElement(MBSIMHYDRAULICSNS"innerRadius");
    setInnerRadius(getDouble(e));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"height");
    setGapHeight(getDouble(e));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"circularLeakagePressureLoss");
    CircularLeakagePressureLoss *p=(CircularLeakagePressureLoss*)(MBSim::ObjectFactory::getInstance()->createFunction1_SS(e->FirstChildElement()));
    setCircularLeakagePressureLoss(p);
    p->initializeUsingXML(e->FirstChildElement());
  }

}
