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
 * Contact: schneidm@users.berlios.de
 */

#include "mbsimHydraulics/leakage_line.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/rigid_line_pressureloss.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimControl/signal_.h"
#include "mbsimHydraulics/obsolet_hint.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;

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
        setSurface1VelocitySignal(getByPath<Signal>(process_signal_string(s1vPath)));
      if (s2vPath!="")
        setSurface2VelocitySignal(getByPath<Signal>(process_signal_string(s2vPath)));
      if (glPath!="")
        setGapLengthSignal(getByPath<Signal>(process_signal_string(glPath)));
    }
    else if (stage==MBSim::modelBuildup) {
      parent->addLink(new RigidLinePressureLoss(name+"/LeakagePressureLoss", this, lpl, false, false));
      RigidHLine::init(stage);
    }
    else
      RigidHLine::init(stage);
  }

  void LeakageLine::initializeUsingXML(TiXmlElement * element) {
    RigidHLine::initializeUsingXML(element);
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

  void PlaneLeakageLine::initializeUsingXML(TiXmlElement * element) {
    LeakageLine::initializeUsingXML(element);
    TiXmlElement * e = element->FirstChildElement(MBSIMHYDRAULICSNS"width");
    setGapWidth(getDouble(e));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"height");
    setGapHeight(getDouble(e));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"planeLeakagePressureLoss");
    PlaneLeakagePressureLoss *p=(PlaneLeakagePressureLoss*)(MBSim::ObjectFactory::getInstance()->createFunction1_SS(e->FirstChildElement()));
    setPlaneLeakagePressureLoss(p);
    p->initializeUsingXML(e->FirstChildElement());
  }


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

  void CircularLeakageLine::initializeUsingXML(TiXmlElement * element) {
    LeakageLine::initializeUsingXML(element);
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

