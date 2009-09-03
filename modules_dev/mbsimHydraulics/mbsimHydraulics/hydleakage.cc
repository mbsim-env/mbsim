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

#include "mbsimHydraulics/hydleakage.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimHydraulics/pressure_loss.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  void PlaneLeakage::init(InitStage stage) {
    if (stage==MBSim::preInit) {  
      RigidHLine::init(stage);
      Mlocal.resize(1, INIT, rho*length/hGap/wGap);
    }
    else
      RigidHLine::init(stage);
  }

  void PlaneLeakage::addPressureLoss(PlaneLeakagePressureLoss * dp) {
    RigidHLine::addPressureLoss(dp);
  }

  void PlaneLeakage::initializeUsingXML(TiXmlElement * element) {
    RigidHLine::initializeUsingXML(element);
    TiXmlElement * e = element->FirstChildElement(MBSIMHYDRAULICSNS"width");
    setGapWidth(atof(e->GetText()));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"height");
    setGapHeight(atof(e->GetText()));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"pressureLoss");
    while (e && e->ValueStr()==MBSIMHYDRAULICSNS"pressureLoss") {
      // TODO Ist das so richtig mit dem factory-cast?
      PressureLoss *p=((HydraulicsObjectFactory*)(ObjectFactory::getInstance()))->createPressureLoss(e->FirstChildElement());
      addPressureLoss(static_cast<PlaneLeakagePressureLoss*>(p));
      p->initializeUsingXML(e->FirstChildElement());
      e=e->NextSiblingElement();
    }
  }


  void CircularLeakage::init(InitStage stage) {
    if (stage==MBSim::preInit) {  
      rO=rI+hGap;
      RigidHLine::init(stage);
      Mlocal.resize(1, INIT, rho*length/(M_PI*(rO*rO-rI*rI)));
    }
    else
      RigidHLine::init(stage);
  }

  void CircularLeakage::addPressureLoss(CircularLeakagePressureLoss * dp) {
    RigidHLine::addPressureLoss(dp);
  }

  void CircularLeakage::initializeUsingXML(TiXmlElement * element) {
    RigidHLine::initializeUsingXML(element);
    TiXmlElement * e = element->FirstChildElement(MBSIMHYDRAULICSNS"innerRadius");
    setInnerRadius(atof(e->GetText()));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"height");
    setGapHeight(atof(e->GetText()));
    e = element->FirstChildElement(MBSIMHYDRAULICSNS"pressureLoss");
    while (e && e->ValueStr()==MBSIMHYDRAULICSNS"pressureLoss") {
      // TODO Ist das so richtig mit dem factory-cast?
      PressureLoss *p=((HydraulicsObjectFactory*)(ObjectFactory::getInstance()))->createPressureLoss(e->FirstChildElement());
      addPressureLoss(static_cast<CircularLeakagePressureLoss*>(p));
      p->initializeUsingXML(e->FirstChildElement());
      e=e->NextSiblingElement();
    }
  }

}

