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

#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/hydnode.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/hydline_pressureloss.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsim/object.h"
#include "mbsim/frame.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  void RigidLine::addPressureLoss(LinePressureLoss * dp) {
    RigidHLine::addPressureLoss(dp); 
  }

  void RigidLine::initializeUsingXML(TiXmlElement * element) {
    RigidHLine::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"diameter");
    setDiameter(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"pressureLoss");
    while (e && e->ValueStr()==MBSIMHYDRAULICSNS"pressureLoss") {
      // TODO Ist das so richtig mit dem factory-cast?
      PressureLoss *p=((HydraulicsObjectFactory*)(ObjectFactory::getInstance()))->createPressureLoss(e->FirstChildElement());
      addPressureLoss(static_cast<LinePressureLoss*>(p));
      p->initializeUsingXML(e->FirstChildElement());
      e=e->NextSiblingElement();
    }
  }

  void RigidLine::init(InitStage stage) {
    if (stage==MBSim::unknownStage) {
      RigidHLine::init(stage);
      double area=M_PI*diameter*diameter/4.;
      Mlocal.resize(1, INIT, rho*length/area);
      cout << "Mlocal=" << Mlocal << endl;
    }
    else
      RigidHLine::init(stage);
  }

}

