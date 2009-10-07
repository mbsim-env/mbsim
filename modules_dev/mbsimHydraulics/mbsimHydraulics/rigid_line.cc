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

#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/rigid_line_pressureloss.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsim/object.h"
#include "mbsim/frame.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsimControl/signal_.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimHydraulics {

  void RigidLine::init(InitStage stage) {
    if (stage==MBSim::modelBuildup) {
      if (pL)
        parent->addLink(new RigidLinePressureLoss(name+"/LinePressureLoss", this, pL, false,false));
      RigidHLine::init(stage);
    }
    else if (stage==MBSim::unknownStage) {
      RigidHLine::init(stage);
      double area=M_PI*diameter*diameter/4.;
      Mlocal.resize(1, INIT, HydraulicEnvironment::getInstance()->getSpecificMass()*length/area);
    }
    else
      RigidHLine::init(stage);
  }

  void RigidLine::initializeUsingXML(TiXmlElement * element) {
    RigidHLine::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"diameter");
    setDiameter(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"linePressureLoss");
    LinePressureLoss *p=(LinePressureLoss*)(MBSim::ObjectFactory::getInstance()->createFunction1_SS(e->FirstChildElement()));
    setLinePressureLoss(p);
    p->initializeUsingXML(e->FirstChildElement());
  }


  bool ClosableRigidLine::isClosed() const {
   return (cpLSignal->getSignal()(0)<cpLMinValue);
  }

  double ClosableRigidLine::getRegularizedValue() const {
    return isClosed()?cpLMinValue:cpLSignal->getSignal()(0);
  }

  void ClosableRigidLine::init(InitStage stage) {
    if (stage==MBSim::modelBuildup) {
      if (cpL)
        parent->addLink(new RigidLinePressureLoss(name+"/ClosablePressureLoss", this, cpL, false,false));
      assert(!(cpLUnilateral && cpLBilateral));
      if (cpLUnilateral)
        parent->addLink(new RigidLinePressureLoss(name+"/UnilateralClosablePressureLoss", this, cpL, false,true));
      if (cpLBilateral)
        parent->addLink(new RigidLinePressureLoss(name+"/BilateralClosablePressureLoss", this, cpL, true, false));

      RigidLine::init(stage);
    }
    else
      RigidLine::init(stage);
  }

  void ClosableRigidLine::initializeUsingXML(TiXmlElement * element) {
    RigidLine::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"closableLinePressureLoss");
    ClosablePressureLoss *p=(ClosablePressureLoss*)(MBSim::ObjectFactory::getInstance()->createFunction1_SS(e->FirstChildElement()));
    setClosablePressureLoss(p);
    p->initializeUsingXML(e->FirstChildElement());
  }
}

