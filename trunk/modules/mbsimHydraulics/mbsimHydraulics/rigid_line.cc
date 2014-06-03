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
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/rigid_line_pressureloss.h"
#include "mbsim/object.h"
#include "mbsim/frame.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsimControl/signal_.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimHydraulics {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(RigidLine,  MBSIMHYDRAULICS%"RigidLine")

  RigidLine::~RigidLine() { 
    delete pL; 
  }

  void RigidLine::init(InitStage stage) {
    if (stage==modelBuildup) {
      if (pL)
        ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"/LinePressureLoss", this, pL, false,false));
      RigidHLine::init(stage);
    }
    else if (stage==plotting) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Reynolds number [-]");
        RigidHLine::init(stage);
      }
    }
    else if (stage==unknownStage) {
      RigidHLine::init(stage);
      double area=M_PI*diameter*diameter/4.;
      Mlocal.resize(1, INIT, HydraulicEnvironment::getInstance()->getSpecificMass()*length/area);
      double nu=HydraulicEnvironment::getInstance()->getKinematicViscosity();
      ReynoldsFactor=diameter/nu/area;
    }
    else
      RigidHLine::init(stage);
  }
  
  void RigidLine::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(fabs(Q(0))*ReynoldsFactor);
      RigidHLine::plot(t, dt);
    }
  }

  void RigidLine::initializeUsingXML(DOMElement * element) {
    RigidHLine::initializeUsingXML(element);
    DOMElement * e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"diameter");
    setDiameter(getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"linePressureLoss");
    LinePressureLoss *p=MBSim::ObjectFactory::createAndInit<LinePressureLoss>(e->getFirstElementChild());
    setLinePressureLoss(p);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ClosableRigidLine,  MBSIMHYDRAULICS%"ClosableRigidLine")

  ClosableRigidLine::~ClosableRigidLine() {
    delete cpL;
  }

  bool ClosableRigidLine::isClosed() const {
   return (cpLSignal->getSignal()(0)<cpLMinValue);
  }

  double ClosableRigidLine::getRegularizedValue() const {
    return isClosed()?cpLMinValue:cpLSignal->getSignal()(0);
  }

  void ClosableRigidLine::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if(refSignalString!="")
        setSignal(getByPath<MBSimControl::Signal>(refSignalString));
      RigidLine::init(stage);
    }
    else if (stage==modelBuildup) {
      if (cpLBilateral)
         ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"/BilateralClosablePressureLoss", this, cpL, true, false));
     else
        ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"/ClosablePressureLoss", this, cpL, false,false));

      RigidLine::init(stage);
    }
    else
      RigidLine::init(stage);
  }

  void ClosableRigidLine::initializeUsingXML(DOMElement * element) {
    RigidLine::initializeUsingXML(element);
    DOMElement * e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"closablePressureLoss");
    DOMElement * ee=e->getFirstElementChild();
    ClosablePressureLoss *p=MBSim::ObjectFactory::createAndInit<ClosablePressureLoss>(ee);
    setClosablePressureLoss(p);
    ee=E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"checksizeSignal");
    refSignalString=E(ee)->getAttribute("ref");
    ee=E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"minimalChecksizeValue");
    setMinimalValue(getDouble(ee));
    ee=E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"setValued");
    if (ee)
      setBilateral(true);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(UnidirectionalRigidLine,  MBSIMHYDRAULICS%"UnidirectionalRigidLine")

  void UnidirectionalRigidLine::init(InitStage stage) {
    if (stage==modelBuildup) {
      if (upL)
        ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"/RegularizedUnidirectionalPressureLoss", this, upL, false,false));
      else
        ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"/UnilateralUnidirectionalPressureLoss", this, NULL, false, true));
      RigidLine::init(stage);
    }
    else
      RigidLine::init(stage);
  }

  void UnidirectionalRigidLine::initializeUsingXML(DOMElement * element) {
    RigidLine::initializeUsingXML(element);
    DOMElement * e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"unidirectionalPressureLoss");
    DOMElement * ee=e->getFirstElementChild();
    UnidirectionalPressureLoss *p=MBSim::ObjectFactory::createAndInit<UnidirectionalPressureLoss>(ee);
    if (p) {
      setUnidirectionalPressureLoss(p);
    }
    ee=E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"minimalPressureDrop");
    setMinimalPressureDrop(getDouble(ee));
  }

}

