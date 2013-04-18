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
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimHydraulics/obsolet_hint.h"
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
        ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"/LinePressureLoss", this, pL, false,false));
      RigidHLine::init(stage);
    }
    else if (stage==MBSim::plot) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Reynolds number [-]");
        RigidHLine::init(stage);
      }
    }
    else if (stage==MBSim::unknownStage) {
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

  void RigidLine::initializeUsingXML(TiXmlElement * element) {
    RigidHLine::initializeUsingXML(element);
    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"diameter");
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
    if (stage==MBSim::resolveXMLPath) {
      if(refSignalString!="")
        setSignal(getByPath<MBSimControl::Signal>(process_signal_string(refSignalString)));
      RigidLine::init(stage);
    }
    else if (stage==MBSim::modelBuildup) {
      if (cpLBilateral)
         ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"/BilateralClosablePressureLoss", this, cpL, true, false));
     else
        ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"/ClosablePressureLoss", this, cpL, false,false));

      RigidLine::init(stage);
    }
    else
      RigidLine::init(stage);
  }

  void ClosableRigidLine::initializeUsingXML(TiXmlElement * element) {
    RigidLine::initializeUsingXML(element);
    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"closablePressureLoss");
    TiXmlElement * ee=e->FirstChildElement();
    ClosablePressureLoss *p=(ClosablePressureLoss*)(MBSim::ObjectFactory::getInstance()->createFunction1_SS(ee));
    setClosablePressureLoss(p);
    p->initializeUsingXML(ee);
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"checksizeSignal");
    refSignalString=ee->Attribute("ref");
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"minimalChecksizeValue");
    setMinimalValue(getDouble(ee));
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"setValued");
    if (ee)
      setBilateral(true);
  }


  void UnidirectionalRigidLine::init(InitStage stage) {
    if (stage==MBSim::modelBuildup) {
      if (upL)
        ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"/RegularizedUnidirectionalPressureLoss", this, upL, false,false));
      else
        ((DynamicSystem*)parent)->addLink(new RigidLinePressureLoss(name+"/UnilateralUnidirectionalPressureLoss", this, NULL, false, true));
      RigidLine::init(stage);
    }
    else
      RigidLine::init(stage);
  }

  void UnidirectionalRigidLine::initializeUsingXML(TiXmlElement * element) {
    RigidLine::initializeUsingXML(element);
    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"unidirectionalPressureLoss");
    TiXmlElement * ee=e->FirstChildElement();
    UnidirectionalPressureLoss *p=(UnidirectionalPressureLoss*)(MBSim::ObjectFactory::getInstance()->createFunction1_SS(ee));
    if (p) {
      setUnidirectionalPressureLoss(p);
      p->initializeUsingXML(ee);
    }
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"minimalPressureDrop");
    setMinimalPressureDrop(getDouble(ee));
  }

}

