
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

#include "mbsimHydraulics/hline.h"
#include "mbsimHydraulics/hydline_pressureloss.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/objectfactory.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  void HLine::init(InitStage stage) {
    if (stage==MBSim::preInit) {
      Object::init(stage);
      if (!nFrom) { cerr<<"ERROR! HLine \""<<name<<"\" has no fromNode!"<<endl; _exit(1); }
      if (!nTo) { cerr<<"ERROR! HLine \""<<name<<"\" has no toNode!"<<endl; _exit(1); }
      if (nFrom==nTo) { cerr<<"ERROR! HLine \""<<name<<"\": fromNode and toNode are the same!"<<endl; _exit(1); }
      rho=HydraulicEnvironment::getInstance()->getSpecificMass();
    }
    else
      Object::init(stage);
  }

  void HLine::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e;
    Object::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"direction");
    setDirection(getVec(e,3));
  }
  
  
  void RigidHLine::addPressureLoss(PressureLoss * dp) {
    if(!parent) {
      cerr << "RigidHLine \"" << name << "\" has to be added to a group before adding a PressureLoss!" << endl;
      throw(123);
    }
    if (pressureLoss==NULL) {
      pressureLoss=new HydlinePressureloss(name+"/PressureLoss", this);
      parent->addLink(pressureLoss);
    }
    pressureLoss->addPressureLoss(dp);
    if (dp->isUnilateral()) {
      HydlinePressureloss * hlpl = new HydlinePressureloss(name+"/UnilateralPressureLoss "+dp->getName(), this);
      parent->addLink(hlpl);
      hlpl->addPressureLoss(dp);
      hlpl->setUnilateral(true);
    }
    else if (dp->isBilateral()) {
      HydlinePressureloss * hlpl = new HydlinePressureloss(name+"/BilateralPressureLoss "+dp->getName(), this);
      parent->addLink(hlpl);
      hlpl->addPressureLoss(dp);
      hlpl->setBilateral(true);
    }
  }
  
  void RigidHLine::updateh(double t) {
    pressureLossGravity=trans(frameOfReference->getOrientation()*MBSimEnvironment::getInstance()->getAccelerationOfGravity())*direction*rho*length;
    h(0)=pressureLossGravity;
  }

  void RigidHLine::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Fluidflow [l/min]");
        plotColumns.push_back("Massflow [kg/min]");
        plotColumns.push_back("pressureLoss due to gravity [bar]");
        HLine::init(stage);
      }
    }
    else
      HLine::init(stage);
  }
  
  void RigidHLine::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(u(0)*6e4);
      plotVector.push_back(u(0)*rho*60.);
      plotVector.push_back(pressureLossGravity*1e-5);
      HLine::plot(t, dt);
    }
  }

  void RigidHLine::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e;
    HLine::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"length");
    setLength(getDouble(e));
  }

}
