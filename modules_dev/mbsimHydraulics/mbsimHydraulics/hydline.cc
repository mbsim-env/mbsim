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
#include "mbsim/object.h"
#include "mbsim/frame.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  HydLineAbstract::HydLineAbstract(const string &name) : ObjectHydraulics(name) , nFrom(NULL), nTo(NULL), d(0), l(0), Area(0), rho(0), direction(0), frameOfReference(NULL) {
  }

  void HydLineAbstract::init(InitStage stage) {
    if (stage==MBSim::preInit) {
      ObjectHydraulics::init(stage);
      if (!nFrom || !nTo || (nFrom==nTo))
        cout << getName() << ": Fehler!" << endl;
      assert(nFrom!=NULL);
      assert(nTo!=NULL);
      assert(nFrom!=nTo);

      Area=M_PI*d*d/4.;
      rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      if (!frameOfReference)
        frameOfReference=parent->getFrame("I");
    }
    else
      ObjectHydraulics::init(stage);
  }


  void HydLine::addPressureLoss(PressureLoss * dp) {
    if (pressureLoss==NULL)
      pressureLoss=new HydlinePressureloss(name+"/PressureLoss", this);
    pressureLoss->addPressureLoss(dp);
    if (dp->isUnilateral()) {
      setvaluedPressureLoss.push_back(new HydlinePressureloss(name+"/UnilateralPressureLoss "+dp->getName(), this));
      setvaluedPressureLoss.back()->addPressureLoss(dp);
      setvaluedPressureLoss.back()->setUnilateral(true);
    }
    else if (dp->isBilateral()) {
      setvaluedPressureLoss.push_back(new HydlinePressureloss(name+"/BilateralPressureLoss "+dp->getName(), this));
      setvaluedPressureLoss.back()->addPressureLoss(dp);
      setvaluedPressureLoss.back()->setBilateral(true);
    }
  }

  void HydLine::init(InitStage stage) {
    if (stage==MBSim::preInit) {
      if (pressureLoss)
        parent->addLink(pressureLoss);
      for (unsigned int i=0; i<setvaluedPressureLoss.size(); i++)
        parent->addLink(setvaluedPressureLoss[i]);
      HydLineAbstract::init(stage);
    }
    else if (stage==MBSim::unknownStage) {
      HydLineAbstract::init(stage);
      MFac=rho*l/Area;
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Fluidflow [l/min]");
        plotColumns.push_back("Massflow [kg/min]");
        if (direction.size()>0)
          plotColumns.push_back("pressureLoss due to gravity [bar]");
        HydLineAbstract::init(stage);
      }
    }
    else
      HydLineAbstract::init(stage);
  }

  void HydLine::updateh(double t) {
    if (direction.size()>0)
      pressureLossGravity=trans(frameOfReference->getOrientation()*MBSimEnvironment::getInstance()->getAccelerationOfGravity())*direction*rho*l;
    h(0)=pressureLossGravity;
  }

  void HydLine::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(u(0)*6e4);
      plotVector.push_back(u(0)*rho*60.);
      if (direction.size()>0)
        plotVector.push_back(pressureLossGravity*1e-5);
      HydLineAbstract::plot(t, dt);
    }
  }

}

