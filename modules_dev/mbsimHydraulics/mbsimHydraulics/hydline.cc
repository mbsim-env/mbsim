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

#include "hydline.h"
#include "hydnode.h"
#include "mbsim/object.h"
#include "hydline_closed.h"
#include "environment.h"
#include "pressure_loss.h"

#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  HydLineAbstract::HydLineAbstract(const string &name) : ObjectHydraulics(name) {
    setPlotFeature(state, enabled);
    setPlotFeature(stateDerivative, enabled);
    setPlotFeature(rightHandSide, enabled);
  }

  HydLineAbstract::~HydLineAbstract() {
  }

  void HydLineAbstract::setFromNode(HydNode * nFrom_) {
    nFrom=nFrom_;
  }

  void HydLineAbstract::setToNode(HydNode * nTo_) {
    nTo=nTo_;
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
    }
    else
      ObjectHydraulics::init(stage);
  }


  void HydLine::addPressureLoss(PressureLoss * dp_) {
    pd.push_back(dp_);
    if (dynamic_cast<PressureLossVar*>(dp_)) {
      assert(pdVar==NULL);
      pdVar=static_cast<PressureLossVar*>(dp_);
    }
  }

  void HydLine::init(InitStage stage) {
    if (stage==MBSim::unknownStage) {
      HydLineAbstract::init(stage);
      MFac=rho*l/Area;
      for (unsigned int i=0; i<pd.size(); i++)
        pd[i]->transferLineData(d, l);
    }
    else if(stage==MBSim::plot) {
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Fluidflow [l/min]");
        plotColumns.push_back("Massflow [kg/min]");
        plotColumns.push_back("p_Loss [bar]");
        for (unsigned int i=0; i<pd.size(); i++)
          pd[i]->initPlot(&plotColumns);
        HydLineAbstract::init(stage);
      }
    }
    else
      HydLineAbstract::init(stage);
  }

  void HydLine::updateh(double t) {
    /* A simple fix for updating  variable pressure losses*/
    if (pdVar)
      pdVar->updateg(t);

    pLossSum=0;
    for (unsigned int i=0; i<pd.size(); i++)
      pLossSum+=(*pd[i])(u(0));
    h(0)-=pLossSum;
  }

  void HydLine::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(u(0)*6e4);
      plotVector.push_back(u(0)*rho*60);
      plotVector.push_back(pLossSum*1e-5);
      for (unsigned int i=0; i<pd.size(); i++)
        pd[i]->plot(&plotVector);
      HydLineAbstract::plot(t, dt);
    }
  }


  void HydLineValveBilateral::init(InitStage stage) {
    if (stage==MBSim::preInit) {
      HydLine::init(stage);
      closed = new HydlineClosedBilateral(name+".Closed", this);
      parent->addLink(closed);
    }
    else if (stage==MBSim::unknownStage) {
      HydLine::init(stage);
      assert(pdVar);
    }
    else
      HydLine::init(stage);
  }


  void HydLineCheckvalveUnilateral::init(InitStage stage) {
    if (stage==MBSim::preInit) {
      HydLine::init(stage);
      closed = new HydlineClosedUnilateral(name+".Closed", this);
      parent->addLink(closed);
    }
  }

}

