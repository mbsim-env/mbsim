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

#include "mbsim/userfunction.h"

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

  void HydLineAbstract::init() {
    ObjectHydraulics::init();

    if (!nFrom || !nTo || (nFrom==nTo))
      cout << getName() << ": Fehler!" << endl;
    assert(nFrom!=NULL);
    assert(nTo!=NULL);
    assert(nFrom!=nTo);

    Area=M_PI*d*d/4.;
    rho=HydraulicEnvironment::getInstance()->getRho();
  }


  HydLine::HydLine(const string &name) : HydLineAbstract(name), pLossSum(1), pdVarArea(NULL) {
  }

  HydLine::~HydLine() {
  }

  void HydLine::addPressureLoss(PressureLoss * dp_) {
    pd.push_back(dp_);
    if (dynamic_cast<PressureLossZetaVarArea*>(dp_)) {
      assert(pdVarArea==NULL);
      pdVarArea=static_cast<PressureLossZetaVarArea*>(dp_);
    }
  }


  void HydLine::init() {
    HydLineAbstract::init();
    MFac=rho*l/Area;
    for (unsigned int i=0; i<pd.size(); i++)
      pd[i]->transferLineData(d, l);
  }

  void HydLine::calcqSize() {
    qSize=0;
  }

  void HydLine::calcuSize(int j) {
    uSize[j]=1;
  }
  
  void HydLine::updateM(double t) {
    M(0,0)=MFac;
  }

  void HydLine::updateStateDependentVariables(double t) {
    HydLineAbstract::updateStateDependentVariables(t);
    if (pdVarArea)
      pdVarArea->updateRelativeArea(t);
  }

  void HydLine::updateh(double t) {
    pLossSum.init(0);
    for (unsigned int i=0; i<pd.size(); i++)
      pLossSum+=(*pd[i])(u(0));
    h-=pLossSum;
  }

  void HydLine::initPlot() {
    plotColumns.push_back("Fluidflow [l/min]");
    plotColumns.push_back("Massflow [kg/min]");
    plotColumns.push_back("p_Loss [bar]");
    for (unsigned int i=0; i<pd.size(); i++)
      pd[i]->initPlot(&plotColumns);
    HydLineAbstract::initPlot();
  }

  void HydLine::plot(double t, double dt) {
    plotVector.push_back(u(0)*6e4);
    plotVector.push_back(u(0)*rho*60);
    plotVector.push_back(pLossSum(0)*1e-5);
    for (unsigned int i=0; i<pd.size(); i++)
      pd[i]->plot(&plotVector);
    HydLineAbstract::plot(t, dt);
  }


  HydLineValve::HydLineValve(const string &name) : HydLine(name) {
  }


  HydLineValveBilateral::HydLineValveBilateral(const string &name) : HydLineValve(name) {
  }

  void HydLineValveBilateral::preinit() {
    HydLine::preinit();
    closed = new HydlineClosedBilateral(name+".Closed", this);
    parent->addLink(closed);
  }

  void HydLineValveBilateral::init() {
    HydLine::init();
    assert(pdVarArea);
  }

  void HydLineValveBilateral::updateStateDependentVariables(double t) {
    HydLine::updateStateDependentVariables(t);
    closed->setStatus(pdVarArea->isClosed());
  }


  HydLineCheckvalveUnilateral::HydLineCheckvalveUnilateral(const string &name) : HydLineValve(name) {
  }

  void HydLineCheckvalveUnilateral::preinit() {
    HydLine::preinit();
    closed = new HydlineClosedUnilateral(name+".Closed", this);
    parent->addLink(closed);
  }

  void HydLineCheckvalveUnilateral::updateStateDependentVariables(double t) {
    HydLine::updateStateDependentVariables(t);
    closed->setStatus((u(0)<=0));
  }


  PressureLoss::PressureLoss(const string &name_) : name(name_), pLoss(1) {
  }

  void PressureLoss::initPlot(vector<string>* plotColumns) {
    plotColumns->push_back(name+"_pLoss [bar]");
  }

  void PressureLoss::plot(vector<double>* plotVector) {
    plotVector->push_back(pLoss(0)*1e-5);
  }


  PressureLossZeta::PressureLossZeta(const string &name, double zeta) : PressureLoss(name) {
    zetaFac=zeta;
  }

  void PressureLossZeta::transferLineData(double d, double l) {
    double area=M_PI*d*d/4.;
    zetaFac*=HydraulicEnvironment::getInstance()->getRho()/2./area/area;
  }

  Vec PressureLossZeta::operator()(double Q){
    return pLoss.init(zetaFac*Q*fabs(Q));
  }


  PressureLossZetaVarArea::PressureLossZetaVarArea(const string &name, double zeta, UserFunction * relAreaFun_, double minRelArea_) : PressureLoss(name) {
    zetaFac=zeta;
    relAreaFun=relAreaFun_;
    minRelArea=minRelArea_;
    updateRelativeArea(0);
  }

  void PressureLossZetaVarArea::transferLineData(double d, double l) {
    double area=M_PI*d*d/4.;
    zetaFac*=HydraulicEnvironment::getInstance()->getRho()/2./area/area;
  }

  void PressureLossZetaVarArea::updateRelativeArea(double t) {
    relArea = (*relAreaFun)(t)(0);
    closed = (relArea<minRelArea);
    relArea = (closed ? minRelArea : relArea);
  }

  void PressureLossZetaVarArea::initPlot(vector<string>* plotColumns) {
    PressureLoss::initPlot(plotColumns);
    plotColumns->push_back(name+"_relArea");
    plotColumns->push_back(name+"_closed");
  }

  void PressureLossZetaVarArea::plot(vector<double>* plotVector) {
    PressureLoss::plot(plotVector);
    plotVector->push_back(relArea);
    plotVector->push_back(closed);
  }
  
  Vec PressureLossZetaVarArea::operator()(double Q){
    return pLoss.init(zetaFac*Q*fabs(Q)/relArea/relArea);
  }


  PressureLossLaminarTubeFlow::PressureLossLaminarTubeFlow(const string &name) : PressureLoss(name) {
  }

  void PressureLossLaminarTubeFlow::transferLineData(double d, double l) {
    double area=M_PI*d*d/4.;
    lossFactor=32.*HydraulicEnvironment::getInstance()->getEta()*l/d/d/area;
    cout << "formelchecken " << endl;
  }

  Vec PressureLossLaminarTubeFlow::operator()(double Q) {
    return pLoss.init(lossFactor * Q);
  }
}

