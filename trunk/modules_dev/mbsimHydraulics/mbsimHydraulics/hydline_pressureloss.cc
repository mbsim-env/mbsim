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

#include "mbsimHydraulics/hydline_pressureloss.h"
#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/nonsmooth_algebra.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  HydlinePressureloss::HydlinePressureloss(const string &name, RigidLine * line_) : Link(name), line(line_), isActive0(false), unilateral(false), bilateral(false), active(false) {
  }

  void HydlinePressureloss::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if (isSetValued()) {
        plotVector.push_back(gd(0));
        plotVector.push_back(active);
        plotVector.push_back(gdn(0));
        plotVector.push_back(la(0));
        plotVector.push_back(rFactor(0));
      }
      else {
        for (unsigned int i=0; i<pressureLosses.size(); i++)
          pressureLosses[i]->plot(&plotVector);
        for (unsigned int i=0; i<variablePressureLoss.size(); i++)
          variablePressureLoss[i]->plot(&plotVector);
      }
      Link::plot(t, dt);
    }
  }

  void HydlinePressureloss::addPressureLoss(PressureLoss * loss) {
    loss->setHydlinePressureloss(this);
    if (dynamic_cast<VariablePressureLoss*>(loss))
      variablePressureLoss.push_back(static_cast<VariablePressureLoss*>(loss));
    else
      pressureLosses.push_back(loss);
  }

  void HydlinePressureloss::init(InitStage stage) {
    if (stage==MBSim::preInit) {
      Link::init(stage);
      if (isSetValued()) {
        isActive0=true;
        active=true;
      }
    }
    if (stage==MBSim::resize) {
      Link::init(stage);
      int j=1;
      W.push_back(Mat(j, 1));
      V.push_back(Mat(j, 1));
      h.push_back(Vec(j));
      hLink.push_back(Vec(j));
      dhdq.push_back(Mat(j, 0));
      dhdu.push_back(SqrMat(j));
      dhdt.push_back(Vec(j));
      r.push_back(Vec(j));
      gd.resize(1);
      gdn.resize(1);
      la.resize(1);
    }
    else if (stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
        if (isSetValued()) {
          plotColumns.push_back("gd");
          plotColumns.push_back("active");
          plotColumns.push_back("gdn");
          plotColumns.push_back("la(0)");
          plotColumns.push_back("rFactor(0)");
        }
        else {
          for (unsigned int i=0; i<pressureLosses.size(); i++)
            pressureLosses[i]->initPlot(&plotColumns);
          for (unsigned int i=0; i<variablePressureLoss.size(); i++)
            variablePressureLoss[i]->initPlot(&plotColumns);
        }
        Link::init(stage);
      }
    }
    else if (stage==MBSim::unknownStage) {
      gdTol*=1e-6;
      if (!isSetValued()) {
        for (unsigned int i=0; i<pressureLosses.size(); i++)
          pressureLosses[i]->transferLineData(line->getDiameter(), line->getLength());
        for (unsigned int i=0; i<variablePressureLoss.size(); i++)
          variablePressureLoss[i]->transferLineData(line->getDiameter(), line->getLength());
      }
      Link::init(stage);
    }
    else
      Link::init(stage);
  }

  void HydlinePressureloss::updatehRef(const Vec& hParent, const Vec& hLinkParent, int i) {
    int hInd = line->gethInd(parent, i);
    Index I=Index(hInd, hInd);
    h[0] >> hParent(I);
    hLink[0].resize() >> hLinkParent(I);
  }

  void HydlinePressureloss::updatedhduRef(const SqrMat& dhduParent, int i) {
    int hInd = line->gethInd(parent, i);
    Index I=Index(hInd, hInd);
    dhdu[0].resize() >> dhduParent(I);
  }

  void HydlinePressureloss::updatedhdtRef(const Vec& dhdtParent, int i) {
    int hInd = line->gethInd(parent, i);
    Index I=Index(hInd, hInd);
    dhdt[0].resize() >> dhdtParent(I);
  }

  void HydlinePressureloss::updaterRef(const Vec &rParent) {
    Index I=Index(line->gethInd(parent), line->gethInd(parent));
    r[0].resize() >> rParent(I);
  }

  void HydlinePressureloss::updateWRef(const Mat &WParent, int j) {
    Index I=Index(line->gethInd(parent,j), line->gethInd(parent,j));
    Index J=Index(laInd, laInd);
    W[0].resize() >> WParent(I,J);
  }

  void HydlinePressureloss::updateVRef(const Mat &VParent, int j) {
    Index J=Index(laInd, laInd);
    Index I=Index(line->gethInd(parent,j), line->gethInd(parent,j));
    V[0].resize() >> VParent(I,J);
  }

  void HydlinePressureloss::updatewbRef(const Vec &wbParent) {
    Link::updatewbRef(wbParent);
    gd.resize() >> wb;
  }

  void HydlinePressureloss::updateg(double t) {
    if (!isSetValued())
      for (unsigned int i=0; i<variablePressureLoss.size(); i++)
        variablePressureLoss[i]->update(line->getu()(0));
  }

  void HydlinePressureloss::checkActiveg() {
    active=variablePressureLoss[0]->isClosed();
  }

  bool HydlinePressureloss::gActiveChanged() {
    bool changed = false;
    if (isActive0 != active)
      changed =true;
    isActive0=active;
    return changed;
  }

  void HydlinePressureloss::updategd(double t) {
    gd=line->getQIn(t);
  }

  void HydlinePressureloss::updateh(double t) {
    for (unsigned int i=0; i<pressureLosses.size(); i++)
      h[0](0)-=(*pressureLosses[i])(line->getu()(0));
    for (unsigned int i=0; i<variablePressureLoss.size(); i++)
      h[0](0)-=(*variablePressureLoss[i])(line->getu()(0));
  }

  void HydlinePressureloss::updateW(double t) {
    W[0]=Mat(1,1,INIT,1.);
  }

  void HydlinePressureloss::updaterFactors() {
    if (active) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();
      double sum = 0;
      for(int j=ia[laIndDS]+1; j<ia[laIndDS+1]; j++)
        sum += fabs(a[j]);
      double ai = a[ia[laIndDS]];
      if(ai > sum) {
        rFactorUnsure(0) = 0;
        rFactor(0) = 1./ai;
      }
      else {
        rFactorUnsure(0) = 1;
        rFactor(0) = 1./ai;
      }
    }
  }

  void HydlinePressureloss::solveImpactsFixpointSingle() {
    if(active) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();
      int *ja = ds->getGs().Jp();
      Vec &laMBS = ds->getla();
      Vec &b = ds->getb();
      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);
      if (bilateral) 
        la(0) -= rFactor(0)*gdn(0);
      else if (unilateral) {
        //lak[k](0) = fnil->project(lak[k](0), gdnk[k](0), gdk[k](0), rFactork[k](0));
        double gd_limit=1e-8;
        double epsilon=0;
        if(fabs(gd(0)) > gd_limit)
          gdn(0) += epsilon*gd(0);
        la(0) = proxCN(la(0)-rFactor(0)*gdn(0));
      }
    }
  }

  void HydlinePressureloss::solveImpactsGaussSeidel() {
    if(active) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();
      int *ja = ds->getGs().Jp();
      Vec &laMBS = ds->getla();
      Vec &b = ds->getb();
      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]+1; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);
      if (bilateral)  
        la(0)= -gdn(0)/a[ia[laIndDS+0]];
      else if (unilateral) {
        // double om = 1.0;
        // double buf = fnil->solve(a[ia[laIndDS+laIndk[k]]], gdnk[k](0), gdk[k](0));
        // lak[k](0) += om*(buf - lak[k](0));
        double gd_limit=1e-8;
        double epsilon=0;
        double om = 1.;
        if(fabs(gd(0)) > gd_limit)
          gdn(0) += epsilon*gd(0);
        double buf = (gdn(0) >= 0) ? 0 : -gdn(0)/a[ia[laIndDS+0]];
        la(0) += om*(buf - la(0));

      }
    }
  }

  void HydlinePressureloss::checkImpactsForTermination() {
    if (active) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();
      int *ja = ds->getGs().Jp();
      Vec &laMBS = ds->getla();
      Vec &b = ds->getb();
      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);
      if (bilateral) {
        if(fabs(gdn(0))>gdTol)
          ds->setTermination(false);
      }
      else if (unilateral) {
        // fnil->isFulfilled(lak[k](0),gdnk[k](0),gdk[k](0),LaTol,gdTol)
        double gd_limit=1e-8;
        double epsilon=0;
        if(fabs(gd(0)) > gd_limit)
          gdn(0) += epsilon*gd(0);
        bool fulfilled=false;
        if(gdn(0) >= -gdTol && fabs(la(0)) <= laTol)
          fulfilled = true;
        else if(la(0) >= -laTol && fabs(gdn(0)) <= gdTol)
          fulfilled = true;
        if(!fulfilled)
          ds->setTermination(false);
      }
    }
  }
}

