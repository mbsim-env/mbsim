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

#include "mbsimHydraulics/rigid_line_pressureloss.h"
#include "mbsimHydraulics/hline.h"
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/nonsmooth_algebra.h"
#include "mbsimControl/signal_.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;

namespace MBSimHydraulics {

  RigidLinePressureLoss::RigidLinePressureLoss(const string &name, RigidHLine * line_, PressureLoss * pressureLoss, bool bilateral_, bool unilateral_) : Link(name), line(line_), isActive0(false), gdn(1), unilateral(unilateral_), bilateral(bilateral_), active(false), pLoss(0), linePressureLoss(NULL), closablePressureLoss(NULL) {
    if (dynamic_cast<LinePressureLoss*>(pressureLoss))
      linePressureLoss = (LinePressureLoss*)(pressureLoss);
    else if (dynamic_cast<ClosablePressureLoss*>(pressureLoss))
      closablePressureLoss = (ClosablePressureLoss*)(pressureLoss);
    else if (dynamic_cast<LeakagePressureLoss*>(pressureLoss))
      leakagePressureLoss = (LeakagePressureLoss*)(pressureLoss);
  }

  void RigidLinePressureLoss::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(pLoss*1e-5);
      if (closablePressureLoss)
        plotVector.push_back(((ClosableRigidLine*)line)->getSignal()->getSignal()(0)<((ClosableRigidLine*)line)->getMinimalValue());
      if (isSetValued()) {
        plotVector.push_back(gd(0));
        plotVector.push_back(active);
        plotVector.push_back(gdn(0));
        plotVector.push_back(la(0));
        plotVector.push_back(rFactor(0));
      }
      Link::plot(t, dt);
    }
  }

  void RigidLinePressureLoss::init(InitStage stage) {
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
        plotColumns.push_back("pressureLoss [bar]");
        if (closablePressureLoss)
          plotColumns.push_back("active");
        if (isSetValued()) {
          plotColumns.push_back("gd");
          plotColumns.push_back("active");
          plotColumns.push_back("gdn");
          plotColumns.push_back("la(0)");
          plotColumns.push_back("rFactor(0)");
        }
        Link::init(stage);
      }
    }
    else if (stage==MBSim::unknownStage) {
      gdTol*=1e-6;
      Link::init(stage);
    }
    else
      Link::init(stage);
  }

  void RigidLinePressureLoss::updatehRef(const Vec& hParent, const Vec& hLinkParent, int i) {
    int hInd = line->gethInd(parent, i);
    Index I=Index(hInd, hInd);
    h[0] >> hParent(I);
    hLink[0].resize() >> hLinkParent(I);
  }

  void RigidLinePressureLoss::updatedhduRef(const SqrMat& dhduParent, int i) {
    int hInd = line->gethInd(parent, i);
    Index I=Index(hInd, hInd);
    dhdu[0].resize() >> dhduParent(I);
  }

  void RigidLinePressureLoss::updatedhdtRef(const Vec& dhdtParent, int i) {
    int hInd = line->gethInd(parent, i);
    Index I=Index(hInd, hInd);
    dhdt[0].resize() >> dhdtParent(I);
  }

  void RigidLinePressureLoss::updaterRef(const Vec &rParent) {
    Index I=Index(line->gethInd(parent), line->gethInd(parent));
    r[0].resize() >> rParent(I);
  }

  void RigidLinePressureLoss::updateWRef(const Mat &WParent, int j) {
    Index I=Index(line->gethInd(parent,j), line->gethInd(parent,j));
    Index J=Index(laInd, laInd);
    W[0].resize() >> WParent(I,J);
  }

  void RigidLinePressureLoss::updateVRef(const Mat &VParent, int j) {
    Index J=Index(laInd, laInd);
    Index I=Index(line->gethInd(parent,j), line->gethInd(parent,j));
    V[0].resize() >> VParent(I,J);
  }

  void RigidLinePressureLoss::updatewbRef(const Vec &wbParent) {
    Link::updatewbRef(wbParent);
    gd.resize() >> wb;
  }

  void RigidLinePressureLoss::updateg(double t) {
    if (!isSetValued()) {
      // for (unsigned int i=0; i<leakagePressureLosses.size(); i++)
      //   leakagePressureLosses[i]->update(line->getu()(0));
      // for (unsigned int i=0; i<variablePressureLosses.size(); i++)
      //   variablePressureLosses[i]->update(line->getu()(0));
    }
  }

  void RigidLinePressureLoss::checkActiveg() {
    if (closablePressureLoss)
      active=(((ClosableRigidLine*)line)->getSignal()->getSignal()(0)<((ClosableRigidLine*)line)->getMinimalValue());
  }

  bool RigidLinePressureLoss::gActiveChanged() {
    bool changed = false;
    if (isActive0 != active)
      changed =true;
    isActive0=active;
    return changed;
  }

  void RigidLinePressureLoss::updategd(double t) {
    gd=line->getQIn(t);
  }

  void RigidLinePressureLoss::updateh(double t) {
    if (linePressureLoss)
      pLoss=(*linePressureLoss)(line->getu()(0), line);
    else if (closablePressureLoss)
      pLoss=(*closablePressureLoss)(line->getu()(0), line);
    else if (leakagePressureLoss)
      pLoss=(*leakagePressureLoss)(line->getu()(0), line);
    h[0](0)-=pLoss;
  }

  void RigidLinePressureLoss::updateW(double t) {
    W[0]=Mat(1,1,INIT,1.);
  }

  void RigidLinePressureLoss::updaterFactors() {
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

  void RigidLinePressureLoss::solveImpactsFixpointSingle() {
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

  void RigidLinePressureLoss::solveImpactsGaussSeidel() {
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

  void RigidLinePressureLoss::solveImpactsRootFinding() {
    if (bilateral) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();
      int *ja = ds->getGs().Jp();
      Vec &laMBS = ds->getla();
      Vec &b = ds->getb();

      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);

      res = rFactor(0)*gdn;
    }
    else
      throw(123);
  }

  void RigidLinePressureLoss::jacobianImpacts() {
    if (bilateral) {
    SqrMat Jprox = ds->getJprox();
    SqrMat G = ds->getG();
    RowVec jp1=Jprox.row(laIndDS);
    jp1.init(0);
    for(int j=0; j<G.size(); j++) 
      jp1(j) = rFactor(0) * G(laIndDS, j);
    }
    else
      throw(123);
  }

  void RigidLinePressureLoss::checkImpactsForTermination() {
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

