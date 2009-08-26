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

#include "hydline_closed.h"

#include "hydline.h"
#include "pressure_loss.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/nonsmooth_algebra.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  HydlineClosed::HydlineClosed(const string &name, HydLineValve * line_) : Link(name), line(line_), isActive0(true) {
  }

  void HydlineClosed::init(InitStage stage) {
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
        plotColumns.push_back("gd");
        plotColumns.push_back("active");
        plotColumns.push_back("gdn");
        plotColumns.push_back("la(0)");
        plotColumns.push_back("rFactor(0)");
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

  void HydlineClosed::updater(double t) {
    r[0] += V[0]*la;
  }

  void HydlineClosed::updateg(double t) {
    line->getPressureLossVar()->updateg(t);
    active=line->getPressureLossVar()->isClosed();
  }

  void HydlineClosed::updategd(double t) {
    gd(0)=line->getQIn(t)(0);
  }

  bool HydlineClosed::gActiveChanged() {
    bool changed = false;
    if (isActive0 != active)
      changed =true;
    isActive0=active;
    return changed;
  }

  void HydlineClosed::updateWRef(const Mat &WParent, int j) {
    Index I=Index(line->gethInd(parent,j), line->gethInd(parent,j));
    Index J=Index(laInd, laInd);
    W[0].resize()>>WParent(I,J);
  }

  void HydlineClosed::updateVRef(const Mat &VParent, int j) {
    Index J=Index(laInd, laInd);
    Index I=Index(line->gethInd(parent,j), line->gethInd(parent,j));
    V[0].resize()>>VParent(I,J);
  }

  void HydlineClosed::updatehRef(const Vec& hParent, const Vec& hLinkParent, int i) {
    int hInd = line->gethInd(parent, i);
    Index I=Index(hInd, hInd);
    h[0]>>hParent(I);
    hLink[0].resize()>>hLinkParent(I);
  }

  void HydlineClosed::updatedhduRef(const SqrMat& dhduParent, int i) {
    int hInd = line->gethInd(parent, i);
    Index I=Index(hInd, hInd);
    dhdu[0].resize()>>dhduParent(I);
  }

  void HydlineClosed::updatedhdtRef(const Vec& dhdtParent, int i) {
    int hInd = line->gethInd(parent, i);
    Index I=Index(hInd, hInd);
    dhdt[0].resize()>>dhdtParent(I);
  }

  void HydlineClosed::updaterRef(const Vec &rParent) {
    Index I=Index(line->gethInd(parent), line->gethInd(parent));
    r[0]>>rParent(I);
  }

  void HydlineClosed::updatewbRef(const Vec &wbParent) {
    Link::updatewbRef(wbParent);
    gd >> wb;
  }

  void HydlineClosed::updateW(double t) {
    W[0]=Mat(1,1,INIT,1.);
  }

  void HydlineClosed::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(gd(0));
      plotVector.push_back(active);
      plotVector.push_back(gdn(0));
      plotVector.push_back(la(0));
      plotVector.push_back(rFactor(0));
      Link::plot(t, dt);
    }
  }



  HydlineClosedBilateral::HydlineClosedBilateral(const string &name, HydLineValve * line) : HydlineClosed(name, line) {
  }

  void HydlineClosedBilateral::updaterFactors() {
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

  void HydlineClosedBilateral::solveImpactsFixpointSingle() {
    if(active) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();
      int *ja = ds->getGs().Jp();
      Vec &laMBS = ds->getla();
      Vec &b = ds->getb();

      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);

      la(0) -= rFactor(0)*gdn(0);
    }
  }

  void HydlineClosedBilateral::solveImpactsGaussSeidel() {
    if(active) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();
      int *ja = ds->getGs().Jp();
      Vec &laMBS = ds->getla();
      Vec &b = ds->getb();

      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]+1; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);

      la(0)= -gdn(0)/a[ia[laIndDS+0]];
    }
  }

  void HydlineClosedBilateral::checkImpactsForTermination() {
    if (active) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();
      int *ja = ds->getGs().Jp();
      Vec &laMBS = ds->getla();
      Vec &b = ds->getb();

      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);

      if(fabs(gdn(0))>gdTol)
        ds->setTermination(false);
    }
  }



  HydlineClosedUnilateral::HydlineClosedUnilateral(const string &name, HydLineCheckvalveUnilateral * line) : HydlineClosed(name, line) {
  }

  void HydlineClosedUnilateral::updaterFactors() {
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

  void HydlineClosedUnilateral::solveImpactsFixpointSingle() {
    if(active) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();
      int *ja = ds->getGs().Jp();
      Vec &laMBS = ds->getla();
      Vec &b = ds->getb();

      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);

      //lak[k](0) = fnil->project(lak[k](0), gdnk[k](0), gdk[k](0), rFactork[k](0));
      double gd_limit=1e-8;
      double epsilon=0;
      if(fabs(gd(0)) > gd_limit)
        gdn(0) += epsilon*gd(0);
      la(0) = proxCN(la(0)-rFactor(0)*gdn(0));
    }
  }

  void HydlineClosedUnilateral::solveImpactsGaussSeidel() {
    if(active) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();
      int *ja = ds->getGs().Jp();
      Vec &laMBS = ds->getla();
      Vec &b = ds->getb();

      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]+1; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);

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

  void HydlineClosedUnilateral::checkImpactsForTermination() {
    if (active) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();
      int *ja = ds->getGs().Jp();
      Vec &laMBS = ds->getla();
      Vec &b = ds->getb();

      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);

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
