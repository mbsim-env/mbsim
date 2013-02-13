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
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/eps.h"
#include "mbsimControl/signal_.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;

namespace MBSimHydraulics {

  RigidLinePressureLoss::RigidLinePressureLoss(const string &name, RigidHLine * line_, PressureLoss * pressureLoss, bool bilateral_, bool unilateral_) : Link(name), line(line_), active(false), active0(false), unilateral(unilateral_), bilateral(bilateral_), gdn(0), gdd(0), dpMin(0), linePressureLoss(NULL), closablePressureLoss(NULL), leakagePressureLoss(NULL), gfl(NULL), gil(NULL) {
    if (dynamic_cast<LinePressureLoss*>(pressureLoss))
      linePressureLoss = (LinePressureLoss*)(pressureLoss);
    else if (dynamic_cast<ClosablePressureLoss*>(pressureLoss))
      closablePressureLoss = (ClosablePressureLoss*)(pressureLoss);
    else if (dynamic_cast<LeakagePressureLoss*>(pressureLoss))
      leakagePressureLoss = (LeakagePressureLoss*)(pressureLoss);
    else if (dynamic_cast<UnidirectionalPressureLoss*>(pressureLoss))
      unidirectionalPressureLoss = (UnidirectionalPressureLoss*)(pressureLoss);

    if (unilateral) {
      gfl=new UnilateralConstraint();
      gil=new UnilateralNewtonImpact();
      dpMin=-((UnidirectionalRigidLine*)(line))->getMinimalPressureDrop();
    }
    else if (bilateral) {
      gfl=new BilateralConstraint();
      gil=new BilateralImpact();
    }

  }

  RigidLinePressureLoss::~RigidLinePressureLoss() {
    if (gfl) {
      delete gfl;
      gfl=NULL;
    }
    if (gil) {
      delete gil;
      gil=NULL;
    }
  }

  void RigidLinePressureLoss::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back((isActive()?la(0)/dt:la(0))*1e-5);
      if (isSetValued())
        plotVector.push_back(active);
      Link::plot(t, dt);
    }
  }

  void RigidLinePressureLoss::init(InitStage stage) {
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
      gd.resize(1);
      la.resize(1);
      if (isSetValued()) {
        r.push_back(Vec(j));
        sv.resize(1);
      }
    }
    else if (stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("pressureLoss [bar]");
        if (isSetValued())
          plotColumns.push_back("active");
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
    const int hInd = line->gethInd(parent, i);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    h[0].resize() >> hParent(I);
    hLink[0].resize() >> hLinkParent(I);
  }

  void RigidLinePressureLoss::updaterRef(const Vec& rParent, int i) {
    const int hInd = line->gethInd(parent, i);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    r[0].resize() >> rParent(I);
  }

  void RigidLinePressureLoss::updatedhduRef(const SqrMat& dhduParent, int i) {
    const int hInd = line->gethInd(parent, i);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    dhdu[0].resize() >> dhduParent(I);
  }

  void RigidLinePressureLoss::updatedhdtRef(const Vec& dhdtParent, int i) {
    const int hInd = line->gethInd(parent, i);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    dhdt[0].resize() >> dhdtParent(I);
  }

  void RigidLinePressureLoss::updaterRef(const Vec &rParent) {
    const int hInd = line->gethInd(parent);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    r[0].resize() >> rParent(I);
  }

  void RigidLinePressureLoss::updateWRef(const Mat &WParent, int j) {
    const int hInd = line->gethInd(parent, j);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    const Index J=Index(laInd, laInd);
    W[0].resize() >> WParent(I,J);
  }

  void RigidLinePressureLoss::updateVRef(const Mat &VParent, int j) {
    const int hInd = line->gethInd(parent, j);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    const Index J=Index(laInd, laInd);
    V[0].resize() >> VParent(I,J);
  }

  void RigidLinePressureLoss::updateg(double t) {
    if (unilateral)
      gd=line->getQIn(t);
  }

  void RigidLinePressureLoss::checkActiveg() {
    if (bilateral)
      active=(((ClosableRigidLine*)line)->getSignal()->getSignal()(0)<((ClosableRigidLine*)line)->getMinimalValue());
    else if (unilateral)
      active=(gd(0)<=(active?gdTol:0));
  }

  void RigidLinePressureLoss::checkActivegdn() {
    if (unilateral && active)
      active=(gdn<=0);
  }

  bool RigidLinePressureLoss::gActiveChanged() {
    bool changed = false;
    if (active0 != active)
      changed =true;
    active0=active;
    return changed;
  }

  void RigidLinePressureLoss::updategd(double t) {
    if (!unilateral)
      gd=line->getQIn(t);
  }

  void RigidLinePressureLoss::updateStopVector(double t) {
    if (bilateral)
      sv(0)=((ClosableRigidLine*)(line))->getSignal()->getSignal()(0)-((ClosableRigidLine*)(line))->getMinimalValue();
    else if (unilateral)
      sv(0)=isActive()?(la(0)-dpMin)*1e-5:-gd(0)*6e4;
  }

  void RigidLinePressureLoss::updateh(double t) {
    if (linePressureLoss)
      la(0)=(*linePressureLoss)(line->getQIn(t)(0), line);
    else if (closablePressureLoss)
      la(0)=(*closablePressureLoss)(line->getQIn(t)(0), line);
    else if (leakagePressureLoss)
      la(0)=(*leakagePressureLoss)(line->getQIn(t)(0), line);
    else if (unilateral || unidirectionalPressureLoss) {
      la(0)=0*dpMin+(unilateral ? 0 : (*unidirectionalPressureLoss)(gd(0), line));
    }
    h[0]-=trans(line->getJacobian())*la(0);
    hLink[0]-=trans(line->getJacobian())*la(0);
  }

  void RigidLinePressureLoss::updateW(double t) {
    W[0]=trans(line->getJacobian())*Mat(1,1,INIT,1.);
  }

  void RigidLinePressureLoss::updatedhdz(double t) {
    const unsigned int nLines=1;
    vector<Vec> hLink0, h0;

    for(unsigned int i=0; i<nLines; i++) { // save old values
      hLink0.push_back(hLink[i].copy());
      h0.push_back(h[i].copy());
    }
    if(nLines)
      updateh(t); 
    vector<Vec> hLinkEnd, hEnd;
    for(unsigned int i=0; i<nLines; i++) { // save with correct state
      hLinkEnd.push_back(hLink[i].copy());
      hEnd.push_back(h[i].copy());
    }

    /****************** velocity dependent calculations ***********************/
    for(unsigned int i=0; i<nLines; i++) 
      for(unsigned int l=0; l<nLines; l++) {
        for(int j=0; j<1; j++) {
          hLink[i] = hLink0[i].copy(); // set to old values
          h[i] = h0[i].copy();

          double uParentj = line->getu()(j); // save correct position

          line->getu()(j) += epsroot(); // update with disturbed positions assuming same active links
          line->updateStateDependentVariables(t); 
          updategd(t);
          updateh(t);

          dhdu[i*nLines+l].col(j) += (hLink[i]-hLinkEnd[i])/epsroot();
          line->getu()(j) = uParentj;
          line->updateStateDependentVariables(t); 
        }
      }

    /****************** position dependent calculations ***********************/
    for(unsigned int i=0; i<nLines; i++) 
      for(unsigned int l=0; l<nLines; l++) {
        for(int j=0; j<0; j++) {
          hLink[i] = hLink0[i].copy(); // set to old values
          h[i] = h0[i].copy();

          double qParentj = line->getq()(j); // save correct position

          line->getq()(j) += epsroot(); // update with disturbed positions assuming same active links
          line->updateStateDependentVariables(t); 
          updateg(t);
          updategd(t);
          line->updateT(t); 
          updateJacobians(t);
          updateh(t);

          dhdq[i*nLines+l].col(j) += (hLink[i]-hLinkEnd[i])/epsroot();
          line->getq()(j) = qParentj;
          line->updateStateDependentVariables(t); 
          line->updateT(t); 
        }
      }

    /******************** time dependent calculations ***************************/
    // for(unsigned int i=0; i<nLines; i++) 
    //   for(unsigned int l=0; l<nLines; l++) {
    //     hLink[i] = hLink0[i].copy(); // set to old values
    //     h[i] = h0[i].copy();

    //     double t0 = t; // save correct position

    //     t += epsroot(); // update with disturbed positions assuming same active links
    //     line->updateStateDependentVariables(t); 
    //     updateg(t);
    //     updategd(t);
    //     line->updateT(t); 
    //     updateJacobians(t);
    //     updateh(t);

    //     dhdt[i] += (hLink[i]-hLinkEnd[i])/epsroot();
    //     t = t0;
    //   }

    /************************ back to initial state ******************************/
    for(unsigned int i=0; i<nLines; i++) {
      line->updateStateDependentVariables(t); 
      updateg(t);
      updategd(t);
      line->updateT(t); 
      updateJacobians(t);
      hLink[i] = hLinkEnd[i].copy();
      h[i] = hEnd[i].copy();
    }
  }

  void RigidLinePressureLoss::updateCondition() {
    if (jsv(0)) {
      if (active)
        active = false;
      else {
        active = true;
        ds->setImpact(true);
      }
    }
  }

  void RigidLinePressureLoss::updaterFactors() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();

    double sum = 0;
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      sum += fabs(a[j]);

    const double ai = a[ia[laInd]];
    if(ai > sum) {
      rFactorUnsure(0) = 0;
      rFactor(0) = 1./ai;
    }
    else {
      rFactorUnsure(0) = 1;
      rFactor(0) = 1./ai;
    }
  }

  void RigidLinePressureLoss::solveImpactsFixpointSingle(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    la(0) = gil->project(la(0), gdn, gd(0), rFactor(0), dpMin*dt);
  }

  void RigidLinePressureLoss::solveConstraintsFixpointSingle() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->project(la(0), gdd, rFactor(0), dpMin);
  }

  void RigidLinePressureLoss::solveImpactsGaussSeidel(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    la(0) = gil->solve(a[ia[laInd]], gdn, gd(0));
    if (unilateral && (la(0) < dpMin*dt))
      la(0) = dpMin*dt;
  }

  void RigidLinePressureLoss::solveConstraintsGaussSeidel() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdd = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->solve(a[ia[laInd]], gdd);
    if (unilateral && (la(0) < dpMin))
      la(0) = dpMin;
  }

  void RigidLinePressureLoss::solveImpactsRootFinding(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    res(0) = la(0) - gil->project(la(0), gdn, gd(0), rFactor(0), dpMin*dt);
  }

  void RigidLinePressureLoss::solveConstraintsRootFinding() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    res(0) = la(0) - gfl->project(la(0), gdd, rFactor(0), dpMin);
  }

  void RigidLinePressureLoss::jacobianImpacts() {
    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->getG();

    RowVec jp1=Jprox.row(laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    Vec diff = gil->diff(la(0), gdn, gd(0), rFactor(0));

    jp1 = e1-diff(0)*e1;
    for(int j=0; j<G.size(); j++) 
      jp1(j) -= diff(1)*G(laInd,j);
  }

  void RigidLinePressureLoss::jacobianConstraints() {
    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->getG();

    RowVec jp1=Jprox.row(laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    Vec diff = gfl->diff(la(0), gdd, rFactor(0));

    jp1 = e1-diff(0)*e1;
    for(int j=0; j<G.size(); j++) 
      jp1(j) -= diff(1)*G(laInd,j);
  }

  void RigidLinePressureLoss::checkImpactsForTermination(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    if(!gil->isFulfilled(la(0),gdn,gd(0),LaTol,gdTol,dpMin*dt))
      ds->setTermination(false);
  }

  void RigidLinePressureLoss::checkConstraintsForTermination() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();
    
    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    if(!gfl->isFulfilled(la(0),gdd,laTol,gddTol,dpMin))
      ds->setTermination(false);
  }

}

