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
#include "mbsimHydraulics/rigid_line_pressureloss.h"
#include "mbsimHydraulics/hline.h"
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/constitutive_laws/bilateral_constraint.h"
#include "mbsim/constitutive_laws/unilateral_constraint.h"
#include "mbsim/constitutive_laws/bilateral_impact.h"
#include "mbsim/constitutive_laws/unilateral_newton_impact.h"
#include "mbsim/utils/eps.h"
#include "mbsimControl/signal_.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;

namespace MBSimHydraulics {

  RigidLinePressureLoss::RigidLinePressureLoss(const string &name, RigidHLine * line_, PressureLoss * pressureLoss, bool bilateral_, bool unilateral_) : Link(name), line(line_), active(false), active0(false), unilateral(unilateral_), bilateral(bilateral_), gdn(0), gdd(0), dpMin(0), linePressureLoss(NULL), closablePressureLoss(NULL), leakagePressureLoss(NULL), unidirectionalPressureLoss(NULL), gfl(NULL), gil(NULL) {
    pressureLoss->setLine(line);
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
      active=true;
      active0=true;
    }
    else if (bilateral) {
      gfl=new BilateralConstraint();
      gil=new BilateralImpact();
      active=true;
      active0=true;
    }
    if(gfl) {
      gfl->setParent(this);
      gfl->setName("gfl");
    }
    if(gil) {
      gil->setParent(this);
      gil->setName("gil");
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

  void RigidLinePressureLoss::plot() {
    if(plotFeature[plotRecursive]) {
      plotVector.push_back(evalGeneralizedForce()(0)*1e-5);
      if (isSetValued())
        plotVector.push_back(active);
    }
    Link::plot();
  }

  void RigidLinePressureLoss::init(InitStage stage, const InitConfigSet &config) {
    if (stage==preInit) {
      if(linePressureLoss) addDependency(linePressureLoss->getDependency());
      if(closablePressureLoss) addDependency(closablePressureLoss->getDependency());
      if(leakagePressureLoss) addDependency(leakagePressureLoss->getDependency());
      if(unidirectionalPressureLoss) addDependency(unidirectionalPressureLoss->getDependency());
      Link::init(stage, config);
      W[0].resize(1);
      W[1].resize(1);
      V[0].resize(1);
      V[1].resize(1);
      h[0].resize(1);
      h[1].resize(1);
      gd.resize(1);
      if (isSetValued()) {
        la.resize(1);
        r[0].resize(1);
        r[1].resize(1);
        sv.resize(1);
      }
    }
    else if (stage==plotting) {
      if(plotFeature[plotRecursive]) {
        plotColumns.push_back("pressureLoss [bar]");
        if (isSetValued())
          plotColumns.push_back("active");
      }
    }
    else if (stage==unknownStage)
      gdTol/=6e4;
    Link::init(stage, config);
    if(gfl) gfl->init(stage, config);
    if(gil) gil->init(stage, config);
  }

  void RigidLinePressureLoss::updatehRef(const Vec& hParent, int i) {
    const int hInd = line->gethInd(i);
    const RangeV I=RangeV(hInd, hInd+line->getGeneralizedVelocitySize()-1);
    h[i][0].resize() >> hParent(I);
  }

  void RigidLinePressureLoss::updatedhduRef(const SqrMat& dhduParent, int i) {
    throw runtime_error("Error in RigidLinePressureLoss::updatedhduRef");
  }

  void RigidLinePressureLoss::updatedhdtRef(const Vec& dhdtParent, int i) {
    const int hInd = line->gethInd(i);
    const RangeV I=RangeV(hInd, hInd+line->getGeneralizedVelocitySize()-1);
    dhdt[i].resize() >> dhdtParent(I);
  }

  void RigidLinePressureLoss::updaterRef(const Vec &rParent, int i) {
    const int hInd = line->gethInd(i);
    const RangeV I=RangeV(hInd, hInd+line->getGeneralizedVelocitySize()-1);
    r[i][0].resize() >> rParent(I);
  }

  void RigidLinePressureLoss::updateWRef(const Mat &WParent, int j) {
    const int hInd = line->gethInd(j);
    const RangeV I=RangeV(hInd, hInd+line->getGeneralizedVelocitySize()-1);
    const RangeV J=RangeV(laInd, laInd+laSize-1);
    W[j][0].resize() >> WParent(I,J);
  }

  void RigidLinePressureLoss::updateVRef(const Mat &VParent, int j) {
    const int hInd = line->gethInd(j);
    const RangeV I=RangeV(hInd, hInd+line->getGeneralizedVelocitySize()-1);
    const RangeV J=RangeV(laInd, laInd+laSize-1);
    V[j][0].resize() >> VParent(I,J);
  }

  void RigidLinePressureLoss::checkActive(int j) {
    if(j==1) {
      if (bilateral)
        active=(((ClosableRigidLine*)line)->getFunction()->operator()(getTime())<((ClosableRigidLine*)line)->getMinimalValue());
      else if (unilateral)
        active=(line->evalQIn()(0)<=(active?gdTol:0));
    }
    else if(j==3) {
      if (unilateral && active)
      active=(gdn<=0);
    }
    else
      throw runtime_error("Error in RigidLinePressureLoss::checkActive");
  }

  bool RigidLinePressureLoss::gActiveChanged() {
    bool changed = false;
    if (active0 != active)
      changed =true;
    active0=active;
    return changed;
  }

  void RigidLinePressureLoss::updategd() {
    if (!unilateral and isActive()) gd=line->evalQIn();
  }

  void RigidLinePressureLoss::updateStopVector() {
    if (bilateral)
      sv(0)=((ClosableRigidLine*)(line))->getFunction()->operator()(getTime())-((ClosableRigidLine*)(line))->getMinimalValue();
    else if (unilateral)
      sv(0)=isActive()?(evalGeneralizedForce()(0)-dpMin)*1e-5:-line->evalQIn()(0)*6e4;
  }

  void RigidLinePressureLoss::updateGeneralizedForces() {
    if(isActive())
      lambda = evalla();
    else {
      if (linePressureLoss) {
        lambda(0)=(*linePressureLoss)(line->evalQIn()(0));
      }
      else if (closablePressureLoss) {
        lambda(0)=(*closablePressureLoss)(line->evalQIn()(0));
      }
      else if (leakagePressureLoss) {
        lambda(0)=(*leakagePressureLoss)(line->evalQIn()(0));
      }
      else if (unilateral || unidirectionalPressureLoss) {
        lambda(0)=0*dpMin+(unilateral ? 0 : (*unidirectionalPressureLoss)(line->evalQIn()(0)));
      }
    }
    updla = false;
  }

  void RigidLinePressureLoss::updateh(int j) {
    if(not(active))
      h[j][0]-=trans(line->getJacobian())*evalGeneralizedForce()(0);
  }

  void RigidLinePressureLoss::updateW(int j) {
    if(active)
      W[j][0]=trans(line->getJacobian())*Mat(1,1,INIT,1.);
  }

  void RigidLinePressureLoss::checkRoot() {
    if (jsv(0)) {
      if (active) {
        active = false;
        ds->setRootID(max(ds->getRootID(),1)); 
      }
      else {
        active = true;
        ds->setRootID(max(ds->getRootID(),3)); // Impact
      }
    }
  }

  void RigidLinePressureLoss::updaterFactors() {
    if(active) {
    const double *a = ds->evalGs()();
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
  }

  void RigidLinePressureLoss::solveImpactsFixpointSingle() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    La(0) = gil->project(La(0), gdn, gd(0), rFactor(0), dpMin*getStepSize());
  }

  void RigidLinePressureLoss::solveConstraintsFixpointSingle() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->project(la(0), gdd, rFactor(0), dpMin);
  }

  void RigidLinePressureLoss::solveImpactsGaussSeidel() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    La(0) = gil->solve(a[ia[laInd]], gdn, gd(0));
    if (unilateral && (La(0) < dpMin*getStepSize()))
      La(0) = dpMin*getStepSize();
  }

  void RigidLinePressureLoss::solveConstraintsGaussSeidel() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->solve(a[ia[laInd]], gdd);
    if (unilateral && (la(0) < dpMin))
      la(0) = dpMin;
  }

  void RigidLinePressureLoss::solveImpactsRootFinding() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    res(0) = La(0) - gil->project(La(0), gdn, gd(0), rFactor(0), dpMin*getStepSize());
  }

  void RigidLinePressureLoss::solveConstraintsRootFinding() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

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
    Vec diff = gil->diff(La(0), gdn, gd(0), rFactor(0));

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

  void RigidLinePressureLoss::checkImpactsForTermination() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    if(!gil->isFulfilled(La(0),gdn,gd(0),LaTol,gdTol,dpMin*getStepSize()))
      ds->setTermination(false);
  }

  void RigidLinePressureLoss::checkConstraintsForTermination() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();
    
    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    if(!gfl->isFulfilled(la(0),gdd,laTol,gddTol,dpMin))
      ds->setTermination(false);
  }

}
