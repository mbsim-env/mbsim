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
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/eps.h"
#include "mbsimControl/signal_.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;

namespace MBSimHydraulics {

  RigidLinePressureLoss::RigidLinePressureLoss(const string &name, RigidHLine * line_, PressureLoss * pressureLoss, bool bilateral_, bool unilateral_) : Link(name), line(line_), active(true), active0(true), unilateral(unilateral_), bilateral(bilateral_), gdn(0), gdd(0), dpMin(0), linePressureLoss(NULL), closablePressureLoss(NULL), leakagePressureLoss(NULL), unidirectionalPressureLoss(NULL), gfl(NULL), gil(NULL) {
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
    }
    else if (bilateral) {
      gfl=new BilateralConstraint();
      gil=new BilateralImpact();
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

  void RigidLinePressureLoss::plot(double t, double dt) {
    //cout << name << " at t = " << t << " active = " << active << " laSIze = " << laSize << " ds " << ds->getlaSize() << endl;
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(getGeneralizedForce(t)(0)*1e-5); 
      if (isSetValued())
        plotVector.push_back(active);
      Element::plot(t, dt);
    }
  }

  void RigidLinePressureLoss::init(InitStage stage) {
    if (stage==preInit) {
      Link::init(stage);
      if(linePressureLoss) addDependency(linePressureLoss->getDependency());
      if(closablePressureLoss) addDependency(closablePressureLoss->getDependency());
      if(leakagePressureLoss) addDependency(leakagePressureLoss->getDependency());
      if(unidirectionalPressureLoss) addDependency(unidirectionalPressureLoss->getDependency());
    }
    else if (stage==resize) {
      Link::init(stage);
      int j=1;
      W[0].push_back(Mat(j, 1));
      V[0].push_back(Mat(j, 1));
      h[0].push_back(Vec(j));
      W[1].push_back(Mat(j, 1));
      V[1].push_back(Mat(j, 1));
      h[1].push_back(Vec(j));
      dhdq.push_back(Mat(j, 0));
      dhdu.push_back(SqrMat(j));
      dhdt.push_back(Vec(j));
      gd.resize(1);
      laSV.resize(1);
      if (isSetValued()) {
        la.resize(1);
        r[0].push_back(Vec(j));
        r[1].push_back(Vec(j));
        sv.resize(1);
        laMV.resize(1);
      }
    }
    else if (stage==plotting) {
      if (line->getPlotFeature(plotRecursive)!=enabled)
        this->setPlotFeature(plotRecursive, disabled);
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("pressureLoss [bar]");
        if (isSetValued())
          plotColumns.push_back("active");
        Element::init(stage);
      }
    }
    else if (stage==unknownStage) {
      gdTol/=6e4;
      Link::init(stage);
    }
    else
      Link::init(stage);
    if(gfl) gfl->init(stage);
    if(gil) gil->init(stage);
  }

  void RigidLinePressureLoss::updatehRef(const Vec& hParent, int i) {
    const int hInd = line->gethInd(i);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    h[i][0].resize() >> hParent(I);
  }

  void RigidLinePressureLoss::updatedhduRef(const SqrMat& dhduParent, int i) {
    throw runtime_error("Error in RigidLinePressureLoss::updatedhduRef");
  }

  void RigidLinePressureLoss::updatedhdtRef(const Vec& dhdtParent, int i) {
    const int hInd = line->gethInd(i);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    dhdt[i].resize() >> dhdtParent(I);
  }

  void RigidLinePressureLoss::updaterRef(const Vec &rParent, int i) {
    const int hInd = line->gethInd(i);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    r[i][0].resize() >> rParent(I);
  }

  void RigidLinePressureLoss::updateWRef(const Mat &WParent, int j) {
    const int hInd = line->gethInd(j);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    const Index J=Index(laInd, laInd+laSize-1);
    W[j][0].resize() >> WParent(I,J);
  }

  void RigidLinePressureLoss::updateVRef(const Mat &VParent, int j) {
    const int hInd = line->gethInd(j);
    const Index I=Index(hInd, hInd+line->getJacobian().cols()-1);
    const Index J=Index(laInd, laInd+laSize-1);
    V[j][0].resize() >> VParent(I,J);
  }

  void RigidLinePressureLoss::updateg(double t) {
    throw;
    if (unilateral) gd=line->getQ(t);
  }

  void RigidLinePressureLoss::checkActive(double t, int j) {
    if(j==1) {
      if (bilateral)
        active=(((ClosableRigidLine*)line)->getFunction()->operator()(t)<((ClosableRigidLine*)line)->getMinimalValue());
      else if (unilateral)
        active=(line->getQ(t)(0)<=(active?gdTol:0));
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

  void RigidLinePressureLoss::updategd(double t) {
    if (!unilateral and isActive()) gd=line->getQ(t);
  }

  void RigidLinePressureLoss::updateStopVector(double t) {
    if (bilateral)
      sv(0)=((ClosableRigidLine*)(line))->getFunction()->operator()(t)-((ClosableRigidLine*)(line))->getMinimalValue();
    else if (unilateral)
      sv(0)=isActive()?(getGeneralizedSetValuedForce(t)(0)-dpMin)*1e-5:-line->getQ(t)(0)*6e4;
  }

  void RigidLinePressureLoss::updateGeneralizedSingleValuedForces(double t) {
    if (linePressureLoss) {
      linePressureLoss->setTime(t);
      laSV(0)=(*linePressureLoss)(line->getQ(t)(0));
    }
    else if (closablePressureLoss) {
      closablePressureLoss->setTime(t);
      laSV(0)=(*closablePressureLoss)(line->getQ(t)(0));
    }
    else if (leakagePressureLoss) {
      leakagePressureLoss->setTime(t);
      laSV(0)=(*leakagePressureLoss)(line->getQ(t)(0));
    }
    else if (unilateral || unidirectionalPressureLoss) {
      unidirectionalPressureLoss->setTime(t);
      laSV(0)=0*dpMin+(unilateral ? 0 : (*unidirectionalPressureLoss)(line->getQ(t)(0)));
    }
    updlaSV = false;
  }

  void RigidLinePressureLoss::updateGeneralizedSetValuedForces(double t) {
    if(isActive())
      laMV = la;
    else
      laMV = getGeneralizedSingleValuedForce(t);
    updlaMV = false;
  }

  void RigidLinePressureLoss::updateh(double t, int j) {
//    if(isSingleValued() or not(active)) {
    h[j][0]-=trans(line->getJacobian())*getGeneralizedSingleValuedForce(t)(0);
//    }
  }

  void RigidLinePressureLoss::updateW(double t, int j) {
    if(active)
      W[j][0]=trans(line->getJacobian())*Mat(1,1,INIT,1.);
  }

  void RigidLinePressureLoss::checkRoot(double t) {
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

  void RigidLinePressureLoss::updaterFactors(double t) {
    if(active) {
    const double *a = ds->getGs(t)();
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

  void RigidLinePressureLoss::solveImpactsFixpointSingle(double t, double dt) {
    const double *a = ds->getGs(t)();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa();
    const Vec &b = ds->getb(false);

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    La(0) = gil->project(La(0), gdn, gd(0), rFactor(0), dpMin*dt);
  }

  void RigidLinePressureLoss::solveConstraintsFixpointSingle(double t) {
    const double *a = ds->getGs(t)();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb(false);

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->project(la(0), gdd, rFactor(0), dpMin);
  }

  void RigidLinePressureLoss::solveImpactsGaussSeidel(double t, double dt) {
    const double *a = ds->getGs(t)();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa();
    const Vec &b = ds->getb(false);

    gdn = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    La(0) = gil->solve(a[ia[laInd]], gdn, gd(0));
    if (unilateral && (La(0) < dpMin*dt))
      La(0) = dpMin*dt;
  }

  void RigidLinePressureLoss::solveConstraintsGaussSeidel(double t) {
    const double *a = ds->getGs(t)();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb(false);

    gdd = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->solve(a[ia[laInd]], gdd);
    if (unilateral && (la(0) < dpMin))
      la(0) = dpMin;
  }

  void RigidLinePressureLoss::solveImpactsRootFinding(double t, double dt) {
    const double *a = ds->getGs(t)();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa();
    const Vec &b = ds->getb(false);

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    res(0) = La(0) - gil->project(La(0), gdn, gd(0), rFactor(0), dpMin*dt);
  }

  void RigidLinePressureLoss::solveConstraintsRootFinding(double t) {
    const double *a = ds->getGs(t)();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb(false);

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    res(0) = la(0) - gfl->project(la(0), gdd, rFactor(0), dpMin);
  }

  void RigidLinePressureLoss::jacobianImpacts(double t) {
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

  void RigidLinePressureLoss::jacobianConstraints(double t) {
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

  void RigidLinePressureLoss::checkImpactsForTermination(double t, double dt) {
    const double *a = ds->getGs(t)();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa();
    const Vec &b = ds->getb(false);

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    if(!gil->isFulfilled(La(0),gdn,gd(0),LaTol,gdTol,dpMin*dt))
      ds->setTermination(false);
  }

  void RigidLinePressureLoss::checkConstraintsForTermination(double t) {
    const double *a = ds->getGs(t)();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb(false);
    
    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    if(!gfl->isFulfilled(la(0),gdd,laTol,gddTol,dpMin))
      ds->setTermination(false);
  }

}
