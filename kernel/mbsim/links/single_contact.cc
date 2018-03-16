/* Copyright (C) 2004-2014 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h> 
#include <mbsim/links/contact.h>
#include <mbsim/frames/contour_frame.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/constitutive_laws/generalized_force_law.h>
#include <mbsim/constitutive_laws/friction_force_law.h>
#include <mbsim/constitutive_laws/generalized_impact_law.h>
#include <mbsim/constitutive_laws/friction_impact_law.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <mbsim/utils/contact_utils.h>
#include <mbsim/utils/utils.h>
#include <mbsim/objectfactory.h>
#include <mbsim/utils/rotarymatrices.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {
  extern double tP;
  extern bool gflag;

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, SingleContact)

  void SingleContact::updatewb() {
      wb -= evalGlobalForceDirection()(RangeV(0,2),RangeV(0,laSize-1)).T() * cFrame[0]->evalGyroscopicAccelerationOfTranslation();
      wb += evalGlobalForceDirection()(RangeV(0,2),RangeV(0,laSize-1)).T() * cFrame[1]->evalGyroscopicAccelerationOfTranslation();
  }

  void SingleContact::resetUpToDate() {
    ContourLink::resetUpToDate();
    updlaN = true;
    updlaT = true;
  }

  bool SingleContact::isSticking() const { 
    return fdf->isSetValued() and laT.size();
  }

  const double& SingleContact::evallaN() {
    if(ds->getUpdatela()) ds->updatela();
    return laN(0);
  }

  const Vec& SingleContact::evallaT() {
    if(ds->getUpdatela()) ds->updatela();
    return laT;
  }

  const double& SingleContact::evalLaN() {
    if(ds->getUpdateLa()) ds->updateLa();
    return LaN(0);
  }

  const Vec& SingleContact::evalLaT() {
    if(ds->getUpdateLa()) ds->updateLa();
    return LaT;
  }

  const double& SingleContact::evalgdnN() {
    if(ds->getUpdateLa()) ds->updateLa();
    return gdnN(0);
  }

  const Vec& SingleContact::evalgdnT() {
    if(ds->getUpdateLa()) ds->updateLa();
    return gdnT;
  }

  const double& SingleContact::evalgddN() {
    if(ds->getUpdatela()) ds->updatela();
    return gddN(0);
  }

  const Vec& SingleContact::evalgddT() {
    if(ds->getUpdatela()) ds->updatela();
    return gddT;
  }

  void SingleContact::updateGeneralizedNormalForceM() {
    if(gdActive[normal])
      lambdaN = evallaN();
    else
      lambdaN = 0;
  }

  void SingleContact::updateGeneralizedNormalForceS() {
    lambdaN = (*fcl)(evalGeneralizedRelativePosition()(0), evalGeneralizedRelativeVelocity()(0));
  }

  void SingleContact::updateGeneralizedNormalForceP() {
    static_cast<Link*>(parent)->updateGeneralizedForces();
  }

  void SingleContact::updateGeneralizedTangentialForceM() {
    if(gdActive[tangential])
      lambdaT = evallaT();
    else if(gdActive[normal])
      lambdaT = fdf->dlaTdlaN(evalGeneralizedRelativeVelocity()(RangeV(1,getFrictionDirections()))) * evalGeneralizedNormalForce();
    else
      lambdaT.init(0);
  }

  void SingleContact::updateGeneralizedTangentialForceS() {
    lambdaT = (*fdf)(evalGeneralizedRelativeVelocity()(RangeV(1,getFrictionDirections())), fabs(evalGeneralizedNormalForce()));
  }

  void SingleContact::updateGeneralizedForces() {
    lambda(0) = evalGeneralizedNormalForce();
    lambda.set(RangeV(1,lambda.size()-1),evalGeneralizedTangentialForce());
    updla = false;
  }

  void SingleContact::updateGeneralizedPositions() {
    if(static_cast<Link*>(parent)->getUpdaterrel())
      static_cast<Link*>(parent)->updateGeneralizedPositions();
    updrrel = false;
  }

  void SingleContact::updateGeneralizedVelocities() {
    if ((fcl->isSetValued() and gdActive[normal]) or (not fcl->isSetValued() and fcl->isClosed(evalGeneralizedRelativePosition()(0), 0))) { // TODO: nicer implementation
      Vec3 Wn = cFrame[0]->evalOrientation().col(0);

      Vec3 WvD = cFrame[1]->evalVelocity() - cFrame[0]->evalVelocity();

      vrel(0) = Wn.T() * WvD;

      if (getFrictionDirections()) {
        Mat3xV Wt(getFrictionDirections());
        Wt.set(0, cFrame[0]->getOrientation().col(1));
        if (getFrictionDirections() > 1)
          Wt.set(1, cFrame[0]->getOrientation().col(2));

        vrel.set(RangeV(1,getFrictionDirections()), Wt.T() * WvD);
      }
    }
    else
      vrel.init(0);
    updvrel = false;
  }

  void SingleContact::updatePositions(Frame *frame) {
    if(static_cast<Link*>(parent)->getUpdaterrel())
      static_cast<Link*>(parent)->updateGeneralizedPositions();
  }

  void SingleContact::updateVelocities() {
    throw;
    if ((fcl->isSetValued() and gdActive[normal]) or (not fcl->isSetValued() and fcl->isClosed(evalGeneralizedRelativePosition()(0), 0))) { // TODO: nicer implementation
      Vec3 Wn = cFrame[0]->evalOrientation().col(0);

      Vec3 WvD = cFrame[1]->evalVelocity() - cFrame[0]->evalVelocity();

      vrel(0) = Wn.T() * WvD;

      if (getFrictionDirections()) {
        Mat3xV Wt(getFrictionDirections());
        Wt.set(0, cFrame[0]->getOrientation().col(1));
        if (getFrictionDirections() > 1)
          Wt.set(1, cFrame[0]->getOrientation().col(2));

        vrel.set(RangeV(1,getFrictionDirections()), Wt.T() * WvD);
      }
    }
    else
      vrel.init(0);
//    updVel = false;
  }

  void SingleContact::updateg() {
    g = evalGeneralizedRelativePosition()(RangeV(0,gSize-1));
  }

  void SingleContact::updategd() {
    int addIndexnormal = fcl->isSetValued()?0:1;
    gd = evalGeneralizedRelativeVelocity()(RangeV(addIndexnormal,gdSize+addIndexnormal-1));
  }

  void SingleContact::updateh(int j) {
    Vec3 F = evalGlobalForceDirection().col(0)*evalGeneralizedNormalForce();
    if(fdf and not fdf->isSetValued())
      F += evalGlobalForceDirection()(Range<Fixed<0>,Fixed<2> >(),Range<Var,Var>(1,getFrictionDirections()))*evalGeneralizedTangentialForce();

    h[j][0] -= cFrame[0]->evalJacobianOfTranslation(j).T() * F;
    h[j][1] += cFrame[1]->evalJacobianOfTranslation(j).T() * F;
  }

  void SingleContact::updateW(int j) {
    int i = fcl->isSetValued()?0:1;
    Mat3xV RF = evalGlobalForceDirection()(Range<Fixed<0>,Fixed<2> >(),Range<Var,Var>(i,i+laSize-1));

    W[j][0] -= cFrame[0]->evalJacobianOfTranslation(j).T() * RF;
    W[j][1] += cFrame[1]->evalJacobianOfTranslation(j).T() * RF;
  }

  void SingleContact::updateV(int j) {
    if (getFrictionDirections()) {
      if (fdf->isSetValued()) {
        if (gdActive[normal] and not gdActive[tangential]) { // with this if-statement for the timestepping integrator it is V=W as it just evaluates checkActive(1)
          Mat3xV RF = evalGlobalForceDirection()(RangeV(0,2),RangeV(1, getFrictionDirections()));
          V[j][0] -= cFrame[0]->evalJacobianOfTranslation(j).T() * RF * fdf->dlaTdlaN(evalGeneralizedRelativeVelocity()(RangeV(1,getFrictionDirections())));
          V[j][1] += cFrame[1]->evalJacobianOfTranslation(j).T() * RF * fdf->dlaTdlaN(evalGeneralizedRelativeVelocity()(RangeV(1,getFrictionDirections())));
        }
      }
    }
  }

  void SingleContact::updateStopVector() {
    // TODO account for regularized normal force
    if (gActive != gdActive[normal])
      throwError("Internal error");
    if (gActive) {
      sv(0) = evalgddN() - gddTol;
      if (gdActive[tangential]) {
        if (getFrictionDirections()) {
          sv(1) = nrm2(gddT) - gddTol;
          if (sv(1) > 0) {
            gddNBuf = gddN;
            gddTBuf = gddT;
          }
        }
      }
      else if(fdf and fdf->isSetValued()) {
        if (getFrictionDirections() == 1)
          sv(1) = evalGeneralizedRelativeVelocity()(1);
        else {
          sv(1) = evalGeneralizedRelativeVelocity()(1) + evalGeneralizedRelativeVelocity()(2); // TODO: is there a better concept?
        }
      }
    }
    else {
      int i=0;
      if(fcl->isSetValued()) sv(i++) = evalGeneralizedRelativePosition()(0);
      if (fdf and fdf->isSetValued())
        sv(i) = 1;
    }
  }

  void SingleContact::updatelaRef(const Vec& laParent) {
    ContourLink::updatelaRef(laParent);
    if (laSize) {
      int laIndSizeNormal = 0;
      if (fcl->isSetValued()) {
        laN >> la(0, 0);
        laIndSizeNormal++;
      }

      if (fdf and fdf->isSetValued())
        laT >> la(laIndSizeNormal, laSize - 1);
    }
  }

  void SingleContact::updateLaRef(const Vec& LaParent) {
    ContourLink::updateLaRef(LaParent);
    if (laSize) {
      int laIndSizeNormal = 0;
      if (fcl->isSetValued()) {
        LaN >> La(0, 0);
        laIndSizeNormal++;
      }

      if (fdf and fdf->isSetValued())
        LaT >> La(laIndSizeNormal, laSize - 1);
    }
  }

  void SingleContact::updategdRef(const Vec& gdParent) {
    ContourLink::updategdRef(gdParent);
    if (gdSize) {
      int gdIndSizeNormal = 0;
      if (fcl->isSetValued()) {
        gdN >> gd(0, 0);
        gdIndSizeNormal++;
      }
      if (fdf and fdf->isSetValued())
        gdT >> gd(gdIndSizeNormal, gdSize - 1);
    }
  }

  void SingleContact::calcSize() {
    ng = 1;
    ngd = 1 + getFrictionDirections();
    nla = ngd;
    updSize = false;
  }

  void SingleContact::calclaSize(int j) {
    ContourLink::calclaSize(j);
    if (j == 0) { // IA
      //Add 1 to lambda size if normal force law is setValued
      if (fcl->isSetValued())
        laSize = 1;
      else
        laSize = 0;

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf and fdf->isSetValued())
        laSize += getFrictionDirections();

    }
    else if (j == 1) { // IG
      //Add 1 to lambda size if normal force law is setValued
      if (fcl->isSetValued())
        laSize = 1;
      else
        laSize = 0;

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf and fdf->isSetValued())
        laSize += getFrictionDirections();

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive;
    }
    else if (j == 2) { // IB
      //Add 1 to lambda size if normal force law is setValued
      if (fcl->isSetValued())
        laSize = 1;
      else
        laSize = 0;

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf and fdf->isSetValued())
        laSize += getFrictionDirections();

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive * gdActive[normal];

    }
    else if (j == 3) { // IH
      //Add 1 to lambda size if normal force law is setValued
      if (fcl->isSetValued())
        laSize = 1;
      else
        laSize = 0;

      //Add number of friction directions to lambda size if friction force law is setValued and active
      if (fdf and fdf->isSetValued())
        laSize += getFrictionDirections() * gdActive[tangential];

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive * gdActive[normal];

    }
    else if (j == 4) { // IG
      //Add 1 to lambda size if normal force law is setValued and active
      if (fcl->isSetValued())
        laSize = gActive;
    }
    else if (j == 5) { // IB
      laSize = gActive * gdActive[normal];
    }
    else
      throwError("Internal error");
  }

  void SingleContact::calcgSize(int j) {
    ContourLink::calcgSize(j);
    if (j == 0) { // IA
      gSize = 1;
    }
    else if (j == 1) { // IG
      gSize = gActive;
    }
    else if (j == 2) { // IB
      gSize = gActive * gdActive[normal];
    }
    else
      throwError("Internal error");
  }

  void SingleContact::calcgdSize(int j) {
    // TODO: avoid code duplication for maintenance
    ContourLink::calcgdSize(j);
    if (j == 0) { // all contacts
      // add 1 to gdSize if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += getFrictionDirections();

    }
    else if (j == 1) { // closed contacts
      // add 1 to gdSize if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += getFrictionDirections();

      gdSize *= gActive;

    }
    else if (j == 2) { // contacts which stay closed
      // add 1 to gdSize if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += getFrictionDirections();

      gdSize *= gActive * gdActive[normal];

    }
    else if (j == 3) { // sticking contacts
      // add 1 to gdSize if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += gdActive[tangential] * getFrictionDirections();

      gdSize *= gActive * gdActive[normal];

    }
    else
      throwError("Internal error");
  }

  void SingleContact::calcrFactorSize(int j) {
    ContourLink::calcrFactorSize(j);
    int addition = 0;
    if(fcl->isSetValued())
      addition += 1;
    if (j == 0) { // IA
      rFactorSize = addition + min(getFrictionDirections(), 1);
    }
    else if (j == 1) { // IG
      rFactorSize = gActive * (addition + min(getFrictionDirections(), 1));
    }
    else if (j == 2) { // IB
      rFactorSize = gActive * gdActive[normal] * (addition + min(getFrictionDirections(), 1));
    }
    else if (j == 3) { // IB
      rFactorSize = gActive * gdActive[normal] * (addition + gdActive[tangential] * min(getFrictionDirections(), 1));
    }
  }

  void SingleContact::calcsvSize() {
    ContourLink::calcsvSize();

    //Add length due to normal direction
    svSize = fcl->isSetValued() ? 1 : 0;

    //Add length due to tangentinal direction
    if (fdf)
      svSize += fdf->isSetValued() ? min(getFrictionDirections(), 1) : 0;
  }

  void SingleContact::calcLinkStatusSize() {
    ContourLink::calcLinkStatusSize();
    LinkStatusSize = 1;
    LinkStatus.resize(LinkStatusSize);
  }

  void SingleContact::calcLinkStatusRegSize() {
    ContourLink::calcLinkStatusRegSize();
    LinkStatusRegSize = 1;
    LinkStatusReg.resize(LinkStatusRegSize);
  }

  void SingleContact::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      gActive = 1;
      gActive0 = 1;

      if(getFrictionDirections()==0) {
        gdActive[normal]=1;
        gdActive[tangential]=0;
        gddActive[normal]=1;
        gddActive[tangential]=0;
      }
      else {
        gdActive[normal]=1;
        gdActive[tangential]=1;
        gddActive[normal]=1;
        gddActive[tangential]=1;
      }

      if(not fcl)
        updateGeneralizedNormalForce_ = &SingleContact::updateGeneralizedNormalForceP;
      else if(fcl->isSetValued())
        updateGeneralizedNormalForce_ = &SingleContact::updateGeneralizedNormalForceM;
      else
        updateGeneralizedNormalForce_ = &SingleContact::updateGeneralizedNormalForceS;

      if(not fdf)
        updateGeneralizedTangentialForce_ = &SingleContact::updateGeneralizedTangentialForce0;
      else if(fdf->isSetValued())
        updateGeneralizedTangentialForce_ = &SingleContact::updateGeneralizedTangentialForceM;
      else
        updateGeneralizedTangentialForce_ = &SingleContact::updateGeneralizedTangentialForceS;

      RF.resize(getGeneralizedForceSize());
      RM.resize(nla);

      iF = RangeV(0,nla-1);
      iM = RangeV(0,-1);
      DF.resize(nla,NONINIT);

      lambdaT.resize(getFrictionDirections());

      gddN.resize(1);
      gddT.resize(getFrictionDirections());
      gdnN.resize(1);
      gdnT.resize(getFrictionDirections());
      gddNBuf.resize(1);
      gddTBuf.resize(getFrictionDirections());

      if (getFrictionDirections() == 0)
        gdActive[tangential] = false;
    }
    ContourLink::init(stage, config);
    if(fcl) fcl->init(stage, config);
    if(fdf) fdf->init(stage, config);
    if(fnil) fnil->init(stage, config);
    if(ftil) ftil->init(stage, config);
  }

  bool SingleContact::isSetValued() const {
    bool flag = fcl->isSetValued();
    if (fdf)
      flag |= fdf->isSetValued();
    return flag;
  }

  bool SingleContact::isSingleValued() const {
    if (fcl->isSetValued()) {
      if (fdf)
        return not fdf->isSetValued();
      return false;
    }
    return true;
  }

  void SingleContact::updateLinkStatus() {
    if (gActive) {
      LinkStatus(0) = 2;
      if (ftil) {
        if (ftil->isSticking(evalLaT(), evalgdnT(), gdT, evalLaN(), LaTol, gdTol))
          LinkStatus(0) = 3;
        else
          LinkStatus(0) = 4;
      }
    }
    else
      LinkStatus(0) = 1;
  }

  void SingleContact::updateLinkStatusReg() {
    if (gActive) {
      LinkStatusReg(0) = 2;
    }
    else {
      LinkStatusReg(0) = 1;
    }
  }

  bool SingleContact::isActive() const {
    return gActive ? true : false;
  }

  bool SingleContact::gActiveChanged() {
    bool changed = (gActive0 != gActive ? true : false);
    gActive0 = gActive;
    return changed;
  }

  bool SingleContact::detectImpact() {
    return gActive0 < gActive ? true : false;
  }

  void SingleContact::setNormalForceLaw(GeneralizedForceLaw *fcl_) { 
    fcl = fcl_; 
    if(fcl) 
      fcl->setParent(this);
  }

  void SingleContact::setNormalImpactLaw(GeneralizedImpactLaw *fnil_) { 
    fnil = fnil_; 
    if(fnil) 
      fnil->setParent(this);
  }

  void SingleContact::setTangentialForceLaw(FrictionForceLaw *fdf_) { 
    fdf = fdf_; 
    if(fdf) 
      fdf->setParent(this);
  }

  void SingleContact::setTangentialImpactLaw(FrictionImpactLaw *ftil_) { 
    ftil = ftil_; 
    if(ftil) 
      ftil->setParent(this);
  }

  void SingleContact::solveImpactsFixpointSingle() {
    if (gActive) {
      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa(false);
      const Vec &b = ds->evalbi();

      int addIndexNormal = 0;
      if (fcl->isSetValued()) {
        addIndexNormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);

        LaN(0) = fnil->project(LaN(0), gdnN(0), gdN(0), rFactor(0));
      }

      if (ftil) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + addIndexNormal + i);
          for (int j = ia[laInd + i + addIndexNormal]; j < ia[laInd + 1 + i + addIndexNormal]; j++)
            gdnT(i) += a[j] * LaMBS(ja[j]);
        }

        //            if (ftil) //There must be a ftil coming with a setValued fdf
        LaT = ftil->project(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), rFactor(addIndexNormal));
      }
    }
  }

  void SingleContact::solveConstraintsFixpointSingle() {
    if (gdActive[normal]) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla(false);
      const Vec &b = ds->evalbc();

      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gddN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        laN(0) = fcl->project(laN(0), gddN(0), rFactor(0));
      }

      if (fdf and fdf->isSetValued() and gdActive[tangential]) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gddT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gddT(i) += a[j] * laMBS(ja[j]);
        }

        laT = fdf->project(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, rFactor(addIndexnormal));
      }
    }
  }

  void SingleContact::solveImpactsGaussSeidel() {
    assert(getFrictionDirections() <= 1);
    if (gActive) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa(false);
      const Vec &b = ds->evalbi();

      //TODO: check indices (in other solution algorithms too!)
      const double om = 1.0;
      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd] + 1; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);

        const double buf = fnil->solve(a[ia[laInd]], gdnN(0), gdN(0));
        LaN(0) += om * (buf - LaN(0));
      }

      if (ftil) {
        gdnT(0) = b(laInd + addIndexnormal);
        for (int j = ia[laInd + addIndexnormal] + 1; j < ia[laInd + addIndexnormal + 1]; j++)
          gdnT(0) += a[j] * LaMBS(ja[j]);

        Vec buf = ftil->solve(ds->getG()(RangeV(laInd + addIndexnormal, laInd + getFrictionDirections())), gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize());
        LaT += om * (buf - LaT);
      }
    }
  }

  void SingleContact::solveConstraintsGaussSeidel() {
    assert(getFrictionDirections() <= 1);

    if (gdActive[normal]) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla(false);
      const Vec &b = ds->evalbc();

      const double om = 1.0; // relaxation parameter omega (cf. Foerg, dissertation, p. 102)

      int addIndexNormal = 0;
      if (fcl->isSetValued()) {
        addIndexNormal++;
        gddN(0) = b(laInd);
        for (int j = ia[laInd] + 1; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        const double buf = fcl->solve(a[ia[laInd]], gddN(0));
        laN(0) += om * (buf - laN(0));
      }

      if (fdf and fdf->isSetValued() and gdActive[tangential]) {
        gddT(0) = b(laInd + addIndexNormal);
        for (int j = ia[laInd + addIndexNormal] + 1; j < ia[laInd + addIndexNormal + 1]; j++)
          gddT(0) += a[j] * laMBS(ja[j]);

        Vec buf = fdf->solve(ds->getG()(RangeV(laInd + addIndexNormal, laInd + addIndexNormal + getFrictionDirections() - 1)), gddT, fcl->isSetValued()?laN(0):lambdaN);
        laT += om * (buf - laT);
      }
    }
  }

  void SingleContact::solveImpactsRootFinding() {
    if (gActive) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa(false);
      const Vec &b = ds->evalbi();

      //compute residuum for normal direction
      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);

        res(0) = LaN(0) - fnil->project(LaN(0), gdnN(0), gdN(0), rFactor(0));
      }

      //compute residuum for tangential directions
      if (ftil) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gdnT(i) += a[j] * LaMBS(ja[j]);
        }
        //            if (ftil) There must be a frictional impact law if fdf is set valued!
        res(addIndexnormal, addIndexnormal + getFrictionDirections() - 1) = LaT - ftil->project(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), rFactor(addIndexnormal));
      }
    }
  }

  void SingleContact::solveConstraintsRootFinding() {
    if (gdActive[normal]) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla(false);
      const Vec &b = ds->evalbc();

      //compute residuum for normal direction
      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gddN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        res(0) = laN(0) - fcl->project(laN(0), gddN(0), rFactor(0));
      }

      //compute residuum for tangential directions
      if (fdf and fdf->isSetValued() and gdActive[tangential]) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gddT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gddT(i) += a[j] * laMBS(ja[j]);
        }
        //            if (ftil) There must be a frictional impact law if fdf is set valued!
        res(addIndexnormal, addIndexnormal + getFrictionDirections() - 1) = laT - fdf->project(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, rFactor(addIndexnormal));
      }
    }
  }

  void SingleContact::jacobianConstraints() {
    if (gdActive[normal]) {

      const SqrMat G = ds->evalG();

      //TODO: separate normal and tangential

      RowVec jp1 = ds->getJprox().row(laInd);
      RowVec e1(jp1.size());
      e1(laInd) = 1;

      int addIndexNormal = 0;
      if(fcl->isSetValued()) {
    	  addIndexNormal++;
    	  Vec diff = fcl->diff(laN(0), gddN(0), rFactor(0));

    	  jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+laIndk)
    	  for (int i = 0; i < G.size(); i++)
    		  jp1(i) -= diff(1) * G(laInd, i);
      }

      if (getFrictionDirections() == 1) {
        Mat diff = fdf->diff(laT, gddT(0, 0), fcl->isSetValued()?laN(0):lambdaN, rFactor(addIndexNormal));
        RowVec jp2 = ds->getJprox().row(laInd + addIndexNormal);
        RowVec e2(jp2.size());
        e2(laInd + 1) = 1;
        Mat e(2, jp2.size());
        e(0, laInd) = 1;
        e(1, laInd + addIndexNormal) = 1;
        jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk)
        //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk)
        for (int i = 0; i < G.size(); i++)
          jp2(i) -= diff(0, 1) * G(laInd + addIndexNormal, i);

      }
      else if (getFrictionDirections() == 2) {
        Mat diff = ftil->diff(laT, gddT, gdT, fcl->isSetValued()?laN(0):lambdaN, rFactor(addIndexNormal));
        Mat jp2 = ds->getJprox()(RangeV(laInd + addIndexNormal, laInd + addIndexNormal + 1), RangeV(0, ds->getJprox().cols() - 1));
        Mat e2(2, jp2.cols());
        e2(0, laInd + addIndexNormal) = 1;
        e2(1, laInd + addIndexNormal + 1) = 1;
        jp2 = e2 - diff(RangeV(0, 1), RangeV(4, 4)) * e1 - diff(RangeV(0, 1), RangeV(0, 1)) * e2; // -diff(RangeV(0,1),RangeV(4,5))*G(RangeV(laInd+laIndk+1,laInd+laIndk+2),RangeV(0,G.size()-1))
        for (int i = 0; i < G.size(); i++) {
          jp2(0, i) = diff(0, 2) * G(laInd + addIndexNormal, i) + diff(0, 3) * G(laInd + addIndexNormal + 1, i);
          jp2(1, i) = diff(1, 2) * G(laInd + addIndexNormal, i) + diff(1, 3) * G(laInd + addIndexNormal + 1, i);
        }
      }
    }
  }

  void SingleContact::jacobianImpacts() {
    if (gActive) {

      const SqrMat G = ds->evalG();

      RowVec jp1 = ds->getJprox().row(laInd);
      RowVec e1(jp1.size());
      e1(laInd) = 1;

      int addIndexNormal = 0;
      if (fcl->isSetValued()) {
    	  addIndexNormal++;

    	  Vec diff = fnil->diff(LaN(0), gdnN(0), gdN(0), rFactor(0));

    	  jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+laIndk)
    	  for (int i = 0; i < G.size(); i++)
    		  jp1(i) -= diff(1) * G(laInd, i);
      }

      if (getFrictionDirections() == 1) {
        Mat diff = ftil->diff(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), rFactor(addIndexNormal));
        RowVec jp2 = ds->getJprox().row(laInd + addIndexNormal);
        RowVec e2(jp2.size());
        e2(laInd + addIndexNormal) = 1;
        Mat e(2, jp2.size());
        e(0, laInd) = 1;
        e(1, laInd + addIndexNormal) = 1;
        jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk)
        //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk)
        for (int i = 0; i < G.size(); i++)
          jp2(i) -= diff(0, 1) * G(laInd + addIndexNormal, i);

      }
      else if (getFrictionDirections() == 2) {
        Mat diff = ftil->diff(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), rFactor(addIndexNormal));
        Mat jp2 = ds->getJprox()(RangeV(laInd + addIndexNormal, laInd + addIndexNormal + 1), RangeV(0, ds->getJprox().cols() - 1));
        Mat e2(2, jp2.cols());
        e2(0, laInd + addIndexNormal) = 1;
        e2(1, laInd + addIndexNormal + 1) = 1;
        jp2 = e2 - diff(RangeV(0, 1), RangeV(4, 4)) * e1 - diff(RangeV(0, 1), RangeV(0, 1)) * e2; // -diff(RangeV(0,1),RangeV(4,5))*G(RangeV(laInd+laIndk+1,laInd+laIndk+2),RangeV(0,G.size()-1))
        for (int i = 0; i < G.size(); i++) {
          jp2(0, i) -= diff(0, 2) * G(laInd + addIndexNormal, i) + diff(0, 3) * G(laInd + addIndexNormal + 1, i);
          jp2(1, i) -= diff(1, 2) * G(laInd + addIndexNormal, i) + diff(1, 3) * G(laInd + addIndexNormal + 1, i);
        }
      }
    }
  }

  void SingleContact::updaterFactors() {
    if (gdActive[normal]) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();

      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        double sumN = 0;
        for (int j = ia[laInd] + 1; j < ia[laInd + 1]; j++)
          sumN += fabs(a[j]);
        const double aN = a[ia[laInd]];
        if (aN > sumN) {
          rFactorUnsure(0) = 0;
          rFactor(0) = 1.0 / aN;
        }
        else {
          rFactorUnsure(0) = 1;
          rFactor(0) = rMax / aN;
        }
      }

      if (fdf and gdActive[tangential] and fdf->isSetValued()) {
        double sumT1 = 0;
        double sumT2 = 0;
        double aT1, aT2;
        if (getFrictionDirections() == 1) {
          for (int j = ia[laInd + addIndexnormal] + 1; j < ia[laInd + addIndexnormal + 1]; j++)
            sumT1 += fabs(a[j]);
          aT1 = a[ia[laInd + addIndexnormal]];
          if (aT1 > sumT1) {
            rFactorUnsure(addIndexnormal) = 0;
            rFactor(addIndexnormal) = 1.0 / aT1;
          }
          else {
            rFactorUnsure(addIndexnormal) = 1;
            rFactor(addIndexnormal) = rMax / aT1;
          }
        }
        else if (getFrictionDirections() == 2) {
          for (int j = ia[laInd + addIndexnormal] + 1; j < ia[laInd + addIndexnormal + 1]; j++)
            sumT1 += fabs(a[j]);
          for (int j = ia[laInd + addIndexnormal + 1] + 1; j < ia[laInd + addIndexnormal + 2]; j++)
            sumT2 += fabs(a[j]);
          aT1 = a[ia[laInd + addIndexnormal]];
          aT2 = a[ia[laInd + addIndexnormal + 1]];

          // TODO rFactorUnsure
          if (aT1 - sumT1 >= aT2 - sumT2)
            if (aT1 + sumT1 >= aT2 + sumT2)
              rFactor(addIndexnormal) = 2.0 / (aT1 + aT2 + sumT1 - sumT2);
            else
              rFactor(addIndexnormal) = 1.0 / aT2;
          else if (aT1 + sumT1 < aT2 + sumT2)
            rFactor(addIndexnormal) = 2.0 / (aT1 + aT2 - sumT1 + sumT2);
          else
            rFactor(addIndexnormal) = 1.0 / aT1;
        }
      }
    }
  }

  void SingleContact::checkConstraintsForTermination() {
    if (gdActive[normal]) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla(false);
      const Vec &b = ds->evalbc();

      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;

        gddN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        if (!fcl->isFulfilled(laN(0), gddN(0), laTol, gddTol)) {
          ds->setTermination(false);
          return;
        }
      }

      if (fdf && gdActive[tangential]) {

        for (unsigned int i = 0; i < gdActive[tangential] * getFrictionDirections(); i++) { //TODO: Is there any other number than 0 or one for gdActive? otherwithe the multiplication could be deleted again...
          gddT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gddT(i) += a[j] * laMBS(ja[j]);
        }

        if (!fdf->isFulfilled(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, laTol, gddTol)) {
          ds->setTermination(false);
          return;
        }
      }
    }
  }

  void SingleContact::checkImpactsForTermination() {
    if (gActive) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa(false);
      const Vec &b = ds->evalbi();

      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);
        if (!fnil->isFulfilled(LaN(0), gdnN(0), gdN(0), LaTol, gdTol)) {
          ds->setTermination(false);
          return;
        }
      }

      if (ftil) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gdnT(i) += a[j] * LaMBS(ja[j]);
        }
        if (!ftil->isFulfilled(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), LaTol, gdTol)) {
          ds->setTermination(false);
          return;
        }
      }
    }
  }

  void SingleContact::checkActive(int j) {
    if (j == 1) { // formerly checkActiveg()
      gActive = fcl->isClosed(evalGeneralizedRelativePosition()(0), gTol) ? 1 : 0;
      gdActive[normal] = gActive;
      gdActive[tangential] = gdActive[normal];
    }
    else if (j == 2) { // formerly checkActivegd()
      gdActive[normal] = gActive ? (fcl->remainsClosed(evalGeneralizedRelativeVelocity()(0), gdTol) ? 1 : 0) : 0;
      gdActive[tangential] = getFrictionDirections() && gdActive[normal] ? (fdf->isSticking(evalGeneralizedRelativeVelocity()(RangeV(1,getFrictionDirections())), gdTol) ? 1 : 0) : 0;
      gddActive[normal] = gdActive[normal];
      gddActive[tangential] = gdActive[tangential];
    }
    else if (j == 3) { // formerly checkActivegdn() (new gap velocities)
      if (gActive) { // contact is closed
        if (evalgdnN() <= gdTol) { // contact stays closed // TODO bilateral contact
          gdActive[normal] = true;
          gddActive[normal] = true;
          if (getFrictionDirections()) {
            if (nrm2(gdnT) <= gdTol) {
              gdActive[tangential] = true;
              gddActive[tangential] = true;
            }
            else {
              gdActive[tangential] = false;
              gddActive[tangential] = false;
            }
          }
        }
        else { // contact will open
          gdActive[normal] = false;
          gdActive[tangential] = false;
          gddActive[normal] = false;
          gddActive[tangential] = false;
        }
      }
    }
    else if (j == 4) { // formerly checkActivegdd()
      if (gActive) {
        if (gdActive[normal]) {
          if (evalgddN() <= gddTol) { // contact stays closed on velocity level
            gddActive[normal] = true;
            if (getFrictionDirections()) {
              if (gdActive[tangential]) {
                if (nrm2(gddT) <= gddTol)
                  gddActive[tangential] = true;
                else
                  gddActive[tangential] = false;
              }
            }
          }
          else { // contact will open on velocity level
            gddActive[normal] = false;
            gddActive[tangential] = false;
          }
        }
      }
    }
    else if (j == 5) { // activity clean-up, if there is no activity on acceleration or velocity level, also more basic levels are set to non-active
      if (gActive) {
        if (gdActive[normal]) {
          if (gdActive[tangential]) {
            if (!gddActive[tangential])
              gdActive[tangential] = false;
          }
          if (!gddActive[normal]) {
            gActive = false;
            gdActive[normal] = false;
            gdActive[tangential] = false;
          }
        }
        else
          gActive = false;
      }
    }
    else if (j == 6) { // just observe closing contact
      if (rootID == 3) {
        gActive = true;
        gdActive[normal] = true;
        gdActive[tangential] = true;
        gddActive[normal] = true;
        gddActive[tangential] = true;
      }
    }
    else if (j == 7) { // just observe slip-stick transitions
      if (getFrictionDirections()) {
        if (rootID == 2) {
          gdActive[tangential] = true;
          gddActive[tangential] = true;
        }
      }
    }
    else if (j == 8) { // just observe opening contacts and stick-slip transitions
      if (jsv(0) && rootID == 1) { // opening contact
        gddActive[normal] = false;
        gddActive[tangential] = false;
      }
      if (getFrictionDirections()) {
        if (jsv(1) && rootID == 1) { // stick-slip transition
          gddActive[tangential] = false;
        }
      }
    }
    else
      throwError("Internal error");
  }

  int SingleContact::getFrictionDirections() const {
    if (fdf)
      return fdf->getFrictionDirections();
    else
      return 0;
  }

  void SingleContact::updatecorr(int j) {
    if (j == 1) { // IG position
      if (gActive) { // Contact was closed
        if (gdActive[normal])
          corr(0) = 0; // Contact stays closed, regular projection
        else
          corr(0) = 1e-14; // Contact opens, projection to positive normal distance
      }
    }
    else if (j == 2) {
      if (gActive && gdActive[normal]) { // Contact was closed
        if (gddActive[normal])
          corr(0) = 0; // Contact stays closed, regular projection
        else
          corr(0) = 1e-14; // Contact opens, projection to positive normal distance
      }
    }
    else if (j == 4) {
      if (rootID == 1) {
        gddN = gddNBuf;
        gddT = gddTBuf;
      }
      if (gActive && gdActive[normal]) { // Contact was closed
        if (gddActive[normal])
          corr(0) = 0; // Contact stays closed, regular projection
        else
          corr(0) = 1e-16; // Contact opens, projection to positive normal distance
        if (getFrictionDirections()) {
          if (gdActive[tangential]) { // Contact was sticking
            if (gddActive[tangential]) {
              corr(1) = 0; // Contact stays sticking, regular projection
              if (getFrictionDirections() > 1)
                corr(2) = 0; // Contact stays sticking, regular projection
            }
            else {
              corr(1) = gddT(0) > 0 ? 1e-16 : -1e-16; // Contact slides, projection to positive normal velocity
              if (getFrictionDirections() > 1)
                corr(2) = gddT(1) > 0 ? 1e-16 : -1e-16; // Contact slides, projection to positive normal velocity
            }
          }
        }
      }
    }
    else
      throwError("Internal error");
  }

  void SingleContact::calccorrSize(int j) {
    ContourLink::calccorrSize(j);
    if (j == 1) { // IG
      corrSize = gActive;
    }
    else if (j == 2) { // IB
      corrSize = gActive * gdActive[normal];
    }
    else if (j == 4) { // IH
      corrSize = gActive * gdActive[normal] * (1 + gdActive[tangential] * getFrictionDirections());
    }
    else
      throwError("Internal error");
  }

  void SingleContact::checkRoot() {
    rootID = 0;
    if (jsv(0)) {
      if (gActive)
        rootID = 1; // contact was closed -> opening
      else
        rootID = 3; // contact was open -> impact
    }
    if (getFrictionDirections()) {
      if (jsv(1)) {
        if (gdActive[tangential])
          rootID = 1; // contact was sticking -> sliding
        else {
          if (getFrictionDirections() == 1)
            rootID = 2; // contact was sliding -> sticking
          else if (nrm2(evalGeneralizedRelativeVelocity()((RangeV(1,getFrictionDirections())))) <= gdTol)
            rootID = 2; // contact was sliding -> sticking
        }
      }
    }
    ds->setRootID(max(ds->getRootID(), rootID));
  }

  void SingleContact::LinearImpactEstimation(double t, Vec &gInActive_, Vec &gdInActive_, int *IndInActive_, Vec &gAct_, int *IndActive_) {
    if (gActive) {
      gAct_(*IndActive_) = evalGeneralizedRelativePosition()(0);
      (*IndActive_)++;
    }
    else {
      // TODO check if already computed
      Vec3 Wn = cFrame[0]->evalOrientation().col(0);
      // TODO check if already computed
      Vec3 WvD = cFrame[1]->evalVelocity() - cFrame[0]->evalVelocity();
      gdInActive_(*IndInActive_) = Wn.T() * WvD;
      gInActive_(*IndInActive_) = evalGeneralizedRelativePosition()(0);
      (*IndInActive_)++;
    }
  }

  void SingleContact::SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) {
    if (gActive)
      (*sizeActive_)++;
    else
      (*sizeInActive_)++;
  }

}
