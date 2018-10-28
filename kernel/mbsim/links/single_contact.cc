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
#include <mbsim/links/single_contact.h>
#include <mbsim/frames/contour_frame.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/constitutive_laws/generalized_force_law.h>
#include <mbsim/constitutive_laws/friction_force_law.h>
#include <mbsim/constitutive_laws/generalized_impact_law.h>
#include <mbsim/constitutive_laws/friction_impact_law.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  void SingleContact::updatewb() {
    wb -= evalGlobalForceDirection()(Range<Fixed<0>,Fixed<2> >(),RangeV(0,laSize-1)).T() * cFrame[0]->evalGyroscopicAccelerationOfTranslation();
    wb += evalGlobalForceDirection()(Range<Fixed<0>,Fixed<2> >(),RangeV(0,laSize-1)).T() * cFrame[1]->evalGyroscopicAccelerationOfTranslation();
  }

  void SingleContact::resetUpToDate() {
    ContourLink::resetUpToDate();
    updlaN = true;
    updlaT = true;
  }

  bool SingleContact::isSticking() const { 
    return laT.size();
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
    else if(gdActive[normal]) {
      Vec gdT = evalGeneralizedRelativeVelocity()(RangeV(1,fdf->getFrictionDirections()));
      lambdaT = fdf->dlaTdlaN(gdT) * evalGeneralizedNormalForce();
      if(gdT.T()*gdTDir<0) lambdaT *= -1.0;
    }
    else
      lambdaT.init(0);
  }

  void SingleContact::updateGeneralizedTangentialForceS() {
    lambdaT = (*fdf)(evalGeneralizedRelativeVelocity()(RangeV(1,fdf->getFrictionDirections())), fabs(evalGeneralizedNormalForce()));
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
    Vec3 Wn = cFrame[0]->evalOrientation().col(0);
    Vec3 WvD = cFrame[1]->evalVelocity() - cFrame[0]->evalVelocity();
    vrel(0) = Wn.T() * WvD;
    if (fdf) {
      Mat3xV Wt(fdf->getFrictionDirections(),NONINIT);
      for(int i=0; i<fdf->getFrictionDirections(); i++)
        Wt.set(i, cFrame[0]->getOrientation().col(i+1));
      vrel.set(RangeV(1,fdf->getFrictionDirections()), Wt.T() * WvD);
    }
    updvrel = false;
  }

  void SingleContact::updatePositions(Frame *frame) {
    if(static_cast<Link*>(parent)->getUpdaterrel())
      static_cast<Link*>(parent)->updateGeneralizedPositions();
  }

  void SingleContact::updateg() {
    g = evalGeneralizedRelativePosition()(RangeV(0,gSize-1));
  }

  void SingleContact::updategd() {
    int IN = fcl->isSetValued()?0:1;
    gd = evalGeneralizedRelativeVelocity()(RangeV(IN,IN+gdSize-1));
  }

  void SingleContact::updateh(int j) {
    Vec3 F = evalGlobalForceDirection().col(0)*evalGeneralizedNormalForce();
    if(fdf and not(fdf->isSetValued() and gdActive[tangential]))
      F += evalGlobalForceDirection()(Range<Fixed<0>,Fixed<2> >(),Range<Var,Var>(1,fdf->getFrictionDirections()))*evalGeneralizedTangentialForce();

    h[j][0] -= cFrame[0]->evalJacobianOfTranslation(j).T() * F;
    h[j][1] += cFrame[1]->evalJacobianOfTranslation(j).T() * F;
  }

  void SingleContact::updateW(int j) {
    int IN = fcl->isSetValued()?0:1;
    Mat3xV RF = evalGlobalForceDirection()(Range<Fixed<0>,Fixed<2> >(),Range<Var,Var>(IN,IN+laSize-1));

    W[j][0] -= cFrame[0]->evalJacobianOfTranslation(j).T() * RF;
    W[j][1] += cFrame[1]->evalJacobianOfTranslation(j).T() * RF;
  }

  void SingleContact::updateV(int j) {
    if (fdf and gdActive[normal] and (not gdActive[tangential] or not fdf->isSetValued())) { // with this if-statement for the timestepping integrator it is V=W as it just evaluates checkActive(1)
      Vec gdT = evalGeneralizedRelativeVelocity()(RangeV(1,fdf->getFrictionDirections()));
      Vec3 F = evalGlobalForceDirection()(Range<Fixed<0>,Fixed<2> >(),RangeV(1, fdf->getFrictionDirections())) * fdf->dlaTdlaN(gdT);
      if(gdT.T()*gdTDir<0) F*=-1.0;
      V[j][0] -= cFrame[0]->evalJacobianOfTranslation(j).T() * F;
      V[j][1] += cFrame[1]->evalJacobianOfTranslation(j).T() * F;
    }
  }

  void SingleContact::updateStopVector() {
    if (gActive != gdActive[normal])
      throwError("Internal error");
    if (gActive) {
      if(fcl->isSetValued())
        sv(0) = evalgddN() - gddTol;
      if (fdf and fdf->isSetValued()) {
        if (gdActive[tangential])
          sv(iN) = nrm2(gddT) - gddTol;
        else
          sv(iN) = evalGeneralizedRelativeVelocity()(RangeV(1,fdf->getFrictionDirections())).T()*gdTDir;
      }
    }
    else {
      sv(0) = evalGeneralizedRelativePosition()(0);
      if (fdf and fdf->isSetValued())
        sv(iN) = 1;
    }
  }

  void SingleContact::updateStopVectorParameters() {
    if(fdf and gdActive[normal] and not gdActive[tangential]) {
      Vec gdT = evalGeneralizedRelativeVelocity()(RangeV(1,fdf->getFrictionDirections()));
      gdTDir = gdT/nrm2(gdT);
    }
  }

  void SingleContact::updatelaRef(const Vec& laParent) {
    ContourLink::updatelaRef(laParent);
    if (laSize) {
      if (fcl->isSetValued())
        laN >> la(0, 0);
      if (fdf and fdf->isSetValued())
        laT >> la(iN, laSize - 1);
    }
  }

  void SingleContact::updateLaRef(const Vec& LaParent) {
    ContourLink::updateLaRef(LaParent);
    if (laSize) {
      if (fcl->isSetValued())
        LaN >> La(0, 0);
      if (fdf and fdf->isSetValued())
        LaT >> La(iN, laSize - 1);
    }
  }

  void SingleContact::updategdRef(const Vec& gdParent) {
    ContourLink::updategdRef(gdParent);
    if (gdSize) {
      if (fcl->isSetValued())
        gdN >> gd(0, 0);
      if (fdf and fdf->isSetValued())
        gdT >> gd(iN, gdSize - 1);
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
      laSize = iN;

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf and fdf->isSetValued())
        laSize += fdf->getFrictionDirections();

    }
    else if (j == 1) { // IG
      //Add 1 to lambda size if normal force law is setValued
      laSize = iN;

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf and fdf->isSetValued())
        laSize += fdf->getFrictionDirections();

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive;
    }
    else if (j == 2) { // IB
      //Add 1 to lambda size if normal force law is setValued
      laSize = iN;

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf and fdf->isSetValued())
        laSize += fdf->getFrictionDirections();

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive * gdActive[normal];

    }
    else if (j == 3) { // IH
      //Add 1 to lambda size if normal force law is setValued
      laSize = iN;

      //Add number of friction directions to lambda size if friction force law is setValued and active
      if (fdf and fdf->isSetValued())
        laSize += fdf->getFrictionDirections() * gdActive[tangential];

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive * gdActive[normal];

    }
    else if (j == 4) { // IG
      //Add 1 to lambda size if normal force law is setValued and active
      laSize = iN * gActive;
    }
    else if (j == 5) { // IB
      laSize = iN * gActive * gdActive[normal];
    }
    else
      throwError("Internal error");
  }

  void SingleContact::calcgSize(int j) {
    ContourLink::calcgSize(j);
    if (j == 0) // IA
      gSize = iN;
    else if (j == 1) // IG
      gSize = iN * gActive;
    else if (j == 2) // IB
      gSize = iN * gActive * gdActive[normal];
    else
      throwError("Internal error");
  }

  void SingleContact::calcgdSize(int j) {
    // TODO: avoid code duplication for maintenance
    ContourLink::calcgdSize(j);
    if (j == 0) { // all contacts
      // add 1 to gdSize if normal force law is setValued
      gdSize = iN;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += fdf->getFrictionDirections();

    }
    else if (j == 1) { // closed contacts
      // add 1 to gdSize if normal force law is setValued
      gdSize = iN;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += fdf->getFrictionDirections();

      gdSize *= gActive;

    }
    else if (j == 2) { // contacts which stay closed
      // add 1 to gdSize if normal force law is setValued
      gdSize = iN;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += fdf->getFrictionDirections();

      gdSize *= gActive * gdActive[normal];

    }
    else if (j == 3) { // sticking contacts
      // add 1 to gdSize if normal force law is setValued
      gdSize = iN;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += gdActive[tangential] * fdf->getFrictionDirections();

      gdSize *= gActive * gdActive[normal];

    }
    else
      throwError("Internal error");
  }

  void SingleContact::calcrFactorSize(int j) {
    ContourLink::calcrFactorSize(j);
    if (j == 0) { // IA
      rFactorSize = iN;
      if(fdf and fdf->isSetValued())
        rFactorSize += 1;
    }
    else if (j == 1) { // IG
      rFactorSize = iN;
      if(fdf and fdf->isSetValued())
        rFactorSize += 1;
      rFactorSize *= gActive;
    }
    else if (j == 2) { // IB
      rFactorSize = iN;
      if(fdf and fdf->isSetValued())
        rFactorSize += 1;
      rFactorSize *= gActive * gdActive[normal];
    }
    else if (j == 3) { // IH
      rFactorSize = iN;
      if(fdf and fdf->isSetValued())
        rFactorSize += gdActive[tangential] * 1;
      rFactorSize *= gActive * gdActive[normal];
    }
  }

  void SingleContact::calcsvSize() {
    ContourLink::calcsvSize();

    //Add length due to normal direction
    svSize = iN;

    //Add length due to tangentinal direction
    if (fdf and fdf->isSetValued())
      svSize += 1;
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
      gdActive[normal]=1;
      gddActive[normal]=1;
      if(fcl) {
        if(fcl->isSetValued()) {
          updateGeneralizedNormalForce_ = &SingleContact::updateGeneralizedNormalForceM;
          iN = 1;
        }
        else
          updateGeneralizedNormalForce_ = &SingleContact::updateGeneralizedNormalForceS;
      }
      else
        updateGeneralizedNormalForce_ = &SingleContact::updateGeneralizedNormalForceP;
      if(fdf) {
        gdActive[tangential]=1;
        gddActive[tangential]=1;
        if(fdf->isSetValued())
          updateGeneralizedTangentialForce_ = &SingleContact::updateGeneralizedTangentialForceM;
        else
          updateGeneralizedTangentialForce_ = &SingleContact::updateGeneralizedTangentialForceS;
      }
      else {
        gdActive[tangential]=0;
        gddActive[tangential]=0;
        updateGeneralizedTangentialForce_ = &SingleContact::updateGeneralizedTangentialForce0;
      }

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

      gdTDir.resize(getFrictionDirections());
    }
    ContourLink::init(stage, config);
    if(fcl) fcl->init(stage, config);
    if(fdf) fdf->init(stage, config);
    if(fnil) fnil->init(stage, config);
    if(ftil) ftil->init(stage, config);
  }

  bool SingleContact::isSetValued() const {
    return ((fcl and fcl->isSetValued()) or (fdf and fdf->isSetValued()));
  }

  bool SingleContact::isSingleValued() const {
    return not(fcl and fcl->isSetValued());
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
    if (gActive)
      LinkStatusReg(0) = 2;
    else
      LinkStatusReg(0) = 1;
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

      if (fcl->isSetValued()) {
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);

        LaN(0) = fnil->project(LaN(0), gdnN(0), gdN(0), rFactor(0));
      }

      if (ftil) {
        for (int i = 0; i < fdf->getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + iN + i);
          for (int j = ia[laInd + i + iN]; j < ia[laInd + 1 + i + iN]; j++)
            gdnT(i) += a[j] * LaMBS(ja[j]);
        }

        LaT = ftil->project(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), rFactor(iN));
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

      if (fcl->isSetValued()) {
        gddN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        laN(0) = fcl->project(laN(0), gddN(0), rFactor(0));
      }

      if (fdf and fdf->isSetValued() and gdActive[tangential]) {
        for (int i = 0; i < fdf->getFrictionDirections(); i++) {
          gddT(i) = b(laInd + i + iN);
          for (int j = ia[laInd + i + iN]; j < ia[laInd + 1 + i + iN]; j++)
            gddT(i) += a[j] * laMBS(ja[j]);
        }

        laT = fdf->project(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, rFactor(iN));
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
      if (fcl->isSetValued()) {
        gdnN(0) = b(laInd);
        for (int j = ia[laInd] + 1; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);

        const double buf = fnil->solve(a[ia[laInd]], gdnN(0), gdN(0));
        LaN(0) += om * (buf - LaN(0));
      }

      if (ftil) {
        gdnT(0) = b(laInd + iN);
        for (int j = ia[laInd + iN] + 1; j < ia[laInd + iN + 1]; j++)
          gdnT(0) += a[j] * LaMBS(ja[j]);

        Vec buf = ftil->solve(ds->getG()(RangeV(laInd + iN, laInd + fdf->getFrictionDirections())), gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize());
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

      if (fcl->isSetValued()) {
        gddN(0) = b(laInd);
        for (int j = ia[laInd] + 1; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        const double buf = fcl->solve(a[ia[laInd]], gddN(0));
        laN(0) += om * (buf - laN(0));
      }

      if (fdf and fdf->isSetValued() and gdActive[tangential]) {
        gddT(0) = b(laInd + iN);
        for (int j = ia[laInd + iN] + 1; j < ia[laInd + iN + 1]; j++)
          gddT(0) += a[j] * laMBS(ja[j]);

        Vec buf = fdf->solve(ds->getG()(RangeV(laInd + iN, laInd + iN + fdf->getFrictionDirections() - 1)), gddT, fcl->isSetValued()?laN(0):lambdaN);
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
      if (fcl->isSetValued()) {
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);

        res(0) = LaN(0) - fnil->project(LaN(0), gdnN(0), gdN(0), rFactor(0));
      }

      //compute residuum for tangential directions
      if (ftil) {
        for (int i = 0; i < fdf->getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + i + iN);
          for (int j = ia[laInd + i + iN]; j < ia[laInd + 1 + i + iN]; j++)
            gdnT(i) += a[j] * LaMBS(ja[j]);
        }
        res(iN, iN + fdf->getFrictionDirections() - 1) = LaT - ftil->project(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), rFactor(iN));
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
      if (fcl->isSetValued()) {
        gddN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        res(0) = laN(0) - fcl->project(laN(0), gddN(0), rFactor(0));
      }

      //compute residuum for tangential directions
      if (fdf and fdf->isSetValued() and gdActive[tangential]) {
        for (int i = 0; i < fdf->getFrictionDirections(); i++) {
          gddT(i) = b(laInd + i + iN);
          for (int j = ia[laInd + i + iN]; j < ia[laInd + 1 + i + iN]; j++)
            gddT(i) += a[j] * laMBS(ja[j]);
        }
        res(iN, iN + fdf->getFrictionDirections() - 1) = laT - fdf->project(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, rFactor(iN));
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

      if(fcl->isSetValued()) {
        Vec diff = fcl->diff(laN(0), gddN(0), rFactor(0));

        jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+laIndk)
        for (int i = 0; i < G.size(); i++)
          jp1(i) -= diff(1) * G(laInd, i);
      }

      if(fdf and gdActive[tangential] and fdf->isSetValued()) {
        if (fdf->getFrictionDirections() == 1) {
          Mat diff = fdf->diff(laT, gddT(0, 0), fcl->isSetValued()?laN(0):lambdaN, rFactor(iN));
          RowVec jp2 = ds->getJprox().row(laInd + iN);
          RowVec e2(jp2.size());
          e2(laInd + 1) = 1;
          Mat e(2, jp2.size());
          e(0, laInd) = 1;
          e(1, laInd + iN) = 1;
          jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk)
          //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk)
          for (int i = 0; i < G.size(); i++)
            jp2(i) -= diff(0, 1) * G(laInd + iN, i);

        }
        else if (fdf->getFrictionDirections() == 2) {
          Mat diff = ftil->diff(laT, gddT, gdT, fcl->isSetValued()?laN(0):lambdaN, rFactor(iN));
          Mat jp2 = ds->getJprox()(RangeV(laInd + iN, laInd + iN + 1), RangeV(0, ds->getJprox().cols() - 1));
          Mat e2(2, jp2.cols());
          e2(0, laInd + iN) = 1;
          e2(1, laInd + iN + 1) = 1;
          jp2 = e2 - diff(RangeV(0, 1), RangeV(4, 4)) * e1 - diff(RangeV(0, 1), RangeV(0, 1)) * e2; // -diff(RangeV(0,1),RangeV(4,5))*G(RangeV(laInd+laIndk+1,laInd+laIndk+2),RangeV(0,G.size()-1))
          for (int i = 0; i < G.size(); i++) {
            jp2(0, i) = diff(0, 2) * G(laInd + iN, i) + diff(0, 3) * G(laInd + iN + 1, i);
            jp2(1, i) = diff(1, 2) * G(laInd + iN, i) + diff(1, 3) * G(laInd + iN + 1, i);
          }
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

      if (fcl->isSetValued()) {

        Vec diff = fnil->diff(LaN(0), gdnN(0), gdN(0), rFactor(0));

        jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+laIndk)
        for (int i = 0; i < G.size(); i++)
          jp1(i) -= diff(1) * G(laInd, i);
      }

      if(fdf and fdf->isSetValued()) {
        if (fdf->getFrictionDirections() == 1) {
          Mat diff = ftil->diff(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), rFactor(iN));
          RowVec jp2 = ds->getJprox().row(laInd + iN);
          RowVec e2(jp2.size());
          e2(laInd + iN) = 1;
          Mat e(2, jp2.size());
          e(0, laInd) = 1;
          e(1, laInd + iN) = 1;
          jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk)
          //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk)
          for (int i = 0; i < G.size(); i++)
            jp2(i) -= diff(0, 1) * G(laInd + iN, i);

        }
        else if (fdf->getFrictionDirections() == 2) {
          Mat diff = ftil->diff(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), rFactor(iN));
          Mat jp2 = ds->getJprox()(RangeV(laInd + iN, laInd + iN + 1), RangeV(0, ds->getJprox().cols() - 1));
          Mat e2(2, jp2.cols());
          e2(0, laInd + iN) = 1;
          e2(1, laInd + iN + 1) = 1;
          jp2 = e2 - diff(RangeV(0, 1), RangeV(4, 4)) * e1 - diff(RangeV(0, 1), RangeV(0, 1)) * e2; // -diff(RangeV(0,1),RangeV(4,5))*G(RangeV(laInd+laIndk+1,laInd+laIndk+2),RangeV(0,G.size()-1))
          for (int i = 0; i < G.size(); i++) {
            jp2(0, i) -= diff(0, 2) * G(laInd + iN, i) + diff(0, 3) * G(laInd + iN + 1, i);
            jp2(1, i) -= diff(1, 2) * G(laInd + iN, i) + diff(1, 3) * G(laInd + iN + 1, i);
          }
        }
      }
    }
  }

  void SingleContact::updaterFactors() {
    if (gdActive[normal]) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();

      if (fcl->isSetValued()) {
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
        if (fdf->getFrictionDirections() == 1) {
          for (int j = ia[laInd + iN] + 1; j < ia[laInd + iN + 1]; j++)
            sumT1 += fabs(a[j]);
          aT1 = a[ia[laInd + iN]];
          if (aT1 > sumT1) {
            rFactorUnsure(iN) = 0;
            rFactor(iN) = 1.0 / aT1;
          }
          else {
            rFactorUnsure(iN) = 1;
            rFactor(iN) = rMax / aT1;
          }
        }
        else if (fdf->getFrictionDirections() == 2) {
          for (int j = ia[laInd + iN] + 1; j < ia[laInd + iN + 1]; j++)
            sumT1 += fabs(a[j]);
          for (int j = ia[laInd + iN + 1] + 1; j < ia[laInd + iN + 2]; j++)
            sumT2 += fabs(a[j]);
          aT1 = a[ia[laInd + iN]];
          aT2 = a[ia[laInd + iN + 1]];

          // TODO rFactorUnsure
          if (aT1 - sumT1 >= aT2 - sumT2)
            if (aT1 + sumT1 >= aT2 + sumT2)
              rFactor(iN) = 2.0 / (aT1 + aT2 + sumT1 - sumT2);
            else
              rFactor(iN) = 1.0 / aT2;
          else if (aT1 + sumT1 < aT2 + sumT2)
            rFactor(iN) = 2.0 / (aT1 + aT2 - sumT1 + sumT2);
          else
            rFactor(iN) = 1.0 / aT1;
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

      if (fcl->isSetValued()) {

        gddN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        if (not fcl->isFulfilled(laN(0), gddN(0), laTol, gddTol)) {
          ds->setTermination(false);
          return;
        }
      }

      if (fdf and fdf->isSetValued() and gdActive[tangential]) {

        for (int i = 0; i < fdf->getFrictionDirections(); i++) {
          gddT(i) = b(laInd + i + iN);
          for (int j = ia[laInd + i + iN]; j < ia[laInd + 1 + i + iN]; j++)
            gddT(i) += a[j] * laMBS(ja[j]);
        }

        if (not fdf->isFulfilled(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, laTol, gddTol)) {
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

      if (fcl->isSetValued()) {
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);
        if (not fnil->isFulfilled(LaN(0), gdnN(0), gdN(0), LaTol, gdTol)) {
          ds->setTermination(false);
          return;
        }
      }

      if (ftil) {
        for (int i = 0; i < fdf->getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + i + iN);
          for (int j = ia[laInd + i + iN]; j < ia[laInd + 1 + i + iN]; j++)
            gdnT(i) += a[j] * LaMBS(ja[j]);
        }
        if (not ftil->isFulfilled(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), LaTol, gdTol)) {
          ds->setTermination(false);
          return;
        }
      }
    }
  }

  void SingleContact::checkActive(int j) {
    if (j == 1) { // formerly checkActiveg()
      if(fcl->isSetValued()) {
        gActive = fcl->isClosed(evalGeneralizedRelativePosition()(0), gTol) ? 1 : 0;
        gdActive[normal] = gActive;
      }
      if(fdf and fdf->isSetValued())
        gdActive[tangential] = gdActive[normal];
    }
    else if (j == 2) { // formerly checkActivegd()
      if(fcl->isSetValued()) {
        gdActive[normal] = gActive ? (fcl->isClosed(evalGeneralizedRelativeVelocity()(0), gdTol) ? 1 : 0) : 0;
        gddActive[normal] = gdActive[normal];
      }
      if(fdf and fdf->isSetValued()) {
        Vec gdT = evalGeneralizedRelativeVelocity()(RangeV(1,fdf->getFrictionDirections()));
        gdActive[tangential] = gdActive[normal] ? (fdf->isSticking(gdT, gdTol) ? 1 : 0) : 0;
        gddActive[tangential] = gdActive[tangential];
        if(gdActive[normal] and not gdActive[tangential])
          gdTDir = gdT/nrm2(gdT);
      }
    }
    else if (j == 3) { // formerly checkActivegdn() (new gap velocities)
      if (gActive) { // contact is closed
        if(fcl->isSetValued()) {
          if (fcl->isClosed(evalgdnN(), gdTol)) { // contact stays closed
            gdActive[normal] = true;
            gddActive[normal] = true;
          }
          else {
            gdActive[normal] = false;
            gddActive[normal] = false;
          }
        }
        if (fdf and fdf->isSetValued()) {
          if(gdActive[normal]) {
            if (fdf->isSticking(gdnT,gdTol)) {
              gdActive[tangential] = true;
              gddActive[tangential] = true;
            }
            else {
              gdActive[tangential] = false;
              gddActive[tangential] = false;
              gdTDir = gdnT/nrm2(gdnT);
            }
          }
          else { // contact will open
            gdActive[tangential] = false;
            gddActive[tangential] = false;
          }
        }
      }
    }
    else if (j == 4) { // formerly checkActivegdd()
      if (gActive and gdActive[normal]) {
        if(fcl->isSetValued()) {
          if (fcl->isClosed(evalgddN(),gddTol)) // contact stays closed on velocity level
            gddActive[normal] = true;
          else // contact will open on velocity level
            gddActive[normal] = false;
        }
        if (fdf and fdf->isSetValued()) {
          if(gddActive[normal]) {
            if (gdActive[tangential]) {
               if (fdf->isSticking(gddT,gddTol))
                 gddActive[tangential] = true;
               else {
                 gddActive[tangential] = false;
                 gdTDir = gddT/nrm2(gddT);
               }
            }
          }
          else // contact will open on velocity level
            gddActive[tangential] = false;
        }
      }
    }
    else if (j == 5) { // activity clean-up, if there is no activity on acceleration or velocity level, also more basic levels are set to non-active
      if (gActive) {
        if (gdActive[normal]) {
          if (gdActive[tangential]) {
            if (not gddActive[tangential])
              gdActive[tangential] = false;
          }
          if (not gddActive[normal]) {
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
      if (rootID == 2) {
        gdActive[tangential] = true;
        gddActive[tangential] = true;
      }
    }
    else if (j == 8) { // just observe opening contacts and stick-slip transitions
      if (jsv(0) and rootID == 1) { // opening contact
        gddActive[normal] = false;
        gddActive[tangential] = false;
      }
      if (getFrictionDirections()) {
        if (jsv(1) and rootID == 1) { // stick-slip transition
          gddActive[tangential] = false;
          gdTDir = gddT/nrm2(gddT);
        }
      }
    }
    else
      throwError("Internal error");
  }

  int SingleContact::getFrictionDirections() const {
    return fdf ? fdf->getFrictionDirections() : 0;
  }

  void SingleContact::updatecorr(int j) {
    if (j == 1) { // IG position
      if (fcl->isSetValued() and gActive) { // Contact was closed
        if (gdActive[normal])
          corr(0) = 0; // Contact stays closed, regular projection
        else
          corr(0) = gCorr; // Contact opens, projection to positive normal distance
      }
    }
    else if (j == 2) {
      if(fcl->isSetValued() and gActive and gdActive[normal]) { // Contact was closed
        if (gddActive[normal])
          corr(0) = 0; // Contact stays closed, regular projection
        else
          corr(0) = gCorr; // Contact opens, projection to positive normal distance
      }
    }
    else if (j == 4) {
      if (gActive and gdActive[normal]) { // Contact was closed
        if(fcl->isSetValued()) {
          if (gddActive[normal])
            corr(0) = 0; // Contact stays closed, regular projection
          else
            corr(0) = gdCorr; // Contact opens, projection to positive normal velocity
        }
        if (fdf and fdf->isSetValued() and gdActive[tangential]) { // Contact was sticking
          if (gddActive[tangential]) {
            for(int i=0; i<fdf->getFrictionDirections(); i++)
            corr(i+1) = 0; // Contact stays sticking, regular projection
          }
          else {
            for(int i=0; i<fdf->getFrictionDirections(); i++)
              corr(i+1) = gddT(i) > 0 ? gdCorr : -gdCorr; // Contact slides, projection to valid tangential velocity
          }
        }
      }
    }
    else
      throwError("Internal error");
  }

  void SingleContact::calccorrSize(int j) {
    ContourLink::calccorrSize(j);
    if (j == 1) // IG
      corrSize = iN * gActive;
    else if (j == 2) // IB
      corrSize = iN * gActive * gdActive[normal];
    else if (j == 4) { // IH
     corrSize = iN;

      //Add number of friction directions to lambda size if friction force law is setValued and active
      if (fdf and fdf->isSetValued())
        corrSize += fdf->getFrictionDirections() * gdActive[tangential];

      //check if contact is active --> else lambda Size will get zero...
      corrSize *= gActive * gdActive[normal];
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
    if (fdf and jsv(1)) {
      if (gdActive[tangential])
        rootID = 1; // contact was sticking -> sliding
      else
        rootID = 2; // contact was sliding -> sticking
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
