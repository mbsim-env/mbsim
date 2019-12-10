/* Copyright (C) 2004-2018 MBSim Development Team
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
#include <mbsim/links/disk_contact.h>
#include <mbsim/contours/disk.h>
#include <mbsim/frames/contour_frame.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/constitutive_laws/generalized_force_law.h>
#include <mbsim/constitutive_laws/friction_force_law.h>
#include <mbsim/constitutive_laws/generalized_impact_law.h>
#include <mbsim/constitutive_laws/friction_impact_law.h>
#include <mbsim/objectfactory.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, DiskContact)

  DiskContact::~DiskContact() {
    delete fcl;
    delete fdf;
    delete fnil;
    delete ftil;
  }

  void DiskContact::resetUpToDate() {
    ContourLink::resetUpToDate();
    updlaN = true;
    updlaT = true;
  }

  bool DiskContact::isSticking() const { 
    return laT.size();
  }

  const double& DiskContact::evallaN() {
    if(ds->getUpdatela()) ds->updatela();
    return laN(0);
  }

  const Vec& DiskContact::evallaT() {
    if(ds->getUpdatela()) ds->updatela();
    return laT;
  }

  const double& DiskContact::evalLaN() {
    if(ds->getUpdateLa()) ds->updateLa();
    return LaN(0);
  }

  const Vec& DiskContact::evalLaT() {
    if(ds->getUpdateLa()) ds->updateLa();
    return LaT;
  }

  const double& DiskContact::evalgdnN() {
    if(ds->getUpdateLa()) ds->updateLa();
    return gdnN(0);
  }

  const Vec& DiskContact::evalgdnT() {
    if(ds->getUpdateLa()) ds->updateLa();
    return gdnT;
  }

  const double& DiskContact::evalgddN() {
    if(ds->getUpdatela()) ds->updatela();
    return gddN(0);
  }

  const Vec& DiskContact::evalgddT() {
    if(ds->getUpdatela()) ds->updatela();
    return gddT;
  }

  void DiskContact::updateForce() {
    F[1] = evalGlobalForceDirection().col(0)*evalGeneralizedForce()(0);
    F[0] = -F[1];
    updF = false;
  }

  void DiskContact::updateMoment() {
    M[1] = evalGlobalMomentDirection().col(0)*rE*evalGeneralizedForce()(1);
    M[0] = -M[1];
    updM = false;
  }

  void DiskContact::updateForceDirections() {
    DF.set(0,cFrame[0]->evalOrientation().col(2));
    DM.set(0,cFrame[0]->getOrientation().col(2));
    updDF = false;
  }

  void DiskContact::updateGeneralizedNormalForceM() {
    if(gdActive[normal])
      lambdaN = evallaN();
    else
      lambdaN = 0;
  }

  void DiskContact::updateGeneralizedNormalForceS() {
    lambdaN = (*fcl)(evalGeneralizedRelativePosition()(0), evalGeneralizedRelativeVelocity()(0));
  }

  void DiskContact::updateGeneralizedTangentialForceM() {
    if(gdActive[tangential])
      lambdaT = evallaT();
    else if(gdActive[normal]) {
      Vec gdT = evalGeneralizedRelativeVelocity()(RangeV(1,1));
      lambdaT = fdf->dlaTdlaN(gdT) * evalGeneralizedNormalForce();
      if(gdT(0)*gdTDir<0) lambdaT *= -1.0;
    }
    else
      lambdaT.init(0);
  }

  void DiskContact::updateGeneralizedTangentialForceS() {
    lambdaT = (*fdf)(evalGeneralizedRelativeVelocity()(RangeV(1,1)), fabs(evalGeneralizedNormalForce()));
  }

  void DiskContact::updateGeneralizedForces() {
    lambda(0) = evalGeneralizedNormalForce();
    lambda.set(RangeV(1,lambda.size()-1),evalGeneralizedTangentialForce());
    updla = false;
  }

  void DiskContact::updatePositions(Frame *frame) {
    if(updrrel)
      updateGeneralizedPositions();
  }

  void DiskContact::updateGeneralizedPositions() {
    for(int i=0; i<2; i++) {
      Disk *disk = static_cast<Disk*>(contour[i]);
      cFrame[i]->setOrientation(disk->getFrame()->evalOrientation());
      cFrame[i]->setPosition(disk->getFrame()->getPosition()+(disk->getWidth()/2)*disk->getFrame()->getOrientation().col(2));
    }
    Vec3 WrD = cFrame[1]->getPosition(false) - cFrame[0]->getPosition(false);
    rrel(0) = cFrame[0]->getOrientation(false).col(2).T() * WrD;
    updrrel = false;
  }

  void DiskContact::updateGeneralizedVelocities() {
    Vec3 WvD = cFrame[1]->evalVelocity() - cFrame[0]->evalVelocity();
    Vec3 WomD = cFrame[1]->getAngularVelocity() - cFrame[0]->getAngularVelocity();
    vrel(0) = evalGlobalForceDirection().col(0).T() * WvD;
    vrel(1) = (evalGlobalMomentDirection().col(0).T() * WomD) * rE;
    updvrel = false;
  }

  void DiskContact::updateg() {
    g = evalGeneralizedRelativePosition()(RangeV(0,gSize-1));
  }

  void DiskContact::updategd() {
    gd = evalGeneralizedRelativeVelocity()(RangeV(not(fcl->isSetValued()),not(fcl->isSetValued())+gdSize-1));
  }

  void DiskContact::updateh(int j) {
    Vec3 F = evalGlobalForceDirection().col(0)*evalGeneralizedNormalForce();
    Vec3 M;
    if(not(fdf->isSetValued() and gdActive[tangential]))
      M = evalGlobalMomentDirection().col(0)*rE*evalGeneralizedTangentialForce();

    h[j][0] -= cFrame[0]->evalJacobianOfTranslation(j).T() * F + cFrame[0]->evalJacobianOfRotation(j).T() * M;
    h[j][1] += cFrame[1]->evalJacobianOfTranslation(j).T() * F + cFrame[1]->evalJacobianOfRotation(j).T() * M;
  }

  void DiskContact::updateW(int j) {
    Mat3xV RF(laSize), RM(laSize);
    if(fcl->isSetValued()) RF.set(0, evalGlobalForceDirection().col(0));
    if(fdf->isSetValued() and laSize>1) RM.set(fcl->isSetValued(), evalGlobalMomentDirection().col(0)*rE);

    W[j][0] -= cFrame[0]->evalJacobianOfTranslation(j).T() * RF + cFrame[0]->evalJacobianOfRotation(j).T() * RM;
    W[j][1] += cFrame[1]->evalJacobianOfTranslation(j).T() * RF + cFrame[1]->evalJacobianOfRotation(j).T() * RM;
  }

  void DiskContact::updateV(int j) {
    if (gdActive[normal] and (not gdActive[tangential] or not fdf->isSetValued())) { // with this if-statement for the timestepping integrator it is V=W as it just evaluates checkActive(1)
      Vec gdT = evalGeneralizedRelativeVelocity()(RangeV(1,1));
      Vec3 M = evalGlobalMomentDirection().col(0) * rE * fdf->dlaTdlaN(gdT);
      if(gdT(0)*gdTDir<0) M*=-1.0;
      V[j][0] -= cFrame[0]->evalJacobianOfRotation(j).T() * M;
      V[j][1] += cFrame[1]->evalJacobianOfRotation(j).T() * M;
    }
  }

  void DiskContact::updatewb() {
    if(fcl->isSetValued()) {
      wb(0) -= evalGlobalForceDirection().col(0).T() * cFrame[0]->evalGyroscopicAccelerationOfTranslation();
      wb(0) += evalGlobalForceDirection().col(0).T() * cFrame[1]->evalGyroscopicAccelerationOfTranslation();
    }
    if(fdf->isSetValued() and gdActive[tangential]) {
      wb(fcl->isSetValued()) -= (evalGlobalMomentDirection().col(0).T() * cFrame[0]->evalGyroscopicAccelerationOfRotation())*rE;
      wb(fcl->isSetValued()) += (evalGlobalMomentDirection().col(0).T() * cFrame[1]->evalGyroscopicAccelerationOfRotation())*rE;
    }
  }

  void DiskContact::updateStopVector() {
    if (gActive != gdActive[normal])
      throwError("Internal error");
    if (gActive) {
      if(fcl->isSetValued())
        sv(0) = evalgddN() - gddTol;
      if (fdf->isSetValued()) {
        if (gdActive[tangential])
          sv(fcl->isSetValued()) = nrm2(evalgddT()) - gddTol;
        else
          sv(fcl->isSetValued()) = evalGeneralizedRelativeVelocity()(1)*gdTDir;
      }
    }
    else {
      sv(0) = evalGeneralizedRelativePosition()(0);
      if (fdf->isSetValued())
        sv(fcl->isSetValued()) = 1;
    }
  }

  void DiskContact::updateStopVectorParameters() {
    if(gdActive[normal] and not gdActive[tangential]) {
      Vec gdT = evalGeneralizedRelativeVelocity()(RangeV(1,1));
      gdTDir = gdT(0)>0?1:-1;
    }
  }

  void DiskContact::updatelaRef(const Vec& laParent) {
    ContourLink::updatelaRef(laParent);
    if (laSize) {
      if (fcl->isSetValued())
        laN &= la(0, 0);
      if (fdf->isSetValued())
        laT &= la(fcl->isSetValued(), laSize - 1);
    }
  }

  void DiskContact::updateLaRef(const Vec& LaParent) {
    ContourLink::updateLaRef(LaParent);
    if (laSize) {
      if (fcl->isSetValued())
        LaN &= La(0, 0);
      if (fdf->isSetValued())
        LaT &= La(fcl->isSetValued(), laSize - 1);
    }
  }

  void DiskContact::updategdRef(const Vec& gdParent) {
    ContourLink::updategdRef(gdParent);
    if (gdSize) {
      if (fcl->isSetValued())
        gdN &= gd(0, 0);
      if (fdf->isSetValued())
        gdT &= gd(fcl->isSetValued(), gdSize - 1);
    }
  }

  void DiskContact::calcSize() {
    ng = 1;
    ngd = 2;
    nla = ngd;
    updSize = false;
  }

  void DiskContact::calclaSize(int j) {
    ContourLink::calclaSize(j);
    if (j == 0) { // IA
      //Add 1 to lambda size if normal force law is setValued
      laSize = fcl->isSetValued();

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf->isSetValued())
        laSize += 1;

    }
    else if (j == 1) { // IG
      //Add 1 to lambda size if normal force law is setValued
      laSize = fcl->isSetValued();

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf->isSetValued())
        laSize += 1;

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive;
    }
    else if (j == 2) { // IB
      //Add 1 to lambda size if normal force law is setValued
      laSize = fcl->isSetValued();

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf->isSetValued())
        laSize += 1;

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive * gdActive[normal];

    }
    else if (j == 3) { // IH
      //Add 1 to lambda size if normal force law is setValued
      laSize = fcl->isSetValued();

      //Add number of friction directions to lambda size if friction force law is setValued and active
      if (fdf->isSetValued())
        laSize += gdActive[tangential];

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive * gdActive[normal];

    }
    else if (j == 4) { // IG
      //Add 1 to lambda size if normal force law is setValued and active
      laSize = fcl->isSetValued() * gActive;
    }
    else if (j == 5) { // IB
      laSize = fcl->isSetValued() * gActive * gdActive[normal];
    }
    else
      throwError("Internal error");
  }

  void DiskContact::calcgSize(int j) {
    ContourLink::calcgSize(j);
    if (j == 0) // IA
      gSize = fcl->isSetValued();
    else if (j == 1) // IG
      gSize = fcl->isSetValued() * gActive;
    else if (j == 2) // IB
      gSize = fcl->isSetValued() * gActive * gdActive[normal];
    else
      throwError("Internal error");
  }

  void DiskContact::calcgdSize(int j) {
    // TODO: avoid code duplication for maintenance
    ContourLink::calcgdSize(j);
    if (j == 0) { // all contacts
      // add 1 to gdSize if normal force law is setValued
      gdSize = fcl->isSetValued();

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf->isSetValued())
        gdSize += 1;

    }
    else if (j == 1) { // closed contacts
      // add 1 to gdSize if normal force law is setValued
      gdSize = fcl->isSetValued();

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf->isSetValued())
        gdSize += 1;

      gdSize *= gActive;

    }
    else if (j == 2) { // contacts which stay closed
      // add 1 to gdSize if normal force law is setValued
      gdSize = fcl->isSetValued();

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf->isSetValued())
        gdSize += 1;

      gdSize *= gActive * gdActive[normal];

    }
    else if (j == 3) { // sticking contacts
      // add 1 to gdSize if normal force law is setValued
      gdSize = fcl->isSetValued();

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf->isSetValued())
        gdSize += gdActive[tangential];

      gdSize *= gActive * gdActive[normal];

    }
    else
      throwError("Internal error");
  }

  void DiskContact::calcrFactorSize(int j) {
    ContourLink::calcrFactorSize(j);
    if (j == 0) { // IA
      rFactorSize = fcl->isSetValued();
      if(fdf->isSetValued())
        rFactorSize += 1;
    }
    else if (j == 1) { // IG
      rFactorSize = fcl->isSetValued();
      if(fdf->isSetValued())
        rFactorSize += 1;
      rFactorSize *= gActive;
    }
    else if (j == 2) { // IB
      rFactorSize = fcl->isSetValued();
      if(fdf->isSetValued())
        rFactorSize += 1;
      rFactorSize *= gActive * gdActive[normal];
    }
    else if (j == 3) { // IH
      rFactorSize = fcl->isSetValued();
      if(fdf->isSetValued())
        rFactorSize += gdActive[tangential] * 1;
      rFactorSize *= gActive * gdActive[normal];
    }
  }

  void DiskContact::calcsvSize() {
    ContourLink::calcsvSize();

    //Add length due to normal direction
    svSize = fcl->isSetValued();

    //Add length due to tangentinal direction
    if (fdf->isSetValued())
      svSize += 1;
  }

  void DiskContact::calcLinkStatusSize() {
    ContourLink::calcLinkStatusSize();
    LinkStatusSize = 1;
    LinkStatus.resize(LinkStatusSize);
  }

  void DiskContact::calcLinkStatusRegSize() {
    ContourLink::calcLinkStatusRegSize();
    LinkStatusRegSize = 1;
    LinkStatusReg.resize(LinkStatusRegSize);
  }

  void DiskContact::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(fdf->getFrictionDirections()>1)
        throwError("Spatial friction for disk contacts not yet implemented!");

      double rO = min(static_cast<Disk*>(contour[0])->getOuterRadius(),static_cast<Disk*>(contour[1])->getOuterRadius());
      double rI = max(static_cast<Disk*>(contour[0])->getInnerRadius(),static_cast<Disk*>(contour[1])->getInnerRadius());
      rE = 2./3.*(pow(rO,3)-pow(rI,3))/(pow(rO,2)-pow(rI,2));
      gActive = 1;
      gActive0 = 1;
      gdActive[normal]=1;
      gddActive[normal]=1;
      if(fcl->isSetValued())
        updateGeneralizedNormalForce_ = &DiskContact::updateGeneralizedNormalForceM;
      else
        updateGeneralizedNormalForce_ = &DiskContact::updateGeneralizedNormalForceS;
      gdActive[tangential]=1;
      gddActive[tangential]=1;
      if(fdf->isSetValued())
        updateGeneralizedTangentialForce_ = &DiskContact::updateGeneralizedTangentialForceM;
      else
        updateGeneralizedTangentialForce_ = &DiskContact::updateGeneralizedTangentialForceS;

      RF.resize(2);
      RM.resize(2);

      iF = RangeV(0,0);
      iM = RangeV(0,0);
      DF.resize(1,NONINIT);
      DM.resize(1,NONINIT);

      lambdaT.resize(1);

      gddN.resize(1);
      gddT.resize(1);
      gdnN.resize(1);
      gdnT.resize(1);
    }
    ContourLink::init(stage, config);
    fcl->init(stage, config);
    fdf->init(stage, config);
    if(fnil) fnil->init(stage, config);
    if(ftil) ftil->init(stage, config);
  }

  bool DiskContact::isSetValued() const {
    return ((fcl->isSetValued()) or (fdf->isSetValued()));
  }

  bool DiskContact::isSingleValued() const {
    return not(fcl->isSetValued());
  }

  void DiskContact::updateLinkStatus() {
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

  void DiskContact::updateLinkStatusReg() {
    if (gActive)
      LinkStatusReg(0) = 2;
    else
      LinkStatusReg(0) = 1;
  }

  bool DiskContact::isActive() const {
    return gActive ? true : false;
  }

  bool DiskContact::gActiveChanged() {
    bool changed = (gActive0 != gActive ? true : false);
    gActive0 = gActive;
    return changed;
  }

  bool DiskContact::detectImpact() {
    return gActive0 < gActive ? true : false;
  }

  void DiskContact::setNormalForceLaw(GeneralizedForceLaw *fcl_) { 
    fcl = fcl_; 
    if(fcl) 
      fcl->setParent(this);
  }

  void DiskContact::setNormalImpactLaw(GeneralizedImpactLaw *fnil_) { 
    fnil = fnil_; 
    if(fnil) 
      fnil->setParent(this);
  }

  void DiskContact::setTangentialForceLaw(FrictionForceLaw *fdf_) { 
    fdf = fdf_; 
    if(fdf) 
      fdf->setParent(this);
  }

  void DiskContact::setTangentialImpactLaw(FrictionImpactLaw *ftil_) { 
    ftil = ftil_; 
    if(ftil) 
      ftil->setParent(this);
  }

  void DiskContact::solveImpactsFixpointSingle() {
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
        gdnT(0) = b(laInd + fcl->isSetValued());
        for (int j = ia[laInd + fcl->isSetValued()]; j < ia[laInd + 1 + fcl->isSetValued()]; j++)
          gdnT(0) += a[j] * LaMBS(ja[j]);

        LaT = ftil->project(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), rFactor(fcl->isSetValued()));
      }
    }
  }

  void DiskContact::solveConstraintsFixpointSingle() {
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

      if (fdf->isSetValued() and gdActive[tangential]) {
        gddT(0) = b(laInd + fcl->isSetValued());
        for (int j = ia[laInd + fcl->isSetValued()]; j < ia[laInd + 1 + fcl->isSetValued()]; j++)
          gddT(0) += a[j] * laMBS(ja[j]);

        laT = fdf->project(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, rFactor(fcl->isSetValued()));
      }
    }
  }

  void DiskContact::solveImpactsGaussSeidel() {
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
        gdnT(0) = b(laInd + fcl->isSetValued());
        for (int j = ia[laInd + fcl->isSetValued()] + 1; j < ia[laInd + fcl->isSetValued() + 1]; j++)
          gdnT(0) += a[j] * LaMBS(ja[j]);

        Vec buf = ftil->solve(ds->getG()(RangeV(laInd + fcl->isSetValued(), laInd + 1)), gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize());
        LaT += om * (buf - LaT);
      }
    }
  }

  void DiskContact::solveConstraintsGaussSeidel() {

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

      if (fdf->isSetValued() and gdActive[tangential]) {
        gddT(0) = b(laInd + fcl->isSetValued());
        for (int j = ia[laInd + fcl->isSetValued()] + 1; j < ia[laInd + fcl->isSetValued() + 1]; j++)
          gddT(0) += a[j] * laMBS(ja[j]);

        Vec buf = fdf->solve(ds->getG()(RangeV(laInd + fcl->isSetValued(), laInd + fcl->isSetValued())), gddT, fcl->isSetValued()?laN(0):lambdaN);
        laT += om * (buf - laT);
      }
    }
  }

  void DiskContact::solveImpactsRootFinding() {
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
        gdnT(0) = b(laInd + fcl->isSetValued());
        for (int j = ia[laInd + fcl->isSetValued()]; j < ia[laInd + 1 + fcl->isSetValued()]; j++)
          gdnT(0) += a[j] * LaMBS(ja[j]);
        res(fcl->isSetValued(), fcl->isSetValued()) = LaT - ftil->project(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), rFactor(fcl->isSetValued()));
      }
    }
  }

  void DiskContact::solveConstraintsRootFinding() {
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
      if (fdf->isSetValued() and gdActive[tangential]) {
        gddT(0) = b(laInd + fcl->isSetValued());
        for (int j = ia[laInd + fcl->isSetValued()]; j < ia[laInd + 1 + fcl->isSetValued()]; j++)
          gddT(0) += a[j] * laMBS(ja[j]);
        res(fcl->isSetValued(), fcl->isSetValued()) = laT - fdf->project(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, rFactor(fcl->isSetValued()));
      }
    }
  }

  void DiskContact::jacobianConstraints() {
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

      if(gdActive[tangential] and fdf->isSetValued()) {
        Mat diff = fdf->diff(laT, gddT(0, 0), fcl->isSetValued()?laN(0):lambdaN, rFactor(fcl->isSetValued()));
        RowVec jp2 = ds->getJprox().row(laInd + fcl->isSetValued());
        RowVec e2(jp2.size());
        e2(laInd + 1) = 1;
        Mat e(2, jp2.size());
        e(0, laInd) = 1;
        e(1, laInd + fcl->isSetValued()) = 1;
        jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk)
        //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk)
        for (int i = 0; i < G.size(); i++)
          jp2(i) -= diff(0, 1) * G(laInd + fcl->isSetValued(), i);
      }
    }
  }

  void DiskContact::jacobianImpacts() {
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

      if(fdf->isSetValued()) {
        Mat diff = ftil->diff(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), rFactor(fcl->isSetValued()));
        RowVec jp2 = ds->getJprox().row(laInd + fcl->isSetValued());
        RowVec e2(jp2.size());
        e2(laInd + fcl->isSetValued()) = 1;
        Mat e(2, jp2.size());
        e(0, laInd) = 1;
        e(1, laInd + fcl->isSetValued()) = 1;
        jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk)
        //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk)
        for (int i = 0; i < G.size(); i++)
          jp2(i) -= diff(0, 1) * G(laInd + fcl->isSetValued(), i);
      }
    }
  }

  void DiskContact::updaterFactors() {
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

      if (gdActive[tangential] and fdf->isSetValued()) {
        double sumT1 = 0;
        double aT1;
        for (int j = ia[laInd + fcl->isSetValued()] + 1; j < ia[laInd + fcl->isSetValued() + 1]; j++)
          sumT1 += fabs(a[j]);
        aT1 = a[ia[laInd + fcl->isSetValued()]];
        if (aT1 > sumT1) {
          rFactorUnsure(fcl->isSetValued()) = 0;
          rFactor(fcl->isSetValued()) = 1.0 / aT1;
        }
        else {
          rFactorUnsure(fcl->isSetValued()) = 1;
          rFactor(fcl->isSetValued()) = rMax / aT1;
        }
      }
    }
  }

  void DiskContact::checkConstraintsForTermination() {
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

      if (fdf->isSetValued() and gdActive[tangential]) {

        gddT(0) = b(laInd + fcl->isSetValued());
        for (int j = ia[laInd + fcl->isSetValued()]; j < ia[laInd + 1 + fcl->isSetValued()]; j++)
          gddT(0) += a[j] * laMBS(ja[j]);

        if (not fdf->isFulfilled(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, laTol, gddTol)) {
          ds->setTermination(false);
          return;
        }
      }
    }
  }

  void DiskContact::checkImpactsForTermination() {
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
        gdnT(0) = b(laInd + fcl->isSetValued());
        for (int j = ia[laInd + fcl->isSetValued()]; j < ia[laInd + 1 + fcl->isSetValued()]; j++)
          gdnT(0) += a[j] * LaMBS(ja[j]);
        if (not ftil->isFulfilled(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*getStepSize(), LaTol, gdTol)) {
          ds->setTermination(false);
          return;
        }
      }
    }
  }

  void DiskContact::checkActive(int j) {
    if (j == 1) { // formerly checkActiveg()
      if(fcl->isSetValued()) {
        gActive = fcl->isClosed(evalGeneralizedRelativePosition()(0), gTol) ? 1 : 0;
        gdActive[normal] = gActive;
      }
      if(fdf->isSetValued())
        gdActive[tangential] = gdActive[normal];
    }
    else if (j == 2) { // formerly checkActivegd()
      if(fcl->isSetValued()) {
        gdActive[normal] = gActive ? (fcl->isClosed(evalGeneralizedRelativeVelocity()(0), gdTol) ? 1 : 0) : 0;
        gddActive[normal] = gdActive[normal];
      }
      if(fdf->isSetValued()) {
        Vec gdT = evalGeneralizedRelativeVelocity()(RangeV(1,1));
        gdActive[tangential] = gdActive[normal] ? (fdf->isSticking(gdT, gdTol) ? 1 : 0) : 0;
        gddActive[tangential] = gdActive[tangential];
        if(gdActive[normal] and not gdActive[tangential])
          gdTDir = gdT(0)>0?1:-1;
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
        if (fdf->isSetValued()) {
          if(gdActive[normal]) {
            if (fdf->isSticking(evalgdnT(),gdTol)) {
              gdActive[tangential] = true;
              gddActive[tangential] = true;
            }
            else {
              gdActive[tangential] = false;
              gddActive[tangential] = false;
              gdTDir = gdnT(0)>0?1:-1;
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
        if (fdf->isSetValued()) {
          if(gddActive[normal]) {
            if (gdActive[tangential]) {
               if (fdf->isSticking(evalgddT(),gddTol))
                 gddActive[tangential] = true;
               else {
                 gddActive[tangential] = false;
                 gdTDir = gddT(0)>0?1:-1;
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
      } else if(gdActive[normal] and not gdActive[tangential]) {
        gdActive[tangential] = true;
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
      if (fcl->isSetValued() and jsv(0) and rootID == 1) { // opening contact
        gddActive[normal] = false;
        gddActive[tangential] = false;
      }
      if (fdf->isSetValued() and jsv(fcl->isSetValued()) and rootID == 1) { // stick-slip transition
        const Vec& gddT = evalgddT();
        gddActive[tangential] = false;
        gdTDir = gddT(0)>0?1:-1;
      }
    }
    else
      throwError("Internal error");
  }

  void DiskContact::updatecorr(int j) {
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
        if (fdf->isSetValued() and gdActive[tangential]) { // Contact was sticking
          if (gddActive[tangential])
            corr(fcl->isSetValued()) = 0; // Contact stays sticking, regular projection
          else
            corr(fcl->isSetValued()) = gddT(0) > 0 ? gdCorr : -gdCorr; // Contact slides, projection to valid tangential velocity
        }
      }
    }
    else
      throwError("Internal error");
  }

  void DiskContact::calccorrSize(int j) {
    ContourLink::calccorrSize(j);
    if (j == 1) // IG
      corrSize = fcl->isSetValued() * gActive;
    else if (j == 2) // IB
      corrSize = fcl->isSetValued() * gActive * gdActive[normal];
    else if (j == 4) { // IH
     corrSize = fcl->isSetValued();

      //Add number of friction directions to lambda size if friction force law is setValued and active
      if (fdf->isSetValued())
        corrSize += gdActive[tangential];

      //check if contact is active --> else lambda Size will get zero...
      corrSize *= gActive * gdActive[normal];
    }
    else
      throwError("Internal error");
  }

  void DiskContact::checkRoot() {
    rootID = 0;
    if (fcl->isSetValued() and jsv(0)) {
      if (gActive)
        rootID = 1; // contact was closed -> opening
      else
        rootID = 3; // contact was open -> impact
    }
    if (fdf->isSetValued() and jsv(fcl->isSetValued())) {
      if (gdActive[tangential])
        rootID = 1; // contact was sticking -> sliding
      else
        rootID = 2; // contact was sliding -> sticking
    }
    ds->setRootID(max(ds->getRootID(), rootID));
  }

  void DiskContact::LinearImpactEstimation(double t, Vec &gInActive_, Vec &gdInActive_, int *IndInActive_, Vec &gAct_, int *IndActive_) {
    if (gActive) {
      gAct_(*IndActive_) = evalGeneralizedRelativePosition()(0);
      (*IndActive_)++;
    }
    else {
      // TODO check if already computed
      Vec3 WvD = cFrame[1]->evalVelocity() - cFrame[0]->evalVelocity();
      gdInActive_(*IndInActive_) = evalGlobalForceDirection().col(0).T() * WvD;
      gInActive_(*IndInActive_) = evalGeneralizedRelativePosition()(0);
      (*IndInActive_)++;
    }
  }

  void DiskContact::SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) {
    if (gActive)
      (*sizeActive_)++;
    else
      (*sizeInActive_)++;
  }

  void DiskContact::initializeUsingXML(DOMElement *element) {
    ContourLink::initializeUsingXML(element);
    DOMElement *e;

    //Set contact law
    e = E(element)->getFirstElementChildNamed(MBSIM%"normalForceLaw");
    GeneralizedForceLaw *gfl = ObjectFactory::createAndInit<GeneralizedForceLaw>(e->getFirstElementChild());
    setNormalForceLaw(gfl);

    //Get Impact law
    e = E(element)->getFirstElementChildNamed(MBSIM%"normalImpactLaw");
    if (e) {
      GeneralizedImpactLaw *gifl = ObjectFactory::createAndInit<GeneralizedImpactLaw>(e->getFirstElementChild());
      setNormalImpactLaw(gifl);
    }

    //Get Friction Force Law
    e = E(element)->getFirstElementChildNamed(MBSIM%"tangentialForceLaw");
    FrictionForceLaw *ffl = ObjectFactory::createAndInit<FrictionForceLaw>(e->getFirstElementChild());
    setTangentialForceLaw(ffl);

    //Get Friction Impact Law
    e = E(element)->getFirstElementChildNamed(MBSIM%"tangentialImpactLaw");
    if (e) {
      FrictionImpactLaw *fil = ObjectFactory::createAndInit<FrictionImpactLaw>(e->getFirstElementChild());
      setTangentialImpactLaw(fil);
    }
  }

}
