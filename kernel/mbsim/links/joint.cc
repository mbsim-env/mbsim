/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/links/joint.h"
#include "mbsim/constitutive_laws/bilateral_impact.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/objectfactory.h"
#include <mbsim/constitutive_laws/generalized_force_law.h>
#include <mbsim/utils/utils.h>
#include <mbsim/objects/rigid_body.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Joint)

  Joint::Joint(const string &name) : FloatingFrameLink(name), ffl(0), fml(0), fifl(0), fiml(0) {
  }

  Joint::~Joint() {
    delete ffl;
    delete fml;
    delete fifl;
    delete fiml;
  }

  void Joint::updatewb() {
    Mat3xV WJT = refFrame->evalOrientation() * JT;
    VecV sdT = WJT.T() * evalGlobalRelativeVelocity();

    wb(0, forceDir.cols() - 1) += evalGlobalForceDirection().T() * (frame[1]->evalGyroscopicAccelerationOfTranslation() - C.evalGyroscopicAccelerationOfTranslation() - crossProduct(C.evalAngularVelocity(), evalGlobalRelativeVelocity() + WJT * sdT));
    wb(forceDir.cols(), momentDir.cols() + forceDir.cols() - 1) += evalGlobalMomentDirection().T() * (frame[1]->evalGyroscopicAccelerationOfRotation() - C.evalGyroscopicAccelerationOfRotation() - crossProduct(C.evalAngularVelocity(), evalGlobalRelativeAngularVelocity()));
  }

  void Joint::updatelaFM() {
    for (int i = 0; i < forceDir.cols(); i++)
      lambdaF(i) = evalla()(i);
  }

  void Joint::updatelaFS() {
    for (int i = 0; i < forceDir.cols(); i++)
      lambdaF(i) = (*ffl)(evalGeneralizedRelativePosition()(i), evalGeneralizedRelativeVelocity()(i));
  }

  void Joint::updatelaMM() {
    for (int i = forceDir.cols(), j=0; i < forceDir.cols() + momentDir.cols(); i++, j++)
      lambdaM(j) = evalla()(i);
  }

  void Joint::updatelaMS() {
    for (int i = forceDir.cols(), j=0; i < forceDir.cols() + momentDir.cols(); i++, j++)
      lambdaM(j) = (*fml)(evalGeneralizedRelativePosition()(i), evalGeneralizedRelativeVelocity()(i));
  }

  void Joint::updatexd() {
    xd = evalGeneralizedRelativeVelocity()(iM);
  }

  void Joint::updateh(int j) {
    Vec3 F = (ffl and not ffl->isSetValued())?evalGlobalForceDirection()*evallaF():Vec3();
    Vec3 M = (fml and not fml->isSetValued())?evalGlobalMomentDirection()*evallaM():Vec3();

    h[j][0] -= C.evalJacobianOfTranslation(j).T() * F + C.evalJacobianOfRotation(j).T() * M;
    h[j][1] += frame[1]->evalJacobianOfTranslation(j).T() * F + frame[1]->evalJacobianOfRotation(j).T() * M;
  }

  void Joint::updateW(int j) {
    int nF = (ffl and ffl->isSetValued())?forceDir.cols():0;
    int nM = (fml and fml->isSetValued())?momentDir.cols():0;
    Mat3xV RF(nF+nM);
    Mat3xV RM(RF.cols());
    RF.set(RangeV(0,2), RangeV(0,nF-1), evalGlobalForceDirection()(RangeV(0,2),RangeV(0,nF-1)));
    RM.set(RangeV(0,2), RangeV(nF,nF+nM-1), evalGlobalMomentDirection()(RangeV(0,2),RangeV(0,nM-1)));

    W[j][0] -= C.evalJacobianOfTranslation(j).T() * RF + C.evalJacobianOfRotation(j).T() * RM;
    W[j][1] += frame[1]->evalJacobianOfTranslation(j).T() * RF + frame[1]->evalJacobianOfRotation(j).T() * RM;
  }

  void Joint::calcxSize() {
    FloatingFrameLink::calcxSize();
    xSize = momentDir.cols();
  }

  void Joint::init(InitStage stage, const InitConfigSet &config) {
    if (stage == unknownStage) {
      gdd.resize(gdSize);
      gdn.resize(gdSize);

      if (ffl) {
        fifl = new BilateralImpact;
        fifl->setParent(this);
      }
      if (fml) {
        fiml = new BilateralImpact;
        fiml->setParent(this);
      }

      JT.resize(3 - forceDir.cols());
      if (forceDir.cols() == 2)
        JT.set(0, crossProduct(forceDir.col(0), forceDir.col(1)));
      else if (forceDir.cols() == 3)
        ;
      else if (forceDir.cols() == 0)
        JT = SqrMat(3, EYE);
      else { // define a coordinate system in the plane perpendicular to the force direction
        JT.set(0, computeTangential(forceDir.col(0)));
        JT.set(1, crossProduct(forceDir.col(0), JT.col(0)));
      }
      if(not ffl)
        updatelaF_ = &Joint::updatelaF0;
      else if(ffl->isSetValued())
        updatelaF_ = &Joint::updatelaFM;
      else
        updatelaF_ = &Joint::updatelaFS;
      if(not fml)
        updatelaM_ = &Joint::updatelaM0;
      else if(fml->isSetValued())
        updatelaM_ = &Joint::updatelaMM;
      else
        updatelaM_ = &Joint::updatelaMS;
    }
    FloatingFrameLink::init(stage, config);
    if(ffl) ffl->init(stage, config);
    if(fml) fml->init(stage, config);
    if(fifl) fifl->init(stage, config);
    if(fiml) fiml->init(stage, config);
  }

  bool Joint::isSetValued() const {
    bool flag = false;
    if (ffl)
      flag |= ffl->isSetValued();
    if (fml)
      flag |= fml->isSetValued();

    return flag;
  }

  bool Joint::isSingleValued() const {
    bool flag = false;
    if (ffl)
      if (not ffl->isSetValued())
        flag = true;
    if (fml)
      if (not fml->isSetValued())
        flag = true;

    return flag;
  }

  void Joint::solveImpactsFixpointSingle() {

    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      La(i) = fifl->project(La(i), gdn(i), gd(i), rFactor(i));
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      La(i) = fiml->project(La(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void Joint::solveConstraintsFixpointSingle() {

    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      la(i) = ffl->project(la(i), gdd(i), rFactor(i));
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      la(i) = fml->project(la(i), gdd(i), rFactor(i));
    }
  }

  void Joint::solveImpactsGaussSeidel() {

    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i] + 1; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      La(i) = fifl->solve(a[ia[laInd + i]], gdn(i), gd(i));
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i] + 1; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      La(i) = fiml->solve(a[ia[laInd + i]], gdn(i), gd(i));
    }
  }

  void Joint::solveConstraintsGaussSeidel() {

    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i] + 1; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      la(i) = ffl->solve(a[ia[laInd + i]], gdd(i));
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i] + 1; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      la(i) = fml->solve(a[ia[laInd + i]], gdd(i));
    }
  }

  void Joint::solveImpactsRootFinding() {

    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      res(i) = La(i) - fifl->project(La(i), gdn(i), gd(i), rFactor(i));
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      res(i) = La(i) - fiml->project(La(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void Joint::solveConstraintsRootFinding() {

    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      res(i) = la(i) - ffl->project(la(i), gdd(i), rFactor(i));
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      res(i) = la(i) - fml->project(la(i), gdd(i), rFactor(i));
    }
  }

  void Joint::jacobianConstraints() {

    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->evalG();

    for (int i = 0; i < forceDir.cols(); i++) {
      RowVec jp1 = Jprox.row(laInd + i);
      RowVec e1(jp1.size());
      e1(laInd + i) = 1;
      Vec diff = ffl->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+i)
      for (int j = 0; j < G.size(); j++)
        jp1(j) -= diff(1) * G(laInd + i, j);
    }

    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {

      RowVec jp1 = Jprox.row(laInd + i);
      RowVec e1(jp1.size());
      e1(laInd + i) = 1;
      Vec diff = fml->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+i)
      for (int j = 0; j < G.size(); j++)
        jp1(j) -= diff(1) * G(laInd + i, j);
    }
  }

  void Joint::jacobianImpacts() {

    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->evalG();

    for (int i = 0; i < forceDir.cols(); i++) {
      RowVec jp1 = Jprox.row(laInd + i);
      RowVec e1(jp1.size());
      e1(laInd + i) = 1;
      Vec diff = fifl->diff(La(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+i)
      for (int j = 0; j < G.size(); j++)
        jp1(j) -= diff(1) * G(laInd + i, j);
    }

    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      RowVec jp1 = Jprox.row(laInd + i);
      RowVec e1(jp1.size());
      e1(laInd + i) = 1;
      Vec diff = fiml->diff(La(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+i)
      for (int j = 0; j < G.size(); j++)
        jp1(j) -= diff(1) * G(laInd + i, j);
    }
  }

  void Joint::updaterFactors() {
    if (isActive()) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();

      for (int i = 0; i < rFactorSize; i++) {
        double sum = 0;
        for (int j = ia[laInd + i] + 1; j < ia[laInd + i + 1]; j++)
          sum += fabs(a[j]);
        double ai = a[ia[laInd + i]];
        if (ai > sum) {
          rFactorUnsure(i) = 0;
          rFactor(i) = 1.0 / ai;
        }
        else {
          rFactorUnsure(i) = 1;
          rFactor(i) = 1.0 / ai;
        }
      }
    }
  }

  void Joint::checkImpactsForTermination() {

    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      if (!fifl->isFulfilled(La(i), gdn(i), gd(i), LaTol, gdTol)) {
        ds->setTermination(false);
        return;
      }
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      if (!fiml->isFulfilled(La(i), gdn(i), gd(i), LaTol, gdTol)) {
        ds->setTermination(false);
        return;
      }
    }
  }

  void Joint::checkConstraintsForTermination() {

    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      if (!ffl->isFulfilled(la(i), gdd(i), laTol, gddTol)) {
        ds->setTermination(false);
        return;
      }
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      if (!fml->isFulfilled(la(i), gdd(i), laTol, gddTol)) {
        ds->setTermination(false);
        return;
      }
    }
  }

  void Joint::setForceLaw(GeneralizedForceLaw * rc) {
    ffl = rc;
    ffl->setParent(this);
  }

  void Joint::setMomentLaw(GeneralizedForceLaw * rc) {
    fml = rc;
    fml->setParent(this);
  }

  void Joint::setForceDirection(const Mat3xV &fd) {

    forceDir = fd;

    for (int i = 0; i < fd.cols(); i++)
      forceDir.set(i, forceDir.col(i) / nrm2(fd.col(i)));
  }

  void Joint::setMomentDirection(const Mat3xV &md) {

    momentDir = md;

    for (int i = 0; i < md.cols(); i++)
      momentDir.set(i, momentDir.col(i) / nrm2(md.col(i)));
  }

  void Joint::initializeUsingXML(DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    if(e) setForceDirection(E(e)->getText<Mat>(3,0));
    e = E(element)->getFirstElementChildNamed(MBSIM%"forceLaw");
    if(e) setForceLaw(ObjectFactory::createAndInit<GeneralizedForceLaw>(e->getFirstElementChild()));
    e = E(element)->getFirstElementChildNamed(MBSIM%"momentDirection");
    if(e) setMomentDirection(E(e)->getText<Mat>(3,0));
    e = E(element)->getFirstElementChildNamed(MBSIM%"momentLaw");
    if(e) setMomentLaw(ObjectFactory::createAndInit<GeneralizedForceLaw>(e->getFirstElementChild()));
  }

  InverseKineticsJoint::InverseKineticsJoint(const string &name) : Joint(name), body(0) {
  }

  void InverseKineticsJoint::calcbSize() {
    if(body) bSize = body->getPJT(0,false).cols();
  }

  void InverseKineticsJoint::updateb() {
    if(body) {
      b(RangeV(0, bSize - 1), RangeV(0, 2)) = body->evalPJT().T();
      b(RangeV(0, bSize - 1), RangeV(3, 5)) = body->evalPJR().T();
    }
  }

  void InverseKineticsJoint::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit)
      x.resize(momentDir.cols());
    Joint::init(stage, config);
  }

}
