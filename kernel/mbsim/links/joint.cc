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
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Joint)

  Joint::~Joint() {
    delete ffl;
    delete fml;
    delete fifl;
    delete fiml;
  }

  void Joint::updateg() {
    if(iFM.size()) g(iFM) = evalGeneralizedRelativePosition()(iF);
    if(iMM.size()) g(iMM) = evalGeneralizedRelativePosition()(iM);;
  }

  void Joint::updategd() {
    if(iFM.size()) gd(iFM) = evalGeneralizedRelativeVelocity()(iF);
    if(iMM.size()) gd(iMM) = evalGeneralizedRelativeVelocity()(iM);
  }

  void Joint::updatewb() {
    if(iFM.size()) wb(iFM) += evalGlobalForceDirection().T() * (frame[1]->evalGyroscopicAccelerationOfTranslation() - C.evalGyroscopicAccelerationOfTranslation() - crossProduct(C.evalAngularVelocity(), 2.0*evalGlobalRelativeVelocity()));
    if(iMM.size()) wb(iMM) += evalGlobalMomentDirection().T() * (frame[1]->evalGyroscopicAccelerationOfRotation() - C.evalGyroscopicAccelerationOfRotation() - crossProduct(C.evalAngularVelocity(), evalGlobalRelativeAngularVelocity()));
  }

  void Joint::updatexd() {
    if(integrateGeneralizedRelativeVelocityOfRotation)
      xd = evalGeneralizedRelativeVelocity()(iM);
  }

  void Joint::updatelaFM() {
    for (int i = 0; i < iFM.size(); i++)
      lambdaF(i) = evalla()(i);
  }

  void Joint::updatelaFS() {
    for (int i = 0; i < forceDir.cols(); i++)
      lambdaF(i) = (*ffl)(evalGeneralizedRelativePosition()(i), evalGeneralizedRelativeVelocity()(i));
  }

  void Joint::updatelaMM() {
    for (int i = iFM.size(), j=0; i < laSize; i++, j++)
      lambdaM(j) = evalla()(i);
  }

  void Joint::updatelaMS() {
    for (int i = forceDir.cols(), j=0; i < forceDir.cols() + momentDir.cols(); i++, j++)
      lambdaM(j) = (*fml)(evalGeneralizedRelativePosition()(i), evalGeneralizedRelativeVelocity()(i));
  }

  void Joint::updateh(int j) {
    Vec3 F = iFM.size()?Vec3():evalGlobalForceDirection()*evallaF();
    Vec3 M = iMM.size()?Vec3():evalGlobalMomentDirection()*evallaM();

    h[j][0] -= C.evalJacobianOfTranslation(j).T() * F + C.evalJacobianOfRotation(j).T() * M;
    h[j][1] += frame[1]->evalJacobianOfTranslation(j).T() * F + frame[1]->evalJacobianOfRotation(j).T() * M;
  }

  void Joint::updateW(int j) {
    if(iFM.size()) RF.set(RangeV(0,2), RangeV(iFM), evalGlobalForceDirection());
    if(iMM.size()) RM.set(RangeV(0,2), RangeV(iMM), evalGlobalMomentDirection());

    W[j][0] -= C.evalJacobianOfTranslation(j).T() * RF + C.evalJacobianOfRotation(j).T() * RM;
    W[j][1] += frame[1]->evalJacobianOfTranslation(j).T() * RF + frame[1]->evalJacobianOfRotation(j).T() * RM;
  }

  void Joint::calclaSize(int j) {
    laSize = iFM.size() + iMM.size();
  }

  void Joint::calcgSize(int j) {
    gSize = iFM.size() + iMM.size();
  }

  void Joint::calcgdSize(int j) {
    gdSize = iFM.size() + iMM.size();
  }

  void Joint::calcrFactorSize(int j) {
    rFactorSize = iFM.size() + iMM.size();
  }

  void Joint::calccorrSize(int j) {
    corrSize = iFM.size() + iMM.size();
  }

  void Joint::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if (ffl) {
        if(ffl->isSetValued()) {
          fifl = new BilateralImpact;
          fifl->setParent(this);
          updatelaF_ = &Joint::updatelaFM;
          iFM = RangeV(0,forceDir.cols()-1);
        }
        else
          updatelaF_ = &Joint::updatelaFS;
      }
      else {
        if(forceDir.cols()>0) throwError("No force law is given!");
        updatelaF_ = &Joint::updatelaF0;
      }
      if (fml) {
        if(fml->isSetValued()) {
          fiml = new BilateralImpact;
          fiml->setParent(this);
          updatelaM_ = &Joint::updatelaMM;
          iMM = RangeV(iFM.size(),iFM.size()+momentDir.cols()-1);
        }
        else
          updatelaM_ = &Joint::updatelaMS;
      }
      else {
        if(momentDir.cols()>0) throwError("No moment law is given!");
        updatelaM_ = &Joint::updatelaM0;
      }

      if(momentDir.cols()==2) {
        if(fabs(momentDir(2,0))<=macheps and fabs(momentDir(2,1))<=macheps)
          iR = 2;
        else if(fabs(momentDir(1,0))<=macheps and fabs(momentDir(1,1))<=macheps)
          iR = 1;
        else if(fabs(momentDir(0,0))<=macheps and fabs(momentDir(0,1))<=macheps)
          iR = 0;
        else
          throwError("Generalized relative velocity of rotation can not be calculated from state for the defined moment direction. Turn on of integration generalized relative velocity of rotation.");
      }
      else if(momentDir.cols()==1) {
        msg(Warn) << "Evaluation of generalized relative velocity of rotation may be wrong for spatial rotation. In this case turn on integration of generalized relative velocity of rotation." << endl;
        if(fabs(momentDir(0,0))<=macheps and fabs(momentDir(2,0))<=macheps)
          iR = 2;
        else if(fabs(momentDir(1,0))<=macheps and fabs(momentDir(2,0))<=macheps)
          iR = 1;
        else if(fabs(momentDir(0,0))<=macheps and fabs(momentDir(1,0))<=macheps)
          iR = 0;
        else
          throwError("Generalized relative velocity of rotation can not be calculated from state for the defined moment direction. Turn on of integration generalized relative velocity of rotation.");
      }
      eR(iR) = 1;
    }
    else if (stage == unknownStage) {
      gdd.resize(gdSize);
      gdn.resize(gdSize);
      RF.resize(laSize);
      RM.resize(laSize);
    }
    FloatingFrameLink::init(stage, config);
    if(ffl) ffl->init(stage, config);
    if(fml) fml->init(stage, config);
    if(fifl) fifl->init(stage, config);
    if(fiml) fiml->init(stage, config);
  }

  bool Joint::isSetValued() const {
    return ((ffl and ffl->isSetValued()) or (fml and fml->isSetValued()));
  }

  bool Joint::isSingleValued() const {
    return ((ffl and not ffl->isSetValued()) or (fml and not fml->isSetValued()));
  }

  void Joint::solveImpactsFixpointSingle() {

    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    for (int i = 0; i < iFM.size(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      La(i) = fifl->project(La(i), gdn(i), gd(i), rFactor(i));
    }
    for (int i = iFM.size(); i < laSize; i++) {
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

    for (int i = 0; i < iFM.size(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      la(i) = ffl->project(la(i), gdd(i), rFactor(i));
    }
    for (int i = iFM.size(); i < laSize; i++) {
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

    for (int i = 0; i < iFM.size(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i] + 1; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      La(i) = fifl->solve(a[ia[laInd + i]], gdn(i), gd(i));
    }
    for (int i = iFM.size(); i < laSize; i++) {
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

    for (int i = 0; i < iFM.size(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i] + 1; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      la(i) = ffl->solve(a[ia[laInd + i]], gdd(i));
    }
    for (int i = iFM.size(); i < laSize; i++) {
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

    for (int i = 0; i < iFM.size(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      res(i) = La(i) - fifl->project(La(i), gdn(i), gd(i), rFactor(i));
    }
    for (int i = iFM.size(); i < laSize; i++) {
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

    for (int i = 0; i < iFM.size(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      res(i) = la(i) - ffl->project(la(i), gdd(i), rFactor(i));
    }
    for (int i = iFM.size(); i < laSize; i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      res(i) = la(i) - fml->project(la(i), gdd(i), rFactor(i));
    }
  }

  void Joint::jacobianConstraints() {

    const SqrMat G = ds->evalG();

    for (int i = 0; i < iFM.size(); i++) {
      RowVec jp1;
      jp1 &= ds->getJprox().row(laInd + i);
      RowVec e1(jp1.size());
      e1(laInd + i) = 1;
      Vec diff = ffl->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+i)
      for (int j = 0; j < G.size(); j++)
        jp1(j) -= diff(1) * G(laInd + i, j);
    }

    for (int i = iFM.size(); i < laSize; i++) {

      RowVec jp1;
      jp1 &= ds->getJprox().row(laInd + i);
      RowVec e1(jp1.size());
      e1(laInd + i) = 1;
      Vec diff = fml->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+i)
      for (int j = 0; j < G.size(); j++)
        jp1(j) -= diff(1) * G(laInd + i, j);
    }
  }

  void Joint::jacobianImpacts() {

    const SqrMat G = ds->evalG();

    for (int i = 0; i < iFM.size(); i++) {
      RowVec jp1;
      jp1 &= ds->getJprox().row(laInd + i);
      RowVec e1(jp1.size());
      e1(laInd + i) = 1;
      Vec diff = fifl->diff(La(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+i)
      for (int j = 0; j < G.size(); j++)
        jp1(j) -= diff(1) * G(laInd + i, j);
    }

    for (int i = iFM.size(); i < laSize; i++) {
      RowVec jp1;
      jp1 &= ds->getJprox().row(laInd + i);
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

    for (int i = 0; i < iFM.size(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * LaMBS(ja[j]);

      if (!fifl->isFulfilled(La(i), gdn(i), gd(i), LaTol, gdTol)) {
        ds->setTermination(false);
        return;
      }
    }
    for (int i = iFM.size(); i < laSize; i++) {
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

    for (int i = 0; i < iFM.size(); i++) {
      gdd(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdd(i) += a[j] * laMBS(ja[j]);

      if (!ffl->isFulfilled(la(i), gdd(i), laTol, gddTol)) {
        ds->setTermination(false);
        return;
      }
    }
    for (int i = iFM.size(); i < laSize; i++) {
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

  VecV Joint::evalGeneralizedRelativePositionOfRotation() {
    if(integrateGeneralizedRelativeVelocityOfRotation)
      return x;
    else
      return evalGlobalMomentDirection().T()*frame[0]->evalOrientation()*evalGlobalRelativeAngle();
  }

  Vec3 Joint::evalGlobalRelativeAngle() {
    WphiK0K1 = crossProduct(eR,evalGlobalRelativeOrientation().col(iR));
    WphiK0K1(iR) = -getGlobalRelativeOrientation()(fmod(iR+1,3),fmod(iR+2,3));
    return WphiK0K1;
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"integrateGeneralizedRelativeVelocityOfRotation");
    if(e) setIntegrateGeneralizedRelativeVelocityOfRotation(E(e)->getText<bool>());
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
