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
#include "mbsim/joint.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/objectfactory.h"
#include "mbsim/contact_kinematics/contact_kinematics.h"
#include <mbsim/utils/utils.h>
#include <mbsim/rigid_body.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Joint, MBSIM%"Joint")

  Joint::Joint(const string &name) : MechanicalLink(name), refFrame(NULL), refFrameID(0), ffl(0), fml(0), fifl(0), fiml(0), C("F") {
    C.setParent(this);
    C.setUpdateGlobalRelativePositionByParent();
  }

  Joint::~Joint() {
    if (ffl) {
      delete ffl;
      ffl = 0;
    }
    if (fml) {
      delete fml;
      fml = 0;
    }
    if (fifl) {
      delete fifl;
      fifl = 0;
    }
    if (fiml) {
      delete fiml;
      fiml = 0;
    }
  }

  void Joint::connect(Frame* frame0, Frame* frame1) {
    MechanicalLink::connect(frame0);
    MechanicalLink::connect(frame1);
  }

  void Joint::updatewb(double t, int j) {
    Mat3xV WJT = refFrame->getOrientation(t) * JT;
    VecV sdT = WJT.T() * (getGlobalRelativeVelocity(t));

    wb(0, Wf.cols() - 1) += getGlobalForceDirection(t).T() * (frame[1]->getGyroscopicAccelerationOfTranslation(t,j) - C.getGyroscopicAccelerationOfTranslation(t,j) - crossProduct(C.getAngularVelocity(t), getGlobalRelativeVelocity(t) + WJT * sdT));
    wb(Wf.cols(), Wm.cols() + Wf.cols() - 1) += getGlobalMomentDirection(t).T() * (frame[1]->getGyroscopicAccelerationOfRotation(t,j) - C.getGyroscopicAccelerationOfRotation(t,j) - crossProduct(C.getAngularVelocity(t), getGlobalRelativeAngularVelocity(t)));
  }

  void Joint::updateW(double t, int j) {
    W[j][0] -= C.getJacobianOfTranslation(t,j).T() * getSetValuedForceDirection(t) + C.getJacobianOfRotation(t,j).T() * getSetValuedMomentDirection(t);
    W[j][1] += frame[1]->getJacobianOfTranslation(t,j).T() * getSetValuedForceDirection(t) + frame[1]->getJacobianOfRotation(t,j).T() * getSetValuedMomentDirection(t);
  }

  void Joint::updateh(double t, int j) {
    h[j][0] -= C.getJacobianOfTranslation(t,j).T() * getSingleValuedForce(t) + C.getJacobianOfRotation(t,j).T() * getSingleValuedMoment(t);
    h[j][1] += frame[1]->getJacobianOfTranslation(t,j).T() * getSingleValuedForce(t) + frame[1]->getJacobianOfRotation(t,j).T() * getSingleValuedMoment(t);
  }

  void Joint::updatePositions(double t) {
    WrP0P1 = frame[1]->getPosition(t) - frame[0]->getPosition(t);
    C.setGlobalRelativePosition(WrP0P1);
    rrel.set(iF, getGlobalForceDirection(t).T() * WrP0P1);
    rrel.set(iM, x);
    updPos = false;
  }

  void Joint::updateVelocities(double t) {
    WvP0P1 = frame[1]->getVelocity(t) - C.getVelocity(t);
    WomP0P1 = frame[1]->getAngularVelocity(t) - C.getAngularVelocity(t);
    vrel.set(iF, getGlobalForceDirection(t).T() * WvP0P1);
    vrel.set(iM, getGlobalMomentDirection(t).T() * WomP0P1);
    updVel = false;
  }

  void Joint::updateForceDirections(double t) {
    Wf = refFrame->getOrientation(t) * forceDir;
    Wm = refFrame->getOrientation(t) * momentDir;
    updFD = false;
  }

  void Joint::updateGeneralizedSetValuedForces(double t) {
    updlaMV = false;
  }

  void Joint::updateGeneralizedSingleValuedForces(double t) {
    if (ffl) {
      for (int i = 0; i < forceDir.cols(); i++)
        laSV(i) = (*ffl)(getRelativePosition(t)(i), getRelativeVelocity(t)(i));
    }
    if (fml) {
      for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++)
        laSV(i) = (*fml)(getRelativePosition(t)(i), getRelativeVelocity(t)(i));
    }
    updlaSV = false;
  }

  void Joint::updateg(double t) {
    g(iF) = getRelativePosition(t)(iF);
    g(iM) = rrel(iM);;
  }

  void Joint::updategd(double t) {
    gd(iF) = getRelativeVelocity(t)(iF);
    gd(iM) = vrel(iM);
  }

  void Joint::updatexd(double t) {
    xd = getRelativeVelocity(t)(iM);
  }

  void Joint::updatedx(double t, double dt) {
    xd = getRelativeVelocity(t)(iM) * dt;
  }

  void Joint::calcxSize() {
    MechanicalLink::calcxSize();
    xSize = momentDir.cols();
  }

  void Joint::init(InitStage stage) {
    if (stage == resolveXMLPath) {
      if (saved_ref1 != "" && saved_ref2 != "")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      if(not(frame.size()))
        THROW_MBSIMERROR("no connection given!");
      MechanicalLink::init(stage);
    }
    else if (stage == resize) {
      MechanicalLink::init(stage);

      iF = Index(0, forceDir.cols() - 1);
      iM = Index(forceDir.cols(), forceDir.cols() + momentDir.cols() - 1);
      rrel.resize(forceDir.cols() + momentDir.cols());
      vrel.resize(forceDir.cols() + momentDir.cols());
      g.resize(forceDir.cols() + momentDir.cols());
      gd.resize(forceDir.cols() + momentDir.cols());
      la.resize(forceDir.cols() + momentDir.cols());
      laSV.resize(forceDir.cols() + momentDir.cols());
      gdd.resize(gdSize);
      gdn.resize(gdSize);
    }
    else if (stage == unknownStage) {
      MechanicalLink::init(stage);

      if (ffl) {
        fifl = new BilateralImpact;
        fifl->setParent(this);
      }
      if (fml) {
        fiml = new BilateralImpact;
        fiml->setParent(this);
      }

      if (forceDir.cols())
        Wf = forceDir;
      else {
      }
      if (momentDir.cols())
        Wm = momentDir;
      else {
      }

      C.setFrameOfReference(frame[0]);
      C.getJacobianOfTranslation(0).resize(frame[0]->getJacobianOfTranslation(0).cols());
      C.getJacobianOfRotation(0).resize(frame[0]->getJacobianOfRotation(0).cols());
      C.getJacobianOfTranslation(1).resize(frame[0]->getJacobianOfTranslation(1).cols());
      C.getJacobianOfRotation(1).resize(frame[0]->getJacobianOfRotation(1).cols());

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
      refFrame = refFrameID ? frame[1] : frame[0];
    }
    else if (stage == plotting) {
      updatePlotFeatures();
      if (getPlotFeature(plotRecursive) == enabled) {
        if (getPlotFeature(generalizedLinkForce) == enabled) {
          for (int j = 0; j < la.size(); ++j)
            plotColumns.push_back("la(" + numtostr(j) + ")");
        }
        if (getPlotFeature(linkKinematics) == enabled) {
          for (int j = 0; j < g.size(); ++j)
            plotColumns.push_back("g(" + numtostr(j) + ")");
          for (int j = 0; j < gd.size(); ++j)
            plotColumns.push_back("gd(" + numtostr(j) + ")");
        }
        MechanicalLink::init(stage);
      }
    }
    else
      MechanicalLink::init(stage);
    if(ffl) ffl->init(stage);
    if(fml) fml->init(stage);
    if(fifl) fifl->init(stage);
    if(fiml) fiml->init(stage);
  }

  void Joint::calclaSize(int j) {
    MechanicalLink::calclaSize(j);
    laSize = forceDir.cols() + momentDir.cols();
  }

  void Joint::calcgSize(int j) {
    MechanicalLink::calcgSize(j);
    gSize = forceDir.cols() + momentDir.cols();
  }

  void Joint::calcgdSize(int j) {
    MechanicalLink::calcgdSize(j);
    gdSize = forceDir.cols() + momentDir.cols();
  }

  void Joint::calcrFactorSize(int j) {
    MechanicalLink::calcrFactorSize(j);
    rFactorSize = isSetValued() ? forceDir.cols() + momentDir.cols() : 0;
  }

  void Joint::calccorrSize(int j) {
    MechanicalLink::calccorrSize(j);
    corrSize = forceDir.cols() + momentDir.cols();
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

  void Joint::solveImpactsFixpointSingle(double dt) {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * laMBS(ja[j]);

      la(i) = fifl->project(la(i), gdn(i), gd(i), rFactor(i));
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * laMBS(ja[j]);

      la(i) = fiml->project(la(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void Joint::solveConstraintsFixpointSingle() {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

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

  void Joint::solveImpactsGaussSeidel(double dt) {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i] + 1; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * laMBS(ja[j]);

      la(i) = fifl->solve(a[ia[laInd + i]], gdn(i), gd(i));
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i] + 1; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * laMBS(ja[j]);

      la(i) = fiml->solve(a[ia[laInd + i]], gdn(i), gd(i));
    }
  }

  void Joint::solveConstraintsGaussSeidel() {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

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

  void Joint::solveImpactsRootFinding(double dt) {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * laMBS(ja[j]);

      res(i) = la(i) - fifl->project(la(i), gdn(i), gd(i), rFactor(i));
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * laMBS(ja[j]);

      res(i) = la(i) - fiml->project(la(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void Joint::solveConstraintsRootFinding() {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

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
    const SqrMat G = ds->getG();

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
    const SqrMat G = ds->getG();

    for (int i = 0; i < forceDir.cols(); i++) {
      RowVec jp1 = Jprox.row(laInd + i);
      RowVec e1(jp1.size());
      e1(laInd + i) = 1;
      Vec diff = fifl->diff(la(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+i)
      for (int j = 0; j < G.size(); j++)
        jp1(j) -= diff(1) * G(laInd + i, j);
    }

    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      RowVec jp1 = Jprox.row(laInd + i);
      RowVec e1(jp1.size());
      e1(laInd + i) = 1;
      Vec diff = fiml->diff(la(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+i)
      for (int j = 0; j < G.size(); j++)
        jp1(j) -= diff(1) * G(laInd + i, j);
    }
  }

  void Joint::updaterFactors() {
    if (isActive()) {

      const double *a = ds->getGs()();
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

  void Joint::checkImpactsForTermination(double dt) {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    for (int i = 0; i < forceDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * laMBS(ja[j]);

      if (!fifl->isFulfilled(la(i), gdn(i), gd(i), LaTol, gdTol)) {
        ds->setTermination(false);
        return;
      }
    }
    for (int i = forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd + i);
      for (int j = ia[laInd + i]; j < ia[laInd + 1 + i]; j++)
        gdn(i) += a[j] * laMBS(ja[j]);

      if (!fiml->isFulfilled(la(i), gdn(i), gd(i), LaTol, gdTol)) {
        ds->setTermination(false);
        return;
      }
    }
  }

  void Joint::checkConstraintsForTermination() {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

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

  void Joint::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      // WF and WM are needed by OpenMBV plotting in MechanicalLink::plot(...)
      if (isSetValued()) {
        WF[0] = fF[0] * la;
        WF[1] = -WF[0];
        WM[0] = fM[0] * la;
        WM[1] = -WM[0];
      }
#endif
      if (getPlotFeature(generalizedLinkForce) == enabled) {
        for (int j = 0; j < la.size(); j++)
          plotVector.push_back(la(j) / (isSetValued() ? dt : 1.)); // TODO (TS, 2014-09-17) why is there dt in the denominator? does this contradict the link implementation?
      }
      if (getPlotFeature(linkKinematics) == enabled) {
        updateg(t);
        updategd(t);
        for (int j = 0; j < g.size(); j++)
          plotVector.push_back(g(j));
        for (int j = 0; j < gd.size(); j++)
          plotVector.push_back(gd(j));
      }
      MechanicalLink::plot(t, dt);
    }
  }

  void Joint::initializeUsingXML(DOMElement *element) {
    MechanicalLink::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"frameOfReferenceID");
    if (e)
      refFrameID = getInt(e);
    e = E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    if (e)
      setForceDirection(getMat(e, 3, 0));
    e = E(element)->getFirstElementChildNamed(MBSIM%"forceLaw");
    if (e)
      setForceLaw(ObjectFactory::createAndInit<GeneralizedForceLaw>(e->getFirstElementChild()));
    e = E(element)->getFirstElementChildNamed(MBSIM%"momentDirection");
    if (e)
      setMomentDirection(getMat(e, 3, 0));
    e = E(element)->getFirstElementChildNamed(MBSIM%"momentLaw");
    if (e)
      setMomentLaw(ObjectFactory::createAndInit<GeneralizedForceLaw>(e->getFirstElementChild()));
    e = E(element)->getFirstElementChildNamed(MBSIM%"connect");
    saved_ref1 = E(e)->getAttribute("ref1");
    saved_ref2 = E(e)->getAttribute("ref2");
#ifdef HAVE_OPENMBVCPPINTERFACE
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toHead, OpenMBV::Arrow::toPoint, 1, 1);
      setOpenMBVForce(ombv.createOpenMBV(e));
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toDoubleHead, OpenMBV::Arrow::toPoint, 1, 1);
      setOpenMBVMoment(ombv.createOpenMBV(e));
    }
#endif
  }

  DOMElement* Joint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = MechanicalLink::writeXMLFile(parent);
//    if (forceDir.cols()) {
//      addElementText(ele0, MBSIM%"forceDirection", forceDir);
//      DOMElement *ele1 = new DOMElement(MBSIM%"forceLaw");
//      if (ffl)
//        ffl->writeXMLFile(ele1);
//      ele0->LinkEndChild(ele1);
//    }
//    if (momentDir.cols()) {
//      addElementText(ele0, MBSIM%"momentDirection", momentDir);
//      DOMElement *ele1 = new DOMElement(MBSIM%"momentLaw");
//      if (fml)
//        fml->writeXMLFile(ele1);
//      ele0->LinkEndChild(ele1);
//    }
//    DOMElement *ele1 = new DOMElement(MBSIM%"connect");
//    //ele1->SetAttribute("ref1", frame[0]->getXMLPath(frame[0])); // absolute path
//    //ele1->SetAttribute("ref2", frame[1]->getXMLPath(frame[1])); // absolute path
//    ele1->SetAttribute("ref1", frame[0]->getXMLPath(this, true)); // relative path
//    ele1->SetAttribute("ref2", frame[1]->getXMLPath(this, true)); // relative path
//    ele0->LinkEndChild(ele1);
    return ele0;
  }

  InverseKineticsJoint::InverseKineticsJoint(const string &name) :
      Joint(name), body(0) {
  }

  void InverseKineticsJoint::calcbSize() {
    bSize = body ? body->getuRelSize() : 0;
  }

  void InverseKineticsJoint::updateb(double t) {
    if (bSize) {
      b(Index(0, bSize - 1), Index(0, 2)) = body->getPJT(t).T();
      b(Index(0, bSize - 1), Index(3, 5)) = body->getPJR(t).T();
    }
  }

  void InverseKineticsJoint::init(InitStage stage) {
    if (stage == resize) {
      Joint::init(stage);
      x.resize(momentDir.cols());
    }
    else
      Joint::init(stage);
  }

}

