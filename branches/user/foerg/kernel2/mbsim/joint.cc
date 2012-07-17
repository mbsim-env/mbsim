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
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/objectfactory.h"
#include "openmbvcppinterface/arrow.h"
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Joint::Joint(const string &name) : LinkMechanics(name), ffl(0), fml(0), fifl(0), fiml(0), C("C") {
  }

  Joint::~Joint() {
    if(ffl) { delete ffl; ffl=0; }
    if(fml) { delete fml; fml=0; }
    if(fifl) { delete fifl; fifl=0; }
    if(fiml) { delete fiml; fiml=0; }
  }

  void Joint::connect(Frame* frame0, Frame* frame1) {
    LinkMechanics::connect(frame0);
    LinkMechanics::connect(frame1);
  }

  void Joint::updatewb(double t, int j) {
    Mat3V WJT = frame[0]->getOrientation()*JT;
    VecV sdT = WJT.T()*(WvP0P1);

    wb(0,Wf.cols()-1) += Wf.T()*(frame[1]->getGyroscopicAccelerationOfTranslation(j) - C.getGyroscopicAccelerationOfTranslation(j) - crossProduct(C.getAngularVelocity(),WvP0P1+WJT*sdT));
    wb(Wf.cols(),Wm.cols()+Wf.cols()-1) += Wm.T()*(frame[1]->getGyroscopicAccelerationOfRotation(j) - C.getGyroscopicAccelerationOfRotation(j) - crossProduct(C.getAngularVelocity(),WomP0P1));
  }

  void Joint::updateW(double t, int j) {
    fF[0].set(Index(0,2),Index(0,Wf.cols()-1), -Wf);
    fM[0].set(Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1), -Wm);
    fF[1] = -fF[0];
    fM[1] = -fM[0];

    W[j][0] += C.getJacobianOfTranslation(j).T()*fF[0] + C.getJacobianOfRotation(j).T()*fM[0];
    W[j][1] += frame[1]->getJacobianOfTranslation(j).T()*fF[1] + frame[1]->getJacobianOfRotation(j).T()*fM[1];
  }

  void Joint::updateh(double t, int j) {
    for(int i=0; i<forceDir.cols(); i++) 
      la(i) = (*ffl)(g(i), gd(i));

    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++)
      la(i) = (*fml)(g(i), gd(i));

    WF[1] = Wf*la(IT);
    WM[1] = Wm*la(IR);
    WF[0] = -WF[1];
    WM[0] = -WM[1];
    h[j][0] += C.getJacobianOfTranslation(j).T()*WF[0] + C.getJacobianOfRotation(j).T()*WM[0];
    h[j][1] += frame[1]->getJacobianOfTranslation(j).T()*WF[1] + frame[1]->getJacobianOfRotation(j).T()*WM[1];
  }

  void Joint::updateg(double t) {
    Wf = frame[0]->getOrientation()*forceDir;
    Wm = frame[0]->getOrientation()*momentDir;

    WrP0P1 = frame[1]->getPosition()-frame[0]->getPosition();
    C.setOrientation(frame[0]->getOrientation());
    C.setPosition(frame[0]->getPosition() + WrP0P1);

    g(IT) = Wf.T()*WrP0P1;
    g(IR) = x;
  }

  void Joint::updategd(double t) {
    C.setAngularVelocity(frame[0]->getAngularVelocity());
    C.setVelocity(frame[0]->getVelocity() + crossProduct(frame[0]->getAngularVelocity(),WrP0P1));

    WvP0P1 = frame[1]->getVelocity()-C.getVelocity();
    WomP0P1 = frame[1]->getAngularVelocity()-C.getAngularVelocity();

    gd(IT) = Wf.T()*WvP0P1;
    gd(IR) = Wm.T()*WomP0P1;
  }

  void Joint::updateJacobians(double t, int j) {
    Mat33 tWrP0P1 = tilde(WrP0P1);

    C.setJacobianOfTranslation(frame[0]->getJacobianOfTranslation(j) - tWrP0P1*frame[0]->getJacobianOfRotation(j),j);
    C.setJacobianOfRotation(frame[0]->getJacobianOfRotation(j),j);
    C.setGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation(j) - tWrP0P1*frame[0]->getGyroscopicAccelerationOfRotation(j) + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrP0P1)),j);
    C.setGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation(j),j);
  }

  void Joint::updatexd(double t) {
    xd = gd(IR);
  }

  void Joint::updatedx(double t, double dt) {
    xd = gd(IR)*dt;
  }

  void Joint::calcxSize() {
    LinkMechanics::calcxSize();
    xSize = momentDir.cols();
  }

  void Joint::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      LinkMechanics::init(stage);
    }
    else if(stage==resize) {
      LinkMechanics::init(stage);

      g.resize(forceDir.cols()+momentDir.cols());
      gd.resize(forceDir.cols()+momentDir.cols());
      la.resize(forceDir.cols()+momentDir.cols());
      gdd.resize(gdSize);
      gdn.resize(gdSize);
    }
    else if(stage==unknownStage) {
      LinkMechanics::init(stage);

      IT = Index(0,forceDir.cols()-1);
      IR = Index(forceDir.cols(),forceDir.cols()+momentDir.cols()-1);
      if(forceDir.cols()) 
        Wf = forceDir;
      else {
      }
      if(momentDir.cols())
        Wm = momentDir;
      else {
      }

      C.getJacobianOfTranslation(0).resize(frame[0]->getJacobianOfTranslation(0).cols());
      C.getJacobianOfRotation(0).resize(frame[0]->getJacobianOfRotation(0).cols());
      C.getJacobianOfTranslation(1).resize(frame[0]->getJacobianOfTranslation(1).cols());
      C.getJacobianOfRotation(1).resize(frame[0]->getJacobianOfRotation(1).cols());

      JT.resize(3-forceDir.cols());
      if(forceDir.cols() == 2)
        JT.set(0, crossProduct(forceDir.col(0),forceDir.col(1)));
      else if(forceDir.cols() == 3);
      else if(forceDir.cols() == 0) JT = SqrMat(3,EYE);
      else { // define a coordinate system in the plane perpendicular to the force direction
        JT.set(0, computeTangential(forceDir.col(0)));
        JT.set(1, crossProduct(forceDir.col(0),JT.col(0)));
      }
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
#endif
        if(getPlotFeature(generalizedLinkForce)==enabled) {
          for(int j=0; j<la.size(); ++j)
            plotColumns.push_back("la("+numtostr(j)+")");
        }
        if(getPlotFeature(linkKinematics)==enabled) {
          for(int j=0; j<g.size(); ++j)
            plotColumns.push_back("g("+numtostr(j)+")");
          for(int j=0; j<gd.size(); ++j)
            plotColumns.push_back("gd("+numtostr(j)+")");
        }
        LinkMechanics::init(stage);
      }
    }
    else
      LinkMechanics::init(stage);
  }

  void Joint::calclaSize(int j) {
    LinkMechanics::calclaSize(j);
    laSize = forceDir.cols()+momentDir.cols();
  }

  void Joint::calcgSize(int j) {
    LinkMechanics::calcgSize(j);
    gSize = forceDir.cols()+momentDir.cols();
  }

  void Joint::calcgdSize(int j) {
    LinkMechanics::calcgdSize(j);
    gdSize = forceDir.cols()+momentDir.cols();
  }

  void Joint::calcrFactorSize(int j) {
    LinkMechanics::calcrFactorSize(j);
    rFactorSize = isSetValued() ? forceDir.cols()+momentDir.cols() : 0;
  }

  bool Joint::isSetValued() const {
    bool flag = false;
    if(ffl) 
      flag |= ffl->isSetValued();
    if(fml) 
      flag |= fml->isSetValued();

    return flag;
  }

  void Joint::solveImpactsFixpointSingle(double dt) {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fifl->project(la(i), gdn(i), gd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fiml->project(la(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void Joint::solveConstraintsFixpointSingle() {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = ffl->project(la(i), gdd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = fml->project(la(i), gdd(i), rFactor(i));
    }
  }

  void Joint::solveImpactsGaussSeidel(double dt) {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laInd+i);
      for(int j=ia[laInd+i]+1; j<ia[laInd+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fifl->solve(a[ia[laInd+i]], gdn(i), gd(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd+i);
      for(int j=ia[laInd+i]+1; j<ia[laInd+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fiml->solve(a[ia[laInd+i]], gdn(i), gd(i));
    }
  }

  void Joint::solveConstraintsGaussSeidel() {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laInd+i);
      for(int j=ia[laInd+i]+1; j<ia[laInd+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = ffl->solve(a[ia[laInd+i]], gdd(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(laInd+i);
      for(int j=ia[laInd+i]+1; j<ia[laInd+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = fml->solve(a[ia[laInd+i]], gdd(i));
    }
  }

  void Joint::solveImpactsRootFinding(double dt) {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - fifl->project(la(i), gdn(i), gd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - fiml->project(la(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void Joint::solveConstraintsRootFinding() {

    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - ffl->project(la(i), gdd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - fml->project(la(i), gdd(i), rFactor(i));
    }
  }

  void Joint::jacobianConstraints() {

    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->getG();

    for(int i=0; i<forceDir.cols(); i++) {
      RowVec jp1=Jprox.row(laInd+i);
      RowVec e1(jp1.size());
      e1(laInd+i) = 1;
      Vec diff = ffl->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laInd+i)
      for(int j=0; j<G.size(); j++) 
        jp1(j) -= diff(1)*G(laInd+i,j);
    }

    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {

      RowVec jp1=Jprox.row(laInd+i);
      RowVec e1(jp1.size());
      e1(laInd+i) = 1;
      Vec diff = fml->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laInd+i)
      for(int j=0; j<G.size(); j++) 
        jp1(j) -= diff(1)*G(laInd+i,j);
    }
  }

  void Joint::jacobianImpacts() {

    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->getG();

    for(int i=0; i<forceDir.cols(); i++) {
      RowVec jp1=Jprox.row(laInd+i);
      RowVec e1(jp1.size());
      e1(laInd+i) = 1;
      Vec diff = fifl->diff(la(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laInd+i)
      for(int j=0; j<G.size(); j++) 
        jp1(j) -= diff(1)*G(laInd+i,j);
    }

    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      RowVec jp1=Jprox.row(laInd+i);
      RowVec e1(jp1.size());
      e1(laInd+i) = 1;
      Vec diff = fiml->diff(la(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laInd+i)
      for(int j=0; j<G.size(); j++) 
        jp1(j) -= diff(1)*G(laInd+i,j);
    }
  }

  void Joint::updaterFactors() {
    if(isActive()) {

      const double *a = ds->getGs()();
      const int *ia = ds->getGs().Ip();

      for(int i=0; i<rFactorSize; i++) {
        double sum = 0;
        for(int j=ia[laInd+i]+1; j<ia[laInd+i+1]; j++)
          sum += fabs(a[j]);
        double ai = a[ia[laInd+i]];
        if(ai > sum) {
          rFactorUnsure(i) = 0;
          rFactor(i) = 1.0/ai;
        } else {
          rFactorUnsure(i) = 1;
          rFactor(i) = 1.0/ai;
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

    for(int i=0; i < forceDir.cols(); i++) {
      gdn(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      if(!fifl->isFulfilled(la(i),gdn(i),gd(i),LaTol,gdTol)) {
        ds->setTermination(false);
        return;
      }
    }
    for(int i=forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      if(!fiml->isFulfilled(la(i),gdn(i),gd(i),LaTol,gdTol)) {
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

    for(int i=0; i < forceDir.cols(); i++) {
      gdd(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      if(!ffl->isFulfilled(la(i),gdd(i),laTol,gddTol)) {
        ds->setTermination(false);
        return;
      }
    }
    for(int i=forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(laInd+i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      if(!fml->isFulfilled(la(i),gdd(i),laTol,gddTol)) {
        ds->setTermination(false);
        return;
      }
    }
  }

  void Joint::setForceDirection(const Mat3V &fd) {

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.set(i, forceDir.col(i)/nrm2(fd.col(i)));
  }

  void Joint::setMomentDirection(const Mat3V &md) {

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.set(i, momentDir.col(i)/nrm2(md.col(i)));
  }

  void Joint::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      // WF and WM are needed by OpenMBV plotting in LinkMechanics::plot(...)
      if(isSetValued()) {
        WF[0]=fF[0]*la;
        WF[1]=-WF[0];
        WM[0]=fM[0]*la;
        WM[1]=-WM[0];
      }
#endif
      if(getPlotFeature(generalizedLinkForce)==enabled) {
        for(int j=0; j<la.size(); j++)
          plotVector.push_back(la(j)/(isSetValued()?dt:1.));
      }
      if(getPlotFeature(linkKinematics)==enabled) {
        for(int j=0; j<g.size(); j++)
          plotVector.push_back(g(j));
        for(int j=0; j<gd.size(); j++)
          plotVector.push_back(gd(j));
      }
      LinkMechanics::plot(t,dt);
    }
  }

  void Joint::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e, *ee;
    LinkMechanics::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMNS"force");
    if(e) {
      ee=e->FirstChildElement(MBSIMNS"direction");
      setForceDirection(getMat3V(ee,0));
      ee=ee->NextSiblingElement();
      GeneralizedForceLaw *gfl=ObjectFactory::getInstance()->createGeneralizedForceLaw(ee->FirstChildElement());
      setForceLaw(gfl);
      gfl->initializeUsingXML(ee->FirstChildElement());
      ee=ee->NextSiblingElement();
      GeneralizedImpactLaw *gifl=ObjectFactory::getInstance()->createGeneralizedImpactLaw(ee->FirstChildElement());
      if(gifl) {
        setImpactForceLaw(gifl);
        gifl->initializeUsingXML(ee->FirstChildElement());
      }
      ee=ee->NextSiblingElement();
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *arrow=dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(ee));
      if(arrow) {
        arrow->initializeUsingXML(ee); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
        setOpenMBVForceArrow(arrow);
        ee=ee->NextSiblingElement();
      }
#endif
    }
    e=element->FirstChildElement(MBSIMNS"moment");
    if(e) {
      ee=e->FirstChildElement(MBSIMNS"direction");
      setMomentDirection(getMat3V(ee,0));
      ee=ee->NextSiblingElement();
      GeneralizedForceLaw *gfl=ObjectFactory::getInstance()->createGeneralizedForceLaw(ee->FirstChildElement());
      setMomentLaw(gfl);
      gfl->initializeUsingXML(ee->FirstChildElement());
      ee=ee->NextSiblingElement();
      GeneralizedImpactLaw *gifl=ObjectFactory::getInstance()->createGeneralizedImpactLaw(ee->FirstChildElement());
      if(gifl) {
        setImpactMomentLaw(gifl);
        gifl->initializeUsingXML(ee->FirstChildElement());
      }
      ee=ee->NextSiblingElement();
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *arrow=dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(ee));
      if(arrow) {
        arrow->initializeUsingXML(ee); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
        setOpenMBVMomentArrow(arrow);
        ee=ee->NextSiblingElement();
      }
#endif
    }
    e=element->FirstChildElement(MBSIMNS"connect");
    saved_ref1=e->Attribute("ref1");
    saved_ref2=e->Attribute("ref2");
  }

  InverseKineticsJoint::InverseKineticsJoint(const string &name) : Joint(name), body(0) {
  }

  void InverseKineticsJoint::calcbSize() {
    bSize = body?body->getuRel().size():0;
  }

  void InverseKineticsJoint::updateb(double t) {
    if(bSize) {
      b(Index(0,bSize-1),Index(0,2)) = body->getPJT().T();
      b(Index(0,bSize-1),Index(3,5)) = body->getPJR().T();
    }
  }

  void InverseKineticsJoint::init(InitStage stage) {
    if(stage==resize) {
      Joint::init(stage);
      x.resize(3);
    }
    else
      Joint::init(stage);
  }

}

