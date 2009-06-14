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
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include "mbsim/joint.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/objectfactory.h"

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

  void Joint::connect(FrameInterface *frame0, FrameInterface* frame1) {
    LinkMechanics::connect(frame0);
    LinkMechanics::connect(frame1);
  }

  void Joint::updatewb(double t) {
    Mat WJT = frame[0]->getOrientation()*JT;
    Vec sdT = trans(WJT)*(WvP0P1);

    wb(0,Wf.cols()-1) += trans(Wf)*(frame[1]->getGyroscopicAccelerationOfTranslation() - C.getGyroscopicAccelerationOfTranslation() - crossProduct(C.getAngularVelocity(),WvP0P1+WJT*sdT));
    wb(Wf.cols(),Wm.cols()+Wf.cols()-1) += trans(Wm)*(frame[1]->getGyroscopicAccelerationOfRotation() - C.getGyroscopicAccelerationOfRotation() - crossProduct(C.getAngularVelocity(),WomP0P1));
  }

  void Joint::updateW(double t) {
    fF[0](Index(0,2),Index(0,Wf.cols()-1)) = -Wf;
    fM[0](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = -Wm;
    fF[1] = -fF[0];
    fM[1] = -fM[0];

    W[0] += trans(C.getJacobianOfTranslation())*fF[0] + trans(C.getJacobianOfRotation())*fM[0];
    W[1] += trans(frame[1]->getJacobianOfTranslation())*fF[1] + trans(frame[1]->getJacobianOfRotation())*fM[1];
  }

  void Joint::updateh(double t) {
    for(int i=0; i<forceDir.cols(); i++) 
      la(i) = (*ffl)(g(i), gd(i));

    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++)
      la(i) = (*fml)(g(i), gd(i));

    WF[1] = Wf*la(IT);
    WM[1] = Wm*la(IR);
    WF[0] = -WF[1];
    WM[0] = -WM[1];
    h[0] += trans(C.getJacobianOfTranslation())*WF[0] + trans(C.getJacobianOfRotation())*WM[0];
    h[1] += trans(frame[1]->getJacobianOfTranslation())*WF[1] + trans(frame[1]->getJacobianOfRotation())*WM[1];
  }

  void Joint::updateg(double t) {
    Wf = frame[0]->getOrientation()*forceDir;
    Wm = frame[0]->getOrientation()*momentDir;

    WrP0P1 = frame[1]->getPosition()-frame[0]->getPosition();

    g(IT) = trans(Wf)*WrP0P1;
    g(IR) = x;
  }

  void Joint::updategd(double t) {
    C.setAngularVelocity(frame[0]->getAngularVelocity());
    C.setVelocity(frame[0]->getVelocity() + crossProduct(frame[0]->getAngularVelocity(),WrP0P1));

    WvP0P1 = frame[1]->getVelocity()-C.getVelocity();
    WomP0P1 = frame[1]->getAngularVelocity()-C.getAngularVelocity();

    gd(IT) = trans(Wf)*WvP0P1;
    gd(IR) = trans(Wm)*WomP0P1;
  }

  void Joint::updateJacobians(double t) {
    Mat tWrP0P1 = tilde(WrP0P1);
    C.setJacobianOfTranslation(frame[0]->getJacobianOfTranslation() - tWrP0P1*frame[0]->getJacobianOfRotation());
    C.setJacobianOfRotation(frame[0]->getJacobianOfRotation());
    C.setGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation() - tWrP0P1*frame[0]->getGyroscopicAccelerationOfRotation() + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrP0P1)));
    C.setGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation());
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

  void Joint::init() {
    LinkMechanics::init();

    g.resize(forceDir.cols()+momentDir.cols());
    gd.resize(forceDir.cols()+momentDir.cols());
    la.resize(forceDir.cols()+momentDir.cols());
    gdd.resize(gdSize);
    gdn.resize(gdSize);

    IT = Index(0,forceDir.cols()-1);
    IR = Index(forceDir.cols(),forceDir.cols()+momentDir.cols()-1);
    if(forceDir.cols()) 
      Wf = forceDir;
    else {
      forceDir.resize(3,0);
      Wf.resize(3,0);
    }
    if(momentDir.cols())
      Wm = momentDir;
    else {
      momentDir.resize(3,0);
      Wm.resize(3,0);
    }
    C.getJacobianOfTranslation().resize(3,frame[0]->getJacobianOfTranslation().cols());
    C.getJacobianOfRotation().resize(3,frame[0]->getJacobianOfRotation().cols());
    JT.resize(3,3-forceDir.cols());
    if(forceDir.cols() == 2)
      JT.col(0) = crossProduct(forceDir.col(0),forceDir.col(1));
    else if(forceDir.cols() == 3);
    else if(forceDir.cols() == 0);
    else {
      cout << "TODO: 1 Force Direction not yet implemented" << endl;
      throw 5;
    }
  }

  void Joint::calclaSize() {
    LinkMechanics::calclaSize();
    laSize = forceDir.cols()+momentDir.cols();
  }

  void Joint::calclaSizeForActiveg() {
    calclaSize();
  }

  void Joint::calcgSize() {
    LinkMechanics::calcgSize();
    gSize = forceDir.cols()+momentDir.cols();
  }

  void Joint::calcgSizeActive() {
    calcgSize();
  }

  void Joint::calcgdSize() {
    LinkMechanics::calcgdSize();
    gdSize = forceDir.cols()+momentDir.cols();
  }

  void Joint::calcgdSizeActive() {
    calcgdSize();
  }

  void Joint::calcrFactorSize() {
    LinkMechanics::calcrFactorSize();
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

  void Joint::solveImpactsFixpointSingle() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laIndDS+i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fifl->project(la(i), gdn(i), gd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fiml->project(la(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void Joint::solveConstraintsFixpointSingle() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laIndDS+i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = ffl->project(la(i), gdd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = fml->project(la(i), gdd(i), rFactor(i));
    }
  }

  void Joint::solveImpactsGaussSeidel() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laIndDS+i);
      for(int j=ia[laIndDS+i]+1; j<ia[laIndDS+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fifl->solve(a[ia[laIndDS+i]], gdn(i), gd(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndDS+i]+1; j<ia[laIndDS+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fiml->solve(a[ia[laIndDS+i]], gdn(i), gd(i));
    }
  }

  void Joint::solveConstraintsGaussSeidel() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laIndDS+i);
      for(int j=ia[laIndDS+i]+1; j<ia[laIndDS+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = ffl->solve(a[ia[laIndDS+i]], gdd(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndDS+i]+1; j<ia[laIndDS+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = fml->solve(a[ia[laIndDS+i]], gdd(i));
    }
  }

  void Joint::solveImpactsRootFinding() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laIndDS+i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - fifl->project(la(i), gdn(i), gd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - fiml->project(la(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void Joint::solveConstraintsRootFinding() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laIndDS+i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - ffl->project(la(i), gdd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - fml->project(la(i), gdd(i), rFactor(i));
    }
  }

  void Joint::jacobianConstraints() {
    SqrMat Jprox = ds->getJprox();
    SqrMat G = ds->getG();

    for(int i=0; i<forceDir.cols(); i++) {
      RowVec jp1=Jprox.row(laIndDS+i);
      RowVec e1(jp1.size());
      e1(laIndDS+i) = 1;
      Vec diff = ffl->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndDS+i)
      for(int j=0; j<G.size(); j++) 
        jp1(j) -= diff(1)*G(laIndDS+i,j);
    }

    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {

      RowVec jp1=Jprox.row(laIndDS+i);
      RowVec e1(jp1.size());
      e1(laIndDS+i) = 1;
      Vec diff = fml->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndDS+i)
      for(int j=0; j<G.size(); j++) 
        jp1(j) -= diff(1)*G(laIndDS+i,j);
    }
  }

  void Joint::jacobianImpacts() {
    SqrMat Jprox = ds->getJprox();
    SqrMat G = ds->getG();

    for(int i=0; i<forceDir.cols(); i++) {
      RowVec jp1=Jprox.row(laIndDS+i);
      RowVec e1(jp1.size());
      e1(laIndDS+i) = 1;
      Vec diff = fifl->diff(la(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndDS+i)
      for(int j=0; j<G.size(); j++) 
        jp1(j) -= diff(1)*G(laIndDS+i,j);
    }

    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {

      RowVec jp1=Jprox.row(laIndDS+i);
      RowVec e1(jp1.size());
      e1(laIndDS+i) = 1;
      Vec diff = fiml->diff(la(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndDS+i)
      for(int j=0; j<G.size(); j++) 
        jp1(j) -= diff(1)*G(laIndDS+i,j);
    }
  }

  void Joint::updaterFactors() {
    if(isActive()) {
      double *a = ds->getGs()();
      int *ia = ds->getGs().Ip();

      for(int i=0; i<rFactorSize; i++) {
        double sum = 0;
        for(int j=ia[laIndDS+i]+1; j<ia[laIndDS+i+1]; j++)
          sum += fabs(a[j]);
        double ai = a[ia[laIndDS+i]];
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

  void Joint::checkImpactsForTermination() {

    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    for(int i=0; i < forceDir.cols(); i++) {
      gdn(i) = b(laIndDS+i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      if(!fifl->isFullfield(la(i),gdn(i),gd(i),LaTol,gdTol)) {
        ds->setTermination(false);
        return;
      }
    }
    for(int i=forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdn(i) += a[j]*laMBS(ja[j]);

      if(!fiml->isFullfield(la(i),gdn(i),gd(i),LaTol,gdTol)) {
        ds->setTermination(false);
        return;
      }
    }
  }

  void Joint::checkConstraintsForTermination() {

    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    for(int i=0; i < forceDir.cols(); i++) {
      gdd(i) = b(laIndDS+i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      if(!ffl->isFullfield(la(i),gdd(i),laTol,gddTol)) {
        ds->setTermination(false);
        return;
      }
    }
    for(int i=forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndDS+i]; j<ia[laIndDS+1+i]; j++)
        gdd(i) += a[j]*laMBS(ja[j]);

      if(!fml->isFullfield(la(i),gdd(i),laTol,gddTol)) {
        ds->setTermination(false);
        return;
      }
    }
  }

  void Joint::resizeJacobians(int j) {
    C.resizeJacobians(); // TODO: hSize of COSY C not set
  }


  void Joint::setForceDirection(const Mat &fd) {
    assert(fd.rows() == 3);

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.col(i) = forceDir.col(i)/nrm2(fd.col(i));
  }

  void Joint::setMomentDirection(const Mat &md) {
    assert(md.rows() == 3);

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.col(i) = momentDir.col(i)/nrm2(md.col(i));
  }

  void Joint::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e, *ee;
    LinkMechanics::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMNS"force");
    if(e) {
      ee=e->FirstChildElement(MBSIMNS"direction");
      setForceDirection(Mat(ee->GetText()));
      ee=ee->NextSiblingElement();
      GeneralizedForceLaw *gfl=ObjectFactory::createGeneralizedForceLaw(ee->FirstChildElement());
      setForceLaw(gfl);
      gfl->initializeUsingXML(ee->FirstChildElement());
      ee=ee->NextSiblingElement();
      GeneralizedImpactLaw *gifl=ObjectFactory::createGeneralizedImpactLaw(ee->FirstChildElement());
      setImpactForceLaw(gifl);
      gifl->initializeUsingXML(ee->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"moment");
    if(e) {
      ee=e->FirstChildElement(MBSIMNS"direction");
      setMomentDirection(Mat(ee->GetText()));
      ee=ee->NextSiblingElement();
      GeneralizedForceLaw *gfl=ObjectFactory::createGeneralizedForceLaw(ee->FirstChildElement());
      setMomentLaw(gfl);
      gfl->initializeUsingXML(ee->FirstChildElement());
      ee=ee->NextSiblingElement();
      GeneralizedImpactLaw *gifl=ObjectFactory::createGeneralizedImpactLaw(ee->FirstChildElement());
      setImpactMomentLaw(gifl);
      gifl->initializeUsingXML(ee->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"connect");
    FrameInterface *ref1=getFrameByPath(e->Attribute("ref1"));
    if(!ref1) { cerr<<"ERROR! Cannot find frame: "<<e->Attribute("ref1")<<endl; _exit(1); }
    FrameInterface *ref2=getFrameByPath(e->Attribute("ref2"));
    if(!ref2) { cerr<<"ERROR! Cannot find frame: "<<e->Attribute("ref2")<<endl; _exit(1); }
    connect(ref1,ref2);
  }

}

