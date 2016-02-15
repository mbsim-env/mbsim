/* Copyright (C) 2004-2010 MBSim Development Team
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
#include "mbsim/constraints/joint_constraint.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/frame.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/links/joint.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/constitutive_laws/bilateral_constraint.h"
#include "mbsim/objectfactory.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/frame.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  JointConstraint::Residuum::Residuum(vector<RigidBody*> body1_, vector<RigidBody*> body2_, const Mat3xV &forceDir_, const Mat3xV &momentDir_,Frame *frame1_, Frame *frame2_, Frame *refFrame_, double t_, vector<Frame*> i1_, vector<Frame*> i2_) : body1(body1_),body2(body2_),forceDir(forceDir_),momentDir(momentDir_),frame1(frame1_), frame2(frame2_), refFrame(refFrame_), t(t_), i1(i1_), i2(i2_) {}
  Vec JointConstraint::Residuum::operator()(const Vec &x) {
    Vec res(x.size(),NONINIT); 
    int nq = 0;
    for(unsigned int i=0; i<body1.size(); i++) {
      int dq = body1[i]->getqRelSize();
      body1[i]->resetPositionsUpToDate();
      body1[i]->setqRel(x(nq,nq+dq-1));
      nq += dq;
    }
    for(unsigned int i=0; i<body2.size(); i++) {
      int dq = body2[i]->getqRelSize();
      body2[i]->resetPositionsUpToDate();
      body2[i]->setqRel(x(nq,nq+dq-1));
      nq += dq;
    }

    Mat3xV dT = refFrame->getOrientation(t)*forceDir;
    Mat3xV dR = refFrame->getOrientation(t)*momentDir;

    if(dT.cols())
      res(Range<Var,Var>(0,dT.cols()-1)) = dT.T()*(frame1->getPosition(t)-frame2->getPosition(t));

    if(dR.cols())
      res(Range<Var,Var>(dT.cols(),dT.cols()+dR.cols()-1)) = dR.T()*AIK2Cardan(frame1->getOrientation().T()*frame2->getOrientation());

    return res;
  } 

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(JointConstraint, MBSIM%"JointConstraint")

  JointConstraint::JointConstraint(const string &name) : Constraint(name), bi(NULL), bi2(NULL), frame1(0), frame2(0), refFrame(NULL), refFrameID(0), nq(0), nu(0), nh(0), saved_ref1(""), saved_ref2("") {
  }

  void JointConstraint::connect(Frame* frame1_, Frame* frame2_) {
    frame1 = frame1_;
    frame2 = frame2_;
  }

  void JointConstraint::setDependentBodiesFirstSide(vector<RigidBody*> bd) {    
    bd1 = bd;
  }

  void JointConstraint::setDependentBodiesSecondSide(vector<RigidBody*> bd) {
    bd2 = bd;
  }

  void JointConstraint::setIndependentBody(RigidBody *bi_) {
    bi = bi_;
  }

  void JointConstraint::setSecondIndependentBody(RigidBody *bi2_) {
    bi2 = bi2_;
  }

  void JointConstraint::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      vector<RigidBody*> rigidBodies;
      if (saved_RigidBodyFirstSide.size()>0) {
        for (unsigned int i=0; i<saved_RigidBodyFirstSide.size(); i++)
          rigidBodies.push_back(getByPath<RigidBody>(saved_RigidBodyFirstSide[i]));
        setDependentBodiesFirstSide(rigidBodies);
      }
      rigidBodies.clear();
      if (saved_RigidBodySecondSide.size()>0) {
        for (unsigned int i=0; i<saved_RigidBodySecondSide.size(); i++)
          rigidBodies.push_back(getByPath<RigidBody>(saved_RigidBodySecondSide[i]));
        setDependentBodiesSecondSide(rigidBodies);
      }
      rigidBodies.clear();
      if (saved_IndependentBody!="")
        setIndependentBody(getByPath<RigidBody>(saved_IndependentBody));
      if (saved_IndependentBody2!="")
        setIndependentBody(getByPath<RigidBody>(saved_IndependentBody2));
      if(bd1.size()) {
        for(unsigned int i=0; i<bd1.size()-1; i++) 
          if1.push_back(bd1[i+1]->getFrameOfReference());
        if1.push_back(frame1);
      }
      if(bd2.size()) {
        for(unsigned int i=0; i<bd2.size()-1; i++) 
          if2.push_back(bd2[i+1]->getFrameOfReference());
        if2.push_back(frame2);
      }
      Constraint::init(stage);
    }
    else if(stage==preInit) {
      Constraint::init(stage);
      for(unsigned int i=0; i<bd1.size(); i++) 
        bd1[i]->addDependency(this);
      for(unsigned int i=0; i<bd2.size(); i++)
        bd2[i]->addDependency(this);
      if(bi)
        addDependency(bi);
      if(bi2)
        addDependency(bi2);
      refFrame=refFrameID?frame2:frame1;
    }
    else if(stage==resize) {
      Constraint::init(stage);
      nq = 0;
      nu = 0;
      nh = 0;
      Iq1.resize(bd1.size());
      Iu1.resize(bd1.size());
      Ih1.resize(bd1.size());
      for(unsigned int i=0; i<bd1.size(); i++) {
        int dq = bd1[i]->getqRelSize();
        int du = bd1[i]->getuRelSize();
        int dh = bd1[i]->gethSize(0);
        Iq1[i] = Index(nq,nq+dq-1);
        Iu1[i] = Index(nu,nu+du-1);
        Ih1[i] = Index(0,dh-1);
        nq += dq;
        nu += du;
        nh = max(nh,dh);
      }
      Iq2.resize(bd2.size());
      Iu2.resize(bd2.size());
      Ih2.resize(bd2.size());
      for(unsigned int i=0; i<bd2.size(); i++) {
        int dq = bd2[i]->getqRelSize();
        int du = bd2[i]->getuRelSize();
        int dh = bd2[i]->gethSize(0);
        Iq2[i] = Index(nq,nq+dq-1);
        Iu2[i] = Index(nu,nu+du-1);
        Ih2[i] = Index(0,dh-1);
        nq += dq;
        nu += du;
        nh = max(nh,dh);
      }

      q.resize(nq);
      JT.resize(3,nu);
      JR.resize(3,nu);
    } else
      Constraint::init(stage);
  }

  void JointConstraint::initz() {
    if(q0.size())
      q = q0;
  }

  void JointConstraint::updateGeneralizedCoordinates(double t) {
    Residuum f(bd1,bd2,forceDir,momentDir,frame1,frame2,refFrame,t,if1,if2);
    MultiDimNewtonMethod newton(&f);
    q = newton.solve(q);
    if(newton.getInfo()!=0)
      msg(Warn) << endl << "Error in JointConstraint: update of state dependent variables failed!" << endl;
    for(unsigned int i=0; i<bd1.size(); i++)
      bd1[i]->setqRel(q(Iq1[i]));
    for(unsigned int i=0; i<bd2.size(); i++)
      bd2[i]->setqRel(q(Iq2[i]));
    dT = refFrame->getOrientation(t)*forceDir;
    dR = refFrame->getOrientation(t)*momentDir;

    for(size_t i=0; i<bd1.size(); i++) {
      bd1[i]->setUpdateByReference(false);
      JT(Index(0,2),Iu1[i]) = frame1->getJacobianOfTranslation(t,2);
      JR(Index(0,2),Iu1[i]) = frame1->getJacobianOfRotation(t,2);
      for(size_t j=i+1; j<bd1.size(); j++)
        bd1[j]->resetJacobiansUpToDate();
      bd1[i]->setUpdateByReference(true);
    }
    for(size_t i=0; i<bd2.size(); i++) {
      bd2[i]->setUpdateByReference(false);
      JT(Index(0,2),Iu2[i]) = -frame2->getJacobianOfTranslation(t,2);
      JR(Index(0,2),Iu2[i]) = -frame2->getJacobianOfRotation(t,2);
      for(size_t j=i+1; j<bd2.size(); j++)
        bd2[j]->resetJacobiansUpToDate();
      bd2[i]->setUpdateByReference(true);
    }
    for(size_t i=0; i<bd1.size(); i++) {
      bd1[i]->resetJacobiansUpToDate();
      bd1[i]->setuRel(Vec(bd1[i]->getuRel().size()));
    }
    for(size_t i=0; i<bd2.size(); i++) {
      bd2[i]->resetJacobiansUpToDate();
      bd2[i]->setuRel(Vec(bd2[i]->getuRel().size()));
    }
    SqrMat A(nu);
    A(Index(0,dT.cols()-1),Index(0,nu-1)) = dT.T()*JT;
    A(Index(dT.cols(),dT.cols()+dR.cols()-1),Index(0,nu-1)) = dR.T()*JR;
    Vec b(nu);

    b(0,dT.cols()-1) = -(dT.T()*(frame1->getVelocity(t)-frame2->getVelocity(t)));
    b(dT.cols(),dT.cols()+dR.cols()-1) = -(dR.T()*(frame1->getAngularVelocity(t)-frame2->getAngularVelocity(t)));
    Vec u = slvLU(A,b);
    for(unsigned int i=0; i<bd1.size(); i++) {
      bd1[i]->resetVelocitiesUpToDate();
      bd1[i]->setuRel(u(Iu1[i]));
    }
    for(unsigned int i=0; i<bd2.size(); i++) {
      bd2[i]->resetVelocitiesUpToDate();
      bd2[i]->setuRel(u(Iu2[i]));
    }
    updGC = false;
  }

  void JointConstraint::updateGeneralizedJacobians(double t, int jj) {
    if(jj == 0) {

      for(unsigned int i=0; i<bd1.size(); i++) {
        bd1[i]->setJRel(Mat(bd1[i]->getuRelSize(),bd1[i]->gethSize()));
        bd1[i]->setjRel(Vec(bd1[i]->getuRelSize()));
      }
      for(unsigned int i=0; i<bd2.size(); i++) {
        bd2[i]->setJRel(Mat(bd2[i]->getuRelSize(),bd2[i]->gethSize()));
        bd2[i]->setjRel(Vec(bd2[i]->getuRelSize()));
      }

      SqrMat A(nu);
      A(Index(0,dT.cols()-1),Index(0,nu-1)) = dT.T()*JT;
      A(Index(dT.cols(),dT.cols()+dR.cols()-1),Index(0,nu-1)) = dR.T()*JR;
      Mat B(nu,nh);
      Mat JT0(3,nh);
      Mat JR0(3,nh);
      if(frame1->getJacobianOfTranslation(0,false).cols()) {
        JT0(Index(0,2),Index(0,frame1->getJacobianOfTranslation(0,false).cols()-1))+=frame1->getJacobianOfTranslation(t);
        JR0(Index(0,2),Index(0,frame1->getJacobianOfRotation(0,false).cols()-1))+=frame1->getJacobianOfRotation(t);
      }
      if(frame2->getJacobianOfTranslation(0,false).cols()) {
        JT0(Index(0,2),Index(0,frame2->getJacobianOfTranslation(0,false).cols()-1))-=frame2->getJacobianOfTranslation(t);
        JR0(Index(0,2),Index(0,frame2->getJacobianOfRotation(0,false).cols()-1))-=frame2->getJacobianOfRotation(t);
      }
      B(Index(0,dT.cols()-1),Index(0,nh-1)) = -(dT.T()*JT0);
      B(Index(dT.cols(),dT.cols()+dR.cols()-1),Index(0,nh-1)) = -(dR.T()*JR0);
      Vec b(nu);
      b(0,dT.cols()-1) = -(dT.T()*(frame1->getGyroscopicAccelerationOfTranslation(t)-frame2->getGyroscopicAccelerationOfTranslation(t)));
      b(dT.cols(),dT.cols()+dR.cols()-1) = -(dR.T()*(frame1->getGyroscopicAccelerationOfRotation(t)-frame2->getGyroscopicAccelerationOfRotation(t)));

      Mat J = slvLU(A,B);
      Vec j = slvLU(A,b);
      for(unsigned int i=0; i<bd1.size(); i++) {
        bd1[i]->resetJacobiansUpToDate();
        bd1[i]->resetGyroscopicAccelerationsUpToDate();
        bd1[i]->setJRel(J(Iu1[i],Ih1[i]));
        bd1[i]->setjRel(j(Iu1[i]));
      }
      for(unsigned int i=0; i<bd2.size(); i++) {
       bd2[i]->resetJacobiansUpToDate();
       bd2[i]->resetGyroscopicAccelerationsUpToDate();
       bd2[i]->setJRel(J(Iu2[i],Ih2[i]));
       bd2[i]->setjRel(j(Iu2[i]));
      }
      updGJ = false;
    }
  }

  void JointConstraint::setUpInverseKinetics() {
    InverseKineticsJoint *joint = new InverseKineticsJoint(string("Joint_")+name);
    static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(joint);
    if(forceDir.cols()) {
      joint->setForceDirection(forceDir);
      joint->setForceLaw(new BilateralConstraint);
    }
    if(momentDir.cols()) {
      joint->setMomentDirection(momentDir);
      joint->setMomentLaw(new BilateralConstraint);
    }
    joint->connect(frame1,frame2);
    if(FArrow)
      joint->setOpenMBVForce(FArrow);
    if(MArrow)
      joint->setOpenMBVMoment(MArrow);
  }

  void JointConstraint::initializeUsingXML(DOMElement *element) {
    DOMElement *e, *ee;
    Constraint::initializeUsingXML(element);
    e=E(element)->getFirstElementChildNamed(MBSIM%"initialGuess");
    if (e) setInitialGuess(getVec(e));
    e=E(element)->getFirstElementChildNamed(MBSIM%"dependentRigidBodiesFirstSide");
    ee=e->getFirstElementChild();
    while(ee) {
      saved_RigidBodyFirstSide.push_back(E(ee)->getAttribute("ref"));
      ee=ee->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"dependentRigidBodiesSecondSide");
    ee=e->getFirstElementChild();
    while(ee) {
      saved_RigidBodySecondSide.push_back(E(ee)->getAttribute("ref"));
      ee=ee->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"independentRigidBody");
    saved_IndependentBody=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"secondIndependentRigidBody");
    if(e) saved_IndependentBody2=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReferenceID");
    if(e) refFrameID=getInt(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    if(e) setForceDirection(getMat3xV(e,0));
    e=E(element)->getFirstElementChildNamed(MBSIM%"momentDirection");
    if(e) setMomentDirection(getMat3xV(e,3));

    e=E(element)->getFirstElementChildNamed(MBSIM%"connect");
    saved_ref1=E(e)->getAttribute("ref1");
    saved_ref2=E(e)->getAttribute("ref2");

#ifdef HAVE_OPENMBVCPPINTERFACE
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      FArrow=ombv.createOpenMBV(e);
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      MArrow=ombv.createOpenMBV(e);
    }
#endif
  }

  DOMElement* JointConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Constraint::writeXMLFile(parent);
//    if(q0.size()) 
//      addElementText(ele0,MBSIM%"initialGeneralizedPosition",q0);
//    DOMElement *ele1 = new DOMElement( MBSIM%"dependentRigidBodiesFirstSide" );
//    for(unsigned int i=0; i<bd1.size(); i++) {
//      DOMElement *ele2 = new DOMElement( MBSIM%"dependentRigidBody" );
//      ele2->SetAttribute("ref", bd1[i]->getXMLPath(this,true)); // relative path
//      ele1->LinkEndChild(ele2);
//    }
//    ele0->LinkEndChild(ele1);
//    ele1 = new DOMElement( MBSIM%"dependentRigidBodiesSecondSide" );
//    for(unsigned int i=0; i<bd2.size(); i++) {
//      DOMElement *ele2 = new DOMElement( MBSIM%"dependentRigidBody" );
//      ele2->SetAttribute("ref", bd2[i]->getXMLPath(this,true)); // relative path
//      ele1->LinkEndChild(ele2);
//    }
//    ele0->LinkEndChild(ele1);
//
//    ele1 = new DOMElement( MBSIM%"independentRigidBody" );
//    ele1->SetAttribute("ref", bi->getXMLPath(this,true)); // relative path
//    ele0->LinkEndChild(ele1);
//
//    if(dT.cols())
//      addElementText(ele0, MBSIM%"forceDirection", dT);
//    if(dR.cols())
//      addElementText(ele0, MBSIM%"momentDirection", dR);
//
//    ele1 = new DOMElement(MBSIM%"connect");
//    ele1->SetAttribute("ref1", frame1->getXMLPath(this,true)); // relative path
//    ele1->SetAttribute("ref2", frame2->getXMLPath(this,true)); // relative path
//    ele0->LinkEndChild(ele1);
//
//    if(FArrow) {
//      ele1 = new DOMElement( MBSIM%"openMBVJointForceArrow" );
//      FArrow->writeXMLFile(ele1);
//      ele0->LinkEndChild(ele1);
//    }
//
//    if(MArrow) {
//      ele1 = new DOMElement( MBSIM%"openMBVJointMomentArrow" );
//      MArrow->writeXMLFile(ele1);
//      ele0->LinkEndChild(ele1);
//    }

    return ele0;
  }

  void JointConstraint::setForceDirection(const Mat3xV &fd) {

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.set(i, forceDir.col(i)/nrm2(fd.col(i)));
  }

  void JointConstraint::setMomentDirection(const Mat3xV &md) {

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.set(i, momentDir.col(i)/nrm2(md.col(i)));
  }


}
