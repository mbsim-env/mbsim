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
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include "constraint.h"
#include "rigid_body.h"
#include "utils/nonlinear_algebra.h"
#include "utils/utils.h"
#include "utils/rotarymatrices.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

namespace MBSim {

  JointConstraint::Residuum::Residuum(vector<RigidBody*> body1_, vector<RigidBody*> body2_, const Mat &dT_, const Mat &dR_,Frame *frame1_, Frame *frame2_,double t_,vector<int> i1_, vector<int> i2_) : body1(body1_),body2(body2_),dT(dT_),dR(dR_),frame1(frame1_), frame2(frame2_), t(t_), i1(i1_), i2(i2_) {}
  Vec JointConstraint::Residuum::operator()(const Vec &x, const void *) {
    Vec res(x.size()); 
    int nq = 0;
    for(unsigned int i=0; i<body1.size(); i++) {
      int dq = body1[i]->getqRel().size();
      body1[i]->getqRel() = x(nq,nq+dq-1);
      nq += dq;
    }
    for(unsigned int i=0; i<body2.size(); i++) {
      int dq = body2[i]->getqRel().size();
      body2[i]->getqRel() = x(nq,nq+dq-1);
      nq += dq;
    }

    for(unsigned int i=0; i<body1.size(); i++) {
      body1[i]->updatePositionAndOrientationOfFrame(t,i1[i]);
    }
    for(unsigned int i=0; i<body2.size(); i++) {
      body2[i]->updatePositionAndOrientationOfFrame(t,i2[i]);
    }

    int nT = dT.cols();
    int nR = dR.cols();

    if(nT) 
      res(0,nT-1) = dT.T()*(frame1->getPosition()-frame2->getPosition()); 

    if(nR) 
      res(nT,nT+nR-1) = dR.T()*AIK2Cardan(trans(frame1->getOrientation())*frame2->getOrientation()); 

    return res;
  } 

  Constraint::Constraint(const std::string &name) : Object(name) {
  }

  Constraint2::Constraint2(const std::string &name, RigidBody* body) : Constraint(name), bd(body) {
    bd->addDependency(this);
  }

  void Constraint2::init(InitStage stage) {
    if(stage==preInit) {
      Constraint::init(stage);
      for(unsigned int i=0; i<bi.size(); i++)
	dependency.push_back(bi[i]);
    }
    else
      Constraint::init(stage);
  }

  void Constraint2::addDependency(RigidBody* body, double ratio_) {
    bi.push_back(body); 
    ratio.push_back(ratio_);
  }

  void Constraint2::updateStateDependentVariables(double t){
    bd->getqRel().init(0);
    bd->getuRel().init(0);
    for(unsigned int i=0; i<bi.size(); i++) {
      bd->getqRel() += bi[i]->getqRel()*ratio[i];
      bd->getuRel() += bi[i]->getuRel()*ratio[i];
    }
  }

  void Constraint2::updateJacobians(double t){
    bd->getJRel().init(0); 
    for(unsigned int i=0; i<bi.size(); i++) {
      bd->getJRel()(Index(0,bi[i]->getJRel().rows()-1),Index(0,bi[i]->getJRel().cols()-1)) += bi[i]->getJRel()*ratio[i];
    }
  }

  Constraint3::Constraint3(const std::string &name, RigidBody* body) : Constraint(name), bd(body) {
    bd->addDependency(this);
  }

  void Constraint3::updateStateDependentVariables(double t) {
    bd->getqRel().init(0);
    bd->getuRel().init(0);
  }

  void Constraint3::updateJacobians(double t) {
    bd->getJRel().init(0); 
  }

  JointConstraint::JointConstraint(const std::string &name, std::vector<RigidBody*> bd1_, std::vector<RigidBody*> bd2_, Frame* frame1_, Frame* frame2_) : Constraint(name), frame1(frame1_), frame2(frame2_) {
    bd1 = bd1_;
    bd2 = bd2_;

    for(unsigned int i=0; i<bd1.size(); i++) {
      bd1[i]->addDependency(this);
    }
    for(unsigned int i=0; i<bd2.size(); i++) {
      bd2[i]->addDependency(this);
    }
    if(bd1.size()) {
      for(unsigned int i=0; i<bd1.size()-1; i++) 
	if1.push_back(bd1[i]->frameIndex(bd1[i+1]->getFrameOfReference()));
      if1.push_back(bd1[bd1.size()-1]->frameIndex(frame1));
    }
    if(bd2.size()) {
      for(unsigned int i=0; i<bd2.size()-1; i++) 
	if2.push_back(bd2[i]->frameIndex(bd2[i+1]->getFrameOfReference()));
      if2.push_back(bd2[bd2.size()-1]->frameIndex(frame2));
    }
  }

  void JointConstraint::init(InitStage stage) {
    if(stage==preInit) {
      Constraint::init(stage);
      if(bd1.size()) {
	Body* obj = dynamic_cast<Body*>(bd1[0]->getFrameOfReference()->getParent());
	if(obj)
	  dependency.push_back(obj);
      }
      if(bd2.size()) {
	Body* obj = dynamic_cast<Body*>(bd2[0]->getFrameOfReference()->getParent());
	if(obj)
	  dependency.push_back(obj);
      }
    } 
    else if(stage==unknownStage) {
      if(!dT.cols()) 
	dT.resize(3,0);
      if(!dR.cols()) 
	dR.resize(3,0);
    } else
      Constraint::init(stage);
  }

  void JointConstraint::initz() {
    nq = 0;
    nu = 0;
    nh = 0;
    for(unsigned int i=0; i<bd1.size(); i++) {
      int dq = bd1[i]->getqRel().size();
      int du = bd1[i]->getuRel().size();
      int dh = bd1[i]->gethSize(0);
      Iq1.push_back(Index(nq,nq+dq-1));
      Iu1.push_back(Index(nu,nu+du-1));
      Ih1.push_back(Index(0,dh-1));
      nq += dq;
      nu += du;
      nh = max(nh,dh);
    }
    for(unsigned int i=0; i<bd2.size(); i++) {
      int dq = bd2[i]->getqRel().size();
      int du = bd2[i]->getuRel().size();
      int dh = bd2[i]->gethSize(0);
      Iq2.push_back(Index(nq,nq+dq-1));
      Iu2.push_back(Index(nu,nu+du-1));
      Ih2.push_back(Index(0,dh-1));
      nq += dq;
      nu += du;
      nh = max(nh,dh);
    }

    q.resize(nq);
    u.resize(nu);
    J.resize(nu,nh);
    j.resize(nu);
    JT.resize(3,nu);
    JR.resize(3,nu);
    for(unsigned int i=0; i<bd1.size(); i++) {
      bd1[i]->getqRel() >> q(Iq1[i]);
      bd1[i]->getuRel() >> u(Iu1[i]);
      bd1[i]->getJRel() >> J(Iu1[i],Ih1[i]);
      bd1[i]->getjRel() >> j(Iu1[i]); 
    }
    for(unsigned int i=0; i<bd2.size(); i++) {
      bd2[i]->getqRel() >> q(Iq2[i]);
      bd2[i]->getuRel() >> u(Iu2[i]);
      bd2[i]->getJRel() >> J(Iu2[i],Ih2[i]);
      bd2[i]->getjRel() >> j(Iu2[i]); 
    }   
    q = q0;
  }


  void JointConstraint::updateStateDependentVariables(double t){
    Residuum* f = new Residuum(bd1,bd2,dT,dR,frame1,frame2,t,if1,if2);
    MultiDimNewtonMethod newton(f);
    q = newton.solve(q);
    assert(newton.getInfo()==0);

    for(unsigned int i=0; i<bd1.size(); i++) {
      bd1[i]->updateRelativeJacobians(t,if1[i]);
      for(unsigned int j=i+1; j<bd1.size(); j++) 
	bd1[j]->updateRelativeJacobians(t,if1[j],bd1[i]->getWJTrel(),bd1[i]->getWJRrel());
    }

    for(unsigned int i=0; i<bd2.size(); i++) {
      bd2[i]->updateRelativeJacobians(t,if2[i]);
      for(unsigned int j=i+1; j<bd2.size(); j++) 
	bd2[j]->updateRelativeJacobians(t,if2[j],bd2[i]->getWJTrel(),bd2[i]->getWJRrel());
    }

    for(unsigned int i=0; i<bd1.size(); i++) {
      JT(Index(0,2),Iu1[i]) = bd1[i]->getWJTrel();
      JR(Index(0,2),Iu1[i]) = bd1[i]->getWJRrel();
    }
    for(unsigned int i=0; i<bd2.size(); i++) {
      JT(Index(0,2),Iu2[i]) = -bd2[i]->getWJTrel();
      JR(Index(0,2),Iu2[i]) = -bd2[i]->getWJRrel();
    }
    SqrMat A(nu);
    A(Index(0,dT.cols()-1),Index(0,nu-1)) = dT.T()*JT;
    A(Index(dT.cols(),dT.cols()+dR.cols()-1),Index(0,nu-1)) = dR.T()*JR;
    Vec b(nu);
    b(0,dT.cols()-1) = -dT.T()*(frame1->getVelocity()-frame2->getVelocity());
    b(dT.cols(),dT.cols()+dR.cols()-1) = -dR.T()*(frame1->getAngularVelocity()-frame2->getAngularVelocity());
    u = slvLU(A,b); 
  }

  void JointConstraint::updateJacobians(double t) {

    for(unsigned int i=0; i<bd1.size(); i++)
      bd1[i]->updateAccelerations(t,if1[i]);
    for(unsigned int i=0; i<bd2.size(); i++)
      bd2[i]->updateAccelerations(t,if2[i]);

    SqrMat A(nu);
    A(Index(0,dT.cols()-1),Index(0,nu-1)) = dT.T()*JT;
    A(Index(dT.cols(),dT.cols()+dR.cols()-1),Index(0,nu-1)) = dR.T()*JR;
    Mat B(nu,nh);
    Mat JT0(3,nh);
    Mat JR0(3,nh);
    if(frame1->getJacobianOfTranslation().cols()) {
      JT0(Index(0,2),Index(0,frame1->getJacobianOfTranslation().cols()-1))+=frame1->getJacobianOfTranslation();
      JR0(Index(0,2),Index(0,frame1->getJacobianOfRotation().cols()-1))+=frame1->getJacobianOfRotation();
    }
    if(frame2->getJacobianOfTranslation().cols()) {
      JT0(Index(0,2),Index(0,frame2->getJacobianOfTranslation().cols()-1))-=frame2->getJacobianOfTranslation();
      JR0(Index(0,2),Index(0,frame2->getJacobianOfRotation().cols()-1))-=frame2->getJacobianOfRotation();
    }
    B(Index(0,dT.cols()-1),Index(0,nh-1)) = -dT.T()*JT0;
    B(Index(dT.cols(),dT.cols()+dR.cols()-1),Index(0,nh-1)) = -dR.T()*JR0;
    Vec b(nu);
    b(0,dT.cols()-1) = -dT.T()*(frame1->getGyroscopicAccelerationOfTranslation()-frame2->getGyroscopicAccelerationOfTranslation());
    b(dT.cols(),dT.cols()+dR.cols()-1) = -dR.T()*(frame1->getGyroscopicAccelerationOfRotation()-frame2->getGyroscopicAccelerationOfRotation());

    J = slvLU(A,B); 
    j = slvLU(A,b); 
  }

}
