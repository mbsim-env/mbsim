/* Copyright (C) 2004-2009 MBSim Development Team
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

using namespace MBSim;
using namespace fmatvec;
using namespace std;

namespace MBSim {

  class Residuum : public Function1<Vec,Vec> {
    RigidBody *body1, *body2;
    Mat d;
    Frame *frame1, *frame2;
    double t;
    int i1,i2;
    public:
    Residuum(RigidBody* body1_, RigidBody* body2_, const Mat &d_,Frame *frame1_, Frame *frame2_,double t_,int i1_, int i2_) : body1(body1_),body2(body2_),d(d_),frame1(frame1_), frame2(frame2_), t(t_), i1(i1_), i2(i2_) {}
    Vec operator()(const Vec &x, const void * =NULL) {
      Vec res(x.size()); 
      int nq1 = body1->getqRel().size();
      int nq2 = body2->getqRel().size();

      body1->getqRel() = x(0,nq1-1);
      body2->getqRel() = x(nq1,nq1+nq2-1);

      body1->updatePositionAndOrientationOfFrame(t,i1);
      body2->updatePositionAndOrientationOfFrame(t,i2);

      res = d.T()*(frame1->getPosition()-frame2->getPosition()); 

      return res;
    } 
  };

  class Residuum2 : public Function1<Vec,Vec> {
    RigidBody *body1, *body2;
    Mat d;
    Frame *frame1, *frame2, *frame3;
    double t;
    int i1,i2;
    public:
    Residuum2(RigidBody* body1_, RigidBody* body2_, const Mat &d_,Frame *frame1_, Frame *frame2_, Frame *frame3_, double t_,int i1_, int i2_) : body1(body1_),body2(body2_),d(d_),frame1(frame1_), frame2(frame2_), frame3(frame3_), t(t_), i1(i1_), i2(i2_) {}
    Vec operator()(const Vec &x, const void * =NULL) {
      Vec res(x.size()); 
      int nq1 = body1->getqRel().size();
      int nq2 = body2->getqRel().size();

      body1->getqRel() = x(0,nq1-1);
      body2->getqRel() = x(nq1,nq1+nq2-1);

      body1->updatePositionAndOrientationOfFrame(t,i1);
      body2->updatePositionAndOrientationOfFrame(t,i2);

      res = d.T()*(frame2->getPosition()-frame3->getPosition()); 

      return res;
    } 
  };

  class Residuum3 : public Function1<Vec,Vec> {
    vector<RigidBody*> body1, body2;
    Mat dT, dR;
    Frame *frame1, *frame2;
    double t;
    vector<int> i1,i2;
    public:
    Residuum3(vector<RigidBody*> body1_, vector<RigidBody*> body2_, const Mat &dT_, const Mat &dR_,Frame *frame1_, Frame *frame2_,double t_,vector<int> i1_, vector<int> i2_) : body1(body1_),body2(body2_),dT(dT_),dR(dR_),frame1(frame1_), frame2(frame2_), t(t_), i1(i1_), i2(i2_) {}
    Vec operator()(const Vec &x, const void * =NULL) {
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
      if(nT) {
	res(0,nT-1) = dT.T()*(frame1->getPosition()-frame2->getPosition()); 
      }
      if(nR) { // TODO
	res(nT,nT+nR-1) = dR.T()*(frame1->getPosition()-frame2->getPosition()); 
      }

      return res;
    } 
  };

  Constraint::Constraint(const std::string &name) : Object(name) {
  }

  Constraint1::Constraint1(const std::string &name, RigidBody *b0, RigidBody* b1, RigidBody* b2, Frame* frame1_, Frame* frame2_) : Constraint(name), frame1(frame1_), frame2(frame2_) {
    bi = b0;
    bd1 = b1;
    bd2 = b2;
    bd1->addDependency(this);
    bd2->addDependency(this);
    if1 = bd1->frameIndex(frame1);
    if2 = bd2->frameIndex(frame2);
  }

  void Constraint1::init(InitStage stage) {
    if(stage==preInit) {
      Constraint::init(stage);
      dependency.push_back(bi);
    }
    else
      Constraint::init(stage);
  }

  void Constraint1::initz() {
    int nq1 = bd1->getqRel().size();
    int nq2 = bd2->getqRel().size();
    int nu1 = bd1->getuRel().size();
    int nu2 = bd2->getuRel().size();
    q12.resize(nq1+nq2);
    u12.resize(nu1+nu2);
    int nh = bd1->getJRel().cols();
    J12.resize(nu1+nu2,nh);
    j12.resize(nu1+nu2);

    bd1->getqRel() >> q12(0,nq1-1);
    bd2->getqRel() >> q12(nq1,nq1+nq2-1);
    bd1->getuRel() >> u12(0,nu1-1);
    bd2->getuRel() >> u12(nu1,nu1+nu2-1);
    bd1->getJRel() >> J12(Index(0,nu1-1),Index(0,nh-1)); //(Index(0,nu1-1),Index(0,nh1-1))
    bd2->getJRel() >> J12(Index(nu1,nu1+nu2-1),Index(0,nh-1));
    bd1->getjRel() >> j12(Index(0,nu1-1)); // (Index(0,nu1-1))
    bd2->getjRel() >> j12(Index(nu1,nu1+nu2-1));
    J12t.resize(3,nu1+nu2);
  }

  void Constraint1::updateStateDependentVariables(double t){
    Residuum* f = new Residuum(bd1,bd2,d,frame1,frame2,t,if1,if2);
    MultiDimNewtonMethod newton(f);
    q12 = newton.solve(q12);

    bd1->updateRelativeJacobians(t,if1);
    bd2->updateRelativeJacobians(t,if2);
    int nu1 = bd1->getWJTrel().cols();
    int nu2 = bd2->getWJTrel().cols();

 //   Mat J12(3,nu1+nu2);
    J12t(Index(0,2),Index(0,nu1-1)) = bd1->getWJTrel();
    J12t(Index(0,2),Index(nu1,nu1+nu2-1)) = -bd2->getWJTrel();
    u12 = slvLU(SqrMat(d.T()*J12t),-d.T()*(frame1->getVelocity()-frame2->getVelocity())); 
  }

  void Constraint1::updateJacobians(double t){
    //int nu1 = bd1->getWJTrel().cols();
    //int nu2 = bd2->getWJTrel().cols();
 //   Mat J12t(3,nu1+nu2);
 //   J12t(Index(0,2),Index(0,nu1-1)) = bd1->getWJTrel();
 //   J12t(Index(0,2),Index(nu1,nu1+nu2-1)) = -bd2->getWJTrel();

    bd1->updateAcclerations(t,if1);
    bd2->updateAcclerations(t,if2);

    J12 = slvLU(SqrMat(d.T()*J12t),-d.T()*(frame1->getJacobianOfTranslation()-frame2->getJacobianOfTranslation())); 
    j12 = slvLU(SqrMat(d.T()*J12t),-d.T()*(frame1->getGyroscopicAccelerationOfTranslation()-frame2->getGyroscopicAccelerationOfTranslation()));

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

  Constraint4::Constraint4(const std::string &name, RigidBody *b0, RigidBody* b1, RigidBody* b2, Frame* frame1_, Frame* frame2_, Frame* frame3_) : Constraint(name), frame1(frame1_), frame2(frame2_), frame3(frame3_) {
    bi = b0;
    bd1 = b1;
    bd2 = b2;
    bd1->addDependency(this);
    bd2->addDependency(this);
    if1 = bd1->frameIndex(frame1);
    if2 = bd2->frameIndex(frame2);
  }

  void Constraint4::initz() {
      int nq1 = bd1->getqRel().size();
      int nq2 = bd2->getqRel().size();
      x0.resize(nq1+nq2);
      x0(0,nq1-1).init(0.52);
      x0(nq1,nq1+nq2-1).init(-2.2);
  }

  void Constraint4::init(InitStage stage) {
    if(stage==preInit) {
      Constraint::init(stage);
      dependency.push_back(bi);
    } 
    else
      Constraint::init(stage);
  }

  void Constraint4::updateStateDependentVariables(double t){
    Residuum2* f = new Residuum2(bd1,bd2,d,frame1,frame2,frame3,t,if1,if2);
    MultiDimNewtonMethod newton(f);
    int nq1 = bd1->getqRel().size();
    int nq2 = bd2->getqRel().size();
    //Vec x0(nq1+nq2);
  //  if(true || t>0) {
  //  x0(0,nq1-1) = bd1->getqRel();
  //  x0(nq1,nq1+nq2-1) = bd2->getqRel();
  //  }
  //  else {
  //  x0(0,nq1-1).init(0.52);
  //  x0(nq1,nq1+nq2-1).init(-2.2);
  //  }
    //Vec q12 = newton.solve(x0);
    //bd1->getqRel() = q12(0,nq1-1);
    //bd2->getqRel() = q12(nq1,nq1+nq2-1);
    x0 = newton.solve(x0);
    bd1->getqRel() = x0(0,nq1-1);
    bd2->getqRel() = x0(nq1,nq1+nq2-1);

    bd1->updateRelativeJacobians(t,if1);
    bd2->updateRelativeJacobians(t,if2);
    bd2->updateRelativeJacobians(t,if2,bd1->getWJTrel(),bd1->getWJRrel());

    int nu1 = bd1->getWJTrel().cols();
    int nu2 = bd2->getWJTrel().cols();

    Mat J12(3,nu1+nu2);
    J12(Index(0,2),Index(0,nu1-1)) = bd1->getWJTrel();
    J12(Index(0,2),Index(nu1,nu1+nu2-1)) = bd2->getWJTrel();
    Vec u12 = slvLU(SqrMat(d.T()*J12),-d.T()*frame2->getVelocity()); 
    bd1->getuRel() = u12(0,nu1-1);
    bd2->getuRel() = u12(nu1,nu1+nu2-1);
  }

  void Constraint4::updateJacobians(double t){
    int nu1 = bd1->getWJTrel().cols();
    int nu2 = bd2->getWJTrel().cols();
    Mat J12t(3,nu1+nu2);
    J12t(Index(0,2),Index(0,nu1-1)) = bd1->getWJTrel();
    J12t(Index(0,2),Index(nu1,nu1+nu2-1)) = bd2->getWJTrel();

    bd1->updateAcclerations(t,if1);
    bd2->updateAcclerations(t,if2);

    Mat J12 = slvLU(SqrMat(d.T()*J12t),-d.T()*frame2->getJacobianOfTranslation()); 
    Vec j12 = slvLU(SqrMat(d.T()*J12t),-d.T()*frame2->getGyroscopicAccelerationOfTranslation());

    int nh1 = bd1->getJRel().cols();
    int nh2 = bd2->getJRel().cols();
    bd1->getJRel() = J12(Index(0,nu1-1),Index(0,nh1-1)); 
    bd2->getJRel() = J12(Index(nu1,nu1+nu2-1),Index(0,nh2-1));
    bd1->getjRel() = j12(Index(0,nu1-1)); 
    bd2->getjRel() = j12(Index(nu1,nu1+nu2-1));

  }

  Constraint5::Constraint5(const std::string &name, RigidBody* bi_, std::vector<RigidBody*> bd1_, std::vector<RigidBody*> bd2_, Frame* frame1_, Frame* frame2_) : Constraint(name), frame1(frame1_), frame2(frame2_) {
    bi = bi_;
    bd1 = bd1_;
    bd2 = bd2_;
    cout << bd1.size() << endl;
    cout << bd2.size() << endl;
    cout << bd1_.size() << endl;
    cout << bd2_.size() << endl;

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

  void Constraint5::init(InitStage stage) {
    if(stage==preInit) {
      Constraint::init(stage);
      dependency.push_back(bi);
    } 
    else if(stage==unknownStage) {
      if(!dT.cols()) 
	dT.resize(3,0);
      if(!dR.cols()) 
	dR.resize(3,0);
    } else
      Constraint::init(stage);
  }

  void Constraint5::initz() {
    nq = 0;
    nu = 0;
    for(unsigned int i=0; i<bd1.size(); i++) {
      int dq = bd1[i]->getqRel().size();
      int du = bd1[i]->getuRel().size();
      Iq1.push_back(Index(nq,nq+dq-1));
      Iu1.push_back(Index(nu,nu+du-1));
      nq += dq;
      nu += du;
    }
     for(unsigned int i=0; i<bd2.size(); i++) {
	int dq = bd2[i]->getqRel().size();
	int du = bd2[i]->getuRel().size();
	Iq2.push_back(Index(nq,nq+dq-1));
	Iu2.push_back(Index(nu,nu+du-1));
	nq += dq;
	nu += du;
      }
    int nh = bd1[0]->getJRel().cols();
    q.resize(nq);
    u.resize(nu);
    J.resize(nu,nh);
    j.resize(nu);
    JT.resize(3,nu);
    JR.resize(3,nu);
    for(unsigned int i=0; i<bd1.size(); i++) {
      bd1[i]->getqRel() >> q(Iq1[i]);
      bd1[i]->getuRel() >> u(Iu1[i]);
      bd1[i]->getJRel() >> J(Iu1[i],Index(0,nh-1));
      bd1[i]->getjRel() >> j(Iu1[i]); 
    }
    for(unsigned int i=0; i<bd2.size(); i++) {
      bd2[i]->getqRel() >> q(Iq2[i]);
      bd2[i]->getuRel() >> u(Iu2[i]);
      bd2[i]->getJRel() >> J(Iu2[i],Index(0,nh-1));
      bd2[i]->getjRel() >> j(Iu2[i]); 
    }   
    q = q0;
  }


  void Constraint5::updateStateDependentVariables(double t){
    Residuum3* f = new Residuum3(bd1,bd2,dT,dR,frame1,frame2,t,if1,if2);
    MultiDimNewtonMethod newton(f);
    q = newton.solve(q);

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

  void Constraint5::updateJacobians(double t) {

    for(unsigned int i=0; i<bd1.size(); i++)
      bd1[i]->updateAcclerations(t,if1[i]);
    for(unsigned int i=0; i<bd2.size(); i++)
      bd2[i]->updateAcclerations(t,if2[i]);

    int nh = bd1[0]->getJRel().cols();

    SqrMat A(nu);
    A(Index(0,dT.cols()-1),Index(0,nu-1)) = dT.T()*JT;
    A(Index(dT.cols(),dT.cols()+dR.cols()-1),Index(0,nu-1)) = dR.T()*JR;
    Mat B(nu,nh);
    Mat JT0(3,nh);
    Mat JR0(3,nh);
    if(frame1->getJacobianOfTranslation().cols()) {
      JT0+=frame1->getJacobianOfTranslation();
      JR0+=frame1->getJacobianOfRotation();
    }
    if(frame2->getJacobianOfTranslation().cols()) {
      JT0-=frame2->getJacobianOfTranslation();
      JR0-=frame2->getJacobianOfRotation();
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
