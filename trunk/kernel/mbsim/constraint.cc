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

  void Constraint1::updateStateDependentVariables(double t){
    Residuum* f = new Residuum(bd1,bd2,d,frame1,frame2,t,if1,if2);
    MultiDimNewtonMethod newton(f);
    int nq1 = bd1->getqRel().size();
    int nq2 = bd2->getqRel().size();
    Vec x0(nq1+nq2);
    x0(0,nq1-1) = bd1->getqRel();
    x0(nq1,nq1+nq2-1) = bd2->getqRel();
    Vec q12 = newton.solve(x0);
    bd1->getqRel() = q12(0,nq1-1);
    bd2->getqRel() = q12(nq1,nq1+nq2-1);

    bd1->updateRelativeJacobians(t,if1);
    bd2->updateRelativeJacobians(t,if2);
    int nu1 = bd1->getWJTrel().cols();
    int nu2 = bd2->getWJTrel().cols();
    //int nu1 = frame1->getJacobianOfTranslation().cols();
    //int nu2 = frame2->getJacobianOfTranslation().cols();

    Mat J12(3,nu1+nu2);
    J12(Index(0,2),Index(0,nu1-1)) = bd1->getWJTrel();
    J12(Index(0,2),Index(nu1,nu1+nu2-1)) = bd2->getWJTrel();
    //J12(Index(0,2),Index(0,nu1-1)) = frame1->getJacobianOfTranslation();
    //J12(Index(0,2),Index(nu1,nu1+nu2-1)) = frame2->getJacobianOfTranslation();
    //Vec u12 = slvLU(SqrMat(d.T()*J12),-d.T()*(bd1->getWjTrel()-bd2->getWjTrel())); 
    Vec u12 = slvLU(SqrMat(d.T()*J12),-d.T()*(frame1->getVelocity()-frame2->getVelocity())); 
    bd1->getuRel() = u12(0,nu1-1);
    bd2->getuRel() = u12(nu1,nu1+nu2-1);
    //bd1->updateVelocities(t,if1);
    //bd2->updateVelocities(t,if2);
    bd1->updateAngularVelocities(t,if1);
    bd2->updateAngularVelocities(t,if2);
  }

  void Constraint1::updateJacobians(double t){
    //int nu1 = frame1->getJacobianOfTranslation().cols();
    //int nu2 = frame2->getJacobianOfTranslation().cols();
    int nu1 = bd1->getWJTrel().cols();
    int nu2 = bd2->getWJTrel().cols();
    Mat J12t(3,nu1+nu2);
    //J12t(Index(0,2),Index(0,nu1-1)) = frame1->getJacobianOfTranslation();
    //J12t(Index(0,2),Index(nu1,nu1+nu2-1)) = frame2->getJacobianOfTranslation();
    J12t(Index(0,2),Index(0,nu1-1)) = bd1->getWJTrel();
    J12t(Index(0,2),Index(nu1,nu1+nu2-1)) = bd2->getWJTrel();

    bd1->updateAcclerations(t,if1);
    bd2->updateAcclerations(t,if2);

    Mat J12 = slvLU(SqrMat(d.T()*J12t),-d.T()*(frame1->getJacobianOfTranslation()-frame2->getJacobianOfTranslation())); 
    Vec j12 = slvLU(SqrMat(d.T()*J12t),-d.T()*(frame1->getGyroscopicAccelerationOfTranslation()-frame2->getGyroscopicAccelerationOfTranslation()));

    int nh1 = bd1->getJRel().cols();
    int nh2 = bd2->getJRel().cols();
    bd1->getJRel() = J12(Index(0,nu1-1),Index(0,nh1-1)); //(Index(0,nu1-1),Index(0,nh1-1))
    bd2->getJRel() = J12(Index(nu1,nu1+nu2-1),Index(0,nh2-1));
    bd1->getjRel() = j12(Index(0,nu1-1)); // (Index(0,nu1-1))
    bd2->getjRel() = j12(Index(nu1,nu1+nu2-1));

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
    Vec x0(nq1+nq2);
    if(t>0) {
    x0(0,nq1-1) = bd1->getqRel();
    x0(nq1,nq1+nq2-1) = bd2->getqRel();
    }
    else {
    x0(0,nq1-1).init(0.52);
    x0(nq1,nq1+nq2-1).init(-2.2);
    }
    Vec q12 = newton.solve(x0);
    bd1->getqRel() = q12(0,nq1-1);
    bd2->getqRel() = q12(nq1,nq1+nq2-1);

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
    //bd1->updateVelocities(t,if1);
    //bd2->updateVelocities(t,if2);
    bd1->updateAngularVelocities(t,if1);
    bd2->updateAngularVelocities(t,if2);
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

}
