/* Copyright (C) 2004-2006  Martin Förg
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */
#include <config.h>
#include "body_rigid_constrained.h"
#include "multi_body_system.h"
#include "userfunction.h"
#include "link.h"
#include "eps.h"

namespace MBSim {

  BodyRigidConstrainedAcc::BodyRigidConstrainedAcc(const string &name) : BodyRigidAbs(name), pos(0), vel(0), acc(0) {
    delta = epsroot();
    sqrtdelta = sqrt(delta);
    m = 1.0;	//Default Mass values; Otherwise BodyRigidAbs::init() : inv(M) not defined!
    I(0,0) = 1.0;
    I(1,1) = 1.0;
    I(2,2) = 1.0; 
  }

  void BodyRigidConstrainedAcc::init() {
    BodyRigidAbs::init();
    if(pos)
      assert((*pos)(0).size() == qSize);
    if(vel)
      assert((*vel)(0).size() == uSize);
    if(acc)
      assert((*acc)(0).size() == uSize);
  }

  void BodyRigidConstrainedAcc::updatezd(double t) {
    (this->*updateT)();
    qd = T*u;
    ud = h;
  }

  void BodyRigidConstrainedAcc::updatedu(double t, double dt) {
    ud = h*dt;
  }

  void BodyRigidConstrainedAcc::updatedq(double t, double dt) {
    qd = T*u*dt;
  }

  void BodyRigidConstrainedAcc::updater(double t) {
  }

  void BodyRigidConstrainedAcc::updateh(double t) {
    if(pos)
      h = ((*pos)(t+sqrtdelta)+(*pos)(t-sqrtdelta)-2*(*pos)(t))/(sqrtdelta*sqrtdelta);
    else if(vel)
      h = ((*vel)(t+delta)-((*vel)(t-delta)))/(2*delta);
    else
      h = (*acc)(t);
  }

  void BodyRigidConstrainedAcc::setPosition(DataInterfaceBase *func_) {
    pos = func_;
  }

  void BodyRigidConstrainedAcc::setVelocity(DataInterfaceBase *func_) {
    vel = func_;
  }

  void BodyRigidConstrainedAcc::setAcceleration(DataInterfaceBase *func_) {
    acc = func_;
  }

  void BodyRigidConstrainedAcc::updateG(double t) {
    BodyRigidAbs::updateW(t);
    vector<Mat>::iterator itW=W.begin(),jtW; 
    vector<Vec>::iterator itw=w.begin(); 
    vector<LinkPortData>::iterator jt1,it1=linkSetValuedPortData.begin(); 
    vector<LinkContourData>::iterator jt2,it2=linkSetValuedContourData.begin(); 
    for(int i=0; i<linkSetValuedPortData.size(); i++) {
      Index I = it1->link->getlaIndex();
      Mat Wi = (*itW);
      mbs->getw()(I) += (*itw); 
      mbs->getb()(I) += trans(Wi)*h; 
      it1++; itW++; itw++;
    }
    for(int i=linkSetValuedPortData.size(); i<linkSetValuedContourData.size() + linkSetValuedPortData.size(); i++) {
      if(it2->link->isActive()) {
	Index I = it2->link->getlaIndex();
	Mat Wi = (*itW);
	mbs->getw()(I) += (*itw); 
	mbs->getb()(I) += trans(Wi)*h; 
      }
      it2++; itW++; itw++;
    }
  }

  BodyRigidConstrainedVel::BodyRigidConstrainedVel(const string &name) : BodyRigidAbs(name), pos(0), vel(0) {
    delta = epsroot();
    sqrtdelta = sqrt(delta);
    m = 1.0;	//Default Mass values; Otherwise BodyRigidAbs::init() : inv(M) not defined!
    I(0,0) = 1.0;
    I(1,1) = 1.0;
    I(2,2) = 1.0; 
  }

  void BodyRigidConstrainedVel::init() {
    BodyRigidAbs::init();
    uSize = 0;
    ud.resize(uSize);
    //for(int i=0; i<JT.cols(); i++)
    //JT.col(i) /= nrm2(JT.col(i));
    //for(int i=0; i<JR.cols(); i++)
    //JR.col(i) /= nrm2(JR.col(i));
    if(pos)
      assert((*pos)(0).size() == qSize);
    if(vel)
      assert((*vel)(0).size() == JT.cols()+JR.cols());
  }

  void BodyRigidConstrainedVel::setPosition(DataInterfaceBase *func_) {
    pos = func_;
  }

  void BodyRigidConstrainedVel::setVelocity(DataInterfaceBase *func_) {
    vel = func_;
  }

  void BodyRigidConstrainedVel::updater(double t) {
  }

  void BodyRigidConstrainedVel::updateh(double t) {
    if(pos)
      h = ((*pos)(t+sqrtdelta)+(*pos)(t-sqrtdelta)-2*(*pos)(t))/(sqrtdelta*sqrtdelta);
    else 
      h = ((*vel)(t+delta)-((*vel)(t-delta)))/(2*delta);
  }

  void BodyRigidConstrainedVel::updateKinematics(double t) {
    if(pos)
      u = ((*pos)(t+delta)-((*pos)(t-delta)))/(2*delta);
    else
      u = (*vel)(t);
    BodyRigidAbs::updateKinematics(t);
  }

  void BodyRigidConstrainedVel::updatezd(double t) {
    (this->*updateT)();
    qd = T*u;
    //  ud = h;
  }

  void BodyRigidConstrainedVel::updatedu(double t, double dt) {
    //  ud = h*dt;
  }

  void BodyRigidConstrainedVel::updatedq(double t, double dt) {
    qd = T*u*dt;
  }

  void BodyRigidConstrainedVel::updateG(double t) {
    vector<Mat>::iterator itW=W.begin(),jtW; 
    vector<Vec>::iterator itw=w.begin(); 
    vector<LinkPortData>::iterator jt1,it1=linkSetValuedPortData.begin(); 
    vector<LinkContourData>::iterator jt2,it2=linkSetValuedContourData.begin(); 
    for(int i=0; i<linkSetValuedPortData.size(); i++) {
      Index I = it1->link->getlaIndex();
      Mat Wi = (*itW);
      it1++; itW++; itw++;
    }
    for(int i=linkSetValuedPortData.size(); i<linkSetValuedContourData.size() + linkSetValuedPortData.size(); i++) {
      if(it2->link->isActive()) {
	Index I = it2->link->getlaIndex();
	Mat Wi = (*itW);
      }
      it2++; itW++; itw++;
    }
  }

  void BodyRigidConstrainedVel::plot(double t, double dt) {
    BodyRigidAbs::plot(t, dt);
  }

  void BodyRigidConstrainedVel::initPlotFiles() {

    BodyRigidAbs::initPlotFiles();
  }

}
