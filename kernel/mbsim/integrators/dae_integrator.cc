/* Copyright (C) 2004-2025  Martin Förg
 
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
 *   martin.o.foerg@googlemail.com
 *
 */

#include <config.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/utils/eps.h>
#include "dae_integrator.h"

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;

namespace MBSim {

  // DAE1
  void DAEIntegrator::par_ud_xd_gdd_par_q_u(Mat &J, const Vec &ud_) {
    const Vec &ud = ud_()?ud_:system->getud(false);
    for(int j=0; j<system->getqSize()+system->getuSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updateud();
      system->updatexd();
      const Vec &gdd1 = system->evalW().T()*ud + system->evalwb();
      for(int i=RuMove.start(),k=0; i<=RuMove.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=RxMove.start(),k=0; i<=RxMove.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      for(int i=RlaMove.start(),k=0; i<=RlaMove.end(); i++,k++)
        J(i,j)=(gdd1(k)-gd0(k))/delta; // we use gd0 for gdd0 here
      system->getState()(j)=zSave;
    }
  }

  // DAE1
  void DAEIntegrator::par_zd_gdd_par_q_u(Mat &J, const Vec &ud_) {
    const Vec &ud = ud_()?ud_:system->getud(false);
    for(int j=0; j<system->getqSize()+system->getuSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updatezd();
      const Vec &gdd1 = system->evalW().T()*ud + system->evalwb();
      for(int i=Rq.start(); i<=Rq.end(); i++)
        J(i,j)=(system->getqd(false)(i)-qd0(i))/delta;
      for(int i=Ru.start(),k=0; i<=Ru.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=Rx.start(),k=0; i<=Rx.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      for(int i=Rla.start(),k=0; i<=Rla.end(); i++,k++)
        J(i,j)=(gdd1(k)-gd0(k))/delta; // we use gd0 for gdd0 here
      system->getState()(j)=zSave;
    }
  }

  // DAE1
  void DAEIntegrator::par_ud_xd_par_x(Mat &J) {
    for(int j=system->getqSize()+system->getuSize(); j<system->getzSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updateud();
      system->updatexd();
      for(int i=RuMove.start(),k=0; i<=RuMove.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=RxMove.start(),k=0; i<=RxMove.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      system->getState()(j)=zSave;
    }
  }

  // DAE2
  void DAEIntegrator::par_ud_xd_gd_par_q(Mat &J) {
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updateud();
      system->updatexd();
      system->updategd();
      for(int i=RuMove.start(),k=0; i<=RuMove.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=RxMove.start(),k=0; i<=RxMove.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      for(int i=RlaMove.start(),k=0; i<=RlaMove.end(); i++,k++)
        J(i,j)=(system->getgd(false)(k)-gd0(k))/delta;
      system->getState()(j)=zSave;
    }
  }

  // DAE2
  void DAEIntegrator::par_zd_gd_par_q(Mat &J) {
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updatezd();
      system->updategd();
      for(int i=Rq.start(); i<=Rq.end(); i++)
        J(i,j)=(system->getqd(false)(i)-qd0(i))/delta;
      for(int i=Ru.start(),k=0; i<=Ru.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=Rx.start(),k=0; i<=Rx.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      for(int i=Rla.start(),k=0; i<=Rla.end(); i++,k++)
        J(i,j)=(system->getgd(false)(k)-gd0(k))/delta;
      system->getState()(j)=zSave;
    }
  }

  // DAE3
  void DAEIntegrator::par_ud_xd_g_par_q(Mat &J) {
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updateud();
      system->updatexd();
      system->updateg();
      for(int i=RuMove.start(),k=0; i<=RuMove.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=RxMove.start(),k=0; i<=RxMove.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      for(int i=RlaMove.start(),k=0; i<=RlaMove.end(); i++,k++)
        J(i,j)=(system->getg(false)(k)-gd0(k))/delta;
      system->getState()(j)=zSave;
    }
  }

  // DAE3
  void DAEIntegrator::par_zd_g_par_q(Mat &J) {
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updatezd();
      system->updateg();
      for(int i=Rq.start(); i<=Rq.end(); i++)
        J(i,j)=(system->getqd(false)(i)-qd0(i))/delta;
      for(int i=Ru.start(),k=0; i<=Ru.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=Rx.start(),k=0; i<=Rx.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      for(int i=Rla.start(),k=0; i<=Rla.end(); i++,k++)
        J(i,j)=(system->getg(false)(k)-gd0(k))/delta; // we use gd0 for g0 here
      system->getState()(j)=zSave;
    }
  }

  // GGL
  void DAEIntegrator::par_ud_xd_gd_g_par_q(Mat &J) {
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updateud();
      system->updatexd();
      system->updategd();
      system->updateg();
      for(int i=Ru.start(),k=0; i<=Ru.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=Rx.start(),k=0; i<=Rx.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      for(int i=Rla.start(),k=0; i<=Rla.end(); i++,k++)
        J(i,j)=(system->getgd(false)(k)-gd0(k))/delta;
      for(int i=Rl.start(),k=0; i<=Rl.end(); i++,k++)
        J(i,j)=(system->getg(false)(k)-g0(k))/delta;
      system->getState()(j)=zSave;
    }
  }

  // GGL
  void DAEIntegrator::par_zd_gd_g_par_q(Mat &J) {
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delta=sqrt(macheps*max(1.e-5,abs(zSave)));
      system->getState()(j)=zSave+delta;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updatezd();
      system->updategd();
      system->updateg();
      for(int i=Rq.start(); i<=Rq.end(); i++)
        J(i,j)=(system->getqd(false)(i)-qd0(i))/delta;
      for(int i=Ru.start(),k=0; i<=Ru.end(); i++,k++)
        J(i,j)=(system->getud(false)(k)-ud0(k))/delta;
      for(int i=Rx.start(),k=0; i<=Rx.end(); i++,k++)
        J(i,j)=(system->getxd(false)(k)-xd0(k))/delta;
      for(int i=Rla.start(),k=0; i<=Rla.end(); i++,k++)
        J(i,j)=(system->getgd(false)(k)-gd0(k))/delta;
      for(int i=Rl.start(),k=0; i<=Rl.end(); i++,k++)
        J(i,j)=(system->getg(false)(k)-g0(k))/delta;
      system->getState()(j)=zSave;
    }
  }

  void DAEIntegrator::calcSize() {
    if(formalism==DAE1 or formalism==DAE2)
      neq = system->getzSize()+system->getlaSize();
    else if(formalism==DAE3)
      neq = system->getzSize()+system->getgSize();
    else if(formalism==GGL)
      neq = system->getzSize()+system->getgdSize()+system->getgSize();
    else
      neq = system->getzSize();
  }

  void DAEIntegrator::reinit() {
    calcSize();
    res0.resize(neq,NONINIT);
    qd0.ref(res0,Rq);
    ud0.ref(res0,Ru);
    xd0.ref(res0,Rx);
    if(formalism>0) {
      Rla = RangeV(system->getzSize(), system->getzSize()+system->getlaSize()-1);
      RlaMove = RangeV(Rla.start()-rowMove, Rla.end()-rowMove);
      gd0.ref(res0,Rla);
    }
    if(formalism==GGL) {
      Rl = RangeV(system->getzSize()+system->getlaSize(), neq-1);
      g0.ref(res0,Rl);
    }
  }

}
