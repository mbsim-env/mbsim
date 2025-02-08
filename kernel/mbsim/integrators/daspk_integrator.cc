/* Copyright (C) 2004-2018  Martin Förg
 
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
#include "fortran/fortran_wrapper.h"
#include "daspk_integrator.h"
#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, DASPKIntegrator)

  DASPKIntegrator::Delta DASPKIntegrator::delt[5];
  DASPKIntegrator::Jac DASPKIntegrator::jac[5];

  // This code is taken from ddaspk.f
  double DASPKIntegrator::delta(int i, double z) const {
    double delt = epsroot*max(max(abs(z),abs(h*yd[i])),abs(1.0/work(lewt+i)));
    delt = h*yd[i]>0?delt:-delt;
    return (z+delt)-z;
  }

  void DASPKIntegrator::deltaODE(double* t, double* z_, double* zd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec zd(ipar[0], zd_);
      Vec delta(ipar[0], delta_);
      self->system->setTime(*t);
      self->system->resetUpToDate();
      delta = self->system->evalzd() - zd;
    }
    catch(...) { // if a exception is thrown catch and store it in self
      *ires = -2;
      self->exception = current_exception();
    }
  }

  void DASPKIntegrator::deltaDAE1(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec yd(ipar[0], yd_);
      Vec delta(ipar[0], delta_);
      self->system->setTime(*t);
      self->system->resetUpToDate();
      self->system->setUpdatela(false);
      delta.set(self->Rz, self->system->evalzd() - yd(self->Rz));
      delta.set(self->Rla, self->system->evalW().T()*yd(self->Ru) + self->system->evalwb());
    }
    catch(...) { // if a exception is thrown catch and store it in self
      *ires = -2;
      self->exception = current_exception();
    }
  }

  void DASPKIntegrator::deltaDAE2(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec yd(ipar[0], yd_);
      Vec delta(ipar[0], delta_);
      self->system->setTime(*t);
      self->system->resetUpToDate();
      self->system->setUpdatela(false);
      delta.set(self->Rz, self->system->evalzd() - yd(self->Rz));
      delta.set(self->Rla, self->system->evalgd());
    }
    catch(...) { // if a exception is thrown catch and store it in self
      *ires = -2;
      self->exception = current_exception();
    }
  }

  void DASPKIntegrator::deltaGGL(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(ipar[0], y_);
      Vec yd(ipar[0], yd_);
      Vec delta(ipar[0], delta_);
      self->system->setTime(*t);
      self->system->resetUpToDate();
      self->system->setUpdatela(false);
      delta.set(self->Rz, self->system->evalzd() - yd(self->Rz));
      delta.set(self->Rla, self->system->evalgd());
      delta.set(self->Rl, self->system->evalg());
      if(self->system->getgSize() != self->system->getgdSize()) {
        self->system->calclaSize(5);
        self->system->updateWRef(self->system->getWParent(0));
        self->system->setUpdateW(true);
        delta.add(self->Rq, self->system->evalW()*y(self->Rl));
        self->system->calclaSize(3);
        self->system->updateWRef(self->system->getWParent(0));
      }
      else
        delta.add(self->Rq, self->system->evalW()*y(self->Rl));
    }
    catch(...) { // if a exception is thrown catch and store it in self
      *ires = -2;
      self->exception = current_exception();
    }
  }

  void DASPKIntegrator::jacODE(double* t, double* z_, double* zd_, double* J_, double* cj, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Mat J(ipar[0], ipar[0], J_); // fmatvec variant of J_

      self->h = J_[0]; // current step size
      self->yd = zd_; // current derivatives of the solution components

      self->zd0 = self->system->getzd();
      auto T = self->system->evalT();

      if(self->system->getqdequ()) {
        setZero(J,self->Rq,self->Rq); // par_qd_par_q
        self->par_ud_xd_par_q(J);
      }
      else
        self->par_zd_par_q(J);
      J.set(self->Rq, self->Ru, T); // par_qd_par_u
      setZero(J,self->Rq,self->Rx); // par_qd_par_x

      self->par_ud_xd_par_u_x(J,true);
      for(int i=0; i<self->system->getzSize(); i++)
        J(i,i) -= *cj;
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void DASPKIntegrator::jacDAE1(double* t, double* y_, double* yd_, double* J_, double* cj, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec yd(ipar[0], yd_); // fmatvec variant of y_
      Mat J(ipar[0], ipar[0], J_); // fmatvec variant of J_

      self->h = J_[0]; // current step size
      self->yd = yd_; // current derivatives of the solution components

      self->zd0 = self->system->getzd();
      auto T = self->system->evalT();
      auto Jrla = self->system->evalJrla();
      auto LLM = self->system->evalLLM();
      auto W = self->system->evalW();
      self->gd0 = W.T()*yd(self->Ru) + self->system->evalwb(); // we use gd0 for gdd0 here

      if(self->system->getqdequ()) {
        setZero(J,self->Rq,self->Rq); // par_qd_par_q
        self->par_ud_xd_gdd_par_q_u(J,yd(self->Ru));
      }
      else
        self->par_zd_gdd_par_q_u(J,yd(self->Ru));
      J.set(self->Rq, self->Ru, T); // par_qd_par_u
      setZero(J,self->Rq,RangeV(self->Rx.start(),self->Rla.end())); // par_qd_par_x_la

      self->par_ud_xd_par_x(J);
      setZero(J,self->Rx,self->Rla); // par_xd_par_la
      setZero(J,self->Rla,self->Rx); // par_gdd_par_x

      Mat Minv_Jrla = slvLLFac(LLM, Jrla);
      J.set(self->Ru, self->Rla, Minv_Jrla); // par_ud_par_la
      for(int i=0; i<self->system->getzSize(); i++)
        J(i,i) -= *cj;
      J.add(self->Rla, self->Ru, *cj*W.T());
      setZero(J,self->Rla,self->Rla); // par_gdd_par_la
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void DASPKIntegrator::jacDAE2(double* t, double* y_, double* yd_, double* J_, double* cj, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec yd(ipar[0], yd_); // fmatvec variant of yd_
      Mat J(ipar[0], ipar[0], J_); // fmatvec variant of J_

      self->h = J_[0]; // current step size
      self->yd = yd_; // current derivatives of the solution components

      self->zd0 = self->system->getzd();
      self->gd0 = self->system->getgd();
      auto T = self->system->evalT();
      auto Jrla = self->system->evalJrla();
      auto LLM = self->system->evalLLM();
      auto W = self->system->evalW();

      if(self->system->getqdequ()) {
        setZero(J,self->Rq,self->Rq); // par_qd_par_q
        self->par_ud_xd_gd_par_q(J);
      }
      else
        self->par_zd_gd_par_q(J);
      J.set(self->Rq, self->Ru, T); // par_qd_par_u
      setZero(J,self->Rq,RangeV(self->Rx.start(),self->Rla.end())); // par_qd_par_x_la
      self->par_ud_xd_par_u_x(J,false);
      setZero(J,self->Rx,self->Rla); // par_xd_par_la
      J.set(self->Rla, self->Ru, W.T()); // par_gd_par_u
      setZero(J,self->Rla,RangeV(self->Rx.start(),self->Rla.end())); // par_gd_par_x_la

      Mat Minv_Jrla = slvLLFac(LLM, Jrla);
      J.set(self->Ru, self->Rla, Minv_Jrla); // par_ud_par_la
      for(int i=0; i<self->system->getzSize(); i++)
        J(i,i) -= *cj;
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void DASPKIntegrator::jacGGL(double* t, double* y_, double* yd_, double* J_, double* cj, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(ipar[0], y_); // fmatvec variant of y_
      Mat J(ipar[0], ipar[0], J_); // fmatvec variant of J_

      self->h = J_[0]; // current step size
      self->yd = yd_; // current derivatives of the solution components

      self->zd0 = self->system->getzd();
      self->gd0 = self->system->getgd();
      self->g0 = self->system->getg();
      auto T = self->system->evalT();
      auto Jrla = self->system->evalJrla();
      auto LLM = self->system->evalLLM();
      auto W = self->system->evalW();
      Mat W2;
      if(self->system->getgSize() != self->system->getgdSize()) {
        self->system->calclaSize(5);
        self->system->updateWRef(self->system->getWParent(0));
        self->system->setUpdateW(true);
        W2 <<= self->system->evalW();
        self->zd0.add(self->Rq, W2*y(self->Rl));
        self->system->calclaSize(3);
        self->system->updateWRef(self->system->getWParent(0));
      }
      else
        self->zd0.add(self->Rq, W*y(self->Rl));

      if(self->reduced)
        self->par_ud_xd_gd_g_par_q(J);
      else {
        if(self->system->getqdequ()) {
          setZero(J,self->Rq,self->Rq); // par_qd_par_q
          self->par_ud_xd_gd_g_par_q(J);
        }
        else
          self->par_zd_gd_g_par_q(J);
        J.set(self->Rq, self->Ru, T); // par_qd_par_u
        setZero(J,self->Rq,RangeV(self->Rx.start(),self->Rla.end())); // par_qd_par_x_la
        if(self->system->getgSize() != self->system->getgdSize())
          J.set(self->Rq, self->Rl, W2); // par_qd_par_l
        else
          J.set(self->Rq, self->Rl, W); // par_qd_par_l
      }
      self->par_ud_xd_par_u_x(J,false);
      setZero(J,self->Ru,self->Rl); // par_ud_par_l
      setZero(J,self->Rx,RangeV(self->Rla.start(),self->Rl.end())); // par_xd_par_la_l
      J.set(self->Rla, self->Ru, W.T()); // par_gd_par_u
      setZero(J,self->Rla,RangeV(self->Rx.start(),self->Rl.end())); // par_gd_par_x_la_l
      setZero(J,self->Rl,RangeV(self->Ru.start(),self->Rl.end())); // par_g_par_u_x_la_l
      for(int i=0; i<self->system->getzSize(); i++)
        J(i,i) -= *cj;

      Mat Minv_Jrla = slvLLFac(LLM, Jrla);
      J.set(self->Ru, self->Rla, Minv_Jrla); // par_ud_par_la
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void DASPKIntegrator::integrate() {
    if(formalism==unknown)
      throwError("(DASPKIntegrator::integrate): formalism unknown");

    delt[0] = &DASPKIntegrator::deltaODE;
    delt[1] = &DASPKIntegrator::deltaDAE1;
    delt[2] = &DASPKIntegrator::deltaDAE2;
    delt[3] = nullptr; // not available
    delt[4] = &DASPKIntegrator::deltaGGL;
    jac[0] = &DASPKIntegrator::jacODE;
    jac[1] = &DASPKIntegrator::jacDAE1;
    jac[2] = &DASPKIntegrator::jacDAE2;
    jac[3] = nullptr; // not available
    jac[4] = &DASPKIntegrator::jacGGL;

    debugInit();

    init();

    if(not neq)
      throwError("(DASPKIntegrator::integrate): dimension of the system must be at least 1");

    double t = tStart;
    double tPlot = t + dtPlot;

    // Enlarge workspace for state vector so that the integrator can use it (avoids copying of state vector)
    system->resizezParent(neq);
    system->updatezRef(system->getzParent());
    if(formalism) {
      system->getlaParent().ref(system->getzParent(), RangeV(system->getzSize(),system->getzSize()+system->getlaSize()-1));
      system->updatelaRef(system->getlaParent());
    }
    // Integrator uses its own workspace for the state derivative
    Vec yd(neq);

    if(z0.size()) {
      if(z0.size() != system->getzSize()+system->getisSize())
        throwError("(DASPKIntegrator::integrate): size of z0 does not match, must be " + to_string(system->getzSize()+system->getisSize()));
      system->setState(z0(RangeV(0,system->getzSize()-1)));
      system->setInternalState(z0(RangeV(system->getzSize(),z0.size()-1)));
    }
    else
      system->evalz0();

    if(aTol.size() == 0)
      aTol.resize(1,INIT,1e-6);
    if(rTol.size() == 0)
      rTol.resize(1,INIT,1e-6);

    info.resize(20);

    // info(0) = 0; first call

    if(aTol.size()>1) {
      info(1) = 1; // aTol und rTol are vectors
      if(aTol.size() != neq)
        throwError("(DASPKIntegrator::integrate): size of aTol does not match, must be " + to_string(neq));
    }
    if(rTol.size() != aTol.size())
      throwError("(DASPKIntegrator::integrate): size of rTol does not match aTol, must be " + to_string(aTol.size()));

    info(2) = 1; // solution only at tOut, no intermediate-output
    // info(3) = 0; // integration does not stop at tStop (rWork(0))
    info(4) = not numericalJacobian; // jacobian is computed
                            // - by finite differences if numericalJacobian is true
                            // - by a combination of finite differences and an analytical solution, otherwise
    // info(5) = 0; // jacobian is a full matrix
    info(6) = dtMax>0; // set maximum stepsize
    info(7) = dt0>0; // set initial stepsize
    // info(8) = 0; // maximum order
    // info(9) = 0; // no components are nonnegative
    // info(10) = 0; // initial t, y, yd are consistent
    // info(11) = 0; // direct method is used to solve the linear systems
    // info(12) = 0; // used for Krylov methods (info(11)==1)
    // info(13) = 0; // used, when an initial condition calculation is requested (info(10)>0)
    // info(14) = 0; // used for Krylov methods (info(11)==1)
    info(15) = excludeAlgebraicVariables; // exclude algebraic variables from the error test
    // info(16) = 0; // used, when an initial condition calculation is requested (info(10)>0)
    // info(17) = 0; // no extra printing in initial condition calculation

    exception=nullptr;

    double rPar[1]; // not used
    iPar.resize(1+sizeof(void*)/sizeof(int)+1);
    DASPKIntegrator *self=this;
    memcpy(&iPar(1), &self, sizeof(void*));

    int lWork = 50+9*neq+neq*neq+neq;
    int liWork = 40+neq+neq;
    iWork.resize(liWork);
    work.resize(lWork);

    int idid;

    system->setTime(t);
    system->resetUpToDate();
    system->computeInitialCondition();
    if(formalism>1) { // DAE2 or GGL
      system->calcgdSize(3); // IH
      system->updategdRef(system->getgdParent());
      if(formalism==GGL) { // GGL
        system->calcgSize(2); // IB
        system->updategRef(system->getgParent());
      }
    }
    system->plot();
    yd.set(RangeV(0,system->getzSize()-1), system->evalzd());
    svLast <<= system->evalsv();

    work(1) = dtMax; // maximum stepsize
    if(info(15)) {
      for(int i=0; i<system->getzSize(); i++)
        iWork(40+i) = 1; // differential variable
    }
    reinit();

    double s0 = clock();
    double time = 0;

    while(t<tEnd-epsroot) {
      DDASPK(*delt[formalism],&neq,&t,system->getState()(),yd(),&tEnd,info(),rTol(),aTol(),&idid,work(),&lWork,iWork(),&liWork,rPar,iPar(),jac[formalism],nullptr);
      if(exception)
        rethrow_exception(exception);
      if(idid==1) {
        double curTimeAndState = -1;
        double tRoot = t;

        // root-finding
        if(system->getsvSize()) {
          system->setTime(t);
          curTimeAndState = t;
          system->resetUpToDate();
          shift = signChangedWRTsvLast(system->evalsv());
          // if a root exists in the current step ...
          double dt = work(6);
          if(shift) {
            // ... search the first root and set step.second to this time
            while(dt>dtRoot) {
              dt/=2;
              double tCheck = tRoot-dt;
              curTimeAndState = tCheck;
              DDATRP(&t, &tCheck, system->getState()(), yd(), &neq, &iWork(7), &work(lphi), &work(38));
              system->setTime(tCheck);
              system->resetUpToDate();
              if(signChangedWRTsvLast(system->evalsv()))
                tRoot = tCheck;
            }
            if(curTimeAndState != tRoot) {
              curTimeAndState = tRoot;
              DDATRP(&t, &tRoot, system->getState()(), yd(), &neq, &iWork(7), &work(lphi), &work(38));
              system->setTime(tRoot);
            }
            system->resetUpToDate();
            auto &sv = system->evalsv();
            auto &jsv = system->getjsv();
            for(int i=0; i<sv.size(); ++i)
              jsv(i)=svLast(i)*sv(i)<0;
          }
        }
        while(tRoot>=tPlot and tPlot<=tEnd+epsroot) {
          if(curTimeAndState != tPlot) {
            curTimeAndState = tPlot;
            DDATRP(&t, &tPlot, system->getState()(), yd(), &neq, &iWork(7), &work(lphi), &work(38));
            system->setTime(tPlot);
          }
          system->resetUpToDate();
          system->setzd(yd(RangeV(0,system->getzSize()-1)));
          system->setUpdatezd(false);
          if(formalism) system->setUpdatela(false);
          system->plot();
          if(msgAct(Status))
            msg(Status) << "   t = " <<  tPlot << ",\tdt = "<< work(6) << flush;

          system->updateInternalState();

          double s1 = clock();
          time += (s1-s0)/CLOCKS_PER_SEC;
          s0 = s1;

          tPlot += dtPlot;
        }

        if(shift) {
          // shift the system
          if(curTimeAndState != tRoot) {
            DDATRP(&t, &tRoot, system->getState()(), yd(), &neq, &iWork(7), &work(lphi), &work(38));
            system->setTime(tRoot);
          }
          if(plotOnRoot) {
            system->resetUpToDate();
//            system->setzd(yd(0,system->getzSize()-1));
//            system->setUpdatezd(false);
//            if(formalism) system->setUpdatela(false);
            system->plot();
//            system->plotAtSpecialEvent();
          }
          system->resetUpToDate();
          system->shift();
          if(formalism>1) { // DAE2 or GGL
            system->calcgdSize(3); // IH
            system->updategdRef(system->getgdParent());
            if(formalism==GGL) { // GGL
              system->calcgSize(2); // IB
              system->updategRef(system->getgParent());
            }
          }
          if(plotOnRoot) {
            system->resetUpToDate();
            system->plot();
          }
          info(0) = 0;
        }
        else {
          // check drift
          bool projVel = true;
          if(gMax>=0) {
            system->setTime(t);
            system->resetUpToDate();
            if(system->positionDriftCompensationNeeded(gMax)) { // project both, first positions and then velocities
              system->projectGeneralizedPositions(3);
              system->projectGeneralizedVelocities(3);
              projVel = false;
              info(0) = 0;
            }
          }
          if(gdMax>=0 and projVel) {
            system->setTime(t);
            system->resetUpToDate();
            if(system->velocityDriftCompensationNeeded(gdMax)) { // project velicities
              system->projectGeneralizedVelocities(3);
              info(0) = 0;
            }
          }
          system->updateStopVectorParameters();
        }
        if(info(0)==0) {
          t = system->getTime();
          system->resetUpToDate();
          yd.set(RangeV(0,system->getzSize()-1), system->evalzd());
          if(shift) {
            svLast = system->evalsv();
            reinit();
          }
        }
        system->updateInternalState();
      }
      else if(idid<0) throwError("Integrator DASPK failed with istate = "+to_string(idid));
    }

    msg(Info)<<string("nrRHS")+(numericalJacobian?" (including jac): ":" (excluding jac): ")<<iWork(11)<<endl;
    msg(Info)<<"nrJac: "<<iWork(12)<<endl;
    msg(Info)<<"nrSteps: "<<iWork(10)<<endl;
    msg(Info)<<"nrStepsAccepted: "<<iWork(10)-iWork(13)<<endl;
    msg(Info)<<"nrStepsRejected: "<<iWork(13)<<endl;
    msg(Info)<<"nrNonlinConvFailures: "<<iWork(14)<<endl;
    msg(Info)<<"nrLinConvFailures: "<<iWork(15)<<endl;
  }

  void DASPKIntegrator::reinit() {
    DAEIntegrator::reinit();

    lphi = excludeAlgebraicVariables?50+4*neq:50+3*neq;
    iPar(0) = neq;
    work(2) = dt0;
    if(info(15)) {
      for(int i=system->getzSize(); i<neq; i++)
        iWork(40+i) = -1; // algebraic variable
    }
    lewt = 50+2*neq;

    gd0.resize(system->getgdSize(),NONINIT);
    if(formalism==GGL)
      g0.resize(system->getgSize(),NONINIT);
  }

  void DASPKIntegrator::initializeUsingXML(DOMElement *element) {
    DAEIntegrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"absoluteTolerance");
    if(e) setAbsoluteTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"absoluteToleranceScalar");
    if(e) setAbsoluteTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"relativeTolerance");
    if(e) setRelativeTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"relativeToleranceScalar");
    if(e) setRelativeTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"initialStepSize");
    if(e) setInitialStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"maximumStepSize");
    if(e) setMaximumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"formalism");
    if(e) {
      string formalismStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(formalismStr=="ODE") formalism=ODE;
      else if(formalismStr=="DAE1") formalism=DAE1;
      else if(formalismStr=="DAE2") formalism=DAE2;
      else if(formalismStr=="GGL") formalism=GGL;
      else formalism=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"excludeAlgebraicVariablesFromErrorTest");
    if(e) setExcludeAlgebraicVariablesFromErrorTest(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"numericalJacobian");
    if(e) setNumericalJacobian(E(e)->getText<bool>());
  }

}
