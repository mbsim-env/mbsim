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
#include "radau_integrator.h"
#include <ctime>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, RADAUIntegrator)

  RADAUIntegrator::Fzdot RADAUIntegrator::fzdot[5];
  RADAUIntegrator::Jac RADAUIntegrator::jac[5];
  RADAUIntegrator::Mass RADAUIntegrator::mass[2];

  // This code is taken from radau.f
  double RADAUIntegrator::delta(int i, double z) const {
    return sqrt(1e-16*max(1.e-5,abs(z)));
  }

  void RADAUIntegrator::fzdotODE(int* zSize, double* t, double* z_, double* zd_, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec zd(*zSize, zd_);
      self->system->setTime(*t);
      self->system->setState(Vec(*zSize, z_));
      self->system->resetUpToDate();
      zd = self->system->evalzd();
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::fzdotDAE1(int* neq, double* t, double* y_, double* yd_, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(*neq, y_);
      Vec yd(*neq, yd_);
      self->system->setTime(*t);
      self->system->setState(y(self->Rz));
      self->system->resetUpToDate();
      self->system->setla(y(self->Rla));
      self->system->setUpdatela(false);
      yd.set(self->Rz, self->system->evalzd());
      yd.set(self->Rla, self->system->evalW().T()*yd(self->Ru) + self->system->evalwb());
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::fzdotDAE2(int* neq, double* t, double* y_, double* yd_, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(*neq, y_);
      Vec yd(*neq, yd_);
      self->system->setTime(*t);
      self->system->setState(y(self->Rz));
      self->system->resetUpToDate();
      self->system->setla(y(self->Rla));
      self->system->setUpdatela(false);
      yd.set(self->Rz, self->system->evalzd());
      yd.set(self->Rla, self->system->evalgd());
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::fzdotDAE3(int* neq, double* t, double* y_, double* yd_, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(*neq, y_);
      Vec yd(*neq, yd_);
      self->system->setTime(*t);
      self->system->setState(y(self->Rz));
      self->system->resetUpToDate();
      self->system->setla(y(self->Rla));
      self->system->setUpdatela(false);
      yd.set(self->Rz, self->system->evalzd());
      yd.set(self->Rla, self->system->evalg());
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::fzdotGGL(int* neq, double* t, double* y_, double* yd_, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(*neq, y_);
      Vec yd(*neq, yd_);
      self->system->setTime(*t);
      self->system->setState(y(self->Rz));
      self->system->resetUpToDate();
      self->system->setla(y(self->Rla));
      self->system->setUpdatela(false);
      yd.set(self->Rz, self->system->evalzd());
      yd.set(self->Rla, self->system->evalgd());
      yd.set(self->Rl, self->system->evalg());
      if(self->system->getgSize() != self->system->getgdSize()) {
        self->system->calclaSize(5);
        self->system->updateWRef(self->system->getWParent(0));
        self->system->setUpdateW(true);
        yd.add(self->Rq, self->system->evalW()*y(self->Rl));
        self->system->calclaSize(3);
        self->system->updateWRef(self->system->getWParent(0));
      }
      else
        yd.add(self->Rq, self->system->evalW()*y(self->Rl));
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::jacODE(int* cols, double *t, double *z_, double *J_, int *rows, double *rpar, int *ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Mat J(*rows, *cols, J_); // fmatvec variant of J_

      self->system->setTime(*t);
      self->system->setState(Vec(*cols, z_));
      self->system->resetUpToDate();
      auto T = self->system->evalT();

      if(self->reduced)
        self->par_ud_xd_par_q(J);
      else {
        if(self->system->getqdequ()) {
          setZero(J,self->Rq,self->Rq); // par_qd_par_q
          self->par_ud_xd_par_q(J);
        }
        else
          self->par_zd_par_q(J);
        J.set(self->Rq, self->Ru, T); // par_qd_par_u
        setZero(J,self->Rq,self->Rx); // par_qd_par_x
      }
      self->par_ud_xd_par_u_x(J,true);
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::jacDAE1(int* cols, double *t, double *y_, double *J_, int *rows, double *rpar, int *ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(*cols, y_); // fmatvec variant of y_
      Mat J(*rows, *cols, J_); // fmatvec variant of J_

      self->system->setTime(*t);
      self->system->setState(y(self->Rz));
      self->system->setla(y(self->Rla));
      self->system->resetUpToDate();
      self->system->setUpdatela(false);
      auto T = self->system->evalT();
      auto Jrla = self->system->evalJrla();
      auto LLM = self->system->evalLLM();
      auto W = self->system->evalW();

      if(self->reduced)
        self->par_ud_xd_gdd_par_q_u(J);
      else {
        if(self->system->getqdequ()) {
          setZero(J,self->Rq,self->Rq); // par_qd_par_q
          self->par_ud_xd_gdd_par_q_u(J);
        }
        else
          self->par_zd_gdd_par_q_u(J);
        J.set(self->Rq, self->Ru, T); // par_qd_par_u
        setZero(J,self->Rq,RangeV(self->Rx.start(),self->Rla.end())); // par_qd_par_x_la
      }
      self->par_ud_xd_par_x(J);
      setZero(J,self->RxMove,self->Rla); // par_xd_par_la
      setZero(J,self->RlaMove,self->Rx); // par_gdd_par_x

      Mat Minv_Jrla = slvLLFac(LLM, Jrla);
      J.set(self->RuMove, self->Rla, Minv_Jrla); // par_ud_par_la
      J.set(self->RlaMove, self->Rla, W.T()*Minv_Jrla); // par_gdd_par_la
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::jacDAE2(int* cols, double *t, double *y_, double *J_, int *rows, double *rpar, int *ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(*cols, y_); // fmatvec variant of y_
      Mat J(*rows, *cols, J_); // fmatvec variant of J_

      self->system->setTime(*t);
      self->system->setState(y(self->Rz));
      self->system->setla(y(self->Rla));
      self->system->resetUpToDate();
      self->system->setUpdatela(false);
      auto T = self->system->evalT();
      auto Jrla = self->system->evalJrla();
      auto LLM = self->system->evalLLM();
      auto W = self->system->evalW();

      if(self->reduced)
        self->par_ud_xd_gd_par_q(J);
      else {
        if(self->system->getqdequ()) {
          setZero(J,self->Rq,self->Rq); // par_qd_par_q
          self->par_ud_xd_gd_par_q(J);
        }
        else
          self->par_zd_gd_par_q(J);
        J.set(self->Rq, self->Ru, T); // par_qd_par_u
        setZero(J,self->Rq,RangeV(self->Rx.start(),self->Rla.end())); // par_qd_par_x_la
      }
      self->par_ud_xd_par_u_x(J,false);
      setZero(J,self->RxMove,self->Rla); // par_xd_par_la
      J.set(self->RlaMove, self->Ru, W.T()); // par_gd_par_u
      setZero(J,self->RlaMove,RangeV(self->Rx.start(),self->Rla.end())); // par_gd_par_x_la

      Mat Minv_Jrla = slvLLFac(LLM, Jrla);
      J.set(self->RuMove, self->Rla, Minv_Jrla); // par_ud_par_la
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::jacDAE3(int* cols, double *t, double *y_, double *J_, int *rows, double *rpar, int *ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(*cols, y_); // fmatvec variant of y_
      Mat J(*rows, *cols, J_); // fmatvec variant of J_

      // the undisturbed call -> this sets the system resetUpToDate
      // res0 is later used for the numerical part of the jacobian
      self->system->setTime(*t);
      self->system->setState(y(self->Rz));
      self->system->setla(y(self->Rla));
      self->system->resetUpToDate();
      self->system->setUpdatela(false);
      auto T = self->system->evalT();
      auto Jrla = self->system->evalJrla();
      auto LLM = self->system->evalLLM();

      if(self->reduced)
        self->par_ud_xd_g_par_q(J);
      else {
        if(self->system->getqdequ()) {
          setZero(J,self->Rq,self->Rq); // par_qd_par_q
          self->par_ud_xd_g_par_q(J);
        }
        else
          self->par_zd_g_par_q(J);
        J.set(self->Rq, self->Ru, T); // / par_qd_par_u
        setZero(J,self->Rq,RangeV(self->Rx.start(),self->Rla.end())); // par_qd_par_x_la
      }
      self->par_ud_xd_par_u_x(J,false);
      setZero(J,self->RxMove,self->Rla); // par_xd_par_la
      setZero(J,self->RlaMove,RangeV(self->Ru.start(),self->Rla.end())); // par_g_par_u_x_la

      Mat Minv_Jrla = slvLLFac(LLM, Jrla);
      J.set(self->RuMove, self->Rla, Minv_Jrla); // par_ud_par_la
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::jacGGL(int* cols, double *t, double *y_, double *J_, int *rows, double *rpar, int *ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(*cols, y_); // fmatvec variant of y_
      Mat J(*rows, *cols, J_); // fmatvec variant of J_

      self->system->setTime(*t);
      self->system->setState(y(self->Rz));
      self->system->setla(y(self->Rla));
      self->system->resetUpToDate();
      self->system->setUpdatela(false);
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
        self->system->calclaSize(3);
        self->system->updateWRef(self->system->getWParent(0));
      }

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

      Mat Minv_Jrla = slvLLFac(LLM, Jrla);
      J.set(self->Ru, self->Rla, Minv_Jrla); // par_ud_par_la
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::massFull(int* cols, double* m_, int* rows, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Mat M(*rows,*cols, m_);
      for(int i=0; i<self->system->getzSize(); i++) M(0,i) = 1;
      for(int i=self->system->getzSize(); i<*cols; i++) M(0,i) = 0;
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::massReduced(int* cols, double* m_, int* rows, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Mat M(*rows,*cols, m_);
      for(int i=0; i<self->system->getuSize()+self->system->getxSize(); i++) M(0,i) = 1;
      for(int i=self->system->getuSize()+self->system->getxSize(); i<*cols; i++) M(0,i) = 0;
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RADAUIntegrator::plot(int* nr, double* told, double* t, double* y, double* cont, int* lrc, int* n, double* rpar, int* ipar, int* irtrn) {
    auto *self = reinterpret_cast<RADAUIntegrator*>(ipar);
    if(self->exception) { // if a exception was already thrown in a call before -> do nothing but set interrupt flag and return
      *irtrn=-1;
      return;
    }
    try { // catch exception -> C code must catch all exceptions
      double curTimeAndState = numeric_limits<double>::min(); // just a value which will never be reached
      double tRoot = *t;

      // root-finding
      if(self->system->getsvSize()) {
        self->system->setTime(*t);
        curTimeAndState = *t;
        self->system->setState(Vec(self->system->getzSize(),y));
        self->system->resetUpToDate();
        self->shift = self->signChangedWRTsvLast(self->system->evalsv());
        // if a root exists in the current step ...
        if(self->shift) {
          // ... search the first root and set step.second to this time
          double dt = *t-*told;
          while(dt>self->dtRoot) {
            dt/=2;
            double tCheck = tRoot-dt;
            self->system->setTime(tCheck);
            curTimeAndState = tCheck;
            for(int i=1; i<=self->system->getzSize(); i++)
              self->system->getState()(i-1) = CONTRA(&i,&tCheck,cont,lrc);
            self->system->resetUpToDate();
            if(self->signChangedWRTsvLast(self->system->evalsv()))
              tRoot = tCheck;
          }
          if(curTimeAndState != tRoot) {
            curTimeAndState = tRoot;
            self->system->setTime(tRoot);
            for(int i=1; i<=self->system->getzSize(); i++)
              self->system->getState()(i-1) = CONTRA(&i,&tRoot,cont,lrc);
          }
          self->system->resetUpToDate();
          auto &sv = self->system->evalsv();
          auto &jsv = self->system->getjsv();
          for(int i=0; i<sv.size(); ++i)
            jsv(i)=self->svLast(i)*sv(i)<0;
        }
      }

      while(tRoot >= self->tPlot) {
        if(curTimeAndState != self->tPlot) {
          curTimeAndState = self->tPlot;
          self->system->setTime(self->tPlot);
          for(int i=1; i<=self->system->getzSize(); i++)
            self->system->getState()(i-1) = CONTRA(&i,&self->tPlot,cont,lrc);
        }
        self->system->resetUpToDate();
        if(self->formalism) {
          for(int i=self->system->getzSize()+1; i<=self->system->getzSize()+self->system->getlaSize(); i++)
            self->system->getla(false)(i-(self->system->getzSize()+1)) = CONTRA(&i,&self->tPlot,cont,lrc);
          self->system->setUpdatela(false);
        }
        self->system->plot();
        if(self->msgAct(Status))
          self->msg(Status) << "   t = " <<  self->tPlot << ",\tdt = "<< *t-*told << flush;

        self->system->updateInternalState();

        double s1 = clock();
        self->time += (s1-self->s0)/CLOCKS_PER_SEC;
        self->s0 = s1;

        self->tPlot += self->dtOut;
      }

      if(self->shift) {
        // shift the system
        if(curTimeAndState != tRoot) {
          self->system->setTime(tRoot);
          for(int i=1; i<=self->system->getzSize(); i++)
            self->system->getState()(i-1) = CONTRA(&i,&tRoot,cont,lrc);
        }
        if(self->plotOnRoot) {
          self->system->resetUpToDate();
          self->system->plot();
        }
        self->system->resetUpToDate();
        self->system->shift();
        if(self->formalism>1) { // DAE2, DAE3 or GGL
          self->system->calcgdSize(3); // IH
          self->system->updategdRef(self->system->getgdParent());
          if(self->formalism>2) { // DAE3 or GGL
            self->system->calcgSize(2); // IB
            self->system->updategRef(self->system->getgParent());
          }
        }
        if(self->plotOnRoot) {
          self->system->resetUpToDate();
          self->system->plot();
        }
        *irtrn = -1;
      }
      else {
        // check drift
        bool projVel = true;
        if(self->getToleranceForPositionConstraints()>=0) {
          self->system->setTime(*t);
          self->system->setState(Vec(self->system->getzSize(),y));
          self->system->resetUpToDate();
          if(self->system->positionDriftCompensationNeeded(self->getToleranceForPositionConstraints())) { // project both, first positions and then velocities
            self->system->projectGeneralizedPositions(3);
            self->system->projectGeneralizedVelocities(3);
            projVel = false;
            self->drift = true;
            *irtrn=-1;
          }
        }
        if(self->getToleranceForVelocityConstraints()>=0 and projVel) {
          self->system->setTime(*t);
          self->system->setState(Vec(self->system->getzSize(),y));
          self->system->resetUpToDate();
          if(self->system->velocityDriftCompensationNeeded(self->getToleranceForVelocityConstraints())) { // project velicities
            self->system->projectGeneralizedVelocities(3);
            self->drift = true;
            *irtrn=-1;
          }
        }
        self->system->updateStopVectorParameters();
      }

      self->system->updateInternalState();
    }
    catch(...) { // if a exception is thrown catch and store it in self and set the interrupt flag
      self->exception = current_exception();
      *irtrn=-1;
    }
  }

  void RADAUIntegrator::integrate() {
    if(formalism==unknown)
      throwError("(RADAUIntegrator::integrate): formalism unknown");

    fzdot[0] = &RADAUIntegrator::fzdotODE;
    fzdot[1] = &RADAUIntegrator::fzdotDAE1;
    fzdot[2] = &RADAUIntegrator::fzdotDAE2;
    fzdot[3] = &RADAUIntegrator::fzdotDAE3;
    fzdot[4] = &RADAUIntegrator::fzdotGGL;
    jac[0] = &RADAUIntegrator::jacODE;
    jac[1] = &RADAUIntegrator::jacDAE1;
    jac[2] = &RADAUIntegrator::jacDAE2;
    jac[3] = &RADAUIntegrator::jacDAE3;
    jac[4] = &RADAUIntegrator::jacGGL;
    mass[0] = &RADAUIntegrator::massFull;
    mass[1] = &RADAUIntegrator::massReduced;

    debugInit();

    init();

    if(not neq)
      throwError("(RADAUIntegrator::integrate): dimension of the system must be at least 1");

    if(formalism==DAE3 and system->getgSize()!=system->getgdSize())
      throwError("(RADAUIntegrator::integrate): size of g (" + to_string(system->getgSize()) + ") must be equal to size of gd (" + to_string(system->getgdSize()) + ") when using the DAE3 formalism");

    double t = tStart;

    Vec y(neq);
    Vec z;
    z.ref(y, Rz);
    if(z0.size()) {
      if(z0.size() != system->getzSize()+system->getisSize())
        throwError("(RADAUIntegrator::integrate): size of z0 does not match, must be " + to_string(system->getzSize()+system->getisSize()));
      z = z0(Rz);
      system->setInternalState(z0(RangeV(system->getzSize(),z0.size()-1)));
    }
    else
      z = system->evalz0();

    if(aTol.size() == 0)
      aTol.resize(1,INIT,1e-6);
    if(rTol.size() == 0)
      rTol.resize(1,INIT,1e-6);

    int iTol;
    if(aTol.size() == 1)
      iTol = 0;
    else {
      iTol = 1;
      if(aTol.size() != neq)
        throwError("(RADAUIntegrator::integrate): size of aTol does not match, must be " + to_string(neq));
    }
    if(rTol.size() != aTol.size())
      throwError("(RADAUIntegrator::integrate): size of rTol does not match aTol, must be " + to_string(aTol.size()));

    int out = 1; // subroutine is available for output

    double rPar[1]; // not used

    exception=nullptr;
    int *iPar = reinterpret_cast<int*>(this);

    int lWork = neq*(neq+1+7*neq+3*7+3)+20;

    // we define a iWork array of size 1 larger then needed for RADUA5 and extend this array to the negative range.
    // iWork[0...] (= iWorkExtended[1...]) is used for RADAU
    // iWork[-1] (= iWorkExtended[0]) is used to pass a special flag for jacobian-recalculation a adapted RADAU code
    // SEE radau5.f MBSIM_EXTENDED_IWORK_ARRAY
    int liWork = 5*neq+20;
    iWorkExtended.resize(liWork + 1);
    iWork=&iWorkExtended[1];

    work.resize(lWork);

    int iMas = formalism>0; // mass-matrix
    int mlMas = 0; // lower bandwith of the mass-matrix
    int muMas = 0; // upper bandwith of the mass-matrix
    int iJac = partiallyAnalyticalJacobian; // jacobian is computed
                            // - by finite differences if partiallyAnalyticalJacobian is false
                            // - by a combination of finite differences and an analytical solution, otherwise

    int idid;

    double dt = dt0;

    tPlot = t + dtPlot;
    dtOut = dtPlot;

    system->setTime(t);
    system->setState(z);
    system->resetUpToDate();
    system->computeInitialCondition();
    if(formalism>1) { // DAE2, DAE3 or GGL
      system->calcgdSize(3); // IH
      system->updategdRef(system->getgdParent());
      if(formalism>2) { // DAE3 or GGL
        system->calcgSize(2); // IB
        system->updategRef(system->getgParent());
      }
    }
    system->plot();
    svLast <<= system->evalsv();
    z = system->getState(); // needed, as computeInitialCondition may change the state

    reinit();

    if(formalism>0)
      y.set(Rla, system->evalla()); // set a proper initial state for lambda

    s0 = clock();

    while(t<tEnd-epsroot) {
      drift = false;

      RADAU(&neq,(*fzdot[formalism]),&t,y(),&tEnd,&dt,
          rTol(),aTol(),&iTol,
          *jac[formalism],&iJac,&mlJac,&muJac,
          *mass[reduced],&iMas,&mlMas,&muMas,
          plot,&out,
          work(),&lWork,iWork,&liWork,rPar,iPar,&idid);
      if(exception)
        rethrow_exception(exception);
      if(idid < 0)
        throw runtime_error("RADAU failed with idid = "+to_string(idid));

      if(shift) {
        system->resetUpToDate();
        svLast = system->evalsv();
        dt = dt0;
        reinit();
      }

      if(shift || drift) {
        // set new state
        t = system->getTime();
        z = system->getState();
        system->resetUpToDate();
        if(formalism>0)
          y.set(Rla, system->evalla()); // set a proper initial state for lambda
      }
    }

    msg(Info)<<"nrRHS (excluding jac): "<<iWork[13]<<endl;
    msg(Info)<<"nrJac: "<<iWork[14]<<endl;
    msg(Info)<<"nrSteps: "<<iWork[15]<<endl;
    msg(Info)<<"nrStepsAccepted: "<<iWork[16]<<endl;
    msg(Info)<<"nrStepsRejected (excluding first step): "<<iWork[17]<<endl;
    msg(Info)<<"nrLUdecom: "<<iWork[18]<<endl;
    msg(Info)<<"nrForwardBackwardSubs: "<<iWork[19]<<endl;
  }

  void RADAUIntegrator::reinit() {
    DAEIntegrator::reinit();

    for(int i=20; i<work.size(); i++)
      work(i) = 0;

    if(dtMax>0)
      work(6) = dtMax; // maximum step size
    work(3) = newtonIterTol;
    work(2) = jacobianRecomputation;
    work(1) = stepSizeSaftyFactor;

    for(size_t i=20; i<iWorkExtended.size(); i++)
      iWorkExtended[i] = 0;

    iWork[-1] = jacobianRecomputationAtRejectedSteps;
    iWork[1] = maxSteps; // maximum number of steps
    iWork[7] = static_cast<int>(stepSizeControl);
    iWork[2] = maxNewtonIter;

    if(formalism==DAE1)
      iWork[4] = system->getzSize() + system->getlaSize();
    else if(formalism==DAE2) {
      iWork[4] = system->getzSize();
      iWork[5] = system->getgdSize();
    }
    else if(formalism==DAE3) {
      iWork[4] = system->getzSize();
      iWork[6] = system->getgSize();
    }
    else if(formalism==GGL) {
      iWork[4] = system->getzSize();
      iWork[5] = system->getgdSize() + system->getgSize();
    }
    if(reduced) {
      if(formalism==GGL)
        throwError("(RADAUIntegrator::reinit): The 'formalism'=='GGL' cannot be used with 'reducedForm'==true.");
      if(not system->getqdequ())
        throwError("(RADAUIntegrator::reinit): The reduced form can only be used if dq/dt = u.");
      iWork[8] = system->getqSize();
      iWork[9] = system->getqSize();
      mlJac = neq - system->getqSize(); // jacobian is a reduced matrix
    }
    else
      mlJac = neq; // jacobian is a full matrix
    muJac = mlJac; // need not to be defined if mlJac = neq

    int zStart = 20+7*neq;
    int zEnd = zStart+system->getzSize();
    int laEnd = zEnd+system->getgdSize();
    zd0.ref(work,RangeV(zStart,zEnd-1));
    gd0.ref(work,RangeV(zEnd,laEnd-1));
    if(formalism==GGL)
      g0.ref(work,RangeV(laEnd,laEnd+system->getgSize()-1));
  }

  void RADAUIntegrator::initializeUsingXML(DOMElement *element) {
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"stepLimit");
    if(e) setStepLimit(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"formalism");
    if(e) {
      string formalismStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(formalismStr=="ODE") formalism=ODE;
      else if(formalismStr=="DAE1") formalism=DAE1;
      else if(formalismStr=="DAE2") formalism=DAE2;
      else if(formalismStr=="DAE3") formalism=DAE3;
      else if(formalismStr=="GGL") formalism=GGL;
      else formalism=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"reducedForm");
    if(e) setReducedForm((E(e)->getText<bool>()));

    e=E(element)->getFirstElementChildNamed(MBSIM%"maximumNumberOfNewtonIterations");
    if(e) setMaximalNumberOfNewtonIterations((E(e)->getText<int>()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"newtonIterationTolerance");
    if(e) setNewtonIterationTolerance((E(e)->getText<double>()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"jacobianRecomputation");
    if(e) setJacobianRecomputation((E(e)->getText<double>()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"jacobianRecomputationAtRejectedSteps");
    if(e) setJacobianRecomputationAtRejectedSteps((E(e)->getText<bool>()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"stepSizeControl");
    if(e) {
      auto ssc = (E(e)->getText<string>());
      ssc = ssc.substr(1, ssc.size()-2);
      if(ssc=="modPred")
        setStepSizeControl(StepSizeControl::ModPred);
      else if(ssc=="classic")
        setStepSizeControl(StepSizeControl::Classic);
      else
        throw DOMEvalException("Unknonwn stepSizeControl "+ssc, e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"stepSizeSaftyFactor");
    if(e) setStepSizeSaftyFactor((E(e)->getText<double>()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"partiallyAnalyticalJacobian");
    if(e) setPartiallyAnalyticalJacobian(E(e)->getText<bool>());
  }

}
