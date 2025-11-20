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
#include "lsodi_integrator.h"
#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, LSODIIntegrator)

  extern bool odePackInUse;

  LSODIIntegrator::Res LSODIIntegrator::res[5];
  LSODIIntegrator::Jac LSODIIntegrator::jac[5];

  // This code is taken from opkda1.f
  double LSODIIntegrator::delta(int i, double z) const {
    return max(epsroot*abs(z),0.01/rWork(lewt+i));
  }

  void LSODIIntegrator::resODE(int* neq, double* t, double* z_, double* zd_, double* res_, int* ires) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec z(neq[0], z_);
      Vec zd(neq[0], zd_);
      Vec res(neq[0], res_);
      self->system->setTime(*t);
      self->system->resetUpToDate();
      res = self->system->evalzd() - zd;
    }
    catch(...) { // if a exception is thrown catch and store it in self
      *ires = 2;
      self->exception = current_exception();
    }
  }

  void LSODIIntegrator::resDAE2(int* neq, double* t, double* y_, double* yd_, double* res_, int* ires) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(neq[0], y_);
      Vec yd(neq[0], yd_);
      Vec res(neq[0], res_);
      self->system->setTime(*t);
      self->system->resetUpToDate();
      self->system->setUpdatela(false);
      res.set(self->Rz, self->system->evalzd() - yd(self->Rz));
      res.set(self->Rla, self->system->evalgd());
    }
    catch(...) { // if a exception is thrown catch and store it in self
      *ires = 2;
      self->exception = current_exception();
    }
  }

  void LSODIIntegrator::resGGL(int* neq, double* t, double* y_, double* yd_, double* res_, int* ires) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(neq[0], y_);
      Vec yd(neq[0], yd_);
      Vec res(neq[0], res_);
      self->system->setTime(*t);
      self->system->resetUpToDate();
      self->system->setUpdatela(false);
      res.set(self->Rz, self->system->evalzd() - yd(self->Rz));
      res.set(self->Rla, self->system->evalgd());
      res.set(self->Rl, self->system->evalg());
      if(self->system->getgSize() != self->system->getgdSize()) {
        self->system->calclaSize(5);
        self->system->updateWRef(self->system->getWParent(0));
        self->system->setUpdateW(true);
        res.add(self->Rq, self->system->evalW()*y(self->Rl));
        self->system->calclaSize(3);
        self->system->updateWRef(self->system->getWParent(0));
      }
      else
        res.add(self->Rq, self->system->getW()*y(self->Rl));
    }
    catch(...) { // if a exception is thrown catch and store it in self
      *ires = 2;
      self->exception = current_exception();
    }
  }

  void LSODIIntegrator::adda(int *neq, double *t, double *y_, int *ml, int *mu, double *P_, int *nrowp) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    SqrMat P(*nrowp, P_);
    for(int i=0; i<self->system->getzSize(); i++) P(i,i) += 1;
  }

  void LSODIIntegrator::jacODE(int *neq, double* t, double* y_, double *yd_, int* ml, int* mu, double* J_, int* nrowp) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Mat J(neq[0], neq[0], J_); // fmatvec variant of J_

      self->lewt = J_[0]-1; // needed in function delta: index in workspace where the array containing multiplicative weights begins

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
        J(i,i) -= 1;
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void LSODIIntegrator::jacDAE2(int *neq, double* t, double* y_, double *yd_, int* ml, int* mu, double* J_, int* nrowp) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Mat J(neq[0], neq[0], J_); // fmatvec variant of J_

      self->lewt = J_[0]-1; // needed in function delta: index in workspace where the array containing multiplicative weights begins

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
        J(i,i) -= 1;
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void LSODIIntegrator::jacGGL(int *neq, double* t, double* y_, double *yd_, int* ml, int* mu, double* J_, int* nrowp) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(neq[0], y_); // fmatvec variant of y_
      Mat J(neq[0], neq[0], J_); // fmatvec variant of J_

      self->lewt = J_[0]-1; // needed in function delta: index in workspace where the array containing multiplicative weights begins

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
        J(i,i) -= 1;

      Mat Minv_Jrla = slvLLFac(LLM, Jrla);
      J.set(self->Ru, self->Rla, Minv_Jrla); // par_ud_par_la
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }
  void LSODIIntegrator::integrate() {
    if(formalism==unknown)
      throwError("(LSODIIntegrator::integrate): formalism unknown");

    res[0] = &LSODIIntegrator::resODE;
    res[1] = nullptr; // not available
    res[2] = &LSODIIntegrator::resDAE2;
    res[3] = nullptr; // not available
    res[4] = &LSODIIntegrator::resGGL;
    jac[0] = &LSODIIntegrator::jacODE;
    jac[1] = nullptr; // not available
    jac[2] = &LSODIIntegrator::jacDAE2;
    jac[3] = nullptr; // not available
    jac[4] = &LSODIIntegrator::jacGGL;

    debugInit();

    if(odePackInUse)
      throwError("(LSODIIntegrator::integrate): Only one integration with LSODEIntegrator, LSODAIntegrator and LSODIIntegrator at a time is possible.");
    odePackInUse = true;

    init();

    if(not neq)
      throwError("(LSODIIntegrator::integrate): dimension of the system must be at least 1");

    exception=nullptr;
    neq_.resize(1+sizeof(void*)/sizeof(int)+1);
    neq_(0) = neq;
    LSODIIntegrator *self=this;
    memcpy(&neq_(1), &self, sizeof(void*));

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
	throwError("(LSODIIntegrator::integrate): size of z0 does not match, must be " + to_string(system->getzSize()+system->getisSize()));
      system->setState(z0(RangeV(0,system->getzSize()-1)));
      system->setInternalState(z0(RangeV(system->getzSize(),z0.size()-1)));
    }
    else
      system->evalz0();

    double t = tStart;
    double tPlot = t + dtPlot;

    int itask=2, iopt=1, istate=1, MF=partiallyAnalyticalJacobian?21:22;
    int lrWork = 22+9*neq+neq*neq;
    int liWork = 20+neq;
    rWork.resize(lrWork);
    rWork(5) = dtMax;
    rWork(6) = dtMin;
    VecInt iWork(liWork);
    iWork(5) = maxSteps;

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

    reinit();

    double s0 = clock();
    double time = 0;

    int zero = 0;
    int iflag;

    int iTol = 4; // aTol and rTol is a vector

    while(t<tEnd-epsroot) {
      DLSODI(*res[formalism], adda, *jac[formalism], neq_(), system->getzParent()(), yd(), &t, &tPlot, &iTol, rTol(), aTol(), &itask, &istate, &iopt, rWork(), &lrWork, iWork(), &liWork, &MF);
      if(exception)
        rethrow_exception(exception);
      if(istate==2 or istate==1) {
        double curTimeAndState = numeric_limits<double>::min(); // just a value which will never be reached
        double tRoot = t;

        // root-finding
        if(system->getsvSize()) {
          system->setTime(t);
          curTimeAndState = t;
          system->resetUpToDate();
          shift = signChangedWRTsvLast(system->evalsv());
          // if a root exists in the current step ...
          double dt = rWork(10);
          if(shift) {
            // ... search the first root and set step.second to this time
            while(dt>dtRoot) {
              dt/=2;
              double tCheck = tRoot-dt;
              curTimeAndState = tCheck;
              DINTDY(&tCheck, &zero, &rWork(20), neq_(), system->getState()(), &iflag);
              system->setTime(tCheck);
              system->resetUpToDate();
              if(signChangedWRTsvLast(system->evalsv()))
                tRoot = tCheck;
            }
            if(curTimeAndState != tRoot) {
              curTimeAndState = tRoot;
              DINTDY(&tRoot, &zero, &rWork(20), neq_(), system->getState()(), &iflag);
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
            DINTDY(&tPlot, &zero, &rWork(20), neq_(), system->getState()(), &iflag);
            system->setTime(tPlot);
          }
          system->resetUpToDate();
          system->setzd(yd(RangeV(0,system->getzSize()-1)));
          system->setUpdatezd(false);
          if(formalism) system->setUpdatela(false);
          system->plot();
          if(msgAct(Status))
            msg(Status) << "   t = " <<  tPlot << ",\tdt = "<< rWork(10) << flush;

          system->updateInternalState();

          double s1 = clock();
          time += (s1-s0)/CLOCKS_PER_SEC;
          s0 = s1;

          tPlot += dtPlot;
        }

        if(shift) {
          // shift the system
          if(curTimeAndState != tRoot) {
            DINTDY(&tRoot, &zero, &rWork(20), neq_(), system->getState()(), &iflag);
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
          istate=1;
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
              istate=1;
            }
          }
          if(gdMax>=0 and projVel) {
            system->setTime(t);
            system->resetUpToDate();
            if(system->velocityDriftCompensationNeeded(gdMax)) { // project velicities
              system->projectGeneralizedVelocities(3);
              istate=1;
            }
          }
          system->updateStopVectorParameters();
        }
        if(istate==1) {
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
      else if(istate<0) throwError("Integrator LSODI failed with istate = "+to_string(istate));
    }

    msg(Info)<<string("nrRHS")+(partiallyAnalyticalJacobian?" (excluding jac): ":" (including jac): ")<<iWork(11)<<endl;
    msg(Info)<<"nrJac: "<<iWork(12)<<endl;
    msg(Info)<<"nrSteps: "<<iWork(10)<<endl;

    odePackInUse = false;
  }

  void LSODIIntegrator::init() {
    DAEIntegrator::init();
    if(partiallyAnalyticalJacobian)
      zd0.resize(system->getzSize(),NONINIT);
  }

  void LSODIIntegrator::reinit() {
    DAEIntegrator::reinit();

    if(excludeAlgebraicVariables)
      for(int i=system->getzSize(); i<neq; i++) aTol(i) = 1e15;

    neq_(0) = neq;
    rWork(4) = dt0;

    if(partiallyAnalyticalJacobian) {
      gd0.resize(system->getgdSize(),NONINIT);
      if(formalism==GGL)
        g0.resize(system->getgSize(),NONINIT);
    }
  }

  void LSODIIntegrator::initializeUsingXML(DOMElement *element) {
    DAEIntegrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"initialStepSize");
    if(e) setInitialStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"maximumStepSize");
    if(e) setMaximumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"minimumStepSize");
    if(e) setMinimumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"stepLimit");
    if(e) setStepLimit(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"formalism");
    if(e) {
      string formalismStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(formalismStr=="ODE") formalism=ODE;
      else if(formalismStr=="DAE2") formalism=DAE2;
      else if(formalismStr=="GGL") formalism=GGL;
      else formalism=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"excludeAlgebraicVariablesFromErrorTest");
    if(e) setExcludeAlgebraicVariablesFromErrorTest(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"partiallyAnalyticalJacobian");
    if(e) setPartiallyAnalyticalJacobian(E(e)->getText<bool>());
  }

}
