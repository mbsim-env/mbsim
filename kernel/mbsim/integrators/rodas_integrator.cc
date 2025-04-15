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
#include "rodas_integrator.h"
#include <ctime>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, RODASIntegrator)

  RODASIntegrator::Fzdot RODASIntegrator::fzdot[2];
  RODASIntegrator::Jac RODASIntegrator::jac[2];
  RODASIntegrator::Pt RODASIntegrator::pt[2];
  RODASIntegrator::Mass RODASIntegrator::mass[2];

  // This code is taken from rodas.f
  double RODASIntegrator::delta(int i, double z) const {
    return sqrt(1e-16*max(1.e-5,abs(z)));
  }

  void RODASIntegrator::par_ud_xd_par_t(Vec &pt) {
    const Vec &zd = system->getzd(false);
    double t=system->getTime();
    double delt=sqrt(1e-16*max(1.e-5,abs(t)));
    system->setTime(t+delt);
    system->resetUpToDate();
    system->updateud();
    system->updatexd();
    for(int i=system->getqSize(); i<system->getzSize(); i++)
      pt(i)=(zd(i)-zd0(i))/delt;
    system->setTime(t);
  }

  void RODASIntegrator::par_ud_xd_gdd_par_t(Vec &pt) {
    const Vec &zd = system->getzd(false);
    const Vec &ud = system->getud(false);
    double t=system->getTime();
    double delt=sqrt(1e-16*max(1.e-5,abs(t)));
    system->setTime(t+delt);
    system->resetUpToDate();
    system->setUpdatela(false);
    system->updateud();
    system->updatexd();
    const Vec &gdd1 = system->evalW().T()*ud + system->evalwb();
    for(int i=system->getqSize(); i<system->getzSize(); i++)
      pt(i)=(zd(i)-zd0(i))/delt;
    for(int i=0; i<system->getgdSize(); i++)
      pt(i+laInd)=(gdd1(i)-gd0(i))/delt; // we use gd0 for gdd0 here
    system->setTime(t);
  }

  void RODASIntegrator::fzdotODE(int* zSize, double* t, double* z_, double* zd_, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RODASIntegrator*>(ipar);
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

  void RODASIntegrator::fzdotDAE1(int* neq, double* t, double* y_, double* yd_, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RODASIntegrator*>(ipar);
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

  void RODASIntegrator::jacODE(int* cols, double *t, double *z_, double *J_, int *rows, double *rpar, int *ipar) {
    auto *self = reinterpret_cast<RODASIntegrator*>(ipar);
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

  void RODASIntegrator::jacDAE1(int* cols, double *t, double *y_, double *J_, int *rows, double *rpar, int *ipar) {
    auto *self = reinterpret_cast<RODASIntegrator*>(ipar);
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

  void RODASIntegrator::ptODE(int* zSize, double* t, double* z_, double* pt_, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RODASIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec pt(*zSize, pt_); // fmatvec variant of pt_

      self->system->setTime(*t);
      self->system->setState(Vec(*zSize, z_));
      self->system->resetUpToDate();

      self->par_ud_xd_par_t(pt);
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RODASIntegrator::ptDAE1(int* neq, double* t, double* y_, double* pt_, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RODASIntegrator*>(ipar);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec y(*neq, y_); // fmatvec variant of y_
      Vec pt(*neq, pt_); // fmatvec variant of pt_

      self->system->setTime(*t);
      self->system->setState(y(self->Rz));
      self->system->resetUpToDate();
      self->system->setla(y(self->Rla));
      self->system->setUpdatela(false);

      self->par_ud_xd_gdd_par_t(pt);
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void RODASIntegrator::massFull(int* cols, double* m_, int* rows, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RODASIntegrator*>(ipar);
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

  void RODASIntegrator::massReduced(int* cols, double* m_, int* rows, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<RODASIntegrator*>(ipar);
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

  void RODASIntegrator::plot(int* nr, double* told, double* t, double* y, double* cont, int* lrc, int* n, double* rpar, int* ipar, int* irtrn) {
    auto *self = reinterpret_cast<RODASIntegrator*>(ipar);
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
              self->system->getState()(i-1) = CONTRO(&i,&tCheck,cont,lrc);
            self->system->resetUpToDate();
            if(self->signChangedWRTsvLast(self->system->evalsv()))
              tRoot = tCheck;
          }
          if(curTimeAndState != tRoot) {
            curTimeAndState = tRoot;
            self->system->setTime(tRoot);
            for(int i=1; i<=self->system->getzSize(); i++)
              self->system->getState()(i-1) = CONTRO(&i,&tRoot,cont,lrc);
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
            self->system->getState()(i-1) = CONTRO(&i,&self->tPlot,cont,lrc);
        }
        self->system->resetUpToDate();
        if(self->formalism) {
          for(int i=self->system->getzSize()+1; i<=self->system->getzSize()+self->system->getlaSize(); i++)
            self->system->getla(false)(i-(self->system->getzSize()+1)) = CONTRO(&i,&self->tPlot,cont,lrc);
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
            self->system->getState()(i-1) = CONTRO(&i,&tRoot,cont,lrc);
        }
        if(self->plotOnRoot) {
          self->system->resetUpToDate();
          self->system->plot();
        }
        self->system->resetUpToDate();
        self->system->shift();
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

  void RODASIntegrator::integrate() {
    if(formalism==unknown)
      throwError("(RODASIntegrator::integrate): formalism unknown");

    fzdot[0] = &RODASIntegrator::fzdotODE;
    fzdot[1] = &RODASIntegrator::fzdotDAE1;
    jac[0] = &RODASIntegrator::jacODE;
    jac[1] = &RODASIntegrator::jacDAE1;
    pt[0] = &RODASIntegrator::ptODE;
    pt[1] = &RODASIntegrator::ptDAE1;
    mass[0] = &RODASIntegrator::massFull;
    mass[1] = &RODASIntegrator::massReduced;

    debugInit();

    init();

    if(not neq)
      throwError("(RODASIntegrator::integrate): dimension of the system must be at least 1");

    double t = tStart;

    Vec y(neq);
    Vec z;
    z.ref(y, RangeV(0,system->getzSize()-1));
    if(z0.size()) {
      if(z0.size() != system->getzSize()+system->getisSize())
        throwError("(RODASIntegrator::integrate): size of z0 does not match, must be " + to_string(system->getzSize()+system->getisSize()));
      z = z0(RangeV(0,system->getzSize()-1));
      system->setInternalState(z0(RangeV(system->getzSize(),z0.size()-1)));
    }
    else
      z = system->evalz0();

    int out = 1; // subroutine is available for output

    double rPar[1]; // not used

    exception=nullptr;
    int *iPar = reinterpret_cast<int*>(this);

    int lWork = neq*(neq+neq+neq+14)+20;
    int liWork = neq+20;
    iWork.resize(liWork);
    work.resize(lWork);

    int ifcn = not autonom;
    int idfx = partiallyAnalyticalJacobian;
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
    system->plot();
    svLast <<= system->evalsv();
    z = system->getState(); // needed, as computeInitialCondition may change the state

    reinit();

    if(formalism>0)
      y.set(Rla, system->evalla()); // set a proper initial state for lambda

    s0 = clock();

    int iTol = 1;

    while(t<tEnd-epsroot) {
      RODAS(&neq,*fzdot[formalism],&ifcn,&t,y(),&tEnd,&dt,
          rTol(),aTol(),&iTol,
          *jac[formalism],&iJac,&mlJac,&muJac,*pt[formalism],&idfx,
          *mass[reduced],&iMas,&mlMas,&muMas,
          plot,&out,
          work(),&lWork,iWork(),&liWork,rPar,iPar,&idid);
      if(exception)
        rethrow_exception(exception);
      if(idid < 0)
        throw runtime_error("RODAS failed with idid = "+to_string(idid));

      if(shift) {
        system->resetUpToDate();
        svLast = system->evalsv();
        reinit();
      }

      if(shift || drift) {
        // set new state
        t = system->getTime();
        z = system->getState();
        if(formalism>0)
          y.set(Rla, system->evalla()); // set a proper initial state for lambda
        dt = dt0;
      }
    }

    msg(Info)<<"nrRHS (excluding jac): "<<iWork(13)<<endl;
    msg(Info)<<"nrJac: "<<iWork(14)<<endl;
    msg(Info)<<"nrSteps: "<<iWork(15)<<endl;
    msg(Info)<<"nrStepsAccepted: "<<iWork(16)<<endl;
    msg(Info)<<"nrStepsRejected (excluding first step): "<<iWork(17)<<endl;
    msg(Info)<<"nrLUdecom: "<<iWork(18)<<endl;
    msg(Info)<<"nrForwardBackwardSubs: "<<iWork(19)<<endl;
  }

  void RODASIntegrator::reinit() {
    DAEIntegrator::reinit();

    for(int i=20; i<work.size(); i++)
      work(i) = 0;

    if(dtMax>0)
      work(1) = dtMax; // maximum step size

    iWork(0) = maxSteps; // maximum number of steps
    if(reduced) {
      iWork(8) = system->getqSize();
      iWork(9) = system->getqSize();
      mlJac = neq - system->getqSize(); // jacobian is a reduced matrix
    }
    else
      mlJac = neq; // jacobian is a full matrix
    muJac = mlJac; // need not to be defined if mlJac = neq

    if(partiallyAnalyticalJacobian) {
      int zStart = 20+neq;
      int zEnd = zStart+system->getzSize();
      int laEnd = zEnd+system->getgdSize();
      zd0.ref(work,RangeV(zStart,zEnd-1));
      gd0.ref(work,RangeV(zEnd,laEnd-1));
    }
  }

  void RODASIntegrator::initializeUsingXML(DOMElement *element) {
    DAEIntegrator::initializeUsingXML(element);
    DOMElement *e;
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
      else formalism=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"reducedForm");
    if(e) setReducedForm((E(e)->getText<bool>()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"autonomousSystem");
    if(e) setAutonomousSystem((E(e)->getText<bool>()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"partiallyAnalyticalJacobian");
    if(e) setPartiallyAnalyticalJacobian(E(e)->getText<bool>());
  }

}
