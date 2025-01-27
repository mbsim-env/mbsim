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

  DASPKIntegrator::Delta DASPKIntegrator::delta[4];

  void DASPKIntegrator::deltaODE(double* t, double* z_, double* zd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec z(ipar[0], z_);
      Vec zd(ipar[0], zd_);
      Vec delta(ipar[0], delta_);
      self->getSystem()->setTime(*t);
      self->getSystem()->resetUpToDate();
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
      Vec y(ipar[0], y_);
      Vec yd(ipar[0], yd_);
      Vec delta(ipar[0], delta_);
      self->getSystem()->setTime(*t);
      self->getSystem()->resetUpToDate();
      self->getSystem()->setUpdatela(false);
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
      Vec y(ipar[0], y_);
      Vec yd(ipar[0], yd_);
      Vec delta(ipar[0], delta_);
      self->getSystem()->setTime(*t);
      self->getSystem()->resetUpToDate();
      self->getSystem()->setUpdatela(false);
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
      self->getSystem()->setTime(*t);
      self->getSystem()->resetUpToDate();
      self->getSystem()->setUpdatela(false);
      delta.set(self->Rz, self->system->evalzd() - yd(self->Rz));
      delta.set(self->Rla, self->system->evalgd());
      delta.set(self->Rl, self->system->evalg());
      if(self->system->getgSize() != self->system->getgdSize()) {
        self->system->calclaSize(5);
        self->system->updateWRef(self->system->getWParent(0));
        self->system->setUpdateW(false);
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

  void DASPKIntegrator::jac(double* t, double* y_, double* yd_, double* pd, double* cj, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Mat J(ipar[0], ipar[0], pd); // fmatvec variant of J_
      int ires;

      // the undisturbed call -> this sets the system resetUpToDate
      // res0 is later used for the numerical part of the jacobian
      self->delta[self->formalism](t,y_,yd_,cj,self->res0(),&ires,rpar,ipar);

      // the columns for la are given analytically
      Mat Minv_Jrla = slvLLFac(self->system->evalLLM(), self->system->evalJrla());
      J.set(self->Ru, self->Rla, Minv_Jrla);
      for(int c=self->Rla.start(); c<=self->Rla.end(); ++c) {
        for(int r=0; r<self->Ru.start(); ++r)
          J(r,c)=0;
        for(int r=self->Ru.end()+1; r<ipar[0]; ++r)
          J(r,c)=0;
      }

      if(self->formalism==GGL) {
        // the columns for algebraic GGL state are given analytically
        if(self->system->getgSize() != self->system->getgdSize()) {
          self->system->calclaSize(5);
          self->system->updateWRef(self->system->getWParent(0));
          self->system->setUpdateW(false);
          J.set(self->Rq, self->Rl, self->system->evalW());
          self->system->calclaSize(3);
          self->system->updateWRef(self->system->getWParent(0));
        }
        else
          J.set(self->Rq, self->Rl, self->system->evalW());
        // the rest of the entries in these columns are 0
        for(int c=self->Rl.start(); c<=self->Rl.end(); ++c) {
          for(int r=self->Ru.start(); r<ipar[0]; ++r)
            J(r,c)=0;
        }
      }

      // now the finite difference of all other columns
      // this code is taken from radau5.f JACOBIAN IS FULL,
      // but converted to C and skipping the last columns of the jacobian for la which is given analytically
      for(int c=0; c<self->system->getzSize(); ++c) {
        double ySafe=y_[c];
        double delta=sqrt(macheps*max(1.e-5,abs(ySafe)));
        y_[c]=ySafe+delta;
        self->delta[self->formalism](t,y_,yd_,cj,self->res1(),&ires,rpar,ipar);
        for(int r=0; r<ipar[0]; ++r)
          pd[(c*ipar[0])+r]=(self->res1(r)-self->res0(r))/delta;
        pd[(c*ipar[0])+c]-=*cj;
        y_[c]=ySafe;
      }
      if(self->formalism==DAE1)
        J.add(self->Rla, self->Ru, *cj*self->system->evalW().T());
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void DASPKIntegrator::integrate() {
    if(formalism==unknown)
      throwError("(DASPKIntegrator::integrate): formalism unknown");

    delta[0] = &DASPKIntegrator::deltaODE;
    delta[1] = &DASPKIntegrator::deltaDAE1;
    delta[2] = &DASPKIntegrator::deltaDAE2;
    delta[3] = &DASPKIntegrator::deltaGGL;

    debugInit();

    calcSize();
    Rq = RangeV(0,system->getqSize()-1);
    Ru = RangeV(system->getqSize(),system->getqSize()+system->getuSize()-1);
    Rz = RangeV(0,system->getzSize()-1);

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

    VecInt info(20);

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
    info(4) = (not numericalJacobian) and (formalism>0); // jacobian is computed
                            // - by finite differences if numericalJacobian is true or formalism is set to ODE
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
    int iPar[1+sizeof(void*)/sizeof(int)+1];
    DASPKIntegrator *self=this;
    memcpy(&iPar[1], &self, sizeof(void*));

    int lWork = 2*(50+9*neq+neq*neq+neq);
    int liWork = 2*(40+neq+neq);
    VecInt iWork(liWork);
    Vec work(lWork);

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

    calcSize();
    iPar[0] = neq;
    work(1) = dtMax; // maximum stepsize
    work(2) = dt0; // initial stepsize
    if(info(15)) {
      for(int i=0; i<system->getzSize(); i++)
        iWork(40+i) = 1; // differential variable
      for(int i=system->getzSize(); i<neq; i++)
        iWork(40+i) = -1; // algebraic variable
    }

    double s0 = clock();
    double time = 0;
    int lphi = excludeAlgebraicVariables?50+4*neq:50+3*neq;

    while(t<tEnd-epsroot) {
      DDASPK(*delta[formalism],&neq,&t,system->getState()(),yd(),&tEnd,info(),rTol(),aTol(),&idid,work(),&lWork,iWork(),&liWork,rPar,iPar,jac,nullptr);
      if(exception)
        rethrow_exception(exception);
      if(idid==1) {
        double curTimeAndState = -1;
        double tRoot = t;

        // root-finding
        if(getSystem()->getsvSize()) {
          getSystem()->setTime(t);
          curTimeAndState = t;
          getSystem()->resetUpToDate();
          shift = signChangedWRTsvLast(getSystem()->evalsv());
          // if a root exists in the current step ...
          double dt = work(6);
          if(shift) {
            // ... search the first root and set step.second to this time
            while(dt>dtRoot) {
              dt/=2;
              double tCheck = tRoot-dt;
              curTimeAndState = tCheck;
              DDATRP(&t, &tCheck, system->getState()(), yd(), &neq, &iWork(7), &work(lphi), &work(38));
              getSystem()->setTime(tCheck);
              getSystem()->resetUpToDate();
              if(signChangedWRTsvLast(getSystem()->evalsv()))
                tRoot = tCheck;
            }
            if(curTimeAndState != tRoot) {
              curTimeAndState = tRoot;
              DDATRP(&t, &tRoot, system->getState()(), yd(), &neq, &iWork(7), &work(lphi), &work(38));
              getSystem()->setTime(tRoot);
            }
            getSystem()->resetUpToDate();
            auto &sv = getSystem()->evalsv();
            auto &jsv = getSystem()->getjsv();
            for(int i=0; i<sv.size(); ++i)
              jsv(i)=svLast(i)*sv(i)<0;
          }
        }
        while(tRoot>=tPlot and tPlot<=tEnd+epsroot) {
          if(curTimeAndState != tPlot) {
            curTimeAndState = tPlot;
            DDATRP(&t, &tPlot, system->getState()(), yd(), &neq, &iWork(7), &work(lphi), &work(38));
            getSystem()->setTime(tPlot);
          }
          getSystem()->resetUpToDate();
          system->setzd(yd(RangeV(0,system->getzSize()-1)));
          system->setUpdatezd(false);
          if(formalism) system->setUpdatela(false);
          getSystem()->plot();
          if(msgAct(Status))
            msg(Status) << "   t = " <<  tPlot << ",\tdt = "<< work(6) << flush;

          getSystem()->updateInternalState();

          double s1 = clock();
          time += (s1-s0)/CLOCKS_PER_SEC;
          s0 = s1;

          tPlot += dtPlot;
        }

        if(shift) {
          // shift the system
          if(curTimeAndState != tRoot) {
            DDATRP(&t, &tRoot, system->getState()(), yd(), &neq, &iWork(7), &work(lphi), &work(38));
            getSystem()->setTime(tRoot);
          }
          if(plotOnRoot) {
            system->resetUpToDate();
//            system->setzd(yd(0,system->getzSize()-1));
//            system->setUpdatezd(false);
//            if(formalism) system->setUpdatela(false);
            system->plot();
//            system->plotAtSpecialEvent();
          }
          getSystem()->resetUpToDate();
          getSystem()->shift();
          if(formalism>1) { // DAE2 or GGL
            system->calcgdSize(3); // IH
            system->updategdRef(system->getgdParent());
            if(formalism==GGL) { // GGL
              system->calcgSize(2); // IB
              system->updategRef(system->getgParent());
            }
          }
          if(plotOnRoot) {
            getSystem()->resetUpToDate();
            getSystem()->plot();
          }
          info(0) = 0;
        }
        else {
          // check drift
          bool projVel = true;
          if(gMax>=0) {
            getSystem()->setTime(t);
            getSystem()->resetUpToDate();
            if(getSystem()->positionDriftCompensationNeeded(gMax)) { // project both, first positions and then velocities
              getSystem()->projectGeneralizedPositions(3);
              getSystem()->projectGeneralizedVelocities(3);
              projVel = false;
              info(0) = 0;
            }
          }
          if(gdMax>=0 and projVel) {
            getSystem()->setTime(t);
            getSystem()->resetUpToDate();
            if(getSystem()->velocityDriftCompensationNeeded(gdMax)) { // project velicities
              getSystem()->projectGeneralizedVelocities(3);
              info(0) = 0;
            }
          }
          getSystem()->updateStopVectorParameters();
        }
        if(info(0)==0) {
          t = system->getTime();
          system->resetUpToDate();
          yd.set(RangeV(0,system->getzSize()-1), system->evalzd());
          if(shift) {
            svLast = system->evalsv();
            calcSize();
            lphi = excludeAlgebraicVariables?50+4*neq:50+3*neq;
            iPar[0] = neq;
            work(2) = dt0;
            if(info(15)) {
              for(int i=system->getzSize(); i<neq; i++)
                iWork(40+i) = -1; // algebraic variable
            }
          }
        }
        getSystem()->updateInternalState();
      }
      else if(idid<0) throwError("Integrator DASPK failed with istate = "+to_string(idid));
    }

    msg(Info)<<string("nrRHS")+(info(4)?" (excluding jac): ":" (including jac): ")<<iWork(11)<<endl;
    msg(Info)<<"nrJac: "<<iWork(12)<<endl;
    msg(Info)<<"nrSteps: "<<iWork(10)<<endl;
    msg(Info)<<"nrStepsAccepted: "<<iWork(10)-iWork(13)<<endl;
    msg(Info)<<"nrStepsRejected: "<<iWork(13)<<endl;
    msg(Info)<<"nrNonlinConvFailures: "<<iWork(14)<<endl;
    msg(Info)<<"nrLinConvFailures: "<<iWork(15)<<endl;
  }

  void DASPKIntegrator::calcSize() {
    if(formalism==DAE1 or formalism==DAE2)
      neq = system->getzSize()+system->getlaSize();
    else if(formalism==GGL)
      neq = system->getzSize()+system->getgdSize()+system->getgSize();
    else
      neq = system->getzSize();
    res0.resize(neq);
    res1.resize(neq);
    Rla = RangeV(system->getzSize(), system->getzSize()+system->getlaSize()-1);
    Rl = RangeV(system->getzSize()+system->getlaSize(), neq-1);
  }

  void DASPKIntegrator::initializeUsingXML(DOMElement *element) {
    RootFindingIntegrator::initializeUsingXML(element);
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
