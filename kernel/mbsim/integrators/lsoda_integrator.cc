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
#include "lsoda_integrator.h"
#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, LSODAIntegrator)

  extern bool odePackInUse;

  // This code is taken from opkda1.f
  double LSODAIntegrator::delta(int i, double z) const {
    return max(epsroot*abs(z),r0/rWork(lewt+i));
  }

  void LSODAIntegrator::fzdot(int* neq, double* t, double* z_, double* zd_) {
    auto self=*reinterpret_cast<LSODAIntegrator**>(&neq[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Vec zd(neq[0], zd_);
      self->system->setTime(*t);
      self->system->resetUpToDate();
      zd = self->system->evalzd();
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void LSODAIntegrator::jac(int *neq, double* t, double* z_, int* ml, int* mu, double* J_, int* nrowp) {
    auto self=*reinterpret_cast<LSODAIntegrator**>(&neq[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      Mat J(neq[0], neq[0], J_); // fmatvec variant of J_

      self->lewt = J_[0]-1; // needed in function delta: index in workspace where the array containing multiplicative weights begins
      self->r0 = J_[1]; // needed in function delta: r0
      self->zd0.ref(self->rWork,RangeV(self->lewt+neq[0],self->lewt+2*neq[0]-1)); // saved zd, which is used for the numerical part of the Jacobian
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
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void LSODAIntegrator::integrate() {
    debugInit();

    if(odePackInUse)
      throwError("(LSODAIntegrator::integrate): Only one integration with LSODEIntegrator, LSODAIntegrator and LSODIIntegrator at a time is possible.");
    odePackInUse = true;

    int zSize=system->getzSize();

    if(not zSize)
      throwError("(LSODAIntegrator::integrate): dimension of the system must be at least 1");

    exception=nullptr;
    int neq[1+sizeof(void*)/sizeof(int)+1];
    neq[0]=zSize;
    LSODAIntegrator *self=this;
    memcpy(&neq[1], &self, sizeof(void*));

    if(z0.size()) {
      if(z0.size() != zSize+system->getisSize())
	throwError("(LSODAIntegrator::integrate): size of z0 does not match, must be " + to_string(zSize+system->getisSize()));
      system->setState(z0(RangeV(0,zSize-1)));
      system->setInternalState(z0(RangeV(zSize,z0.size()-1)));
    }
    else
      system->evalz0();

    double t = tStart;
    double tPlot = t + dtPlot;

    if(aTol.size() == 0)
      aTol.resize(1,INIT,1e-6);
    if(rTol.size() == 0)
      rTol.resize(1,INIT,1e-6);

    int iTol;
    if(rTol.size() == 1) {
      if(aTol.size() == 1)
        iTol = 1;
      else {
        iTol = 2;
        if(aTol.size() != zSize)
          throwError("(LSODAIntegrator::integrate): size of aTol does not match, must be " + to_string(zSize));
      }
    }
    else {
      if(aTol.size() == 1)
        iTol = 3;
      else {
        iTol = 4;
        if(aTol.size() != zSize)
          throwError("(LSODAIntegrator::integrate): size of aTol does not match, must be " + to_string(zSize));
      }
      if(rTol.size() != zSize)
        throwError("(LSODAIntegrator::integrate): size of rTol does not match, must be " + to_string(zSize));
    }

    int itask=2, iopt=1, istate=1, jt=partiallyAnalyticalJacobian?1:2;
    int lrWork = 22+zSize*max(16,zSize+9);
    int liWork = 20+zSize;
    rWork.resize(lrWork);
    rWork(4) = dt0;
    rWork(5) = dtMax;
    rWork(6) = dtMin;
    VecInt iWork(liWork);
    iWork(5) = maxSteps;

    system->setTime(t);
    system->resetUpToDate();
    system->computeInitialCondition();
    system->plot();
    svLast <<= system->evalsv();

    double s0 = clock();
    double time = 0;

    int zero = 0;
    int iflag;

    init();

    while(t<tEnd-epsroot) {
      DLSODA(fzdot, neq, system->getState()(), &t, &tEnd, &iTol, rTol(), aTol(),
          &itask, &istate, &iopt, rWork(), &lrWork, iWork(), &liWork, jac, &jt);
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
              DINTDY (&tCheck, &zero, &rWork(20), neq, system->getState()(), &iflag);
              system->setTime(tCheck);
              system->resetUpToDate();
              if(signChangedWRTsvLast(system->evalsv()))
                tRoot = tCheck;
            }
            if(curTimeAndState != tRoot) {
              curTimeAndState = tRoot;
              DINTDY (&tRoot, &zero, &rWork(20), neq, system->getState()(), &iflag);
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
            DINTDY (&tPlot, &zero, &rWork(20), neq, system->getState()(), &iflag);
            system->setTime(tPlot);
          }
          system->resetUpToDate();
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
            DINTDY (&tRoot, &zero, &rWork(20), neq, system->getState()(), &iflag);
            system->setTime(tRoot);
          }
          if(plotOnRoot) {
            system->resetUpToDate();
            system->plot();
          }
          system->resetUpToDate();
          system->shift();
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
          if(shift) {
            system->resetUpToDate();
            svLast = system->evalsv();
          }
          t = system->getTime();
        }

        system->updateInternalState();
      }
      else if(istate<0) throwError("Integrator LSODA failed with istate = "+to_string(istate));
    }

    msg(Info)<<string("nrRHS")+(partiallyAnalyticalJacobian?" (excluding jac): ":" (including jac): ")<<iWork(11)<<endl;
    msg(Info)<<"nrJac: "<<iWork(12)<<endl;
    msg(Info)<<"nrSteps: "<<iWork(10)<<endl;

    odePackInUse = false;
  }

  void LSODAIntegrator::init() {
    ImplicitIntegrator::init();
    if(partiallyAnalyticalJacobian)
      zd0.resize(system->getzSize(),NONINIT);
  }

  void LSODAIntegrator::initializeUsingXML(DOMElement *element) {
    ImplicitIntegrator::initializeUsingXML(element);
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"minimumStepSize");
    if(e) setMinimumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"stepLimit");
    if(e) setStepLimit(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"partiallyAnalyticalJacobian");
    if(e) setPartiallyAnalyticalJacobian(E(e)->getText<bool>());
  }

}
