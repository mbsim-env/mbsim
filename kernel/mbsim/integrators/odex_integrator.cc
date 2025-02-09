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
 *   martin.o.foerg@googlemail.com
 *
 */

#include <config.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/utils/eps.h>
#include "fortran/fortran_wrapper.h"
#include "odex_integrator.h"
#include <ctime>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, ODEXIntegrator)

  void ODEXIntegrator::fzdot(int* zSize, double* t, double* z_, double* zd_, double* rpar, int* ipar) {
    auto *self = reinterpret_cast<ODEXIntegrator*>(ipar);
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

  void ODEXIntegrator::plot(int* nr, double* told, double* t, double* z, int* n, double* con, int *ncon, int* icomp, int* nd, double* rpar, int* ipar, int* irtrn) {
    auto *self = reinterpret_cast<ODEXIntegrator*>(ipar);
    if(self->exception) { // if a exception was already thrown in a call before -> do nothing but set interrupt flag of ODEX and return
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
        self->system->setState(Vec(self->system->getzSize(),z));
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
            for(int i=1; i<=*n; i++)
              self->system->getState()(i-1) = CONTEX(&i,&tCheck,con,ncon,icomp,nd);
            self->system->resetUpToDate();
            if(self->signChangedWRTsvLast(self->system->evalsv()))
              tRoot = tCheck;
          }
          if(curTimeAndState != tRoot) {
            curTimeAndState = tRoot;
            self->system->setTime(tRoot);
            for(int i=1; i<=*n; i++)
              self->system->getState()(i-1) = CONTEX(&i,&tRoot,con,ncon,icomp,nd);
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
          for(int i=1; i<=*n; i++)
            self->system->getState()(i-1) = CONTEX(&i,&self->tPlot,con,ncon,icomp,nd);
        }
        self->system->resetUpToDate();
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
          for(int i=1; i<=*n; i++)
            self->system->getState()(i-1) = CONTEX(&i,&tRoot,con,ncon,icomp,nd);
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
          self->system->setState(Vec(self->system->getzSize(),z));
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
          self->system->setState(Vec(self->system->getzSize(),z));
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
    catch(...) { // if a exception is thrown catch and store it in self and set the interrupt flag of ODEX
      self->exception = current_exception();
      *irtrn=-1;
    }
  }

  void ODEXIntegrator::integrate() {
    debugInit();

    int zSize=system->getzSize();

    if(not zSize)
      throwError("(ODEXIntegrator::integrate): dimension of the system must be at least 1");

    double t = tStart;

    Vec z(zSize);
    if(z0.size()) {
      if(z0.size() != system->getzSize()+system->getisSize())
        throwError("(ODEXIntegrator::integrate): size of z0 does not match, must be " + to_string(system->getzSize()+system->getisSize()));
      z = z0(RangeV(0,system->getzSize()-1));
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
      if(aTol.size() != zSize)
        throwError("(ODEXIntegrator::integrate): size of aTol does not match, must be " + to_string(zSize));
    }
    if(rTol.size() != aTol.size())
      throwError("(ODEXIntegrator::integrate): size of rTol does not match aTol, must be " + to_string(aTol.size()));

    int out = 2; // dense output is performed in plot

    double rPar[1]; // not used

    exception=nullptr;
    int *iPar = reinterpret_cast<int*>(this);

    int lWork = zSize*(9+5)+5*9+20+(2*9*(9+2)+5)*zSize;
    int liWork = 2*9+21+zSize;
    VecInt iWork(liWork);
    Vec work(lWork);
    if(dtMax>0)
      work(1) = dtMax; // maximum step size
    iWork(0) = maxSteps; // maximum number of steps
    iWork(7) = zSize;

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

    s0 = clock();

    while(t<tEnd-epsroot) {
      drift = false;

      ODEX(&zSize,fzdot,&t,z(),&tEnd, &dt,rTol(),aTol(),&iTol,plot,&out,
          work(),&lWork,iWork(),&liWork,rPar,iPar,&idid);
      if(exception)
        rethrow_exception(exception);

      if(shift) {
        system->resetUpToDate();
        svLast = system->evalsv();
      }

      if(shift || drift) {
        // set new state
        t = system->getTime();
        z = system->getState();
        dt = dt0;
      }
    }

    msg(Info)<<"nrRHS: "<<iWork(16)<<endl;
    msg(Info)<<"nrSteps: "<<iWork(17)<<endl;
    msg(Info)<<"nrStepsAccepted: "<<iWork(18)<<endl;
    msg(Info)<<"nrStepsRejected: "<<iWork(19)<<endl;
  }

  void ODEXIntegrator::initializeUsingXML(DOMElement *element) {
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"stepLimit");
    if(e) setStepLimit(E(e)->getText<int>());
  }

}
