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
    auto self=*reinterpret_cast<ODEXIntegrator**>(&ipar[0]);
    Vec zd(*zSize, zd_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(Vec(*zSize, z_));
    self->getSystem()->resetUpToDate();
    zd = self->getSystem()->evalzd();
  }

  void ODEXIntegrator::plot(int* nr, double* told, double* t, double* z, int* n, double* con, int *ncon, int* icomp, int* nd, double* rpar, int* ipar, int* irtrn) {
    auto self=*reinterpret_cast<ODEXIntegrator**>(&ipar[0]);

    double curTimeAndState = numeric_limits<double>::min(); // just a value which will never be reached
    double tRoot = *t;

    // root-finding
    if(self->getSystem()->getsvSize()) {
      self->getSystem()->setTime(*t);
      curTimeAndState = *t;
      self->getSystem()->setState(Vec(self->getSystem()->getzSize(),z));
      self->getSystem()->resetUpToDate();
      self->shift = self->signChangedWRTsvLast(self->getSystem()->evalsv());
      // if a root exists in the current step ...
      if(self->shift) {
        // ... search the first root and set step.second to this time
        double dt = *t-*told;
        while(dt>self->dtRoot) {
          dt/=2;
          double tCheck = tRoot-dt;
          self->getSystem()->setTime(tCheck);
          curTimeAndState = tCheck;
          for(int i=1; i<=*n; i++)
            self->getSystem()->getState()(i-1) = CONTEX(&i,&tCheck,con,ncon,icomp,nd);
          self->getSystem()->resetUpToDate();
          if(self->signChangedWRTsvLast(self->getSystem()->evalsv()))
            tRoot = tCheck;
        }
        if(curTimeAndState != tRoot) {
          curTimeAndState = tRoot;
          self->getSystem()->setTime(tRoot);
          for(int i=1; i<=*n; i++)
            self->getSystem()->getState()(i-1) = CONTEX(&i,&tRoot,con,ncon,icomp,nd);
        }
        self->getSystem()->resetUpToDate();
        auto &sv = self->getSystem()->evalsv();
        auto &jsv = self->getSystem()->getjsv();
        for(int i=0; i<sv.size(); ++i)
          jsv(i)=self->svLast(i)*sv(i)<0;
      }
    }

    while(tRoot >= self->tPlot) {
      if(curTimeAndState != self->tPlot) {
        curTimeAndState = self->tPlot;
        self->getSystem()->setTime(self->tPlot);
        for(int i=1; i<=*n; i++)
          self->getSystem()->getState()(i-1) = CONTEX(&i,&self->tPlot,con,ncon,icomp,nd);
      }
      self->getSystem()->resetUpToDate();
      self->getSystem()->plot();
      if(self->msgAct(Status))
	self->msg(Status) << "   t = " <<  self->tPlot << ",\tdt = "<< *t-*told << flush;

      self->getSystem()->updateInternalState();

      double s1 = clock();
      self->time += (s1-self->s0)/CLOCKS_PER_SEC;
      self->s0 = s1;

      self->tPlot += self->dtOut;
    }

    if(self->shift) {
      // shift the system
      if(curTimeAndState != tRoot) {
        self->getSystem()->setTime(tRoot);
        for(int i=1; i<=*n; i++)
          self->getSystem()->getState()(i-1) = CONTEX(&i,&tRoot,con,ncon,icomp,nd);
      }
      if(self->plotOnRoot) {
        self->getSystem()->resetUpToDate();
        self->getSystem()->plot();
      }
      self->getSystem()->resetUpToDate();
      self->getSystem()->shift();
      if(self->plotOnRoot) {
        self->getSystem()->resetUpToDate();
        self->getSystem()->plot();
      }
      *irtrn = -1;
    }
    else {
      // check drift
      bool projVel = true;
      if(self->getToleranceForPositionConstraints()>=0) {
        self->getSystem()->setTime(*t);
        self->getSystem()->setState(Vec(self->getSystem()->getzSize(),z));
        self->getSystem()->resetUpToDate();
        if(self->getSystem()->positionDriftCompensationNeeded(self->getToleranceForPositionConstraints())) { // project both, first positions and then velocities
          self->getSystem()->projectGeneralizedPositions(3);
          self->getSystem()->projectGeneralizedVelocities(3);
          projVel = false;
          *irtrn=-1;
        }
      }
      if(self->getToleranceForVelocityConstraints()>=0 and projVel) {
        self->getSystem()->setTime(*t);
        self->getSystem()->setState(Vec(self->getSystem()->getzSize(),z));
        self->getSystem()->resetUpToDate();
        if(self->getSystem()->velocityDriftCompensationNeeded(self->getToleranceForVelocityConstraints())) { // project velicities
          self->getSystem()->projectGeneralizedVelocities(3);
          *irtrn=-1;
        }
      }
      self->getSystem()->updateStopVectorParameters();
    }

    self->getSystem()->updateInternalState();
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

    double rPar;
    int iPar[sizeof(void*)/sizeof(int)+1];
    ODEXIntegrator *self=this;
    memcpy(&iPar[0], &self, sizeof(void*));

    int lWork = 2*(zSize*(9+5)+5*9+20+(2*9*(9+2)+5)*zSize);
    int liWork = 2*(2*9+21+zSize);
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
      ODEX(&zSize,fzdot,&t,z(),&tEnd, &dt,rTol(),aTol(),&iTol,plot,&out,
          work(),&lWork,iWork(),&liWork,&rPar,iPar,&idid);

      self->getSystem()->updateInternalState();

      if(shift) {
        system->resetUpToDate();
        svLast = system->evalsv();
        dt = dt0;
      }
      t = system->getTime();
      z = system->getState();
    }
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
