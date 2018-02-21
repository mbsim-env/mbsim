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
#include "fortran/fortran_wrapper.h"
#include "dopri5_integrator.h"
#include <fstream>
#include <ctime>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, DOPRI5Integrator)

  void DOPRI5Integrator::fzdot(int* zSize, double* t, double* z_, double* zd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DOPRI5Integrator**>(&ipar[0]);
    Vec zd(*zSize, zd_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(Vec(*zSize, z_));
    self->getSystem()->resetUpToDate();
    zd = self->getSystem()->evalzd();
  }

  void DOPRI5Integrator::plot(int* nr, double* told, double* t,double* z, int* n, double* con, int* icomp, int* nd, double* rpar, int* ipar, int* irtrn) {
    auto self=*reinterpret_cast<DOPRI5Integrator**>(&ipar[0]);

    double curTimeAndState = -1;
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
        while(dt>1e-10) {
          dt/=2;
          double tCheck = tRoot-dt;
          self->getSystem()->setTime(tCheck);
          curTimeAndState = tCheck;
          for(int i=1; i<=*n; i++)
            self->getSystem()->getState()(i-1) = CONTD5(&i,&tCheck,con,icomp,nd);
          self->getSystem()->resetUpToDate();
          if(self->signChangedWRTsvLast(self->getSystem()->evalsv()))
            tRoot = tCheck;
        }
        // root found increase time by dt and set jsv
        if(curTimeAndState != tRoot) {
          curTimeAndState = tRoot;
          self->getSystem()->setTime(tRoot);
          for(int i=1; i<=*n; i++)
            self->getSystem()->getState()(i-1) = CONTD5(&i,&tRoot,con,icomp,nd);
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
          self->getSystem()->getState()(i-1) = CONTD5(&i,&self->tPlot,con,icomp,nd);
        self->getSystem()->resetUpToDate();
      }
      self->getSystem()->plot();
      if(self->output)
	cout << "   t = " <<  self->tPlot << ",\tdt = "<< *t-*told << "\r"<<flush;

      double s1 = clock();
      self->time += (s1-self->s0)/CLOCKS_PER_SEC;
      self->s0 = s1; 

      if(self->plotIntegrationData) self->integPlot<< self->tPlot << " " << *t-*told << " " << self->time << endl;
      self->tPlot += self->dtOut;
    }

    if(self->shift) {
      // shift the system
      if(curTimeAndState != tRoot) {
        self->getSystem()->setTime(tRoot);
        for(int i=1; i<=*n; i++)
          self->getSystem()->getState()(i-1) = CONTD5(&i,&tRoot,con,icomp,nd);
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
      self->getSystem()->resetUpToDate();
      self->svLast=self->getSystem()->evalsv();
      *irtrn = -1;
    }
    else {

    // check drift
    if(self->getToleranceForPositionConstraints()>=0) {
      if(curTimeAndState != *t) {
        self->getSystem()->setTime(*t);
        self->getSystem()->setState(Vec(self->getSystem()->getzSize(),z));
        self->getSystem()->resetUpToDate();
      }
      if(self->getSystem()->positionDriftCompensationNeeded(self->getToleranceForPositionConstraints())) { // project both, first positions and then velocities
        self->getSystem()->projectGeneralizedPositions(3);
        self->getSystem()->projectGeneralizedVelocities(3);
        *irtrn=-1;
      }
    }
    else if(self->getToleranceForVelocityConstraints()>=0) {
      if(curTimeAndState != *t) {
        self->getSystem()->setTime(*t);
        self->getSystem()->setState(Vec(self->getSystem()->getzSize(),z));
        self->getSystem()->resetUpToDate();
      }
      if(self->getSystem()->velocityDriftCompensationNeeded(self->getToleranceForVelocityConstraints())) { // project velicities
        self->getSystem()->projectGeneralizedVelocities(3);
        *irtrn=-1;
      }
    }
    }
  }

  bool DOPRI5Integrator::signChangedWRTsvLast(const fmatvec::Vec &svStepEnd) const {
    for(int i=0; i<svStepEnd.size(); i++)
      if(svLast(i)*svStepEnd(i)<0)
        return true;
    return false;
  }

  void DOPRI5Integrator::integrate() {
    debugInit();

    int zSize = system->getzSize();

    double t = tStart;

    Vec z(zSize);
    if(z0.size()) {
      if(z0.size() != zSize)
        throw MBSimError("(DOPRI5Integrator::integrate): size of z0 does not match, must be " + toStr(zSize));
      z = z0;
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
        throw MBSimError("(DOPRI5Integrator::integrate): size of aTol does not match, must be " + toStr(zSize));
    }
    if(rTol.size() != aTol.size())
      throw MBSimError("(DOPRI5Integrator::integrate): size of rTol does not match aTol, must be " + toStr(aTol.size()));

    int out = 2; // dense output is performed in plot

    double rPar;
    int iPar[sizeof(void*)/sizeof(int)+1]; // store this at iPar[0..]
    DOPRI5Integrator *self=this;
    memcpy(&iPar[0], &self, sizeof(void*));

    int lWork = 2*(8*zSize+5*zSize+21);
    int liWork = 2*(zSize+21);
    VecInt iWork(liWork);
    Vec work(lWork);
    if(dtMax>0)
      work(5) = dtMax; // maximum step size
    work(6) = dt0; // initial step size
    iWork(0) = maxSteps; // maximum number of steps
    iWork(4) = zSize;

    int idid;

    tPlot = t + dtPlot;
    dtOut = dtPlot;

    system->setTime(t);
    system->setState(z);
    system->computeInitialCondition();
    system->resetUpToDate();
    system->plot();
    svLast=system->evalsv();

    if(plotIntegrationData) integPlot.open((name + ".plt").c_str());

    cout.setf(ios::scientific, ios::floatfield);

    s0 = clock();

    while(t<tEnd) {
      DOPRI5(&zSize,fzdot,&t,z(),&tEnd,rTol(),aTol(),&iTol,plot,&out,
          work(),&lWork,iWork(),&liWork,&rPar,iPar,&idid);

      if(shift) work(6) = dt0;
      t = system->getTime();
      z = system->getState();
    }

    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      ofstream integSum((name + ".sum").c_str());
      integSum.precision(8);
      integSum << "Integration time: " << time << endl;
      integSum << "Simulation time: " << t << endl;
      //integSum << "Integration steps: " << integrationSteps << endl;
      integSum.close();
    }

    cout.unsetf (ios::scientific);
    cout << endl;
  }

  void DOPRI5Integrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteTolerance");
    if(e) setAbsoluteTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteToleranceScalar");
    if(e) setAbsoluteTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeTolerance");
    if(e) setRelativeTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeToleranceScalar");
    if(e) setRelativeTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialStepSize");
    if(e) setInitialStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"maximumStepSize");
    if(e) setMaximumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stepLimit");
    if(e) setStepLimit(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"plotOnRoot");
    if(e) setPlotOnRoot(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(E(e)->getText<double>());
  }

}
