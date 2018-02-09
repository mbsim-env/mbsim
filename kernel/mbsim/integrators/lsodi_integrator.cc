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
#include <mbsim/utils/utils.h>
#include "fortran/fortran_wrapper.h"
#include "lsodi_integrator.h"
#include <fstream>
#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, LSODIIntegrator)

  void LSODIIntegrator::res(int* neq, double* t, double* y_, double* yd_, double* res_, int* ires) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    int nq = self->system->getqSize();
    int nu = self->system->getuSize();
    int nx = self->system->getxSize();
    Vec y(neq[0], y_);
    Vec yd(neq[0], yd_);
    Vec res(neq[0], res_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(y(0,nq+nu+nx-1));
    self->getSystem()->resetUpToDate();
    res(0,nq-1) = self->system->evaldq() - yd(0,nq-1);
    res(nq,nq+nu-1) = self->system->evalh() + self->system->evalV()*y(nq+nu+nx,neq[0]-1) - self->system->evalM()*yd(nq,nq+nu-1); 
    res(nq+nu,nq+nu+nx-1) = self->system->evaldx() - yd(nq+nu,nq+nu+nx-1);
    res(nq+nu+nx,neq[0]-1) = self->system->evalgd();
  }

  void LSODIIntegrator::adda(int *neq, double *t, double *y_, int *ml, int *mu, double *P_, int *nrowp) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    int nq = self->system->getqSize();
    int nu = self->system->getuSize();
    int nx = self->system->getxSize();
    SqrMat P(*nrowp, P_);
    for(int i=0; i<nq; i++) P(i,i) += 1;
    P(RangeV(nq,nq+nu-1),RangeV(nq,nq+nu-1)) += self->system->evalM(); // system is up to date, as res is called immediately before
    for(int i=nq+nu; i<nq+nu+nx-1; i++) P(i,i) += 1;
  }

  void LSODIIntegrator::integrate() {
    debugInit();

    if(odePackInUse)
      throw MBSimError("Only one integration with LSODARIntegrator, LSODKRIntegrator and LSODEIntegrator at a time is possible.");
    odePackInUse = true;

    int N=system->getzSize()+system->getgdSize();
    int neq[1+sizeof(void*)/sizeof(int)+1];
    neq[0]=N;
    LSODIIntegrator *self=this;
    memcpy(&neq[1], &self, sizeof(void*));

    Vec y(N);
    Vec yd(N);

    if(z0.size()) {
      if(z0.size() != system->getzSize())
        throw MBSimError("(LSODIIntegrator::integrate): size of z0 does not match");
      y(0,system->getzSize()-1) = z0;
    }
    else
      y(0,system->getzSize()-1) = system->evalz0();

    double t = tStart;
    double tPlot = min(tEnd,t + dtPlot);

    Vec absTol(N,NONINIT);
    if(aTol.size() == 0) 
      absTol(0,system->getzSize()-1).init(1e-6);
    else if(aTol.size() == 1)
      absTol(0,system->getzSize()-1).init(aTol(0));
    else {
      absTol(0,system->getzSize()-1) = aTol;
      absTol(system->getzSize(),N-1).init(1e15);
      assert (aTol.size() == system->getzSize());
    }
    absTol(system->getzSize(),N-1).init(1e15);

    int iTol = 2; // Vektor

    int itask=1, istate=1, iopt=0;
    int lrWork = (22+9*N+N*N)*2;
    Vec rWork(lrWork);
    rWork(4) = dt0;
    rWork(5) = dtMax;
    rWork(6) = dtMin;
    int liWork=(20+N)*2;
    VecInt iWork(liWork);
    iWork(5) = maxSteps;

    system->setStepSize(1);
    system->setTime(t);
    system->setState(y(0,system->getzSize()-1));
    system->resetUpToDate();
    system->plot();
    yd(0,system->getzSize()-1) = system->evalzd();
    y(system->getzSize(),N-1) = system->getla();

    double s0 = clock();
    double time = 0;
    int integrationSteps = 0;

    ofstream integPlot;
    if(plotIntegrationData) {
      integPlot.open((name + ".plt").c_str());
      integPlot << "#1 t [s]:" << endl;
      integPlot << "#1 dt [s]:" << endl;
      integPlot << "#1 calculation time [s]:" << endl;
    }

    int MF = 22;

    cout.setf(ios::scientific, ios::floatfield);
    while(t<tEnd) {
      DLSODI(res, adda, 0, neq, y(), yd(), &t, &tPlot, &iTol, &rTol, absTol(), &itask, &istate, &iopt, rWork(), &lrWork, iWork(), &liWork, &MF);
      if(istate==2 || fabs(t-tPlot)<epsroot) {
        system->setTime(t);
        system->setState(y(0,system->getzSize()-1));
        system->resetUpToDate();
        system->plot();
        if(output)
          cout << "   t = " <<  t << ",\tdt = "<< rWork(10) << "\r"<<flush;
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        if(plotIntegrationData) integPlot<< t << " " << rWork(10) << " " << time << endl;
        tPlot = min(tEnd,tPlot + dtPlot);

        // check drift
        if(gMax>=0 and system->positionDriftCompensationNeeded(gMax)) { // project both, first positions and then velocities
          system->projectGeneralizedPositions(3);
          system->projectGeneralizedVelocities(3);
          istate=1;
        }
        else if(gdMax>=0 and system->velocityDriftCompensationNeeded(gdMax)) { // project velicities
          system->projectGeneralizedVelocities(3);
          istate=1;
        }
      }
      if(istate<0) throw MBSimError("Integrator LSODI failed with istate = "+toString(istate));
    }

    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      ofstream integSum((name + ".sum").c_str());
      integSum << "Integration time: " << time << endl;
      integSum << "Simulation time: " << t << endl;
      integSum << "Integration steps: " << integrationSteps << endl;
      integSum.close();
    }

    cout.unsetf (ios::scientific);
    cout << endl;

    odePackInUse = false;
  }

  void LSODIIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteTolerance");
    if(e) setAbsoluteTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteToleranceScalar");
    if(e) setAbsoluteTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeToleranceScalar");
    if(e) setRelativeTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialStepSize");
    if(e) setInitialStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"maximumStepSize");
    if(e) setMaximumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"minimumStepSize");
    if(e) setMinimumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stepLimit");
    if(e) setStepLimit(E(e)->getText<int>());
   e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(E(e)->getText<double>());
  }

}
