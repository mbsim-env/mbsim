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

  LSODIIntegrator::Res LSODIIntegrator::res[3];

  void LSODIIntegrator::resODE(int* neq, double* t, double* z_, double* zd_, double* res_, int* ires) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    Vec z(neq[0], z_);
    Vec zd(neq[0], zd_);
    Vec res(neq[0], res_);
    self->getSystem()->setTime(*t);
    self->getSystem()->resetUpToDate();
    res = self->system->evalzd() - zd;
  }

  void LSODIIntegrator::resDAE2(int* neq, double* t, double* y_, double* yd_, double* res_, int* ires) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    Vec y(neq[0], y_);
    Vec yd(neq[0], yd_);
    Vec res(neq[0], res_);
    self->getSystem()->setTime(*t);
    self->getSystem()->resetUpToDate();
    self->getSystem()->setUpdatela(false);
    res(0,self->system->getzSize()-1) = self->system->evalzd() - yd(0,self->system->getzSize()-1);
    res(self->system->getzSize(),neq[0]-1) = self->system->evalgd();
  }

  void LSODIIntegrator::resGGL(int* neq, double* t, double* y_, double* yd_, double* res_, int* ires) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    Vec y(neq[0], y_);
    Vec yd(neq[0], yd_);
    Vec res(neq[0], res_);
    self->getSystem()->setTime(*t);
    self->getSystem()->resetUpToDate();
    self->getSystem()->setUpdatela(false);
    res(0,self->system->getzSize()-1) = self->system->evalzd() - yd(0,self->system->getzSize()-1);
    res(0,self->system->getqSize()-1) += self->system->evalW()*y(self->system->getzSize()+self->system->getlaSize(),neq[0]-1);
    res(self->system->getzSize(),self->system->getzSize()+self->system->getgdSize()-1) = self->system->evalgd();
    res(self->system->getzSize()+self->system->getgdSize(),neq[0]-1) = self->system->evalg();
  }

  void LSODIIntegrator::adda(int *neq, double *t, double *y_, int *ml, int *mu, double *P_, int *nrowp) {
    auto self=*reinterpret_cast<LSODIIntegrator**>(&neq[1]);
    SqrMat P(*nrowp, P_);
    for(int i=0; i<self->system->getzSize(); i++) P(i,i) += 1;
  }

  void LSODIIntegrator::integrate() {
    res[0] = &LSODIIntegrator::resODE;
    res[1] = &LSODIIntegrator::resDAE2;
    res[2] = &LSODIIntegrator::resGGL;

    debugInit();

    if(odePackInUse)
      throwError("Only one integration with LSODARIntegrator, LSODKRIntegrator and LSODEIntegrator at a time is possible.");
    odePackInUse = true;

    int N;
    if(formalism==DAE2)
      N = system->getzSize()+system->getgdSize();
    else if(formalism==GGL)
      N = system->getzSize()+system->getgdSize()+system->getgSize();
    else
      N = system->getzSize();

    if(not N)
      throwError("(LSODIIntegrator::integrate): dimension of the system must be at least 1");

    int neq[1+sizeof(void*)/sizeof(int)+1];
    neq[0] = N;
    LSODIIntegrator *self=this;
    memcpy(&neq[1], &self, sizeof(void*));

    // Enlarge workspace for state vector so that the integrator can use it (avoids copying of state vector)
    system->resizezParent(N);
    system->updatezRef(system->getzParent());
    if(formalism) {
      system->getlaParent() >> system->getzParent()(system->getzSize(),system->getzSize()+system->getlaSize()-1);
      system->updatelaRef(system->getlaParent());
    }
    // Integrator uses its own workspace for the state derivative
    Vec yd(N);

    if(z0.size()) {
      if(z0.size() != system->getzSize())
        throwError("(LSODIIntegrator::integrate): size of z0 does not match, must be " + toStr(system->getzSize()));
      system->setState(z0);
    }
    else
      system->evalz0();

    double t = tStart;
    double tPlot = min(tEnd, t+dtPlot);

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
        if(aTol.size() != N)
          throwError("(LSODIIntegrator::integrate): size of aTol does not match, must be " + toStr(N));
      }
    }
    else {
      if(aTol.size() == 1)
        iTol = 3;
      else {
        iTol = 4;
        if(aTol.size() != N)
          throwError("(LSODIIntegrator::integrate): size of aTol does not match, must be " + toStr(N));
      }
      if(rTol.size() != N)
        throwError("(LSODIIntegrator::integrate): size of rTol does not match, must be " + toStr(N));
    }

    int itask=1, istate=1, iopt=0;
    int lrWork = 2*(22+9*N+N*N);
    Vec rWork(lrWork);
    rWork(4) = dt0;
    rWork(5) = dtMax;
    rWork(6) = dtMin;
    int liWork = 2*(20+N);
    VecInt iWork(liWork);
    iWork(5) = maxSteps;

    system->setTime(t);
    system->resetUpToDate();
    system->plot();
    yd(0,system->getzSize()-1) = system->evalzd();

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

    while(t<tEnd-epsroot) {
      DLSODI(*res[formalism], adda, 0, neq, system->getzParent()(), yd(), &t, &tPlot, &iTol, rTol(), aTol(), &itask, &istate, &iopt, rWork(), &lrWork, iWork(), &liWork, &MF);
      if(istate==2 or istate==1) {
        system->setTime(t);
        system->resetUpToDate();
        if(formalism) system->setUpdatela(false);
        system->plot();
        if(msgAct(Status))
          msg(Status) << "   t = " <<  t << ",\tdt = "<< rWork(10) << flush;
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1;
        if(plotIntegrationData) integPlot<< t << " " << rWork(10) << " " << time << endl;
        tPlot = min(tEnd, tPlot+dtPlot);

        // check drift
        if(gMax>=0 and system->positionDriftCompensationNeeded(gMax)) { // project both, first positions and then velocities
          system->projectGeneralizedPositions(3);
          system->projectGeneralizedVelocities(3);
          system->resetUpToDate();
          yd(0,system->getzSize()-1) = system->evalzd();
          istate=1;
        }
        else if(gdMax>=0 and system->velocityDriftCompensationNeeded(gdMax)) { // project velicities
          system->projectGeneralizedVelocities(3);
          system->resetUpToDate();
          yd(0,system->getzSize()-1) = system->evalzd();
          istate=1;
        }
      }
      else if(istate<0) throwError("Integrator LSODI failed with istate = "+toString(istate));
    }

    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      ofstream integSum((name + ".sum").c_str());
      integSum << "Integration time: " << time << endl;
      integSum << "Simulation time: " << t << endl;
      integSum << "Integration steps: " << integrationSteps << endl;
      integSum.close();
    }

    odePackInUse = false;
  }

  void LSODIIntegrator::initializeUsingXML(DOMElement *element) {
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
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"minimumStepSize");
    if(e) setMinimumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stepLimit");
    if(e) setStepLimit(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"formalism");
    if(e) {
      string formalismStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(formalismStr=="ODE") formalism=ODE;
      else if(formalismStr=="DAE2") formalism=DAE2;
      else if(formalismStr=="GGL") formalism=GGL;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(E(e)->getText<double>());
  }

}