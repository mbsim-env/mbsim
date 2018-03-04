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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, DASPKIntegrator)

  DASPKIntegrator::Delta DASPKIntegrator::delta[4];

  void DASPKIntegrator::deltaODE(double* t, double* z_, double* zd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    Vec z(ipar[0], z_);
    Vec zd(ipar[0], zd_);
    Vec delta(ipar[0], delta_);
    self->getSystem()->setTime(*t);
    self->getSystem()->resetUpToDate();
    delta = zd - self->system->evalzd();
  }

  void DASPKIntegrator::deltaDAE1(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    Vec y(ipar[0], y_);
    Vec yd(ipar[0], yd_);
    Vec delta(ipar[0], delta_);
    self->getSystem()->setTime(*t);
    self->getSystem()->resetUpToDate();
    self->getSystem()->setUpdatela(false);
    delta(0,self->system->getzSize()-1) = self->system->evalzd() - yd(0,self->system->getzSize()-1);
    delta(self->system->getzSize(),ipar[0]-1) = self->system->evalW().T()*yd(self->system->getqSize(),self->system->getqSize()+self->system->getuSize()-1) + self->system->evalwb();
  }

  void DASPKIntegrator::deltaDAE2(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    Vec y(ipar[0], y_);
    Vec yd(ipar[0], yd_);
    Vec delta(ipar[0], delta_);
    self->getSystem()->setTime(*t);
    self->getSystem()->resetUpToDate();
    self->getSystem()->setUpdatela(false);
    delta(0,self->system->getzSize()-1) = self->system->evalzd() - yd(0,self->system->getzSize()-1);
    delta(self->system->getzSize(),ipar[0]-1) = self->system->evalgd();
  }

  void DASPKIntegrator::deltaGGL(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASPKIntegrator**>(&ipar[1]);
    Vec y(ipar[0], y_);
    Vec yd(ipar[0], yd_);
    Vec delta(ipar[0], delta_);
    self->getSystem()->setTime(*t);
    self->getSystem()->resetUpToDate();
    self->getSystem()->setUpdatela(false);
    delta(0,self->system->getzSize()-1) = self->system->evalzd() - yd(0,self->system->getzSize()-1);
    delta(0,self->system->getqSize()-1) += self->system->evalW()*y(self->system->getzSize()+self->system->getlaSize(),ipar[0]-1);
    delta(self->system->getzSize(),self->system->getzSize()+self->system->getgdSize()-1) = self->system->evalgd();
    delta(self->system->getzSize()+self->system->getgdSize(),ipar[0]-1) = self->system->evalg();
  }

  void DASPKIntegrator::integrate() {
    delta[0] = &DASPKIntegrator::deltaODE;
    delta[1] = &DASPKIntegrator::deltaDAE1;
    delta[2] = &DASPKIntegrator::deltaDAE2;
    delta[3] = &DASPKIntegrator::deltaGGL;

    debugInit();

    int neq;
    if(formalism==DAE1 or formalism==DAE2)
      neq = system->getzSize()+system->getgdSize();
    else if(formalism==GGL)
      neq = system->getzSize()+system->getgdSize()+system->getgSize();
    else
      neq = system->getzSize();

    if(not neq)
      throw MBSimError("(DASPKIntegrator::integrate): dimension of the system must be at least 1");

    double t = tStart;
    double tPlot = min(tEnd,t + dtPlot);

    // Enlarge workspace for state vector so that the integrator can use it (avoids copying of state vector)
    system->resizezParent(neq);
    system->updatezRef(system->getzParent());
    if(formalism) {
      system->getlaParent() >> system->getzParent()(system->getzSize(),system->getzSize()+system->getlaSize()-1);
      system->updatelaRef(system->getlaParent());
    }
    // Integrator uses its own workspace for the state derivative
    Vec yd(neq);

    if(z0.size()) {
      if(z0.size() != system->getzSize())
        throw MBSimError("(DASPKIntegrator::integrate): size of z0 does not match, must be " + toStr(system->getzSize()));
      system->setState(z0);
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
        throw MBSimError("(DASPKIntegrator::integrate): size of aTol does not match, must be " + toStr(neq));
    }
    if(rTol.size() != aTol.size())
      throw MBSimError("(DASPKIntegrator::integrate): size of rTol does not match aTol, must be " + toStr(aTol.size()));

    // info(2) = 0; // solution only at tOut, no intermediate-output
    // info(3) = 0; // integration does not stop at tStop (rWork(0))
    // info(4) = 0; // jacobian is computed internally
    // info(5) = 0; // jacobian is a full matrix
    info(6) = dtMax>0; // set maximum stepsize
    info(7) = dt0>0; // set initial stepsize
    //  info(8) = 0; // maximum order
    // info(9) = 0; // no components are nonnegative
    // info(10) = 0; // initial t, y, yd are consistent
    // info(11) = 0; // direct method is used to solve the linear systems
    // info(12) = 0; // used for Krylov methods (info(11)==1)
    // info(13) = 0; // used, when an initial condition calculation is requested (info(10)>0)
    // info(14) = 0; // used for Krylov methods (info(11)==1)
    info(15) = excludeAlgebraicVariables; // exclude algebraic variables from the error test
    // info(16) = 0; // used, when an initial condition calculation is requested (info(10)>0)
    // info(17) = 0; // no extra printing in initial condition calculation

    double rPar;
    int iPar[1+sizeof(void*)/sizeof(int)+1];
    iPar[0] = neq;
    DASPKIntegrator *self=this;
    memcpy(&iPar[1], &self, sizeof(void*));

    int lWork = 2*(50+9*neq+neq*neq+neq);
    int liWork = 2*(40+neq+neq);
    VecInt iWork(liWork);
    Vec work(lWork);

    work(1) = dtMax; // maximum stepsize
    work(2) = dt0; // initial stepsize
    if(info(15)) {
      for(int i=0; i<system->getzSize(); i++)
        iWork(40+i) = 1; // differential variable
      for(int i=system->getzSize(); i<neq; i++)
        iWork(40+i) = -1; // algebraic variable
    }

    int idid;

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

    while(t<tEnd) {
      DDASPK(*delta[formalism],&neq,&t,system->getState()(),yd(),&tPlot,info(),rTol(),aTol(),&idid,work(),&lWork,iWork(),&liWork,&rPar,iPar,nullptr,nullptr);
      if(idid==3 or idid==2) {
        system->setTime(t);
        system->resetUpToDate();
        system->setzd(yd(0,system->getzSize()-1));
        system->setUpdatezd(false);
        if(formalism) system->setUpdatela(false);
        system->plot();
        if(msgAct(Status))
          msg(Status) << "   t = " <<  t << ",\tdt = "<< work(6) << flush;
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1;
        if(plotIntegrationData) integPlot<< t << " " << work(6) << " " << time << endl;
        tPlot = min(tEnd,tPlot + dtPlot);

        // check drift
        if(gMax>=0 and system->positionDriftCompensationNeeded(gMax)) { // project both, first positions and then velocities
          system->projectGeneralizedPositions(3);
          system->projectGeneralizedVelocities(3);
          system->resetUpToDate();
          yd(0,system->getzSize()-1) = system->evalzd();
          info(0)=0;
        }
        else if(gdMax>=0 and system->velocityDriftCompensationNeeded(gdMax)) { // project velicities
          system->projectGeneralizedVelocities(3);
          system->resetUpToDate();
          yd(0,system->getzSize()-1) = system->evalzd();
          info(0)=0;
        }
      }
      else if(idid<0) throw MBSimError("Integrator DASPK failed with istate = "+toString(idid));
    }

    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      ofstream integSum((name + ".sum").c_str());
      integSum << "Integration time: " << time << endl;
      integSum << "Simulation time: " << t << endl;
      integSum << "Integration steps: " << integrationSteps << endl;
      integSum.close();
    }
  }

  void DASPKIntegrator::initializeUsingXML(DOMElement *element) {
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
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"formalism");
    if(e) {
      string formalismStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(formalismStr=="ODE") formalism=ODE;
      else if(formalismStr=="DAE1") formalism=DAE1;
      else if(formalismStr=="DAE2") formalism=DAE2;
      else if(formalismStr=="GGL") formalism=GGL;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"excludeAlgebraicVariablesFromErrorTest");
    if(e) setExcludeAlgebraicVariablesFromErrorTest(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(E(e)->getText<double>());
  }

}
