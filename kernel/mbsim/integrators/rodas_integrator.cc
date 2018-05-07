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
#include <fstream>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, RODASIntegrator)

  RODASIntegrator::Fzdot RODASIntegrator::fzdot[2];
  RODASIntegrator::Mass RODASIntegrator::mass[2];

  void RODASIntegrator::fzdotODE(int* zSize, double* t, double* z_, double* zd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<RODASIntegrator**>(&ipar[0]);
    Vec zd(*zSize, zd_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(Vec(*zSize, z_));
    self->getSystem()->resetUpToDate();
    zd = self->getSystem()->evalzd();
  }

  void RODASIntegrator::fzdotDAE1(int* neq, double* t, double* y_, double* yd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<RODASIntegrator**>(&ipar[0]);
    Vec y(*neq, y_);
    Vec yd(*neq, yd_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(y(0,self->system->getzSize()-1));
    self->getSystem()->resetUpToDate();
    self->getSystem()->setla(y(self->system->getzSize(),*neq-1));
    self->getSystem()->setUpdatela(false);
    yd(0,self->system->getzSize()-1) = self->system->evalzd();
    yd(self->system->getzSize(),*neq-1) = self->system->evalW().T()*yd(self->system->getqSize(),self->system->getqSize()+self->system->getuSize()-1) + self->system->evalwb();
  }

  void RODASIntegrator::massFull(int* zSize, double* m_, int* lmas, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<RODASIntegrator**>(&ipar[0]);
    Mat M(*lmas,*zSize, m_);
    for(int i=0; i<self->system->getzSize(); i++) M(0,i) = 1;
  }

  void RODASIntegrator::massReduced(int* zSize, double* m_, int* lmas, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<RODASIntegrator**>(&ipar[0]);
    Mat M(*lmas,*zSize, m_);
    for(int i=0; i<self->system->getqSize(); i++) M(0,i) = 1;
  }

  void RODASIntegrator::plot(int* nr, double* told, double* t, double* y, double* cont, int* lrc, int* n, double* rpar, int* ipar, int* irtrn) {
    auto self=*reinterpret_cast<RODASIntegrator**>(&ipar[0]);

    double curTimeAndState = -1;
    double tRoot = *t;
    // root-finding
    if(self->getSystem()->getsvSize()) {
      self->getSystem()->setTime(*t);
      curTimeAndState = *t;
      self->getSystem()->setState(Vec(self->getSystem()->getzSize(),y));
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
          for(int i=1; i<=self->system->getzSize(); i++)
            self->getSystem()->getState()(i-1) = CONTRO(&i,&tCheck,cont,lrc);
          self->getSystem()->resetUpToDate();
          if(self->signChangedWRTsvLast(self->getSystem()->evalsv()))
            tRoot = tCheck;
        }
        if(curTimeAndState != tRoot) {
          curTimeAndState = tRoot;
          self->getSystem()->setTime(tRoot);
          for(int i=1; i<=self->system->getzSize(); i++)
            self->getSystem()->getState()(i-1) = CONTRO(&i,&tRoot,cont,lrc);
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
        for(int i=1; i<=self->system->getzSize(); i++)
          self->getSystem()->getState()(i-1) = CONTRO(&i,&self->tPlot,cont,lrc);
      }
      self->getSystem()->resetUpToDate();
      if(self->formalism) {
        for(int i=self->system->getzSize()+1; i<=self->system->getzSize()+self->system->getlaSize(); i++)
          self->getSystem()->getla(false)(i-(self->system->getzSize()+1)) = CONTRO(&i,&self->tPlot,cont,lrc);
        self->getSystem()->setUpdatela(false);
      }
      self->getSystem()->plot();
      if(self->msgAct(Status))
	self->msg(Status) << "   t = " <<  self->tPlot << ",\tdt = "<< *t-*told << flush;

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
        for(int i=1; i<=self->getSystem()->getzSize(); i++)
          self->getSystem()->getState()(i-1) = CONTRO(&i,&tRoot,cont,lrc);
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
        self->getSystem()->setState(Vec(self->getSystem()->getzSize(),y));
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
        self->getSystem()->setState(Vec(self->getSystem()->getzSize(),y));
        self->getSystem()->resetUpToDate();
        if(self->getSystem()->velocityDriftCompensationNeeded(self->getToleranceForVelocityConstraints())) { // project velicities
          self->getSystem()->projectGeneralizedVelocities(3);
          *irtrn=-1;
        }
      }
    }
  }

  void RODASIntegrator::integrate() {
    if(formalism==unknown)
      throwError("(RODASIntegrator::integrate): formalism unknown");

    fzdot[0] = &RODASIntegrator::fzdotODE;
    fzdot[1] = &RODASIntegrator::fzdotDAE1;
    mass[0] = &RODASIntegrator::massFull;
    mass[1] = &RODASIntegrator::massReduced;

    debugInit();

    calcSize();

    if(not neq)
      throwError("(RODASIntegrator::integrate): dimension of the system must be at least 1");

    double t = tStart;

    Vec y(neq);
    Vec z = y(0,system->getzSize()-1);
    if(z0.size()) {
      if(z0.size() != system->getzSize())
        throwError("(RODASIntegrator::integrate): size of z0 does not match, must be " + to_string(system->getzSize()));
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
      if(aTol.size() != neq)
        throwError("(RODASIntegrator::integrate): size of aTol does not match, must be " + to_string(neq));
    }
    if(rTol.size() != aTol.size())
      throwError("(RODASIntegrator::integrate): size of rTol does not match aTol, must be " + to_string(aTol.size()));

    int out = 1; // subroutine is available for output

    double rPar;
    int iPar[sizeof(void*)/sizeof(int)+1];
    RODASIntegrator *self=this;
    memcpy(&iPar[0], &self, sizeof(void*));

    int lWork = 2*(neq*(neq+neq+neq+14)+20);
    int liWork = 2*(neq+20);
    iWork.resize(liWork);
    work.resize(lWork);
    if(dtMax>0)
      work(1) = dtMax; // maximum step size
    iWork(0) = maxSteps; // maximum number of steps
    int ifcn = not autonom;
    int idfx = 0;
    int iMas = formalism>0; // mass-matrix
    int mlMas = 0; // lower bandwith of the mass-matrix
    int muMas = 0; // upper bandwith of the mass-matrix
    int iJac = 0; // jacobian is computed internally by finite differences
    int idid;

    double dt = dt0;

    tPlot = t + dtPlot;
    dtOut = dtPlot;

    system->setTime(t);
    system->setState(z);
    system->resetUpToDate();
    system->computeInitialCondition();
    system->plot();
    svLast = system->evalsv();

    calcSize();
    reinit();

    if(plotIntegrationData) {
      integPlot.open((name + ".plt").c_str());

      integPlot << "#1 t [s]:" << endl;
      integPlot << "#1 dt [s]:" << endl;
      integPlot << "#1 calculation time [s]:" << endl;
    }

    s0 = clock();

    while(t<tEnd-epsroot) {
      RODAS(&neq,(*fzdot[formalism]),&ifcn,&t,y(),&tEnd,&dt,
          rTol(),aTol(),&iTol,
          nullptr,&iJac,&mlJac,&muJac,nullptr,&idfx,
          *mass[reduced],&iMas,&mlMas,&muMas,
          plot,&out,
          work(),&lWork,iWork(),&liWork,&rPar,iPar,&idid);

      if(shift) {
        self->getSystem()->resetUpToDate();
        self->svLast=self->getSystem()->evalsv();
        dt = dt0;
        calcSize();
        reinit();
      }

      t = system->getTime();
      z = system->getState();
      if(formalism)
        y(system->getzSize(),neq-1).init(0);
    }

    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      ofstream integSum((name + ".sum").c_str());
      integSum << "Integration time: " << time << endl;
      //integSum << "Integration steps: " << integrationSteps << endl;
      integSum.close();
    }
  }

  void RODASIntegrator::calcSize() {
    if(formalism==DAE1)
      neq = system->getzSize()+system->getlaSize();
    else
      neq = system->getzSize();
  }

  void RODASIntegrator::reinit() {
    work(20,work.size()-1).init(0);
    if(reduced) {
      iWork(8) = system->getqSize();
      iWork(9) = system->getqSize();
      mlJac = neq - system->getqSize(); // jacobian is a reduced matrix
    }
    else
      mlJac = neq; // jacobian is a full matrix
    muJac = mlJac; // need not to be defined if mlJac = neq
  }

  void RODASIntegrator::initializeUsingXML(DOMElement *element) {
    RootFindingIntegrator::initializeUsingXML(element);
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
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"formalism");
    if(e) {
      string formalismStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(formalismStr=="ODE") formalism=ODE;
      else if(formalismStr=="DAE1") formalism=DAE1;
      else formalism=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"reducedForm");
    if(e) setReducedForm((E(e)->getText<bool>()));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"autonomousSystem");
    if(e) setAutonomousSystem((E(e)->getText<bool>()));
  }

}
