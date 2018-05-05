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
#include "seulex_integrator.h"
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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, SEULEXIntegrator)

  SEULEXIntegrator::Fzdot SEULEXIntegrator::fzdot[2];
  SEULEXIntegrator::Mass SEULEXIntegrator::mass[2];

  void SEULEXIntegrator::fzdotODE(int* zSize, double* t, double* z_, double* zd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<SEULEXIntegrator**>(&ipar[0]);
    Vec zd(*zSize, zd_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(Vec(*zSize, z_));
    self->getSystem()->resetUpToDate();
    zd = self->getSystem()->evalzd();
  }

  void SEULEXIntegrator::fzdotDAE1(int* neq, double* t, double* y_, double* yd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<SEULEXIntegrator**>(&ipar[0]);
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

  void SEULEXIntegrator::massFull(int* zSize, double* m_, int* lmas, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<SEULEXIntegrator**>(&ipar[0]);
    Mat M(*lmas,*zSize, m_);
    for(int i=0; i<self->system->getzSize(); i++) M(0,i) = 1;
  }

  void SEULEXIntegrator::massReduced(int* zSize, double* m_, int* lmas, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<SEULEXIntegrator**>(&ipar[0]);
    Mat M(*lmas,*zSize, m_);
    for(int i=0; i<self->system->getqSize(); i++) M(0,i) = 1;
  }

  void SEULEXIntegrator::plot(int* nr, double* told, double* t, double *y, double *rc, int* lrc, int* ic, int* lic, int* n, double* rpar, int* ipar, int* irtrn) {
    auto self=*reinterpret_cast<SEULEXIntegrator**>(&ipar[0]);

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
        while(dt>1e-10) {
          dt/=2;
          double tCheck = tRoot-dt;
          self->getSystem()->setTime(tCheck);
          curTimeAndState = tCheck;
          for(int i=1; i<=self->system->getzSize(); i++)
            self->getSystem()->getState()(i-1) = CONTSX(&i,&tCheck,rc,lrc,ic,lic);
          self->getSystem()->resetUpToDate();
          if(self->signChangedWRTsvLast(self->getSystem()->evalsv()))
            tRoot = tCheck;
        }
        if(curTimeAndState != tRoot) {
          curTimeAndState = tRoot;
          self->getSystem()->setTime(tRoot);
          for(int i=1; i<=self->system->getzSize(); i++)
            self->getSystem()->getState()(i-1) = CONTSX(&i,&tRoot,rc,lrc,ic,lic);
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
          self->getSystem()->getState()(i-1) = CONTSX(&i,&self->tPlot,rc,lrc,ic,lic);
      }
      self->getSystem()->resetUpToDate();
      if(self->formalism) {
        for(int i=self->system->getzSize()+1; i<=self->system->getzSize()+self->system->getlaSize(); i++)
          self->getSystem()->getla(false)(i-(self->system->getzSize()+1)) = CONTSX(&i,&self->tPlot,rc,lrc,ic,lic);
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
          self->getSystem()->getState()(i-1) = CONTSX(&i,&tRoot,rc,lrc,ic,lic);
      }
      if(self->plotOnRoot) {
        self->getSystem()->resetUpToDate();
        self->getSystem()->plot();
      }
      self->getSystem()->resetUpToDate();
      self->getSystem()->shift();
      if(self->formalism>1) { // DAE2, DAE3 or GGL
        self->system->calcgdSize(3); // IH
        self->system->updategdRef(self->system->getgdParent()(0,self->system->getgdSize()-1));
        if(self->formalism>2) { // DAE3 or GGL
          self->system->calcgSize(2); // IB
          self->system->updategRef(self->system->getgParent()(0,self->system->getgSize()-1));
        }
      }
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

  bool SEULEXIntegrator::signChangedWRTsvLast(const fmatvec::Vec &svStepEnd) const {
    for(int i=0; i<svStepEnd.size(); i++)
      if(svLast(i)*svStepEnd(i)<0)
        return true;
    return false;
  }

  void SEULEXIntegrator::integrate() {
    if(formalism==unknown)
      throwError("(SEULEXIntegrator::integrate): formalism unknown");

    fzdot[0] = &SEULEXIntegrator::fzdotODE;
    fzdot[1] = &SEULEXIntegrator::fzdotDAE1;
    mass[0] = &SEULEXIntegrator::massFull;
    mass[1] = &SEULEXIntegrator::massReduced;

    debugInit();

    calcSize();

    if(not neq)
      throwError("(SEULEXIntegrator::integrate): dimension of the system must be at least 1");

    double t = tStart;

    Vec y(neq);
    Vec z = y(0,system->getzSize()-1);
    if(z0.size()) {
      if(z0.size() != system->getzSize())
        throwError("(SEULEXIntegrator::integrate): size of z0 does not match, must be " + to_string(system->getzSize()));
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
        throwError("(SEULEXIntegrator::integrate): size of aTol does not match, must be " + to_string(neq));
    }
    if(rTol.size() != aTol.size())
      throwError("(SEULEXIntegrator::integrate): size of rTol does not match aTol, must be " + to_string(aTol.size()));

    int out = 2; // dense output is performed in plot

    double rPar;
    int iPar[sizeof(void*)/sizeof(int)+1];
    SEULEXIntegrator *self=this;
    memcpy(&iPar[0], &self, sizeof(void*));
    int lWork = 2*(neq*(neq+neq+neq+20)+4*12+20+2+12*8*neq);
    int liWork = 2*(2*neq+12+20+neq);
    iWork.resize(liWork);
    work.resize(lWork);
    if(dtMax>0)
      work(1) = dtMax; // maximum step size
    iWork(1) = maxSteps; // maximum number of steps
    int ifcn = not autonom;
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
    if(formalism>1) { // DAE2, DAE3 or GGL
      system->calcgdSize(3); // IH
      system->updategdRef(system->getgdParent()(0,system->getgdSize()-1));
      if(formalism>2) { // DAE3 or GGL
        system->calcgSize(2); // IB
        system->updategRef(system->getgParent()(0,system->getgSize()-1));
      }
    }
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
      SEULEX(&neq,(*fzdot[formalism]),&ifcn,&t,y(),&tEnd,&dt,
          rTol(),aTol(),&iTol,
          nullptr,&iJac,&mlJac,&muJac,
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

  void SEULEXIntegrator::calcSize() {
    if(formalism==DAE1)
      neq = system->getzSize()+system->getlaSize();
    else
      neq = system->getzSize();
  }

  void SEULEXIntegrator::reinit() {
    work(20,work.size()-1).init(0);
    iWork(5) = neq; // number of components, for which dense output is required
    for(int i=0; i<neq; i++)
      iWork(20+i) = i+1;
    if(reduced) {
      iWork(8) = system->getqSize();
      iWork(9) = system->getqSize();
      mlJac = neq - system->getqSize(); // jacobian is a reduced matrix
    }
    else
      mlJac = neq; // jacobian is a full matrix
    muJac = mlJac; // need not to be defined if mlJac = neq
  }

  void SEULEXIntegrator::initializeUsingXML(DOMElement *element) {
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
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"plotOnRoot");
    if(e) setPlotOnRoot(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(E(e)->getText<double>());
  }

}
