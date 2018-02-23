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
#include "radau5_integrator.h"
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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, RADAU5Integrator)

  RADAU5Integrator::Fzdot RADAU5Integrator::fzdot[5];
  RADAU5Integrator::Mass RADAU5Integrator::mass[2];

  void RADAU5Integrator::fzdotODE(int* zSize, double* t, double* z_, double* zd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<RADAU5Integrator**>(&ipar[0]);
    Vec zd(*zSize, zd_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(Vec(*zSize, z_));
    self->getSystem()->resetUpToDate();
    zd = self->getSystem()->evalzd();
  }

  void RADAU5Integrator::fzdotDAE1(int* neq, double* t, double* y_, double* yd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<RADAU5Integrator**>(&ipar[0]);
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

  void RADAU5Integrator::fzdotDAE2(int* neq, double* t, double* y_, double* yd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<RADAU5Integrator**>(&ipar[0]);
    Vec y(*neq, y_);
    Vec yd(*neq, yd_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(y(0,self->system->getzSize()-1));
    self->getSystem()->resetUpToDate();
    self->getSystem()->setla(y(self->system->getzSize(),*neq-1));
    self->getSystem()->setUpdatela(false);
    yd(0,self->system->getzSize()-1) = self->system->evalzd();
    yd(self->system->getzSize(),*neq-1) = self->system->evalgd();
  }

  void RADAU5Integrator::fzdotDAE3(int* neq, double* t, double* y_, double* yd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<RADAU5Integrator**>(&ipar[0]);
    Vec y(*neq, y_);
    Vec yd(*neq, yd_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(y(0,self->system->getzSize()-1));
    self->getSystem()->resetUpToDate();
    self->getSystem()->setla(y(self->system->getzSize(),*neq-1));
    self->getSystem()->setUpdatela(false);
    yd(0,self->system->getzSize()-1) = self->system->evalzd();
    yd(self->system->getzSize(),*neq-1) = self->system->evalg();
  }

  void RADAU5Integrator::fzdotGGL(int* neq, double* t, double* y_, double* yd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<RADAU5Integrator**>(&ipar[0]);
    Vec y(*neq, y_);
    Vec yd(*neq, yd_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(y(0,self->system->getzSize()-1));
    self->getSystem()->resetUpToDate();
    self->getSystem()->setla(y(self->system->getzSize(),self->system->getzSize()+self->system->getlaSize()-1));
    self->getSystem()->setUpdatela(false);
    yd(0,self->system->getzSize()-1) = self->system->evalzd();
    yd(self->system->getzSize(),self->system->getzSize()+self->system->getgdSize()-1) = self->system->evalgd();
    yd(self->system->getzSize()+self->system->getgdSize(),*neq-1) = self->system->evalg();
    if(self->system->getgSize() != self->system->getgdSize()) {
      self->system->calclaSize(5);
      self->system->updateWRef(self->system->getWParent(0)(RangeV(0, self->system->getuSize()-1),RangeV(0,self->system->getlaSize()-1)));
      self->system->setUpdateW(false);
      yd(0,self->system->getqSize()-1) += self->system->evalW()*y(self->system->getzSize()+self->system->getgdSize(),*neq-1);
      self->system->calclaSize(3);
      self->system->updateWRef(self->system->getWParent(0)(RangeV(0, self->system->getuSize()-1),RangeV(0,self->system->getlaSize()-1)));
    }
    else
      yd(0,self->system->getqSize()-1) += self->system->evalW()*y(self->system->getzSize()+self->system->getgdSize(),*neq-1);
  }

  void RADAU5Integrator::massFull(int* zSize, double* m_, int* lmas, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<RADAU5Integrator**>(&ipar[0]);
    Mat M(*lmas,*zSize, m_);
    for(int i=0; i<self->system->getzSize(); i++) M(0,i) = 1;
  }

  void RADAU5Integrator::massReduced(int* zSize, double* m_, int* lmas, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<RADAU5Integrator**>(&ipar[0]);
    Mat M(*lmas,*zSize, m_);
    for(int i=0; i<self->system->getqSize(); i++) M(0,i) = 1;
  }

  void RADAU5Integrator::plot(int* nr, double* told, double* t, double* y, double* cont, int* lrc, int* n, double* rpar, int* ipar, int* irtrn) {
    auto self=*reinterpret_cast<RADAU5Integrator**>(&ipar[0]);

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
            self->getSystem()->getState()(i-1) = CONTR5(&i,&tCheck,cont,lrc);
          self->getSystem()->resetUpToDate();
          if(self->signChangedWRTsvLast(self->getSystem()->evalsv()))
            tRoot = tCheck;
        }
        if(curTimeAndState != tRoot) {
          curTimeAndState = tRoot;
          self->getSystem()->setTime(tRoot);
          for(int i=1; i<=self->system->getzSize(); i++)
            self->getSystem()->getState()(i-1) = CONTR5(&i,&tRoot,cont,lrc);
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
          self->getSystem()->getState()(i-1) = CONTR5(&i,&self->tPlot,cont,lrc);
      }
      self->getSystem()->resetUpToDate();
      if(self->formalism) {
        for(int i=self->system->getzSize()+1; i<=self->system->getzSize()+self->system->getlaSize(); i++)
          self->getSystem()->getla(false)(i-(self->system->getzSize()+1)) = CONTR5(&i,&self->tPlot,cont,lrc);
        self->getSystem()->setUpdatela(false);
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
        for(int i=1; i<=self->getSystem()->getzSize(); i++)
          self->getSystem()->getState()(i-1) = CONTR5(&i,&tRoot,cont,lrc);
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
      self->getSystem()->resetUpToDate();
      self->svLast=self->getSystem()->evalsv();
      *irtrn = -1;
    }
    else {
      // check drift
      if(self->getToleranceForPositionConstraints()>=0) {
        self->getSystem()->setTime(*t);
        self->getSystem()->setState(Vec(self->getSystem()->getzSize(),y));
        self->getSystem()->resetUpToDate();
        if(self->getSystem()->positionDriftCompensationNeeded(self->getToleranceForPositionConstraints())) { // project both, first positions and then velocities
          self->getSystem()->projectGeneralizedPositions(3);
          self->getSystem()->projectGeneralizedVelocities(3);
          *irtrn=-1;
        }
      }
      else if(self->getToleranceForVelocityConstraints()>=0) {
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

  bool RADAU5Integrator::signChangedWRTsvLast(const fmatvec::Vec &svStepEnd) const {
    for(int i=0; i<svStepEnd.size(); i++)
      if(svLast(i)*svStepEnd(i)<0)
        return true;
    return false;
  }

  void RADAU5Integrator::integrate() {
    fzdot[0] = &RADAU5Integrator::fzdotODE;
    fzdot[1] = &RADAU5Integrator::fzdotDAE1;
    fzdot[2] = &RADAU5Integrator::fzdotDAE2;
    fzdot[3] = &RADAU5Integrator::fzdotDAE3;
    fzdot[4] = &RADAU5Integrator::fzdotGGL;
    mass[0] = &RADAU5Integrator::massFull;
    mass[1] = &RADAU5Integrator::massReduced;

    debugInit();

    int zSize = system->getzSize();
    calcSize();

    if(not neq)
      throw MBSimError("(RADAU5Integrator::integrate): dimension of the system must be at least 1");

    if(formalism==DAE3 and system->getgSize()!=system->getgdSize())
      throw MBSimError("(RADAU5Integrator::integrate): size of g (" + toStr(system->getgSize()) + ") must be equal to size of gd (" + toStr(system->getgdSize()) + ") when using the DAE3 formalism");

    double t = tStart;

    Vec y(neq);
    Vec z = y(0,zSize-1);
    if(z0.size()) {
      if(z0.size() != zSize)
        throw MBSimError("(RADAU5Integrator::integrate): size of z0 does not match, must be " + toStr(zSize));
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
        throw MBSimError("(RADAU5Integrator::integrate): size of aTol does not match, must be " + toStr(neq));
    }
    if(rTol.size() != aTol.size())
      throw MBSimError("(RADAU5Integrator::integrate): size of rTol does not match aTol, must be " + toStr(aTol.size()));

    int out = 1; // subroutine is available for output

    double rPar;
    int iPar[sizeof(void*)/sizeof(int)+1];
    RADAU5Integrator *self=this;
    memcpy(&iPar[0], &self, sizeof(void*));

    int lWork = 2*(neq*(neq+neq+3*neq+12)+20);
    int liWork = 2*(3*neq+20);
    iWork.resize(liWork);
    work.resize(lWork);
    if(dtMax>0)
      work(6) = dtMax; // maximum step size
    iWork(1) = maxSteps; // maximum number of steps
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

    cout.setf(ios::scientific, ios::floatfield);

    s0 = clock();

    while(t<=tEnd-dt) {
      RADAU5(&neq,(*fzdot[formalism]),&t,y(),&tEnd,&dt,
          rTol(),aTol(),&iTol,
          nullptr,&iJac,&mlJac,&muJac,
          *mass[reduced],&iMas,&mlMas,&muMas,
          plot,&out,
          work(),&lWork,iWork(),&liWork,&rPar,iPar,&idid);

      if(shift) {
        dt = dt0;
        calcSize();
        reinit();
      }

      t = system->getTime();
      z = system->getState();
      if(formalism)
        y(zSize,neq-1).init(0);
    }

    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      ofstream integSum((name + ".sum").c_str());
      integSum << "Integration time: " << time << endl;
      //integSum << "Integration steps: " << integrationSteps << endl;
      integSum.close();
    }

    cout.unsetf (ios::scientific);
    cout << endl;
  }

  void RADAU5Integrator::calcSize() {
    if(formalism==DAE1 or formalism==DAE2)
      neq = system->getzSize()+system->getlaSize();
    else if(formalism==DAE3)
      neq = system->getzSize()+system->getgSize();
    else if(formalism==GGL)
      neq = system->getzSize()+system->getgdSize()+system->getgSize();
    else
      neq = system->getzSize();
  }

  void RADAU5Integrator::reinit() {
    work(20,work.size()-1).init(0);
    if(formalism==DAE1)
      iWork(4) = system->getzSize() + system->getlaSize();
    else if(formalism==DAE2) {
      iWork(4) = system->getzSize();
      iWork(5) = system->getgdSize();
    }
    else if(formalism==DAE3) {
      iWork(4) = system->getzSize();
      iWork(6) = system->getgSize();
    }
    else if(formalism==GGL) {
      iWork(4) = system->getzSize();
      iWork(5) = system->getgdSize() + system->getgSize();
    }
    if(reduced) {
      iWork(8) = system->getqSize();
      iWork(9) = system->getqSize();
      mlJac = neq - system->getqSize(); // jacobian is a reduced matrix
    }
    else
      mlJac = neq; // jacobian is a full matrix
    muJac = mlJac; // need not to be defined if mlJac = neq
  }

  void RADAU5Integrator::initializeUsingXML(DOMElement *element) {
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
      else if(formalismStr=="DAE2") formalism=DAE2;
      else if(formalismStr=="DAE3") formalism=DAE3;
      else if(formalismStr=="GGL") formalism=GGL;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"reducedForm");
    if(e) setReducedForm((E(e)->getText<bool>()));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"plotOnRoot");
    if(e) setPlotOnRoot(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(E(e)->getText<double>());
  }

}
