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
#include "daskr_integrator.h"
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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, DASKRIntegrator)

  DASKRIntegrator::Delta DASKRIntegrator::delta[4];

  void DASKRIntegrator::deltaODE(double* t, double* z_, double* zd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASKRIntegrator**>(&ipar[1]);
    Vec z(ipar[0], z_);
    Vec zd(ipar[0], zd_);
    Vec delta(ipar[0], delta_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(z);
    self->getSystem()->resetUpToDate();
    delta = zd - self->system->evalzd();
  }

  void DASKRIntegrator::deltaDAE1(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASKRIntegrator**>(&ipar[1]);
    Vec y(ipar[0], y_);
    Vec yd(ipar[0], yd_);
    Vec delta(ipar[0], delta_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(y(0,self->system->getzSize()-1));
    self->getSystem()->resetUpToDate();
    self->getSystem()->setla(y(self->system->getzSize(),ipar[0]-1));
    self->getSystem()->setUpdatela(false);
    delta(0,self->system->getzSize()-1) = self->system->evalzd() - yd(0,self->system->getzSize()-1);
    delta(self->system->getzSize(),ipar[0]-1) = self->system->evalW().T()*yd(self->system->getqSize(),self->system->getqSize()+self->system->getuSize()-1) + self->system->evalwb();
  }

  void DASKRIntegrator::deltaDAE2(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASKRIntegrator**>(&ipar[1]);
    Vec y(ipar[0], y_);
    Vec yd(ipar[0], yd_);
    Vec delta(ipar[0], delta_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(y(0,self->system->getzSize()-1));
    self->getSystem()->resetUpToDate();
    self->getSystem()->setla(y(self->system->getzSize(),ipar[0]-1));
    self->getSystem()->setUpdatela(false);
    delta(0,self->system->getzSize()-1) = self->system->evalzd() - yd(0,self->system->getzSize()-1);
    delta(self->system->getzSize(),ipar[0]-1) = self->system->evalgd();
  }

  void DASKRIntegrator::deltaGGL(double* t, double* y_, double* yd_, double* cj, double* delta_, int *ires, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASKRIntegrator**>(&ipar[1]);
    Vec y(ipar[0], y_);
    Vec yd(ipar[0], yd_);
    Vec delta(ipar[0], delta_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(y(0,self->system->getzSize()-1));
    self->getSystem()->resetUpToDate();
    self->getSystem()->setla(y(self->system->getzSize(),self->system->getzSize()+self->system->getgdSize()-1));
    self->getSystem()->setUpdatela(false);
    delta(0,self->system->getzSize()-1) = self->system->evalzd() - yd(0,self->system->getzSize()-1);
    delta(0,self->system->getqSize()-1) += self->system->evalW()*y(self->system->getzSize()+self->system->getlaSize(),ipar[0]-1);
    delta(self->system->getzSize(),self->system->getzSize()+self->system->getgdSize()-1) = self->system->evalgd();
    delta(self->system->getzSize()+self->system->getgdSize(),ipar[0]-1) = self->system->evalg();
  }

  void DASKRIntegrator::rt(int* neq, double* t, double* y_, double* yd_, int* nrt, double* rval_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DASKRIntegrator**>(&nrt[1]);
    Vec y(ipar[0], y_);
    Vec yd(ipar[0], yd_);
    Vec rval(*nrt, rval_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(y(0,self->getSystem()->getzSize()-1));
    self->getSystem()->resetUpToDate();
    // The following code would be faster but does not work, maybe due to round-off errors
//    self->getSystem()->setzd(yd(0,self->getSystem()->getzSize()-1));
//    self->getSystem()->setUpdatezd(false);
//    if(self->getFormalism()) {
//      self->getSystem()->setla(y(self->getSystem()->getzSize(),self->getSystem()->getzSize()+self->getSystem()->getlaSize()-1));
//      self->getSystem()->setUpdatela(false);
//    }
    rval = self->getSystem()->evalsv();
  }

  void DASKRIntegrator::integrate() {
    delta[0] = &DASKRIntegrator::deltaODE;
    delta[1] = &DASKRIntegrator::deltaDAE1;
    delta[2] = &DASKRIntegrator::deltaDAE2;
    delta[3] = &DASKRIntegrator::deltaGGL;

    debugInit();

    int zSize = system->getzSize();
    int neq;
    if(formalism==DAE1 or formalism==DAE2)
      neq = zSize+system->getgdSize();
    else if(formalism==GGL) {
      if(system->getgSize() != system->getgdSize())
        throw MBSimError("(DASKRIntegrator::integrate): size of g and gd must be equal for GGL formalism");
      neq = zSize+system->getgdSize()+system->getgSize();
    }
    else
      neq = zSize;

    double t = tStart;
    double tPlot = min(tEnd,t + dtPlot);

    Vec y(neq);
    Vec yd(neq);
    Vec z = y(0,zSize-1);
    if(z0.size()) {
      if(z0.size() != zSize)
        throw MBSimError("(DASKRIntegrator::integrate): size of z0 does not match, must be " + toStr(zSize));
      z = z0;
    }
    else
      z = system->evalz0();

    if(aTol.size() == 0)
      aTol.resize(1,INIT,1e-6);
    if(rTol.size() == 0)
      rTol.resize(1,INIT,1e-6);

    VecInt info(20);

    // info(0) = 0; first call

    if(aTol.size()>1) {
      info(1) = 1; // aTol und rTol are vectors
      if(aTol.size() != neq)
        throw MBSimError("(DASKRIntegrator::integrate): size of aTol does not match, must be " + toStr(neq));
    }
    if(rTol.size() != aTol.size())
      throw MBSimError("(DASKRIntegrator::integrate): size of rTol does not match aTol, must be " + toStr(aTol.size()));

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

    int lWork = 2*(60+9*neq+neq*neq+neq+3*system->getsvSize());
    int liWork = 2*(40+neq+neq);

    system->setTime(t);
    system->setState(y(0,zSize-1));
    system->resetUpToDate();
    system->computeInitialCondition();
    if(formalism>1) { // DAE2 or GGL
      system->calcgdSize(3); // IH
      system->updategdRef(system->getgdParent()(0,system->getgdSize()-1));
      if(formalism==GGL) { // GGL
        system->calcgSize(2); // IB
        system->updategRef(system->getgParent()(0,system->getgSize()-1));
      }
    }
    system->plot();
    yd(0,zSize-1) = system->evalzd();
    if(formalism) { // DAE or GGL
      y(zSize,zSize+system->getlaSize()-1) = system->getla();
      if(formalism<3) // DAE1 or DAE2
        neq = zSize+system->getlaSize();
      else // GGL
        neq = zSize+system->getgdSize()+system->getgSize();
    }

    double rPar;
    int iPar[1+sizeof(void*)/sizeof(int)+1];
    iPar[0] = neq;
    DASKRIntegrator *self=this;
    memcpy(&iPar[1], &self, sizeof(void*));
    int nrt[1+sizeof(void*)/sizeof(int)+1];
    nrt[0] = system->getsvSize();
    memcpy(&nrt[1], &self, sizeof(void*));

    VecInt iWork(liWork);
    Vec work(lWork);

    work(1) = dtMax; // maximum stepsize
    work(2) = dt0; // initial stepsize
    if(info(15)) {
      for(int i=0; i<zSize; i++)
        iWork(40+i) = 1; // differential variable
      for(int i=zSize; i<neq; i++)
        iWork(40+i) = -1; // algebraic variable
    }

    int idid;

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

    cout.setf(ios::scientific, ios::floatfield);
    while(t<tEnd) {
      DDASKR(*delta[formalism],&neq,&t,y(),yd(),&tPlot,info(),rTol(),aTol(),&idid,work(),&lWork,iWork(),&liWork,&rPar,iPar,nullptr,nullptr,rt,nrt,system->getjsv()());
      if(idid==3 || fabs(t-tPlot)<epsroot) {
        system->setTime(t);
        system->setState(y(0,zSize-1));
        system->resetUpToDate();
        system->setzd(yd(0,zSize-1));
        system->setUpdatezd(false);
        if(formalism) {
          system->setla(y(zSize,zSize+system->getlaSize()-1));
          system->setUpdatela(false);
        }
        system->plot();
        if(output)
          cout << "   t = " <<  t << ",\tdt = "<< work(6) << "\r"<<flush;
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1;
        if(plotIntegrationData) integPlot<< t << " " << work(6) << " " << time << endl;
        tPlot = min(tEnd,tPlot + dtPlot);

        // check drift
        if(gMax>=0 and system->positionDriftCompensationNeeded(gMax)) { // project both, first positions and then velocities
          system->projectGeneralizedPositions(3);
          system->projectGeneralizedVelocities(3);
//          info(0)=1;
        }
        else if(gdMax>=0 and system->velocityDriftCompensationNeeded(gdMax)) { // project velicities
          system->projectGeneralizedVelocities(3);
//          info(0)=1;
        }
      }
      if(idid==5) {
        if(plotOnRoot) { // plot before shifting
          system->setTime(t);
          system->setState(y(0,zSize-1));
          system->resetUpToDate();
          system->setzd(yd(0,zSize-1));
          system->setUpdatezd(false);
          if(formalism) {
            system->setla(y(zSize,zSize+system->getlaSize()-1));
            system->setUpdatela(false);
          }
          system->plot();
          system->plotAtSpecialEvent();
        }
        system->setTime(t);
        system->setState(y(0,zSize-1));
//        system->setjsv(jsv);
        system->resetUpToDate();
        system->shift();
        if(formalism>1) { // DAE2 or GGL
          system->calcgdSize(3); // IH
          system->updategdRef(system->getgdParent()(0,system->getgdSize()-1));
          if(formalism==GGL) { // GGL
            system->calcgSize(2); // IB
            system->updategRef(system->getgParent()(0,system->getgSize()-1));
          }
        }
        y(0,zSize-1) = system->getState();
        system->resetUpToDate();
        yd(0,zSize-1) = system->evalzd();
        if(formalism) {
          y(zSize,zSize+system->getlaSize()-1) = system->getla();
          if(formalism<3) // DAE1 or DAE2
            neq = zSize+system->getlaSize();
          else // GGL
            neq = zSize+system->getgdSize()+system->getgSize();
        }
        if(plotOnRoot) { // plot after shifting
          system->setTime(t);
          system->plot();
          system->plotAtSpecialEvent();
        }
        iPar[0] = neq;
        info(0)=0;
        work(2)=dt0;
        for(int i=zSize; i<neq; i++)
          iWork(40+i) = -1; // algebraic variable
      }
      if(idid<0) throw MBSimError("Integrator DASKR failed with istate = "+toString(idid));
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
  }

  void DASKRIntegrator::initializeUsingXML(DOMElement *element) {
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
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"plotOnRoot");
    if(e) setPlotOnRoot(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(E(e)->getText<double>());
  }

}
