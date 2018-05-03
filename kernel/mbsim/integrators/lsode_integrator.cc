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
#include "lsode_integrator.h"
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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, LSODEIntegrator)

  bool odePackInUse = false;

  void LSODEIntegrator::fzdot(int* neq, double* t, double* z_, double* zd_) {
    auto self=*reinterpret_cast<LSODEIntegrator**>(&neq[1]);
    Vec zd(neq[0], zd_);
    self->getSystem()->setTime(*t);
//    self->getSystem()->setState(Vec(neq[0], z_)); Not needed as the integrator uses the state of the system
    self->getSystem()->resetUpToDate();
    zd = self->getSystem()->evalzd();
  }

  bool LSODEIntegrator::signChangedWRTsvLast(const fmatvec::Vec &svStepEnd) const {
    for(int i=0; i<svStepEnd.size(); i++)
      if(svLast(i)*svStepEnd(i)<0)
        return true;
    return false;
  }

  void LSODEIntegrator::integrate() {
    if(method==unknown)
      throwError("(LSODEIntegrator::integrate): method unknown");

    debugInit();

    if(odePackInUse)
      throwError("Only one integration with LSODARIntegrator, LSODERIntegrator and LSODEIntegrator at a time is possible.");
    odePackInUse = true;

    int zSize=system->getzSize();

    if(not zSize)
      throwError("(LSODEIntegrator::integrate): dimension of the system must be at least 1");

    int neq[1+sizeof(void*)/sizeof(int)+1];
    neq[0]=zSize;
    LSODEIntegrator *self=this;
    memcpy(&neq[1], &self, sizeof(void*));

    if(z0.size()) {
      if(z0.size() != zSize)
        throwError("(LSODEIntegrator::integrate): size of z0 does not match, must be " + to_string(zSize));
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
        if(aTol.size() != zSize)
          throwError("(LSODEIntegrator::integrate): size of aTol does not match, must be " + to_string(zSize));
      }
    }
    else {
      if(aTol.size() == 1)
        iTol = 3;
      else {
        iTol = 4;
        if(aTol.size() != zSize)
          throwError("(LSODEIntegrator::integrate): size of aTol does not match, must be " + to_string(zSize));
      }
      if(rTol.size() != zSize)
        throwError("(LSODEIntegrator::integrate): size of rTol does not match, must be " + to_string(zSize));
    }

    int itask=2, iopt=1, istate=1;
    int lrWork = 2*(22+9*zSize+zSize*zSize);
    Vec rWork(lrWork);
    rWork(4) = dt0;
    rWork(5) = dtMax;
    rWork(6) = dtMin;
    int liWork = 2*(20+zSize);
    VecInt iWork(liWork);
    iWork(5) = maxSteps;

    system->setTime(t);
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->resetUpToDate();
    system->computeInitialCondition();
    system->plot();
    svLast = system->evalsv();

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

    int MF = method;

    int zero = 0;
    int iflag;

    while(t<tEnd-epsroot) {
      DLSODE(fzdot, neq, system->getState()(), &t, &tEnd, &iTol, rTol(), aTol(),
          &itask, &istate, &iopt, rWork(), &lrWork, iWork(), &liWork, 0, &MF);
      if(istate==2 or istate==1) {
        double curTimeAndState = -1;
        double tRoot = t;
        // root-finding
        if(getSystem()->getsvSize()) {
          getSystem()->setTime(t);
          curTimeAndState = t;
//          getSystem()->setState(z);
          getSystem()->resetUpToDate();
          shift = signChangedWRTsvLast(getSystem()->evalsv());
          // if a root exists in the current step ...
          double dt = rWork(10);
          if(shift) {
            // ... search the first root and set step.second to this time
            while(dt>1e-10) {
              dt/=2;
              double tCheck = tRoot-dt;
              curTimeAndState = tCheck;
              DINTDY (&tCheck, &zero, &rWork(20), neq, system->getState()(), &iflag);
              getSystem()->setTime(tCheck);
//              getSystem()->setState(zWant);
              getSystem()->resetUpToDate();
              if(signChangedWRTsvLast(getSystem()->evalsv()))
                tRoot = tCheck;
            }
            if(curTimeAndState != tRoot) {
              curTimeAndState = tRoot;
              DINTDY (&tRoot, &zero, &rWork(20), neq, system->getState()(), &iflag);
              getSystem()->setTime(tRoot);
//              getSystem()->setState(zWant);
            }
            getSystem()->resetUpToDate();
            auto &sv = getSystem()->evalsv();
            auto &jsv = getSystem()->getjsv();
            for(int i=0; i<sv.size(); ++i)
              jsv(i)=svLast(i)*sv(i)<0;
          }
        }

        while(tRoot >= tPlot and tPlot<=tEnd) {
          if(curTimeAndState != tPlot) {
            curTimeAndState = tPlot;
            DINTDY (&tPlot, &zero, &rWork(20), neq, system->getState()(), &iflag);
            getSystem()->setTime(tPlot);
//            getSystem()->setState(zWant);
          }
          getSystem()->resetUpToDate();
          getSystem()->plot();
          if(msgAct(Status))
            msg(Status) << "   t = " <<  tPlot << ",\tdt = "<< rWork(10) << flush;

          double s1 = clock();
          time += (s1-s0)/CLOCKS_PER_SEC;
          s0 = s1;

          if(plotIntegrationData) integPlot<< tPlot << " " << rWork(10) << " " << time << endl;
          tPlot += dtPlot;
        }

        if(shift) {
          // shift the system
          if(curTimeAndState != tRoot) {
            DINTDY (&tRoot, &zero, &rWork(20), neq, system->getState()(), &iflag);
            getSystem()->setTime(tRoot);
//            getSystem()->setState(zWant);
          }
          if(plotOnRoot) {
            getSystem()->resetUpToDate();
            getSystem()->plot();
          }
          getSystem()->resetUpToDate();
          getSystem()->shift();
          if(plotOnRoot) {
            getSystem()->resetUpToDate();
            getSystem()->plot();
          }
          getSystem()->resetUpToDate();
          svLast=getSystem()->evalsv();
          istate=1;
        }
        else {
          // check drift
          bool projVel = true;
          if(gMax>=0) {
            getSystem()->setTime(t);
//            getSystem()->setState(z);
            getSystem()->resetUpToDate();
            if(getSystem()->positionDriftCompensationNeeded(gMax)) { // project both, first positions and then velocities
              getSystem()->projectGeneralizedPositions(3);
              getSystem()->projectGeneralizedVelocities(3);
              projVel = false;
              istate=1;
            }
          }
          if(gdMax>=0 and projVel) {
            getSystem()->setTime(t);
//            getSystem()->setState(z);
            getSystem()->resetUpToDate();
            if(getSystem()->velocityDriftCompensationNeeded(gdMax)) { // project velicities
              getSystem()->projectGeneralizedVelocities(3);
              istate=1;
            }
          }
        }
        if(istate==1) t = system->getTime();
      }
      else if(istate<0) throwError("Integrator LSODE failed with istate = "+to_string(istate));
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

  void LSODEIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"method");
    if(e) {
      string methodStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(methodStr=="nonstiff" or methodStr=="Adams") method=nonstiff;
      else if(methodStr=="stiff" or methodStr=="BDF") method=stiff;
      else method=unknown;
    }
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
   e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(E(e)->getText<double>());
  }

}
