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
#include "lsoda_integrator.h"
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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, LSODAIntegrator)

  void LSODAIntegrator::fzdot(int* neq, double* t, double* z_, double* zd_) {
    auto self=*reinterpret_cast<LSODAIntegrator**>(&neq[1]);
    Vec zd(neq[0], zd_);
    self->getSystem()->setTime(*t);
//    self->getSystem()->setState(Vec(neq[0], z_)); Not needed as the integrator uses the state of the system
    self->getSystem()->resetUpToDate();
    zd = self->getSystem()->evalzd();
  }

  void LSODAIntegrator::integrate() {
    debugInit();

    if(odePackInUse)
      throw MBSimError("Only one integration with LSODARIntegrator, LSODERIntegrator and LSODEIntegrator at a time is possible.");
    odePackInUse = true;

    int zSize=system->getzSize();

    if(not zSize)
      throw MBSimError("(LSODAIntegrator::integrate): dimension of the system must be at least 1");

    int neq[1+sizeof(void*)/sizeof(int)+1];
    neq[0]=zSize;
    LSODAIntegrator *self=this;
    memcpy(&neq[1], &self, sizeof(void*));

    if(z0.size()) {
      if(z0.size() != zSize)
        throw MBSimError("(LSODAIntegrator::integrate): size of z0 does not match, must be " + toStr(zSize));
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
          throw MBSimError("(LSODAIntegrator::integrate): size of aTol does not match, must be " + toStr(zSize));
      }
    }
    else {
      if(aTol.size() == 1)
        iTol = 3;
      else {
        iTol = 4;
        if(aTol.size() != zSize)
          throw MBSimError("(LSODAIntegrator::integrate): size of aTol does not match, must be " + toStr(zSize));
      }
      if(rTol.size() != zSize)
        throw MBSimError("(LSODAIntegrator::integrate): size of rTol does not match, must be " + toStr(zSize));
    }

    int one=1, two=2, istate=1;
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
    system->plot();

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
    while(t<tEnd-epsroot) {
      DLSODA(fzdot, neq, system->getState()(), &t, &tPlot, &iTol, rTol(), aTol(), &one,
          &istate, &one, rWork(), &lrWork, iWork(), &liWork, NULL, &two);
      if(istate==2 or istate==1) {
        system->setTime(t);
//        system->setState(z); Not needed as the integrator uses the state of the system
        system->resetUpToDate();
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
          istate=1;
        }
        else if(gdMax>=0 and system->velocityDriftCompensationNeeded(gdMax)) { // project velicities
          system->projectGeneralizedVelocities(3);
          istate=1;
        }
      }
      else if(istate<0) throw MBSimError("Integrator LSODA failed with istate = "+toString(istate));
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

  void LSODAIntegrator::initializeUsingXML(DOMElement *element) {
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
   e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(E(e)->getText<double>());
  }

}
