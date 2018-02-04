/* Copyright (C) 2004-2009 MBSim Development Team
 *
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/utils/eps.h>
#include <mbsim/utils/utils.h>
#include "fortran/fortran_wrapper.h"
#include "lsodar_integrator.h"
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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, LSODARIntegrator)

  void LSODARIntegrator::fzdot(int* neq, double* t, double* z_, double* zd_) {
    auto self=*reinterpret_cast<LSODARIntegrator**>(&neq[1]);
    Vec zd(neq[0], zd_);
    self->getSystem()->setTime(*t);
//    self->getSystem()->setState(z); Not needed as the integrator uses the state of the system
    self->getSystem()->resetUpToDate();
    zd = self->getSystem()->evalzd();
  }

  void LSODARIntegrator::fsv(int* neq, double* t, double* z_, int* nsv, double* sv_) {
    auto self=*reinterpret_cast<LSODARIntegrator**>(&neq[1]);
    Vec sv(*nsv, sv_);
    self->getSystem()->setTime(*t);
//    self->getSystem()->setState(z); Not needed as the integrator uses the state of the system
    self->getSystem()->resetUpToDate();
    sv = self->getSystem()->evalsv();
  }

  void LSODARIntegrator::initializeUsingXML(DOMElement *element) {
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
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"minimumStepSize");
    if(e) setMinimumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"maximumStepSize");
    if(e) setMaximumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"plotOnRoot");
    if(e) setPlotOnRoot(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(E(e)->getText<double>());
  }

  void LSODARIntegrator::integrate() {
    preIntegrate();
    subIntegrate(tEnd);
    postIntegrate();
  }

  void LSODARIntegrator::preIntegrate() {
    debugInit();

    if(odePackInUse)
      throw MBSimError("Only one integration with LSODARIntegrator, LSODERIntegrator and LSODEIntegrator at a time is possible.");
    odePackInUse = true;

    int zSize=system->getzSize();
    neq[0]=zSize;
    LSODARIntegrator *self=this;
    memcpy(&neq[1], &self, sizeof(void*));

    if(z0.size()) {
      if(z0.size() != zSize)
        throw MBSimError("(LSODEIntegrator::integrate): size of z0 does not match");
      system->setState(z0);
    }
    else
      system->evalz0();
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->computeInitialCondition();
    t=tStart;
    tPlot=t+dtPlot;
    if(aTol.size() == 0) 
      aTol.resize(1,INIT,1e-6);
    if(aTol.size() == 1) {
      iTol = 1; // Skalar
    } else {
      iTol = 2; // Vektor
      assert (aTol.size() >= zSize);
    }
    istate=1;
    nsv=system->getsvSize();
    lrWork = (22 + zSize * max(16, zSize + 9) + 3 * nsv) * 2;
    rWork.resize(lrWork);
    rWork(4) = dt0; 
    rWork(5) = dtMax;
    rWork(6) = dtMin;
    liWork = (20+zSize)*10;
    iWork.resize(liWork);
    iWork(5) = 10000;
    s0 = clock();
    time = 0;
    integrationSteps = 0;
    if(plotIntegrationData) integPlot.open((name + ".plt").c_str());

    // plot initial state
    system->resetUpToDate();
    system->plot();
  }

  void LSODARIntegrator::subIntegrate(double tStop) {
    int one = 1;
    int two = 2;
    rWork(4) = dt0;
    system->setTime(t);
//    system->setState(z); Not needed as the integrator uses the state of the system
    while(t < tStop-epsroot) {  
      integrationSteps++;
      double tOut = min(tPlot, tStop);
      DLSODAR(fzdot, neq, system->getState()(), &t, &tOut, &iTol, &rTol, aTol(), &one,
          &istate, &one, rWork(), &lrWork, iWork(),
          &liWork, NULL, &two, fsv, &nsv, system->getjsv()());
      if(istate==2 || fabs(t-tPlot)<epsroot) {
        system->setTime(t);
//        system->setState(z); Not needed as the integrator uses the state of the system
        system->resetUpToDate();
        system->plot();
        if(output)
          msg(Info) << "   t = " <<  t << ",\tdt = "<< rWork(10) << "\r"<<flush;
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        if(plotIntegrationData) integPlot<< t << " " << rWork(10) << " " << time << endl;
        tPlot += dtPlot;
//        if (tPlot > tStop)
//          tPlot = tStop;

        // check drift
        if(system->positionDriftCompensationNeeded(gMax)) { // project both, first positions and then velocities
          system->projectGeneralizedPositions(3);
          system->projectGeneralizedVelocities(3);
          istate=1;
        }
        else if(system->velocityDriftCompensationNeeded(gdMax)) { // project velicities
          system->projectGeneralizedVelocities(3);
          istate=1;
        }
      }
      if(istate==3) {
        if(plotOnRoot) { // plot before shifting
          system->setTime(t);
//          system->setState(z); Not needed as the integrator uses the state of the system
          system->resetUpToDate();
          system->plot();
          system->plotAtSpecialEvent();
        }
        system->setTime(t);
//        system->setState(z); Not needed as the integrator uses the state of the system
//        system->setjsv(jsv);
        system->resetUpToDate();
        system->shift();
        if(plotOnRoot) { // plot after shifting
          system->setTime(t);
//          system->setState(z); Not needed as the integrator uses the state of the system
          system->resetUpToDate();
          system->plot();
          system->plotAtSpecialEvent();
        }
        istate=1;
        rWork(4)=dt0;
      }
      if(istate<0) throw MBSimError("Integrator LSODAR failed with istate = "+toString(istate));
    }
  }

  void LSODARIntegrator::postIntegrate() {
    system->setTime(t);
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->resetUpToDate();
    system->plot();
    system->plotAtSpecialEvent();
    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      ofstream integSum((name + ".sum").c_str());
      integSum << "Integration time: " << time << endl;
      integSum << "Integration steps: " << integrationSteps << endl;
      integSum.close();
    }

    msg(Info) << endl;

    odePackInUse = false;
  }

}
