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
#include "implicit_euler_integrator.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include <ctime>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, ImplicitEulerIntegrator)

    ImplicitEulerIntegrator::Residuum::Residuum(DynamicSystemSolver *sys_, double dt_) : sys(sys_), dt(dt_) {
      zk = sys->getState();
    }

  Vec ImplicitEulerIntegrator::Residuum::operator()(const Vec &z) {
    Vec res;
    sys->setState(z);
    sys->resetUpToDate();
    res =  z - zk - sys->evalzd()*dt;
    return res;
  } 

  void ImplicitEulerIntegrator::preIntegrate() {
    debugInit();
    assert(dtPlot >= dt);

    system->setTime(tStart);

    if(z0.size()) {
      if(z0.size() != system->getzSize())
        throw MBSimError("(ImplicitEulerIntegrator::integrate): size of z0 does not match, must be " + toStr(system->getzSize()));
      system->setState(z0);
    }
    else
      system->evalz0();

    tPlot = 0.;
    if(plotIntegrationData) integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);
    
    stepPlot =(int) (dtPlot/dt + 0.5);
    assert(fabs(stepPlot*dt - dtPlot) < dt*dt);
    
    step = 0;
    integrationSteps = 0;
    
    s0 = clock();
    time = 0;
  }

  void ImplicitEulerIntegrator::subIntegrate(double tStop) {
    while(system->getTime()<tStop) { // time loop
      integrationSteps++;
      if((step*stepPlot - integrationSteps) < 0) {
        step++;
        system->resetUpToDate();
        system->plot();
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1;
        if(plotIntegrationData) integPlot<< system->getTime() << " " << dt << " " << time << endl;
        if(output) cout << "   t = " << system->getTime() << ",\tdt = "<< dt << "\r"<<flush;
        tPlot += dtPlot;
      }

      Residuum f(system,dt);
      MultiDimNewtonMethod newton(&f);
//      newton.setLinearAlgebra(1);
      system->getTime() += dt;
      system->getState() = newton.solve(system->getState());
      if(newton.getInfo() != 0)
        throw MBSimError("(Eigenanalyzer::computeEigenvalues): computation of equilibrium state failed!");
    }
  }

  void ImplicitEulerIntegrator::postIntegrate() {
    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      ofstream integSum((name + ".sum").c_str());
      integSum << "Integration time: " << time << endl;
      integSum << "Integration steps: " << integrationSteps << endl;
      integSum.close();
    }

    cout.unsetf(ios::scientific);
    cout << endl;
  }

  void ImplicitEulerIntegrator::integrate() {
    preIntegrate();
    subIntegrate(tEnd);
    postIntegrate();
  }

  void ImplicitEulerIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stepSize");
    if(e) setStepSize(E(e)->getText<double>());
  }

}
