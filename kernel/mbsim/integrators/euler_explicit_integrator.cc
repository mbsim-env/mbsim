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
#include "euler_explicit_integrator.h"
#include <ctime>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, EulerExplicitIntegrator)

  EulerExplicitIntegrator::EulerExplicitIntegrator()  {
  }

  void EulerExplicitIntegrator::preIntegrate() {
    debugInit();
    assert(dtPlot >= dt);

    system->setTime(tStart);

    if(z0.size())
      system->setState(z0);
    else
      system->evalz0();

    tPlot = 0.;
    integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);
    
    stepPlot =(int) (dtPlot/dt + 0.5);
    assert(fabs(stepPlot*dt - dtPlot) < dt*dt);
    
    step = 0;
    integrationSteps = 0;
    
    s0 = clock();
    time = 0;
  }

  void EulerExplicitIntegrator::subIntegrate(double tStop) { 
    while(system->getTime()<tStop) { // time loop
      integrationSteps++;
      if((step*stepPlot - integrationSteps) < 0) {
        step++;
        system->resetUpToDate();
        system->plot();
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1;
        integPlot<< system->getTime() << " " << dt << " " << time << endl;
        if(output) cout << "   t = " << system->getTime() << ",\tdt = "<< dt << "\r"<<flush;
        tPlot += dtPlot;
      }

      system->resetUpToDate();
      system->getState() += system->evalzd()*dt;
      system->getTime() += dt;
    }
  }

  void EulerExplicitIntegrator::postIntegrate() {
    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum.close();

    cout.unsetf(ios::scientific);
    cout << endl;
  }

  void EulerExplicitIntegrator::integrate() {
    preIntegrate();
    subIntegrate(tEnd);
    postIntegrate();
  }

  void EulerExplicitIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stepSize");
    if(e) setStepSize(E(e)->getText<double>());
  }

}
