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
 * Contact: mfoerg@users.berlios.de
 */

#include<config.h>
#include<mbsim/dynamic_system_solver.h>
#include "time_stepping_integrator.h"

#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;

namespace MBSim {

  TimeSteppingIntegrator::TimeSteppingIntegrator() : dt(1e-3), driftCompensation(false) {}

  void TimeSteppingIntegrator::preIntegrate(DynamicSystemSolver& system) {
    // initialisation
    assert(dtPlot >= dt);

    t = tStart;

    int nq = system.getqSize(); // size of positions, velocities, state
    int nu = system.getuSize();
    int nx = system.getxSize();
    int n = nq + nu + nx;

    Index Iq(0,nq-1);
    Index Iu(nq,nq+nu-1);
    Index Ix(nq+nu,n-1);
    z.resize(n);
    q>>z(Iq);
    u>>z(Iu);
    x>>z(Ix);

    if(z0.size()) z = z0; // define initial state
    else system.initz(z);

    tPlot = 0.;
    integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);

    stepPlot =(int) (dtPlot/dt + 0.5);
    assert(fabs(stepPlot*dt - dtPlot) < dt*dt);

    iter = 0;
    step = 0;   
    integrationSteps = 0;
    maxIter = 0;
    sumIter = 0;

    s0 = clock();
    time = 0;
  }

  void TimeSteppingIntegrator::subIntegrate(DynamicSystemSolver& system, double tStop) {
    while(t<tStop) { // time loop
      integrationSteps++;
      if((step*stepPlot - integrationSteps) < 0) {
        step++;
        if(driftCompensation) system.projectGeneralizedPositions(t);
        system.plot(z,t,dt);
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        integPlot<< t << " " << dt << " " <<  iter << " " << time << " "<<system.getlaSize() <<endl;
        if(output) cout << "   t = " <<  t << ",\tdt = "<< dt << ",\titer = "<<setw(5)<<setiosflags(ios::left) << iter <<  "\r"<<flush;
        tPlot += dtPlot;
      }

      q += system.deltaq(z,t,dt);

      t += dt;

      system.update(z,t); 

      system.getb().resize() = system.getgd() + system.getW().T()*slvLLFac(system.getLLM(),system.geth())*dt;
      iter = system.solveImpacts(dt);

      if(iter>maxIter) maxIter = iter;
      sumIter += iter;

      u += system.deltau(z,t,dt);
      x += system.deltax(z,t,dt);
    }
  }

  void TimeSteppingIntegrator::postIntegrate(DynamicSystemSolver& system) {
    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum << "Maximum number of iterations: " << maxIter << endl;
    integSum << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;
    integSum.close();

    cout.unsetf(ios::scientific);
    cout << endl;
  }


  void TimeSteppingIntegrator::integrate(DynamicSystemSolver& system) {
    preIntegrate(system);
    subIntegrate(system, tEnd);
    postIntegrate(system);
  }

  void TimeSteppingIntegrator::initializeUsingXML(TiXmlElement *element) {
    Integrator::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMINTNS"stepSize");
    setStepSize(Element::getDouble(e));
  }

}

