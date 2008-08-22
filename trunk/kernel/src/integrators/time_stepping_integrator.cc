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
 *   mfoerg@users.berlios.de
 *
 */

#include<config.h>
#include<multi_body_system.h>
#include "time_stepping_integrator.h"

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;

namespace MBSim {

  TimeSteppingIntegrator::TimeSteppingIntegrator() : dt(1e-3), driftCompensation(false) {}

  void TimeSteppingIntegrator::integrate(MultiBodySystem& system) 
  {
    assert(dtPlot >= dt);

    double t = 0.; // starting time without restriction because of flow property of dynamical systems

    int nq = system.getqSize(); // size of positions, velocities, state
    int nu = system.getuSize();
    int nx = system.getxSize();
    int n = nq + nu + nx;

    Index Iq(0,nq-1);
    Index Ix(nq,nq+nx-1);
    Index Iu(nx+nq, n-1);
    Vec z(n);
    Vec q(z(Iq));
    Vec u(z(Iu));
    Vec x(z(Ix));

    if(z0.size()) z = z0; // define initial state
    else system.initz(z);

    double tPlot = 0.;
    ofstream integPlot((system.getDirectoryName() + name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);
    //	int stepPlot =(int) (1./dtPlot);

    int stepPlot =(int) (dtPlot/dt + 0.5);
    assert(fabs(stepPlot*dt - dtPlot) < dt*dt);


    int iter = 0;
    int step = 0;   
    int integrationSteps = 0;
    int maxIter = 0;
    int sumIter = 0;

    double s0 = clock();
    double time = 0;
    while(t<tEnd) {
      integrationSteps++;
      if( (step*stepPlot - integrationSteps) < 0) { // effort can be neglected with factor between dt and dtPlot
	step++;
	system.plot(z,t,dt); // is executed with current data t_i,q_i,u_i
	double s1 = clock();
	time += (s1-s0)/CLOCKS_PER_SEC;
	s0 = s1; 
	integPlot<< t << " " << dt << " " <<  iter << " " << time << " "<<system.getlaSize() <<endl;
	if(output) cout << "   t = " <<  t << ",\tdt = "<< dt << ",\titer = "<<setw(5)<<setiosflags(ios::left) << iter <<  "\r"<<flush;
	tPlot += dtPlot;
      }

      q += system.deltaq(z,t,dt);

      t += dt;

      system.update(z,t); // is executed with t_{i+1},q_{i+1},u_i

      iter = system.solve(dt);

      if(iter>maxIter) maxIter = iter;
      sumIter += iter;

      u += system.deltau(z,t,dt);
      x += system.deltax(z,t,dt);

      if(driftCompensation) system.projectViolatedConstraints(t);
    }

    integPlot.close();

    ofstream integSum((system.getDirectoryName() + name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum << "Maximum number of iterations: " << maxIter << endl;
    integSum << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;
    integSum.close();

    cout.unsetf(ios::scientific);
    cout << endl;
  }

}
