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
#include<mbsim/multi_body_system.h>
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

    double t = 0.;

    int nq = system.getqSize(); // size of positions, velocities, state
    int nu = system.getuSize();
    int nx = system.getxSize();
    int n = nq + nu + nx;

    Index Iq(0,nq-1);
    Index Iu(nq,nq+nu-1);
    Index Ix(nq+nu,n-1);
    Vec z(n);
    Vec q(z(Iq));
    Vec u(z(Iu));
    Vec x(z(Ix));

    if(z0.size()) z = z0; // define initial state
    else system.initz(z);

    double tPlot = 0.;
    ofstream integPlot((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);
//	int stepPlot =(int) (1./dtPlot);

    int stepPlot =(int) (dtPlot/dt + 0.5);
    assert(fabs(stepPlot*dt - dtPlot) < dt*dt);


    int iter = 0;
//    double dt0 = dt;
    int step = 0;   
    int integrationSteps = 0;
    int maxIter = 0;
    int sumIter = 0;

    // unsigned long s0 = clock();
    double s0 = clock();
    double time = 0;
    // ofstream rzeit("rzeit.asc");
    while(t<tEnd) {
      // rzeit << "Zeit: " << t << endl;
      // long clock1 = clock();
      integrationSteps++;
      if( (step*stepPlot - integrationSteps) < 0) {
	step++;
	system.plot(z,t,dt);
	double s1 = clock();
	time += (s1-s0)/CLOCKS_PER_SEC;
	s0 = s1; 
	integPlot<< t << " " << dt << " " <<  iter << " " << time << " "<<system.getlaSize() <<endl;
	if(output)
	  cout << "   t = " <<  t << ",\tdt = "<< dt << ",\titer = "<<setw(5)<<setiosflags(ios::left) << iter <<  "\r"<<flush;
	tPlot += dtPlot;
      }
////      if((t+dt)*stepPlot >= step) { // plotten
////		step++;
////		system.plot(z,t,dt);
////		double s1 = clock();
////		time += (s1-s0)/CLOCKS_PER_SEC;
////		s0 = s1; 
////		integPlot<< t << " " << dt << " " <<  iter << " " << time << " "<<system.getlaSize() <<endl;
////		if(output) cout << "   t = " <<  t << ",\tdt = "<< dt << ",\titer = " << setw(5) << setiosflags(ios::left) << iter <<  "\r"<< flush;
////		tPlot += dtPlot;
////      }
      // long clock1b = clock();

      // if(fabs(tPlot-t)*dt < 1e-14) {
      // 	// Interpolation macht wahrscheinlich keinen Sinn
      //    // Vec zPlot = z + (z-zOld)/(t-tOld)*(t-tPlot)
      //    system.plot(z,t,dt);
      //    integPlot<< t << " " << dt << " " <<  iter << " " << (clock()-s0)/CLOCKS_PER_SEC << " "<< dt <<endl;
      //    if(output) cout << "   t = " <<  t << ",\tdt = "<< dt << ",\titer = " << setw(4) << setiosflags(ios::left) << iter <<  "\r" << flush;
      //    tPlot += dtPlot;
      // }

      // if ((tPlot-(t+dt))*dt < -1e-14) {
      //    cout << "Warning " << t << " " << tPlot << " " << t+dt << " "<< t+dt - tPlot << endl;
      //    dt = t+dt - tPlot;
      // } 
      // else dt = dt0;

      q += system.deltaq(z,t,dt);
      
      t += dt;
      // long clock2 = clock();

      system.update(z,t); 
      // long clock3 = clock();

      iter = system.solveImpacts(dt);
      // long clock4 = clock();

      if(iter>maxIter) maxIter = iter;
      sumIter += iter;

      u += system.deltau(z,t,dt);
      x += system.deltax(z,t,dt);

      if(driftCompensation) 
	system.projectGeneralizedPositions(t);
    }

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

}
