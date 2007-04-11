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

#include <config.h>
#include <multi_body_system.h>
#include "euler_explicit_integrator.h"
#include <fstream>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;

namespace MBSim {

  EulerExplicitIntegrator::EulerExplicitIntegrator() : dt(1e-3) {
  }

  void EulerExplicitIntegrator::integrate(MultiBodySystem& system) {

    assert(dtPlot >= dt);

    double t = 0;

    int nq = system.getqSize();
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

    if(z0.size())
      z = z0;
    else
      system.initz(z);

    double tPlot = 0.0;

    ofstream integPlot((system.getDirectoryName() + name + ".plt").c_str());
    double s0 = clock();
    double time = 0;
    int integrationSteps = 0;
 
    cout.setf(ios::scientific, ios::floatfield);

    double dt0 = dt;
    int step = 0;
    int stepPlot =(int) (1.0/dtPlot);
    while(t<tEnd) {
      integrationSteps++;
      if((t+dt)*stepPlot >= step) {
	step++;
	system.plot(z,t);
	integPlot<< t << " " << dt << " " << (clock()-s0)/CLOCKS_PER_SEC << " "<<dt <<endl;
	if(output)
	  cout << "   t = " <<  t << ",\tdt = "<< dt <<  "\r"<<flush;
	tPlot += dtPlot;
      }

      z += system.zdot(z,t)*dt; 
      t += dt;
    }
    integPlot.close();

    ofstream integSum((system.getDirectoryName() + name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

}
