/* Copyright (C) 2004-2006  Roland Zander, Martin Förg
 
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
 *   rzander@users.berlios.de
 *
 */

#include "theta_time_stepping_integrator.h"
#include "multi_body_system.h"

#include "eps.h"
#include <cmath>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;

namespace MBSim {

  ThetaTimeSteppingIntegrator::ThetaTimeSteppingIntegrator() : dt(1e-3), theta(0.5), driftCompensation(false) {
  }

  void ThetaTimeSteppingIntegrator::integrate(MultiBodySystem& system) {
    assert(dtPlot >= dt);

    double t0 = 0.0;
    double  t = t0;

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

    int iter = 0;

    cout.setf(ios::scientific, ios::floatfield);

    int step = 0;
    int stepPlot =(int) (dtPlot/dt + 0.5);

    assert(fabs(stepPlot*dt - dtPlot) < dt*dt);

    int integrationSteps = 0;
    int maxIter = 0;
    int sumIter = 0;

    double s0 = clock();
    double time = 0;
    while(t<=tEnd+dt/2.) {
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

      t += dt;

   //   q += system.deltaq(z,t,dt);
      //q += system.getT()*(u)*dt;

      // TODO T updaten (passiert sonst in deltaq)
      system.updateKinematics(t);
      system.updateLinksStage1(t);
      system.checkActiveConstraints();
      system.updateLinksStage2(t);
      system.updateT(t); 
      system.updateM(t); 
      system.updateh(t); 
      system.updateW(t); 

      SymMat M = system.getM().copy();
      Mat W = system.getW().copy();
      Vec h = system.geth().copy();
      Mat T = system.getT().copy();
      //system.updateJh(t);
      //Mat Jh2 = system.getJh();

Mat Jh(nu,n);
static const double eps = epsroot();
for(int i=0;i<n;i++) {
  double zSave = z(i);
  z(i) += eps;
  system.updateKinematics(t);
  system.updateLinksStage1(t);
  system.updateLinksStage2(t);
  system.updateh(t); 
  Vec hm = system.geth();
  Jh.col(i) << (hm-h)/(eps);
  z(i) = zSave;
}

      Vector<int> ipiv(M.size());
      SqrMat luMeff = SqrMat(facLU(M - theta*dt*Jh(Index(0,nu-1),Index(nq,nq+nu-1)) - theta*theta*dt*dt*Jh(Index(0,nu-1),Index(0,nq-1))*T,ipiv));
      SqrMat Geff = SqrMat(trans(W)*slvLUFac(luMeff,W,ipiv));
      system.getGs().resize();
      system.getGs() << Geff;
      system.getb() = trans(W)*(slvLUFac(luMeff,h+theta*Jh(Index(0,nu-1),Index(0,nq-1))*T*u*dt,ipiv) );

      iter = system.solve(dt);
      if(iter>maxIter)
	maxIter = iter;
      sumIter += iter;

      system.updater(t);
      Vec du = slvLUFac(luMeff,h * dt + W*system.getla() + theta*Jh(Index(0,nu-1),Index(0,nq-1))*T*u*dt*dt,ipiv);
      q += T*(u+theta*du)*dt;
      u += du;
      x += system.deltax(z,t,dt);

///cout << "system. Jh = " << system.getJh() << endl;
///cout << "numeric.Jh = " << Jh2 << endl;
//cout << "      diff = " << Jh - Jh2 << endl;

      if(driftCompensation)
	system.projectViolatedConstraints(t);
    }

    integPlot.close();

    ofstream integSum((system.getDirectoryName() + name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum << "Maximum number of iterations: " << maxIter << endl;
    integSum << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;
    integSum.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

}
