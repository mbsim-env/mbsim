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
 *          rzander@users.berlios.de
 */

#include <config.h>

#include "mbsim/integrators/theta_time_stepping_integrator.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/eps.h"

#include <cmath>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;

namespace MBSim {

  ThetaTimeSteppingIntegrator::ThetaTimeSteppingIntegrator() : dt(1e-3), theta(0.5), driftCompensation(false) {}

  void ThetaTimeSteppingIntegrator::update(DynamicSystemSolver& system, const Vec& z, double t) {
    if(system.getq()()!=z()) system.updatezRef(z);

    system.updateStateDependentVariables(t);
    system.updateg(t);
    system.checkActiveg();
    system.checkActiveLinks();
    if(system.gActiveChanged()) {
      system.checkAllgd(); // TODO necessary?
      system.calcgdSizeActive();
      system.calclaSize();
      system.calcrFactorSize();

      system.setlaIndDS(system.getlaInd());

      system.updateWRef(system.getWParent()(Index(0,system.getuSize()-1),Index(0,system.getlaSize()-1)));
      system.updateVRef(system.getVParent()(Index(0,system.getuSize()-1),Index(0,system.getlaSize()-1)));
      system.updatelaRef(system.getlaParent()(0,system.getlaSize()-1));
      system.updategdRef(system.getgdParent()(0,system.getgdSize()-1));
      system.updateresRef(system.getresParent()(0,system.getlaSize()-1));
      system.updaterFactorRef(system.getrFactorParent()(0,system.getrFactorSize()-1));
    }
    system.updategd(t);

    system.updateT(t); 
    system.updateJacobians(t);
    system.updatedhdz(t);
    system.updateM(t); 
    system.facLLM(); 
    system.updateW(t); 
    system.updateV(t); 
  }

  void ThetaTimeSteppingIntegrator::integrate(DynamicSystemSolver& system) {

    // initialisation
    assert(dtPlot >= dt);

    double t = tStart;

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

    if(z0.size()) z = z0;
    else system.initz(z);

    double tPlot = 0.;

    ofstream integPlot((name + ".plt").c_str());

    int iter = 0;

    cout.setf(ios::scientific, ios::floatfield);

    int step = 0;
    int stepPlot = (int)(dtPlot/dt + 0.5);

    assert(fabs(stepPlot*dt - dtPlot) < dt*dt);

    int integrationSteps = 0;
    int maxIter = 0;
    int sumIter = 0;

    double s0 = clock();
    double time = 0;

    while(t<=tEnd+dt/2.) { // time loop
      integrationSteps++;
      if((step*stepPlot - integrationSteps) < 0) {
        step++;
        if(driftCompensation) system.projectGeneralizedPositions(t);
        system.plot(z,t,dt);
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        integPlot << t << " " << dt << " " <<  iter << " " << time << " "<<system.getlaSize() << endl;
        if(output) cout << "   t = " <<  t << ",\tdt = "<< dt << ",\titer = "<< setw(5) << setiosflags(ios::left) << iter << "\r" << flush;
        tPlot += dtPlot;
      }

      double te = t + dt;
      t += theta*dt;
      update(system,z,t);

      Mat T = system.getT().copy();
      SymMat M = system.getM().copy();
      Vec h = system.geth().copy();
      Mat W = system.getW().copy();
      Mat V = system.getV().copy();
      Mat dhdq = system.getdhdq();
      Mat dhdu = system.getdhdu();

      Vector<int> ipiv(M.size());
      SqrMat luMeff = SqrMat(facLU(M - theta*dt*dhdu - theta*theta*dt*dt*dhdq*T,ipiv));
      SqrMat Geff = SqrMat(trans(W)*slvLUFac(luMeff,V,ipiv));
      system.getGs().resize() << Geff;
      system.getb() = trans(W)*slvLUFac(luMeff,h+theta*dhdq*T*u*dt,ipiv);

      iter = system.solveImpacts(dt);
      if(iter>maxIter) maxIter = iter;
      sumIter += iter;

      system.updater(t);
      Vec du = slvLUFac(luMeff,h * dt + V*system.getla() + theta*dhdq*T*u*dt*dt,ipiv);
      q += T*(u+theta*du)*dt;
      u += du;
      x += system.deltax(z,t,dt);
      t = te;
    }

    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum << "Maximum number of iterations: " << maxIter << endl;
    integSum << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;
    integSum.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

}

