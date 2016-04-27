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
#include "fortran/fortran_wrapper.h"
#include "lsoder_integrator.h"

// TODO wieder entfernen
#include <fstream>
#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;

namespace MBSimIntegrator {

  LSODERIntegrator::LSODERIntegrator() : dtMax(0), dtMin(0), aTol(1,INIT,1e-6), rTol(1e-6), dt0(0), plotOnRoot(true) {
  }

  void LSODERIntegrator::fzdot(int* zSize, double* t, double* z_, double* zd_) {
    Vec zd(*zSize, zd_);
    system->setTime(*t);
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->resetUpToDate();
    zd = system->evalzd();
  }

  void LSODERIntegrator::fsv(int* zSize, double* t, double* z_, int* nsv, double* sv_) {
    Vec sv(*nsv, sv_);
    system->setTime(*t);
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->resetUpToDate();
    sv = system->evalsv();
  }

  void LSODERIntegrator::integrate(DynamicSystemSolver& system_) {
    debugInit();
    system = &system_;

    int zSize=system->getzSize();
    if(z0.size())
      system->setState(z0);
    else
      system->evalz0();
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->computeInitialCondition();
    double t=tStart;
    double tPlot=t+dtPlot;

    int iTol; 
    if(aTol.size() == 1) {
      iTol = 1; // Skalar
    } else {
      iTol = 2; // Vektor
      assert (aTol.size() >= zSize);
    }

    int one=1, two=2, istate=1;

    int nsv=system->getsvSize();
    int lrWork = (22+zSize*max(16,zSize+9)+3*nsv)*2;
    Vec rWork(lrWork);
    rWork(4) = dt0; 
    rWork(5) = 1e-2;
    rWork(6) = 1e-14;
    int liWork=(20+zSize)*10;
    VecInt iWork(liWork);
    iWork(5) = 10000;

    system->setTime(t);
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->resetUpToDate();
    system->plot();

    double s0 = clock();
    double time = 0;
    int integrationSteps = 0;

    ofstream integPlot((name + ".plt").c_str());

    cout.setf(ios::scientific, ios::floatfield);

    while(t<tEnd) {

      integrationSteps++;

      DLSODER(fzdot, &zSize, system->getState()(), &t, &tPlot, &iTol, &rTol, aTol(), &one,
          &istate, &one, rWork(), &lrWork, iWork(),
          &liWork, NULL, &two, fsv, &nsv, system->getjsv()());
      if(istate==2 || fabs(t-tPlot)<epsroot()) {
        system->setTime(t);
//        system->setState(z); Not needed as the integrator uses the state of the system
        system->resetUpToDate();
        system->plot();
        if(output)
          cout << "   t = " <<  t << ",\tdt = "<< rWork(10) << "\r"<<flush;
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        integPlot<< t << " " << rWork(10) << " " << time << endl;
        tPlot += dtPlot;
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
      if(istate<0) exit(istate);
    }
    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }
}

