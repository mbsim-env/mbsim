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
#include <mbsim/multi_body_system.h>
#include "fortran_wrapper.h"
#include "dopri5_integrator.h"
#include <fstream>

#ifndef NO_ISO_14882
using namespace std;
#endif

namespace MBSim {

  DOPRI5Integrator::DOPRI5Integrator() : aTol(1,INIT,1e-6), rTol(1,INIT,1e-6), dt0(0),maxSteps(0) {
  }

  double DOPRI5Integrator::tPlot = 0;
  double DOPRI5Integrator::dtOut = 0;
  double DOPRI5Integrator::s0;
  double DOPRI5Integrator::time = 0;
  //int DOPRI5Integrator::integrationSteps = 0;
  ofstream DOPRI5Integrator::integPlot;
  Vec DOPRI5Integrator::zInp;
  bool DOPRI5Integrator::output_;

  void DOPRI5Integrator::fzdot(int* zSize, double* t, double* z_, double* zd_, double* rpar, int* ipar) {
    Vec z(*zSize, z_);
    Vec zd(*zSize, zd_);
    system->zdot(z, zd, *t);
  }

  void DOPRI5Integrator::plot(int* nr, double* told, double* t,double* z, int* n, double* con, int* icomp, int* nd, double* rpar, int* ipar, int* irtrn) {

    while(*t >= tPlot) {
      for(int i=1; i<=*n; i++)
	zInp(i-1) = CONTD5(&i,&tPlot,con,icomp,nd);
      system->plot(zInp, tPlot);
      if(output_)
	cout << "   t = " <<  tPlot << ",\tdt = "<< *t-*told << "\r"<<flush;

      double s1 = clock();
      time += (s1-s0)/CLOCKS_PER_SEC;
      s0 = s1; 

      integPlot<< tPlot << " " << *t-*told << " " << time << endl;
      tPlot += dtOut;
    }
  }

  void DOPRI5Integrator::integrate(MultiBodySystem& system_) {

    system = &system_;
    int zSize=system->getzSize();
    int nrDens = zSize;

    double t = 0.0;

    Vec z(zSize);
    if(z0.size())
      z = z0;
    else
      system->initz(z);          

    assert(aTol.size() == rTol.size());

    int iTol;
    if(aTol.size() == 1) {
      iTol = 0; // Skalar
    } else {
      iTol = 1; // Vektor
      assert (aTol.size() >= zSize);
    }

    int out = 2; // TODO

    double rPar;
    int iPar;

    int lWork = 2*(8*zSize+5*nrDens+21);
    int liWork = 2*(nrDens+21);
    Vector<int> iWork(liWork);
    Vec work(lWork);

    //Maximum Step Numbers
    iWork(0)=maxSteps; 
    // if(warnLevel)
    //   iWork(2) = warnLevel;
    // else
    //   iWork(2) = -1;

    iWork(4) = nrDens;

    int idid;

    tPlot = t + dtPlot;
    dtOut = dtPlot;
    system->plot(z, t);

    zInp.resize(zSize);

    integPlot.open((name + ".plt").c_str());

    cout.setf(ios::scientific, ios::floatfield);

    output_ = output;

    s0 = clock();

    DOPRI5(&zSize,fzdot,&t,z(),&tEnd, rTol(),aTol(),&iTol, plot,&out,
	work(),&lWork,iWork(),&liWork,&rPar,&iPar,&idid);

    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    //integSum << "Integration steps: " << integrationSteps << endl;
    integSum.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

}
