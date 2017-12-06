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
 *   martin.o.foerg@googlemail.com
 *
 */

#include <config.h>
#include <mbsim/dynamic_system_solver.h>
#include "fortran/fortran_wrapper.h"
#include "dop853_integrator.h"
#include <ctime>
#include <fstream>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimIntegrator {

  DOP853Integrator::DOP853Integrator() : aTol(1,INIT,1e-6), rTol(1,INIT,1e-6) {
  }

  void DOP853Integrator::fzdot(int* zSize, double* t, double* z_, double* zd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<DOP853Integrator**>(&ipar[0]);
    Vec zd(*zSize, zd_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(Vec(*zSize, z_));
    self->getSystem()->resetUpToDate();
    zd = self->getSystem()->evalzd();
  }

  void DOP853Integrator::plot(int* nr, double* told, double* t,double* z, int* n, double* con, int* icomp, int* nd, double* rpar, int* ipar, int* irtrn) {
    auto self=*reinterpret_cast<DOP853Integrator**>(&ipar[0]);

    while(*t >= self->tPlot) {
      self->getSystem()->setTime(self->tPlot);
      for(int i=1; i<=*n; i++)
	self->getSystem()->getState()(i-1) = CONTD8(&i,&self->tPlot,con,icomp,nd);
      self->getSystem()->resetUpToDate();
      self->getSystem()->plot();
      if(self->output)
	cout << "   t = " <<  self->tPlot << ",\tdt = "<< *t-*told << "\r"<<flush;

      double s1 = clock();
      self->time += (s1-self->s0)/CLOCKS_PER_SEC;
      self->s0 = s1; 

      self->integPlot<< self->tPlot << " " << *t-*told << " " << self->time << endl;
      self->tPlot += self->dtOut;
    }
  }

  void DOP853Integrator::integrate() {
    debugInit();

    int zSize=system->getzSize();
    int nrDens = zSize;

    double t = tStart;

    Vec z(zSize);
    if(z0.size()) {
      if(z0.size() != zSize)
        throw MBSimError("(DOP853Integrator::integrate): size of z0 does not match");
      z = z0;
    }
    else
      z = system->evalz0();

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
    int iPar[sizeof(void*)/sizeof(int)+1]; // store this at iPar[0..]
    DOP853Integrator *self=this;
    memcpy(&iPar[0], &self, sizeof(void*));

    int lWork = 2*(11*zSize+8*nrDens+21);
    int liWork = 2*(nrDens+21);
    VecInt iWork(liWork);
    Vec work(lWork);

    // if(warnLevel)
    //   iWork(2) = warnLevel;
    // else
    //   iWork(2) = -1;

    iWork(4) = nrDens;

    int idid;

    tPlot = t + dtPlot;
    dtOut = dtPlot;

    system->setTime(t);
    system->setState(z);
    system->resetUpToDate();
    system->plot();

    integPlot.open((name + ".plt").c_str());

    cout.setf(ios::scientific, ios::floatfield);

    s0 = clock();

    DOP853(&zSize,fzdot,&t,z(),&tEnd,rTol(),aTol(),&iTol,plot,&out,
	work(),&lWork,iWork(),&liWork,&rPar,iPar,&idid);

    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    //integSum << "Integration steps: " << integrationSteps << endl;
    integSum.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

}
