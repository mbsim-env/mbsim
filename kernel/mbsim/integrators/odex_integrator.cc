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
#include "odex_integrator.h"
#include <fstream>
#include <ctime>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, ODEXIntegrator)

  void ODEXIntegrator::fzdot(int* zSize, double* t, double* z_, double* zd_, double* rpar, int* ipar) {
    auto self=*reinterpret_cast<ODEXIntegrator**>(&ipar[0]);
    Vec zd(*zSize, zd_);
    self->getSystem()->setTime(*t);
    self->getSystem()->setState(Vec(*zSize, z_));
    self->getSystem()->resetUpToDate();
    zd = self->getSystem()->evalzd();
  }

  void ODEXIntegrator::plot(int* nr, double* told, double* t,double* z, int* n, double* con, int *ncon, int* icomp, int* nd, double* rpar, int* ipar, int* irtrn) {
    auto self=*reinterpret_cast<ODEXIntegrator**>(&ipar[0]);

    while(*t >= self->tPlot) {
      self->getSystem()->setTime(self->tPlot);
      for(int i=1; i<=*n; i++)
	self->getSystem()->getState()(i-1) = CONTEX(&i,&self->tPlot,con,ncon,icomp,nd);
      self->getSystem()->resetUpToDate();
      self->getSystem()->plot();
      if(self->output)
	cout << "   t = " <<  self->tPlot << ",\tdt = "<< *t-*told << "\r"<<flush;

      double s1 = clock();
      self->time += (s1-self->s0)/CLOCKS_PER_SEC;
      self->s0 = s1; 

      if(self->plotIntegrationData) self->integPlot<< self->tPlot << " " << *t-*told << " " << self->time << endl;
      self->tPlot += self->dtOut;
    }
  }

  void ODEXIntegrator::integrate() {
    debugInit();

    int zSize=system->getzSize();

    double t = tStart;

    Vec z(zSize);
    if(z0.size()) {
      if(z0.size() != zSize)
        throw MBSimError("(ODEXIntegrator::integrate): size of z0 does not match, must be " + toStr(zSize));
      z = z0;
    }
    else
      z = system->evalz0();

    if(aTol.size() == 0)
      aTol.resize(1,INIT,1e-6);
    if(rTol.size() == 0)
      rTol.resize(1,INIT,1e-6);

    int iTol;
    if(aTol.size() == 1)
      iTol = 0;
    else {
      iTol = 1;
      if(aTol.size() != zSize)
        throw MBSimError("(ODEXIntegrator::integrate): size of aTol does not match, must be " + toStr(zSize));
    }
    if(rTol.size() != aTol.size())
      throw MBSimError("(ODEXIntegrator::integrate): size of rTol does not match aTol, must be " + toStr(aTol.size()));

    int out = 2; // dense output is performed in plot

    double rPar;
    int iPar[sizeof(void*)/sizeof(int)+1];
    ODEXIntegrator *self=this;
    memcpy(&iPar[0], &self, sizeof(void*));

    int lWork = 2*(zSize*(9+5)+5*9+20+(2*9*(9+2)+5)*zSize);
    int liWork = 2*(2*9+21+zSize);
    VecInt iWork(liWork);
    Vec work(lWork);
    if(dtMax>0)
      work(1)=dtMax; // maximum step size
    iWork(0)=maxSteps; // maximum number of steps
    iWork(7) = zSize;

    int idid;

    tPlot = t + dtPlot;
    dtOut = dtPlot;

    system->setTime(t);
    system->setState(z);
    system->resetUpToDate();
    system->plot();

    if(plotIntegrationData) integPlot.open((name + ".plt").c_str());

    cout.setf(ios::scientific, ios::floatfield);

    s0 = clock();

    ODEX(&zSize,fzdot,&t,z(),&tEnd, &dt0,rTol(),aTol(),&iTol,plot,&out,
	work(),&lWork,iWork(),&liWork,&rPar,iPar,&idid);

    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      ofstream integSum((name + ".sum").c_str());
      integSum.precision(8);
      integSum << "Integration time: " << time << endl;
      integSum << "Simulation time: " << t << endl;
      //integSum << "Integration steps: " << integrationSteps << endl;
      integSum << "Number of function evaluations" << iWork(16) <<endl;
      integSum.close();
    }

    cout.unsetf (ios::scientific);
    cout << endl;
  }

  void ODEXIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteTolerance");
    if(e) setAbsoluteTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteToleranceScalar");
    if(e) setAbsoluteTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeTolerance");
    if(e) setRelativeTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeToleranceScalar");
    if(e) setRelativeTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialStepSize");
    if(e) setInitialStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"maximumStepSize");
    if(e) setMaximumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stepLimit");
    if(e) setStepLimit(E(e)->getText<int>());
  }

}
