/* Copyright (C) 2004-2010  Martin Förg
 
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
#include <fmatvec.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/utils/eps.h>
#include "fortran_wrapper.h"
#include "daspk_integrator.h"

// TODO wieder entfernen
#include <fstream>
#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;

namespace MBSim {

  DASPKIntegrator::DASPKIntegrator() : dtMax(0), dtMin(0), aTol(1,INIT,1e-6), rTol(1e-6), dt0(0), maxSteps(10000), stiff(false) {
  }

  int DASPKIntegrator::zSize;

  void DASPKIntegrator::res(double *t, double *z_, double *zd_, double *cj, double *delta_, int *ires, double *rpar, int *ipar) {
    Vec z(zSize, z_);
    Vec zd(zSize, zd_);
    Vec zds(zSize);
    Vec delta(zSize, delta_);
    system->zdot(z, zds, *t);
    delta = zd - zds; 
  }

  void DASPKIntegrator::fsv(int* zSize, double* t, double* z_, int* nsv, double* sv_) {
    Vec z(*zSize, z_);
    Vec sv(*nsv, sv_);
    system->getsv(z, sv, *t);
  }

  void DASPKIntegrator::integrate(DynamicSystemSolver& system_) {
    system = &system_;

    zSize=system->getzSize();
    Vec z(zSize);
    Vec zdot(zSize);
    if(z0.size())
      z = z0;
    else
      system->initz(z);

    double t = tStart;
    double tPlot = min(tEnd,t + dtPlot);

    int nsv=system->getsvSize();
    int lrWork = (22+zSize*max(16,zSize+9)+3*nsv)*2;
    Vec rWork(lrWork);
    rWork(4) = dt0; // integrator chooses the step size (dont use dt0)
    rWork(5) = dtMax;
    rWork(6) = dtMin;
    int liWork=(20+zSize)                             *10;//////////////;
    Vector<int> iWork(liWork);
    iWork(5) = maxSteps;

    Vector<int> info(20);

    system->plot(z, t);

    double s0 = clock();
    double time = 0;
    int integrationSteps = 0;

    ofstream integPlot((name + ".plt").c_str());
    integPlot << "#1 t [s]:" << endl; 
    integPlot << "#1 dt [s]:" << endl; 
    integPlot << "#1 calculation time [s]:" << endl;

    Vector<int> jsv(nsv);  
    int idid;
    Vec rpar(1);
    Vector<int> ipar(1);

    cout.setf(ios::scientific, ios::floatfield);
    while(t<tEnd) {
      DDASPK (res, &zSize, &t, z(), zdot(), &tPlot, info(), &rTol, aTol(), 
	  &idid, rWork(), &lrWork, iWork(), &liWork, rpar(),ipar(), 0, 0);
      if(fabs(t-tPlot)<epsroot()) {
	system->plot(z, t);
	if(output)
	  cout << "   t = " <<  t << ",\tdt = "<< rWork(10) << "\r"<<flush;
	double s1 = clock();
	time += (s1-s0)/CLOCKS_PER_SEC;
	s0 = s1; 
	integPlot<< t << " " << rWork(10) << " " << time << endl;
	tPlot = min(tEnd,tPlot + dtPlot);
      }
    }

    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Simulation time: " << t << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

  void DASPKIntegrator::initializeUsingXML(TiXmlElement * element) {
    Integrator::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMINTNS"absoluteTolerance");
    if (e)
      setAbsoluteTolerance(Element::getVec(e));
    else {
      e=element->FirstChildElement(MBSIMINTNS"absoluteToleranceScalar");
      setAbsoluteTolerance(Element::getDouble(e));
    }
    e=element->FirstChildElement(MBSIMINTNS"relativeToleranceScalar");
    setRelativeTolerance(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"initialStepSize");
    setInitialStepSize(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"maximalStepSize");
    setMaximalStepSize(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"minimalStepSize");
    setMinimalStepSize(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"numberOfMaximalSteps");
    setmaxSteps(Element::getInt(e));
    setStiff(element->FirstChildElement(MBSIMINTNS"stiffModus"));
      
  }

}
