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
#include <mbsim/utils/eps.h>
#include <mbsim/utils/utils.h>
#include "fortran/fortran_wrapper.h"
#include "lsode_integrator.h"
#include <fstream>
#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LSODEIntegrator, MBSIMINT%"LSODEIntegrator")

  LSODEIntegrator::LSODEIntegrator() : dtMax(0), dtMin(0), rTol(1e-6), dt0(0), maxSteps(10000), stiff(false) {
  }

  void LSODEIntegrator::fzdot(int* zSize, double* t, double* z_, double* zd_) {
    Vec z(*zSize, z_);
    Vec zd(*zSize, zd_);
    system->setTime(*t);
    system->setState(z);
    system->resetUpToDate();
    zd = system->evalzd();
  }

  void LSODEIntegrator::integrate(DynamicSystemSolver& system_) {
    debugInit();
    system = &system_;

    int zSize=system->getzSize();
    Vec z(zSize);
    if(z0.size())
      z = z0;
    else
      z = system->evalz0();

    double t = tStart;
    double tPlot = min(tEnd,t + dtPlot);

    int iTol; 
    if(aTol.size() == 0) 
      aTol.resize(1,INIT,1e-6);
    if(aTol.size() == 1) {
      iTol = 1; // Skalar
    } else {
      iTol = 2; // Vektor
      assert (aTol.size() >= zSize);
    }

//    int i, one=1, zero=0, two=2, istate=1;
    int one=1, istate=1;
    int nsv=system->getsvSize();
    int lrWork = (22+zSize*max(16,zSize+9)+3*nsv)*2;
    Vec rWork(lrWork);
    rWork(4) = dt0; // integrator chooses the step size (dont use dt0)
    rWork(5) = dtMax;
    rWork(6) = dtMin;
    int liWork=(20+zSize)                             *10;//////////////;
    VecInt iWork(liWork);
    iWork(5) = maxSteps;

    system->setTime(t);
    system->setState(z);
    system->resetUpToDate();
    system->solveAndPlot();

    double s0 = clock();
    double time = 0;
    int integrationSteps = 0;

    ofstream integPlot((name + ".plt").c_str());
    integPlot << "#1 t [s]:" << endl; 
    integPlot << "#1 dt [s]:" << endl; 
    integPlot << "#1 calculation time [s]:" << endl;

    int MF;
    if(stiff) 
      MF = 22; // Stiff (BDF) method, internally generated full Jacobian.
    else
      MF = 10; // Nonstiff (Adams) method, no Jacobian used.

    VecInt jsv(nsv);  
//    bool donedrift;

    cout.setf(ios::scientific, ios::floatfield);
    while(t<tEnd) {
      DLSODE (fzdot, &zSize, z(), &t, &tPlot, &iTol, &rTol, aTol(), 
        &one, &istate, &one, rWork(), &lrWork, iWork(), 
        &liWork, 0, &MF);
      if(istate==2 || fabs(t-tPlot)<epsroot()) {
        system->setTime(t);
        system->setState(z);
        system->resetUpToDate();
        system->solveAndPlot();
        if(output)
          cout << "   t = " <<  t << ",\tdt = "<< rWork(10) << "\r"<<flush;
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        integPlot<< t << " " << rWork(10) << " " << time << endl;
        tPlot = min(tEnd,tPlot + dtPlot);
      }
      if(istate<0) exit(istate);
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

  void LSODEIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteTolerance");
    if (e)
      setAbsoluteTolerance(Element::getVec(e));
    else {
      e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteToleranceScalar");
      setAbsoluteTolerance(Element::getDouble(e));
    }
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeToleranceScalar");
    setRelativeTolerance(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialStepSize");
    setInitialStepSize(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"maximalStepSize");
    setMaximalStepSize(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"minimalStepSize");
    setMinimalStepSize(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"numberOfMaximalSteps");
    setmaxSteps(Element::getInt(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stiffModus");
    if(e)
      setStiff(Element::getBool(e));
      
  }

  DOMElement* LSODEIntegrator::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Integrator::writeXMLFile(parent);
//    if(aTol.size() > 1) 
//      addElementText(ele0,MBSIMINT%"absoluteTolerance",aTol);
//    else
//      addElementText(ele0,MBSIMINT%"absoluteToleranceScalar",aTol(0));
//    addElementText(ele0,MBSIMINT%"relativeToleranceScalar",rTol);
//    addElementText(ele0,MBSIMINT%"initialStepSize",dt0);
//    addElementText(ele0,MBSIMINT%"maximalStepSize",dtMax);
//    addElementText(ele0,MBSIMINT%"minimalStepSize",dtMin);
//    if(maxSteps != 2000000000)
//      addElementText(ele0,MBSIMINT%"numberOfMaximalSteps",maxSteps);
//    addElementText(ele0,MBSIMINT%"stiffModus",stiff);
    return ele0;
  }


}
