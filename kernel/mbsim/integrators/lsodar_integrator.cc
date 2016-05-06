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
#include <mbsim/utils/utils.h>
#include "fortran/fortran_wrapper.h"
#include "lsodar_integrator.h"
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

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LSODARIntegrator, MBSIMINT%"LSODARIntegrator")

  LSODARIntegrator::LSODARIntegrator() : dtMax(0), dtMin(0), rTol(1e-6), dt0(0), plotOnRoot(true), gMax(1e-5), gdMax(1e+5) {
  }

  void LSODARIntegrator::fzdot(int* zSize, double* t, double* z_, double* zd_) {
    Vec zd(*zSize, zd_);
    system->setTime(*t);
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->resetUpToDate();
    zd = system->evalzd();
  }

  void LSODARIntegrator::fsv(int* zSize, double* t, double* z_, int* nsv, double* sv_) {
    Vec sv(*nsv, sv_);
    system->setTime(*t);
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->resetUpToDate();
    sv = system->evalsv();
  }

  void LSODARIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteTolerance");
    if(e) setAbsoluteTolerance(Element::getVec(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteToleranceScalar");
    if(e) setAbsoluteTolerance(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeToleranceScalar");
    if(e) setRelativeTolerance(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialStepSize");
    setInitialStepSize(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"minimalStepSize");
    setMinimalStepSize(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"maximalStepSize");
    setMaximalStepSize(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"plotOnRoot");
    setPlotOnRoot(Element::getBool(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(Element::getDouble(e));
  }

  DOMElement* LSODARIntegrator::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Integrator::writeXMLFile(parent);
//    if(aTol.size()==0)
//      addElementText(ele0,MBSIMINT%"absoluteToleranceScalar",1e-6);
//    else if(aTol.size()==1)
//      addElementText(ele0,MBSIMINT%"absoluteToleranceScalar",aTol(0));
//    else
//      addElementText(ele0,MBSIMINT%"absoluteTolerance",aTol);
//    addElementText(ele0,MBSIMINT%"relativeToleranceScalar",rTol);
//    addElementText(ele0,MBSIMINT%"initialStepSize",dt0);
//    addElementText(ele0,MBSIMINT%"minimalStepSize",dtMin);
//    addElementText(ele0,MBSIMINT%"maximalStepSize",dtMax);
//    addElementText(ele0,MBSIMINT%"plotOnRoot",plotOnRoot);
    return ele0;
  }

  void LSODARIntegrator::integrate(DynamicSystemSolver& system) {
    debugInit();
    preIntegrate(system);
    subIntegrate(system, tEnd);
    postIntegrate(system);
  }

  void LSODARIntegrator::preIntegrate(DynamicSystemSolver& system_) {
    system = &system_;
    zSize=system->getzSize();
    if(z0.size())
      system->setState(z0);
    else
      system->evalz0();
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->computeInitialCondition();
    t=tStart;
    tPlot=t+dtPlot;
    if(aTol.size() == 0) 
      aTol.resize(1,INIT,1e-6);
    if(aTol.size() == 1) {
      iTol = 1; // Skalar
    } else {
      iTol = 2; // Vektor
      assert (aTol.size() >= zSize);
    }
    istate=1;
    nsv=system->getsvSize();
    lrWork = (22 + zSize * max(16, zSize + 9) + 3 * nsv) * 2;
    rWork.resize(lrWork);
    rWork(4) = dt0; 
    rWork(5) = dtMax;
    rWork(6) = dtMin;
    liWork = (20+zSize)*10;
    iWork.resize(liWork);
    iWork(5) = 10000;
    s0 = clock();
    time = 0;
    integrationSteps = 0;
    integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);
  }

  void LSODARIntegrator::subIntegrate(DynamicSystemSolver& system_, double tStop) {
    int one = 1;
    int two = 2;
    rWork(4) = dt0;
    system->setTime(t);
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->resetUpToDate();
    system->plot();
    cout << "System shiftet and plotted" << endl;
    while(t < tStop) {  
      integrationSteps++;
      DLSODAR(fzdot, &zSize, system->getState()(), &t, &tPlot, &iTol, &rTol, aTol(), &one,
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
        if (tPlot > tStop)
          tPlot = tStop;

        // check drift
        if(system->positionDriftCompensationNeeded(gMax)) { // project both, first positions and then velocities//MFMF
          system->projectGeneralizedPositions(3);
          system->projectGeneralizedVelocities(3);
          istate=1;
        }
        else if(system->velocityDriftCompensationNeeded(gdMax)) { // project velicities//MFMF
          system->projectGeneralizedVelocities(3);
          istate=1;
        }
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
  }

  void LSODARIntegrator::postIntegrate(DynamicSystemSolver& system_) {
    system->setTime(t);
//    system->setState(z); Not needed as the integrator uses the state of the system
    system->resetUpToDate();
    system->plot();
    system->plotAtSpecialEvent();
    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum.close();
    
    cout.unsetf (ios::scientific);
    cout << endl;
  }

}
