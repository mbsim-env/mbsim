/* Copyright (C) 2004-2010 MBSim Development Team
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

#include "mbsim/integrators/theta_time_stepping_integrator.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/eps.h"
#include <ctime>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, ThetaTimeSteppingIntegrator)

  void ThetaTimeSteppingIntegrator::preIntegrate() {
    debugInit();
    // initialisation
    assert(dtPlot >= dt);

    system->setTime(tStart);

    system->setStepSize(dt);

    if(z0.size()) {
      if(z0.size() != system->getzSize())
        throw MBSimError("(ThetaTimeSteppingIntegrator::integrate): size of z0 does not match");
      system->setState(z0);
    }
    else
      system->evalz0();

    if(plotIntegrationData) integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);
    
    stepPlot = (int) (dtPlot/dt + 0.5);
    if(fabs(stepPlot*dt - dtPlot) > dt*dt) {
      cout << "WARNING: Due to the plot-Step settings it is not possible to plot exactly at the correct times." << endl;
    }

    s0 = clock();
  }

  void ThetaTimeSteppingIntegrator::subIntegrate(double tStop) {
    while(system->getTime()<tStop) { // time loop
      integrationSteps++;
      if((step*stepPlot - integrationSteps) < 0) {
        step++;
        if(system->positionDriftCompensationNeeded(gMax))
          system->projectGeneralizedPositions(3);
        system->setUpdatela(false);
        system->setUpdateLa(false);
        system->setUpdatezd(false);
        system->plot();
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1;
        if(plotIntegrationData) integPlot<< system->getTime() << " " << dt << " " <<  system->getIterI() << " " << time << " "<<system->getlaSize() <<endl;
        if(output) cout << "   t = " << system->getTime() << ",\tdt = "<< dt << ",\titer = "<<setw(5)<<setiosflags(ios::left) << system->getIterI() <<  "\r"<<flush;
        tPlot += dtPlot;
      }

      Vec hOld = system->evalh();
      Mat dhdq(system->gethSize(),system->getqSize(),NONINIT);
      for (int i=0; i<system->getqSize(); i++) {
        double qtmp = system->getq()(i);
        system->getq()(i) += epsroot;
        system->resetUpToDate();
        dhdq.col(i) = (system->evalh()-hOld)/epsroot;
        system->getq()(i) = qtmp;
      }
      Mat dhdu(system->gethSize(),system->getuSize(),NONINIT);
      for (int i=0; i<system->getuSize(); i++) {
        double utmp = system->getu()(i);
        system->getu()(i) += epsroot;
        system->resetUpToDate();
        dhdu.col(i) = (system->evalh()-hOld)/epsroot;
        system->getu()(i) = utmp;
      }

      double te = system->getTime() + dt;
      system->getTime() += theta*dt;
      system->resetUpToDate();

      system->checkActive(1);
      if (system->gActiveChanged()) system->resize_();

      const Mat &T = system->evalT();
      const SymMat &M = system->evalM();
      const Vec &h = system->evalh();
      const Mat &W = system->evalW();
      const Mat &V = system->evalV();

      VecInt ipiv(M.size());
      SqrMat luMeff = SqrMat(facLU(M - (theta*dt)*dhdu - (theta*theta*dt*dt)*dhdq*T,ipiv));
      Vec heff = h + (theta*dt)*dhdq*T*system->getu();
      system->getG(false) << SqrMat(W.T()*slvLUFac(luMeff,V,ipiv));
      system->getGs(false) << system->getG(false);
      system->setUpdateG(false);
      system->getbi(false) << system->evalgd() + W.T()*slvLUFac(luMeff,heff,ipiv)*dt;
      system->setUpdatebi(false);

      Vec du = slvLUFac(luMeff,heff*dt+system->evalrdt(),ipiv);

      system->getq() += T*(system->getu()+theta*du)*dt;
      system->getu() += du;
      system->getx() += system->evaldx();

      //cout << system->getgd() + W.T()*du << endl;
      //cout << W.T()*system->getu() << endl;
      system->setTime(te);

      //cout << "resetUpToDate" << endl;
      system->resetUpToDate();
      //cout << system->evalgd() << endl;
      //cout << "end" << endl;

      if(system->getIterI()>maxIter) maxIter = system->getIterI();
      sumIter += system->getIterI();
    }
  }

  void ThetaTimeSteppingIntegrator::postIntegrate() {
    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      ofstream integSum((name + ".sum").c_str());
      integSum << "Integration time: " << time << endl;
      integSum << "Integration steps: " << integrationSteps << endl;
      integSum << "Maximum number of iterations: " << maxIter << endl;
      integSum << "Average number of iterations: " << double(sumIter) / integrationSteps << endl;
      integSum.close();
    }

    cout.unsetf(ios::scientific);
    cout << endl;
  }

  void ThetaTimeSteppingIntegrator::integrate() {
    preIntegrate();
    subIntegrate(tEnd);
    postIntegrate();
  }

  void ThetaTimeSteppingIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIMINT%"stepSize");
    if(e) setStepSize(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIMINT%"theta");
    if(e) setTheta(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
  }

}
