/* Copyright (C) 2004-2014 MBSim Development Team
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
#include "time_stepping_integrator.h"

#include <ctime>
#include <boost/iostreams/tee.hpp>
#include <boost/iostreams/stream.hpp>

#ifndef NO_ISO_14882
using namespace std;
#endif

namespace bio = boost::iostreams;
using bio::tee_device;
using bio::stream;

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, TimeSteppingIntegrator)

  void TimeSteppingIntegrator::preIntegrate() {
    debugInit();
    // initialisation
    assert(dtPlot >= dt);

    system->setTime(tStart);

    system->setStepSize(dt);

    if(z0.size()) {
      if(z0.size() != system->getzSize())
        throw MBSimError("(TimeSteppingIntegrator::integrate): size of z0 does not match, must be " + toStr(system->getzSize()));
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

  void TimeSteppingIntegrator::subIntegrate(double tStop) {
    while(system->getTime()<tStop) { // time loop
      integrationSteps++;
      if((step*stepPlot - integrationSteps) < 0) {
        step++;
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

      system->getq() += system->evaldq();
      system->getTime() += dt;
      system->resetUpToDate();

      system->checkActive(1);
      if (system->gActiveChanged()) system->resize_();

      if(gMax>=0 and system->positionDriftCompensationNeeded(gMax))
        system->projectGeneralizedPositions(3);

      system->getbi(false) << system->evalgd() + system->evalW().T()*slvLLFac(system->evalLLM(),system->evalh())*dt;
      system->setUpdatebi(false);

      system->getu() += system->evaldu();
      system->getx() += system->evaldx();

      system->setla(system->getLa()/dt);
      system->setqd(system->getdq(false)/dt);
      system->setud(system->getdu(false)/dt);
      system->setxd(system->getdx(false)/dt);

      system->resetUpToDate();

      if(system->getIterI()>maxIter) maxIter = system->getIterI();
      sumIter += system->getIterI();
    }
  }

  void TimeSteppingIntegrator::postIntegrate() {
    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      typedef tee_device<ostream, ofstream> TeeDevice;
      typedef stream<TeeDevice> TeeStream;
      ofstream integSum((name + ".sum").c_str());
      TeeDevice ts_tee(cout, integSum);
      TeeStream ts_split(ts_tee);

      ts_split << endl << endl << "******************************" << endl;
      ts_split << "INTEGRATION SUMMARY: " << endl;
      ts_split << "End time [s]: " << tEnd << endl;
      ts_split << "Integration time [s]: " << time << endl;
      ts_split << "Integration steps: " << integrationSteps << endl;
      ts_split << "Maximum number of iterations: " << maxIter << endl;
      ts_split << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;
      ts_split << "******************************" << endl;
      ts_split.flush();
      ts_split.close();
    }

    cout.unsetf(ios::scientific);
    cout << endl;
  }

  void TimeSteppingIntegrator::integrate() {
    preIntegrate();
    subIntegrate(tEnd);
    postIntegrate();
  }

  void TimeSteppingIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stepSize");
    if(e) setStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
  }

}
