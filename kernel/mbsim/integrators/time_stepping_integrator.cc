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

#include <time.h>
#include <boost/iostreams/tee.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/bind.hpp>

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

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(TimeSteppingIntegrator, MBSIMINT%"TimeSteppingIntegrator")

  TimeSteppingIntegrator::TimeSteppingIntegrator() : dt(1e-3), tPlot(0.), step(0), integrationSteps(0), maxIter(0), sumIter(0), s0(0.), time(0.), stepPlot(0), driftCompensation(false) {}

  void TimeSteppingIntegrator::preIntegrate(DynamicSystemSolver& system) {
    // initialisation
    assert(dtPlot >= dt);

    system.setTime(tStart);

    system.setStepSize(dt);

    if(z0.size())
      system.setState(z0);
    else
      system.evalz0();

    integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);

    stepPlot =(int) (dtPlot/dt + 0.5);
    if(fabs(stepPlot*dt - dtPlot) > dt*dt) {
      cout << "WARNING: Due to the plot-Step settings it is not possible to plot exactly at the correct times." << endl;
    }

    s0 = clock();
  }

  void TimeSteppingIntegrator::subIntegrate(DynamicSystemSolver& system, double tStop) {
    while(system.getTime()<tStop) { // time loop
      integrationSteps++;
      if((step*stepPlot - integrationSteps) < 0) {
        step++;
        if(driftCompensation) system.projectGeneralizedPositions(0);
        system.plot();
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        integPlot<< system.getTime() << " " << dt << " " <<  system.getIterI() << " " << time << " "<<system.getlaSize() <<endl;
        if(output) cout << "   t = " << system.getTime() << ",\tdt = "<< dt << ",\titer = "<<setw(5)<<setiosflags(ios::left) << system.getIterI() <<  "\r"<<flush;
        tPlot += dtPlot;
      }

      system.getq() += system.evaldq();
      system.getTime() += dt;
      system.resetUpToDate();

      system.checkActive(1);
      if (system.gActiveChanged()) system.resize_();

      system.getu() += system.evaldu();
      system.getx() += system.evaldx();
      system.resetUpToDate();

      if(system.getIterI()>maxIter) maxIter = system.getIterI();
      sumIter += system.getIterI();

    }
  }

  void TimeSteppingIntegrator::postIntegrate(DynamicSystemSolver& system) {
    integPlot.close();

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

    cout.unsetf(ios::scientific);
    cout << endl;
  }

  void TimeSteppingIntegrator::integrate(DynamicSystemSolver& system) {
    this->system = &system;
    system.setUpdatebiCallBack(boost::bind(&TimeSteppingIntegrator::updatebi,this));
    debugInit();
    preIntegrate(system);
    subIntegrate(system, tEnd);
    postIntegrate(system);
  }

  void TimeSteppingIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stepSize");
    setStepSize(Element::getDouble(e));
  }

  void TimeSteppingIntegrator::updatebi() {
    system->getbi(false) << system->evalgd() + system->evalW().T()*slvLLFac(system->evalLLM(),system->evalh())*dt;
  }

}
