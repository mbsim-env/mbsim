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
#include "rksuite_integrator.h"
#include <mbsim/utils/eps.h>
#include "fortran/fortran_wrapper.h"
#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, RKSuiteIntegrator)

  RKSuiteIntegrator::RKSuiteIntegrator() : method(RK45), thres(1,INIT,1e-10), rTol(1e-6), dt0(0), ndworkarray(100000), messages(0), integrationSteps(0), t(0), tPlot(0), s0(0), time(0), z(0), zdGot(0), zMax(0) {
  }

  void RKSuiteIntegrator::preIntegrate() {
    debugInit();

    if(selfStatic)
      throw MBSimError("RKSuiteIntegrator can only integrate one system.");
    selfStatic = this;
    zSize=system->getzSize();

    z.resize(zSize);
    if(z0.size()) {
      if(z0.size() != zSize)
        throw MBSimError("(RKSuiteIntegrator::integrate): size of z0 does not match");
      z = z0;
    }
    else
      z = system->evalz0();

    if(thres.size() == 1) {
      double buf = thres(0);
      thres.resize(zSize,INIT,buf);
    } 
    assert (thres.size() == zSize);

    dworkarray=new double[ndworkarray];
    if (warnLevel)
      messages=1;

    zdGot.resize(zSize);
    zMax.resize(zSize);

    t=tStart;
    system->setTime(t);
    system->setState(z);
    system->resetUpToDate();
    system->plot();

    tPlot = t + dtPlot;
    if(plotIntegrationData) integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);

    integrationSteps = 0;

    s0 = clock();
    time = 0;
  }

    void RKSuiteIntegrator::subIntegrate(double tStop) {

      int result=0, errass=0;
      double tEND=tEnd+dtPlot; // tEND must be greater than tEnd
      char task='U';
      int method_ = method;

      SETUP(&zSize, &t, z(), &tEND, &rTol, thres(), &method_, &task,
          &errass, &dt0, dworkarray, &ndworkarray, &messages);

      while((tStop-t)>epsroot) {

        integrationSteps++;

        double dtLast = 0;
        UT(fzdot, &tPlot, &t, z(), zdGot(), zMax(), dworkarray, &result, &dtLast);

        if(result==1 || result==2 || fabs(t-tPlot)<epsroot) {
          system->setTime(t);
          system->setState(z);
          system->resetUpToDate();
          system->plot();

          if(output) cout << "   t = " <<  t << ",\tdt = "<< dtLast << "\r"<<flush;

          const double s1 = clock();
          time += (s1-s0)/CLOCKS_PER_SEC;
          s0 = s1; 
          if(plotIntegrationData) integPlot<< t << " " << dtLast << " " << time << endl;

          tPlot += dtPlot;
        }

        if(result==3 || result==4)
          continue;
        if(result>=5) 
          throw MBSimError("Integrator RKSUITE failed with result = "+toString(result));

        if (tPlot>tStop)
          tPlot=tStop;
      }
    }

    void RKSuiteIntegrator::postIntegrate() {
      if(plotIntegrationData) integPlot.close();

      if(writeIntegrationSummary) {
        ofstream integSum((name + ".sum").c_str());
        integSum << "Integration time: " << time << endl;
        integSum << "Integration steps: " << integrationSteps << endl;
        integSum.close();
      }

      cout.unsetf (ios::scientific);
      cout << endl;

      selfStatic = nullptr;
    }

  void RKSuiteIntegrator::integrate() {
    preIntegrate();
    subIntegrate(tEnd);
    postIntegrate();
  }

  void RKSuiteIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIMINT%"method");
    if(e) {
      string methodStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(methodStr=="RK23") method=RK23;
      if(methodStr=="RK45") method=RK45;
      if(methodStr=="RK78") method=RK78;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeToleranceScalar");
    if(e) setrTol(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"thresholdScalar");
    if(e) setThreshold(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialStepsize");
    if(e) setInitialStepSize(E(e)->getText<double>());
  }

  void RKSuiteIntegrator::fzdot(double* t, double* z_, double* zd_) {
    Vec zd(selfStatic->zSize, zd_);
    selfStatic->getSystem()->setTime(*t);
    selfStatic->getSystem()->setState(Vec(selfStatic->zSize, z_));
    selfStatic->getSystem()->resetUpToDate();
    zd = selfStatic->getSystem()->evalzd();
  }

  RKSuiteIntegrator *RKSuiteIntegrator::selfStatic = nullptr;

}
