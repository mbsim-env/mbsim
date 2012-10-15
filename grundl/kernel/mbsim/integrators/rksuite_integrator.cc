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
#include "fortran_wrapper.h"

#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;

namespace MBSim {

  RKSuiteIntegrator::RKSuiteIntegrator() : method(2), thres(1,INIT,1e-10), rTol(1e-6), dt0(0), ndworkarray(100000), messages(0), integrationSteps(0), t(0), tPlot(0), s0(0), time(0), z(0), zdGot(0), zMax(0), integPlot(0) {
  }

  void RKSuiteIntegrator::preIntegrate(DynamicSystemSolver& system_) {

    system=&system_;

    zSize=system->getzSize();

    z.resize(zSize);
    if(z0.size()) z = z0; // define initial state
    else system->initz(z);

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
    system->plot(z, t);

    tPlot = t + dtPlot;
    integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);

    integrationSteps = 0;

    s0 = clock();
    time = 0;
  }

  void RKSuiteIntegrator::subIntegrate(DynamicSystemSolver& system_, double tStop) {

    int result=0, errass=0;
    double tEND=tEnd+dtPlot; // tEND must be greater than tEnd
    char task='U';

    SETUP(&zSize, &t, z(), &tEND, &rTol, thres(), &method, &task,
        &errass, &dt0, dworkarray, &ndworkarray, &messages);

    while((tStop-t)>epsroot()) {

      integrationSteps++;

      double dtLast = 0;
      UT(fzdot, &tPlot, &t, z(), zdGot(), zMax(), dworkarray, &result, &dtLast);

      if(result==1 || result==2 || fabs(t-tPlot)<epsroot()) {
        system->plot(z, t);

        if(output) cout << "   t = " <<  t << ",\tdt = "<< dtLast << "\r"<<flush;

        const double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        integPlot<< t << " " << dtLast << " " << time << endl;

        tPlot += dtPlot;
      }

      if(result==3 || result==4)
        continue;
      if(result>=5) 
        exit(result);

      if (tPlot>tStop)
        tPlot=tStop;
    }
  }

  void RKSuiteIntegrator::postIntegrate(DynamicSystemSolver& system_) {
    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

  void RKSuiteIntegrator::integrate(DynamicSystemSolver& system) {
    preIntegrate(system);
    subIntegrate(system, tEnd);
    postIntegrate(system);
  }

  void RKSuiteIntegrator::initializeUsingXML(TiXmlElement *element) {
    Integrator::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMINTNS"method23");
    if (e)
      setMethod(1);
    e=element->FirstChildElement(MBSIMINTNS"method45");
    if (e)
      setMethod(2);
    e=element->FirstChildElement(MBSIMINTNS"method78");
    if (e)
      setMethod(3);
    e=element->FirstChildElement(MBSIMINTNS"relativeToleranceScalar");
    setrTol(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"thresholdScalar");
    setThreshold(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"initialStepsize");
    if (e)
      setInitialStepSize(Element::getDouble(e));
  }

  void RKSuiteIntegrator::fzdot(double* t, double* z_, double* zd_) {
    Vec z(zSize, z_);
    Vec zd(zSize, zd_);
    system->zdot(z, zd, *t);
  }

  int RKSuiteIntegrator::zSize = 0;


}
