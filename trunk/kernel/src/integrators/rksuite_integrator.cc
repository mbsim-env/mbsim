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
 *   mfoerg@users.berlios.de
 *
 */

#include<config.h>
#include<ctime>
#include<fstream>
#include "multi_body_system.h"
#include "fortran_wrapper.h"
#include "rksuite_integrator.h"

#ifndef NO_ISO_14882
using namespace std;
#endif

namespace MBSim {

  RKSuiteIntegrator::RKSuiteIntegrator() : method(2), thres(1,INIT,1e-10), rTol(1e-6), dt0(0) {
  }

  int RKSuiteIntegrator::zSize = 0;

  void RKSuiteIntegrator::fzdot(double* t, double* z_, double* zd_) {
    Vec z(zSize, z_);
    Vec zd(zSize, zd_);
    system->zdot(z, zd, *t);
  }

  void RKSuiteIntegrator::integrate(MultiBodySystem& system_) {

    system = &system_;
    zSize=system->getzSize();

    double t=0.0;
    Vec z(zSize);
    if(z0.size())
      z = z0;
    else
      system->initz(z);
    double tEND=tEnd+dtPlot; // tEND must be greater than tEnd
    if(thres.size() == 1) {
      double buf = thres(0);
      thres.resize(zSize,INIT,buf);
    } 
    assert (thres.size() == zSize);

    char task='U';
    int errass=0;
    double dtStart=0.0; // automatical detection of dt start
    int ndworkarray=100000; /////////////////
    double *dworkarray=new double[ndworkarray];
    int messages;
    if(warnLevel)
      messages = 1;
    else 
      messages = 0;
    SETUP(&zSize, &t, z(), &tEND, &rTol, thres(), &method, &task,
	&errass, &dtStart, dworkarray, &ndworkarray, &messages);

    double tPlot=t+dtPlot;
    Vec zdGot(zSize);
    Vec zMax(zSize);
    int result;
//    bool donedrift;

    system->plot(z, t);

    double s0 = clock();
    double time = 0;
    int integrationSteps = 0;

    ofstream integPlot((system->getDirectoryName() + name + ".plt").c_str());

    cout.setf(ios::scientific, ios::floatfield);
    double dtLast = 0;
    while(t<=tEnd) {

      integrationSteps++;

      UT(fzdot, &tPlot, &t, z(), zdGot(), zMax(), dworkarray, 
	  &result, &dtLast);
      if(result==1 || result==2 || t==tPlot) {
	system->plot(z, t);

	if(output)
	  cout << "   t = " <<  t << ",\tdt = "<< dtLast << "\r"<<flush;

	double s1 = clock();
	time += (s1-s0)/CLOCKS_PER_SEC;
	s0 = s1; 
	integPlot<< t << " " << dtLast << " " << time << endl;

	tPlot += dtPlot;
      }
      if(result==3 || result==4)
	continue;
      if(result>=5) exit(result);
    }
    integPlot.close();

    ofstream integSum((system->getDirectoryName() + name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

}
