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
#include <fmatvec.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/utils/eps.h>
#include <mbsim/utils/utils.h>
#include "fortran_wrapper.h"
#include "lsodar_integrator.h"
#include <fstream>
#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBXMLUtils;

namespace MBSim {

  LSODARIntegrator::LSODARIntegrator() : dtMax(0), dtMin(0), rTol(1e-6), dt0(0), plotOnRoot(true) {
  }

  void LSODARIntegrator::fzdot(int* zSize, double* t, double* z_, double* zd_) {
    Vec z(*zSize, z_);
    Vec zd(*zSize, zd_);
    system->zdot(z, zd, *t);
  }

  void LSODARIntegrator::fsv(int* zSize, double* t, double* z_, int* nsv, double* sv_) {
    Vec z(*zSize, z_);
    Vec sv(*nsv, sv_);
    system->getsv(z, sv, *t);
  }

  void LSODARIntegrator::initializeUsingXML(TiXmlElement *element) {
    Integrator::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMINTNS"absoluteTolerance");
    if(e) setAbsoluteTolerance(Element::getVec(e));
    e=element->FirstChildElement(MBSIMINTNS"absoluteToleranceScalar");
    if(e) setAbsoluteTolerance(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"relativeToleranceScalar");
    if(e) setRelativeTolerance(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"initialStepSize");
    setInitialStepSize(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"minimalStepSize");
    setMinimalStepSize(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"maximalStepSize");
    setMaximalStepSize(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"plotOnRoot");
    setPlotOnRoot(Element::getBool(e));
  }

  TiXmlElement* LSODARIntegrator::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
    if(aTol.size()==0)
      addElementText(ele0,MBSIMINTNS"absoluteToleranceScalar",1e-6);
    else if(aTol.size()==1)
      addElementText(ele0,MBSIMINTNS"absoluteToleranceScalar",aTol(0));
    else
      addElementText(ele0,MBSIMINTNS"absoluteTolerance",aTol);
    addElementText(ele0,MBSIMINTNS"relativeToleranceScalar",rTol);
    addElementText(ele0,MBSIMINTNS"initialStepSize",dt0);
    addElementText(ele0,MBSIMINTNS"minimalStepSize",dtMin);
    addElementText(ele0,MBSIMINTNS"maximalStepSize",dtMax);
    addElementText(ele0,MBSIMINTNS"plotOnRoot",plotOnRoot);
    return ele0;
  }

  void LSODARIntegrator::integrate(DynamicSystemSolver& system) {
    preIntegrate(system);
    subIntegrate(system, tEnd);
    postIntegrate(system);
  }

  void LSODARIntegrator::preIntegrate(DynamicSystemSolver& system_) {
    system = &system_;
    zSize=system->getzSize();
    z.resize(zSize);
    if(z0.size())
      z = z0;
    else
      system->initz(z);
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
    //system->plot(z, t);
    s0 = clock();
    time = 0;
    integrationSteps = 0;
    integPlot.open((name + ".plt").c_str());
    jsv.resize(nsv);  
    cout.setf(ios::scientific, ios::floatfield);
  }

  void LSODARIntegrator::subIntegrate(DynamicSystemSolver& system_, double tStop) {
    int one = 1;
    int two = 2;
    rWork(4) = dt0;
    //system->shift(z, jsv, t);
    system->plot(z, t);
    cout << "System shiftet and plotted" << endl;
    while(t < tStop) {  
      integrationSteps++;
      DLSODAR(fzdot, &zSize, z(), &t, &tPlot, &iTol, &rTol, aTol(), &one,
          &istate, &one, rWork(), &lrWork, iWork(),
          &liWork, NULL, &two, fsv, &nsv, jsv());
      if(istate==2 || fabs(t-tPlot)<epsroot()) {
        system->plot(z, t);
        if(output)
          cout << "   t = " <<  t << ",\tdt = "<< rWork(10) << "\r"<<flush;
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        integPlot<< t << " " << rWork(10) << " " << time << endl;
        tPlot += dtPlot;
        if (tPlot > tStop)
          tPlot = tStop;
      }
      if(istate==3) {
        system->shift(z, jsv, t);
        if(plotOnRoot) {
          system->plot(z, t);
          system->plotAtSpecialEvent(t);
        }
        istate=1;
        rWork(4)=dt0;
      }
      if(istate<0) exit(istate);
    }
  }

  void LSODARIntegrator::postIntegrate(DynamicSystemSolver& system_) {
    system->plot(z, t);
    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum.close();
    
    cout.unsetf (ios::scientific);
    cout << endl;
  }

}
