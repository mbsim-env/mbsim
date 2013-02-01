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
#include "fortran_wrapper.h"
#include "radau5_integrator.h"
#include <time.h>
#include <fstream>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;

namespace MBSim {

  RADAU5Integrator::RADAU5Integrator() : dt0(0), maxSteps(0), dtMax(0) {
  }

  double RADAU5Integrator::tPlot = 0;
  double RADAU5Integrator::dtOut = 0;
  Vec RADAU5Integrator::zInp;
  ofstream RADAU5Integrator::integPlot;
  double RADAU5Integrator::s0;
  double RADAU5Integrator::time = 0;
  bool RADAU5Integrator::output_;

  void RADAU5Integrator::fzdot(int* zSize, double* t, double* z_, double* zd_, double* rpar, int* ipar) {
    Vec z(*zSize, z_);
    Vec zd(*zSize, zd_);
    system->zdot(z, zd, *t);
  }

  void  RADAU5Integrator::plot(int* nr, double* told, double* t, double* z, double* cont, int* lrc, int* n, double* rpar, int* ipar, int* irtrn) {

    while(*t >= tPlot) {
      for(int i=1; i<=*n; i++)
	zInp(i-1) = CONTR5(&i,&tPlot,cont,lrc);
      system->plot(zInp, tPlot);
      if(output_)
	cout << "   t = " <<  tPlot << ",\tdt = "<< *t-*told << "\r"<<flush;

      double s1 = clock();
      time += (s1-s0)/CLOCKS_PER_SEC;
      s0 = s1; 

      integPlot<< tPlot << " " << *t-*told << " " << time << endl;
      tPlot += dtOut;
    }
  }

  void RADAU5Integrator::integrate(DynamicSystemSolver& system_) {

    system = &system_;
    int zSize=system->getzSize();
//    int nrDens = zSize;

    double t = tStart;

    Vec z(zSize);
    if(z0.size())
      z = z0;
    else
      system->initz(z);

    if(aTol.size() == 0) 
      aTol.resize(1,INIT,1e-6);
    if(rTol.size() == 0) 
      rTol.resize(1,INIT,1e-6);

    assert(aTol.size() == rTol.size());

    int iTol;
    if(aTol.size() == 1) {
      iTol = 0; // Skalar
    } else {
      iTol = 1; // Vektor
      assert (aTol.size() >= zSize);
    }

    int out = 2; // TODO

    double rPar;
    int iPar;

    int iJac = 0; // TODO
    int mlJac = zSize; // TODO
    int muJac = 0; // TODO
    int iMas = 0; // TODO
    int mlMas = zSize; // TODO
    int muMas = 0; // TODO

    int lJac = zSize; // TODO
    int lMas = 0; // TODO
    int le = zSize; // TODO

    int lWork =2*(zSize*(lJac+lMas+3*le+12)+20);
    int liWork = 2*(3*zSize+20);
    VecInt iWork(liWork);
    Vec work(lWork);

    if(dtMax>0)
      work(6) = dtMax;

    //Maximum Step Numbers
    iWork(1)=maxSteps;

    // if(warnLevel)
    //   iWork(2) = warnLevel;
    // else
    //   iWork(2) = -1;

    int idid;

    tPlot = t + dtPlot;
    dtOut = dtPlot;
    system->plot(z, t);

    zInp.resize(zSize);

    integPlot.open((name + ".plt").c_str());

    integPlot << "#1 t [s]:" << endl; 
    integPlot << "#1 dt [s]:" << endl; 
    integPlot << "#1 calculation time [s]:" << endl; 

    cout.setf(ios::scientific, ios::floatfield);

    output_ = output;

    s0 = clock();

    RADAU5(&zSize,fzdot,&t,z(),&tEnd,&dt0,
	rTol(),aTol(),&iTol,
	0,&iJac,&mlJac,&muJac,
	0,&iMas,&mlMas,&muMas,
	plot,&out,
	work(),&lWork,iWork(),&liWork,&rPar,&iPar,&idid);

    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    //integSum << "Integration steps: " << integrationSteps << endl;
    integSum.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

  void RADAU5Integrator::initializeUsingXML(TiXmlElement *element) {
    Integrator::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMINTNS"absoluteTolerance");
    if(e) setAbsoluteTolerance(Element::getVec(e));
    e=element->FirstChildElement(MBSIMINTNS"absoluteToleranceScalar");
    if(e) setAbsoluteTolerance(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"relativeTolerance");
    if(e) setRelativeTolerance(Element::getVec(e));
    e=element->FirstChildElement(MBSIMINTNS"relativeToleranceScalar");
    if(e) setRelativeTolerance(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"initialStepSize");
    setInitialStepSize(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"maximalStepSize");
    setMaximalStepSize(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"maximalNumberOfSteps");
    if (e)
      setMaxStepNumber(atoi(e->GetText()));
  }

}
