/* Copyright (C) 2004-2007  Robert Huber

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
 *   rhuber@users.berlios.de
 *
 */

#include<config.h>
#include<multi_body_system.h>
#include "time_stepping_ssc_integrator.h"
#include "eps.h"


#ifndef NO_ISO_14882
using namespace std;

#endif


using namespace fmatvec;

namespace MBSim {


  TimeSteppingSSCIntegrator::TimeSteppingSSCIntegrator() : dt(1e-6), dtMax(1e-2),driftCompensation(false), StepsWithUnchangedConstraints(-1), FlagSSC(true), aTol(1,INIT,1e-5), rTol(1,INIT,1e-4), gapTol(0), maxGainSSC(2.5), safetyFactorSSC(0.6), FlagPlotIntegrator(true), FlagPlotIntegrationSum(true), FlagCoutInfo(true), plotEveryStep(false), outputInterpolation(true), optimisedtforgaps(false), time(0.0), iter(0), maxIter(0), sumIter(0), integrationSteps(0), refusedSteps(0)
  {
    name = "TimeStepperSSC";
    dtMin = epsroot();
    safetyFactorGapControl = 1.0 + rTol(0)*10.0*2.0;
  }

  void TimeSteppingSSCIntegrator::integrate(MultiBodySystem& system_) { 
    initIntegrator(system_);
    IntegrationStep();
    closeIntegrator();
  }

  void TimeSteppingSSCIntegrator::initIntegrator(MultiBodySystem &system_) {
    system = &system_;
    int nq = system->getqSize(); // size of positions, velocities, state
    int nu = system->getuSize();
    int nx = system->getxSize();
    zSize = nq + nu + nx;

    Index Iq(0,nq-1);
    Index Ix(nq,nq+nx-1);
    Index Iu(nx+nq, zSize-1);

    z.resize(zSize);
    q.resize() >> z(Iq);
    x.resize() >> z(Ix);
    u.resize() >> z(Iu);

    zi.resize(zSize,INIT,0.0); 	// starting value for ith step
    z1e.resize(zSize);		// one step integration
    z2e.resize(zSize);  	// end of two step integration

    iter = 0;
    integrationSteps = 0;
    refusedSteps = 0;
    maxIter = 0;
    sumIter = 0;
    maxdtUsed = dtMin;
    mindtUsed = dtMax;

    if (aTol.size() < zSize) aTol.resize(zSize,INIT,aTol(0));
    if (rTol.size() < zSize) rTol.resize(zSize,INIT,rTol(0));
    assert(aTol.size() == zSize);
    assert(rTol.size() == zSize);

    if (dtPlot) plotEveryStep = false; 
    else plotEveryStep = true;  

    cout.setf(ios::scientific, ios::floatfield);
    if (FlagPlotIntegrator) {
      integPlot.open((system->getDirectoryName() + name + ".plt").c_str());
      integPlot << "#1 t [s]:" << endl; 
      integPlot << "#2 dt [s]:" << endl; 
      integPlot << "#3 iter  :" << endl;      
      integPlot << "#4 laSize:" << endl;
      integPlot << "#5 calculation time [s]:" << endl;
    }
  }

  void TimeSteppingSSCIntegrator::IntegrationStep() {
    s0 = clock();
    t = tStart;
    tPlot = tStart;
    if(z0.size()) zi = z0; 					// define initial state
    else system->initz(zi); 
    if (outputInterpolation) {
      laiBi.resize()  = system->getAllBilateralla();		// la = LA/dt [N]
      laiUni.resize() = system->getAllUnilateralla();
      laiBi.init(0.0);
      laiUni.init(0.0);
    }
    if(optimisedtforgaps) {
      z = zi;
      z2e = zi;
      system->updatezRef(z);
      system->updateKinematics(t);
      system->updateLinksStage1(t);
      Vec g_0;
      g_0 = system->getg();        
      bool finished;
      do {
	finished = gapControl(g_0);
      }
      while(! finished);
    }

    int StepFinished = 0;
    bool ExitIntegration = (t>=tEnd);
    int UnchangedSteps =0;
    bool ConstraintsChanged;
     
    while(! ExitIntegration) {
      while(StepFinished==0) {
	// two step integration
	double dtHalf = dt/2.0;
	if (FlagSSC) {
	  z  << zi;
	  q += system->deltaq(z,t,dtHalf);
	  system->update(z,t+dtHalf);
	  iter  = system->solve(dtHalf);
	  u += system->deltau(z,t+dtHalf,dtHalf);
	  x += system->deltax(z,t+dtHalf,dtHalf);

	  q += system->deltaq(z,t+dtHalf,dtHalf);
	  system->update(z,t+dt);
	  iter  += system->solve(dtHalf);
	  u += system->deltau(z,t+dt,dtHalf);
	  x += system->deltax(z,t+dt,dtHalf);
	  z2e << z;
	} 
	// one step integration
	z  << zi;
	q += system->deltaq(z,t,dt);
	ConstraintsChanged = system->update(z,t+dt);
	iter  = system->solve(dt);
	u += system->deltau(z,t+dt,dt);
	x += system->deltax(z,t+dt,dt);
	z1e << z;
	if (! FlagSSC) z2e >> z1e; 
	dtOld = dt;
        Vec g0_;
        g0_ = system->getg(); 
	if (testTolerances()) {
          StepFinished = 1;
	  sumIter += iter;
	  if(iter > maxIter) maxIter = iter;
	  if (maxdtUsed < dtOld) maxdtUsed = dtOld;
	  if (mindtUsed > dtOld) mindtUsed = dtOld;
          if(outputInterpolation) {
	    la1eBi.resize()  = system->getAllBilateralla()/dtOld;
	    la1eUni.resize() = system->getAllUnilateralla()/dtOld;
	  }
	  if(driftCompensation) {
	    system->updatezRef(z2e);
	    system->projectViolatedConstraints(t+dtOld);
	  }
	}
	else {
	  refusedSteps++;
	  if (dtOld == dtMin) {
	    StepFinished = -1;
	    cout << " TimeStepperSSC reached minimum stepsize dt= "<<dt<<" at t= "<<t<<endl;
	    //exit(StepFinished);
	  }
	}
	if (optimisedtforgaps) {
	  bool finished;
	  do {
	    finished = gapControl(g0_);
	  }
	  while(! finished);
	}
      }

      StepFinished = 0;
      integrationSteps++;
      ti = t;
      t += dtOld;
      plot();
      zi << z2e; 
      ExitIntegration = (t>=tEnd);
      if (outputInterpolation) {
	if (laiBi.size() != la1eBi.size()) laiBi.resize();
	laiBi = la1eBi;
	if (laiUni.size() != la1eUni.size()) laiUni.resize();
	laiUni = la1eUni;
      }
     if (StepsWithUnchangedConstraints>=0) {
       if (! ConstraintsChanged) UnchangedSteps++;
       else UnchangedSteps =0;
       if (UnchangedSteps >= StepsWithUnchangedConstraints) ExitIntegration = true;
     }
    }
  }

  void TimeSteppingSSCIntegrator::plot() {
    if ((plotEveryStep) || ((t>=tPlot)&&(outputInterpolation==false))) {
      tPlot+=dtPlot;
      z = z2e;
      system->plot(z,t,dtOld);
    } 
    else if ((t>=tPlot) && outputInterpolation) {
      while (t>=tPlot) { 
	if (laiBi.size() > la1eBi.size()) laiBi.resize()  = laiBi(0,la1eBi.size()-1);
	if (laiBi.size() < la1eBi.size()) la1eBi.resize() = la1eBi(0,laiBi.size()-1);
	if (laiUni.size() > la1eUni.size()) laiUni.resize()  = laiUni(0,la1eUni.size()-1);
	if (laiUni.size() < la1eUni.size()) la1eUni.resize() = la1eUni(0,laiUni.size()-1);
	Vec laBi, laUni;
	z = zi + (z1e-zi)*(tPlot-ti)/dtOld;
	laBi  = laiBi + (la1eBi-laiBi)*(tPlot-ti)/dtOld;
	laUni = laiUni + (la1eUni-laiUni)*(tPlot-ti)/dtOld;
	system->setAllBilateralla(laBi);
	system->setAllUnilateralla(laUni);
	system->plot(z,tPlot,1);			// la [N] !! (la!=N*s)
	tPlot +=dtPlot;
      }
    }
    else return;
    if (FlagPlotIntegrator) {
      double s1 = clock();
      time += (s1-s0)/CLOCKS_PER_SEC;
      s0 = s1; 
      integPlot<< t << " " << dtOld << " " <<  iter << " " << system->getlaSize()  << " "<< time  <<endl;
    }
    if(output) cout << "   t = " <<  t << ",\tdt = "<< dtOld << ",\titer = "<<setw(5)<<setiosflags(ios::left) <<iter<<  "\r"<<flush;
  }

  void TimeSteppingSSCIntegrator::closeIntegrator() {
    double s1 = clock();
    time += (s1-s0)/CLOCKS_PER_SEC;
    cout.unsetf(ios::scientific);
    if (FlagPlotIntegrator) integPlot.close();
    if (FlagPlotIntegrationSum) {
      ofstream integSum((system->getDirectoryName() + name + ".sum").c_str());
      integSum << "Integration time: " << time << endl;
      integSum << "Integration steps: " << integrationSteps << endl;
      if (FlagSSC) {
        integSum << "Refused steps: " << refusedSteps << endl;
        integSum << "Maximum step size: " << maxdtUsed << endl;
        integSum << "Minimum step size: " << mindtUsed << endl;
      }
      integSum << "Maximum number of iterations: " << maxIter << endl;
      integSum << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;
      integSum.close();
    }   
    if (FlagCoutInfo) {
      if (output) cout << endl <<endl;
      cout << "Summary Integration with TimeStepperSSC: "<<endl;
      cout << "Integration time: " << time << endl;
      cout << "Integration steps: " << integrationSteps << endl;
      if (FlagSSC) {
        cout << "Refused steps: " << refusedSteps << endl;
        cout << "Maximum step size: " << maxdtUsed << endl;
        cout << "Minimum step size: " << mindtUsed << endl;
      }
      cout << "Maximum number of iterations: " << maxIter << endl;
      cout << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;
    }
  }

  bool TimeSteppingSSCIntegrator::testTolerances() {
    if (! FlagSSC) return true;
    Vec EstErrorLocal;
    EstErrorLocal = z2e -z1e;

    bool testOK = true;
    double dtNewRel=0; 
    double dtNewRel_i;
    double ResTol_i;
    for (int i=0; i< zSize; i++) {
      EstErrorLocal(i) = fabs(EstErrorLocal(i));
      ResTol_i   = aTol(i) + rTol(i)*fabs(zi(i));
      dtNewRel_i = ResTol_i / EstErrorLocal(i);
      if (! i) dtNewRel = dtNewRel_i;
      else if (dtNewRel_i < dtNewRel) dtNewRel = dtNewRel_i;
      if ((testOK)&&(EstErrorLocal(i)> ResTol_i)) testOK = false;
    }
    if(dtNewRel > maxGainSSC) dtNewRel=maxGainSSC;   //  0.6 oder 0.7 und 2.5 oder 2
    if(dtNewRel < 1.0/maxGainSSC) dtNewRel = 1/maxGainSSC;
    dt = dt*safetyFactorSSC*dtNewRel;
    if (dt > dtMax) dt = dtMax;
    if (dt < dtMin) dt = dtMin;

    return testOK;
  }

  bool TimeSteppingSSCIntegrator::gapControl(const Vec &g) {
    Index gInd = system->getgIndUnilateral();
    Vec g0,ge;
    int gSize;
    
    g0 = g(gInd);
    gSize = g0.size();
    z << z2e;
    q += system->deltaq(z,t+dtOld,dt);
    system->updateKinematics(t+dtOld+dt);
    system->updateLinksStage1(t+dtOld+dt);
    ge = system->getg()(gInd);
    if (ge.size()<gSize) gSize = ge.size();

    Mat DTiMatTmp(gSize,2,INIT,-1.0);
    double DTiGTdtMin = 10*dt;
    int iDTiGTdtMin = -1;
    int tiSize=0;
    for(int i=0; i<gSize; i++) {
      if ((g0(i)>0)&&(1.5*ge(i)-0.5*g0(i)<0)) {
	double DTi = g0(i)*dt/(g0(i)-ge(i));
	if ((DTi>dt)&&(DTi<DTiGTdtMin)) {
	  DTiGTdtMin = DTi;
	  iDTiGTdtMin = i; 
	}
	if (DTi<=dt) {
	  DTiMatTmp(tiSize,0)= DTi;
	  DTiMatTmp(tiSize,1)= i;
	  tiSize++;
	}
      }
    }
    if (iDTiGTdtMin>-1) {
      DTiMatTmp(tiSize,0)= DTiGTdtMin;
      DTiMatTmp(tiSize,1)= iDTiGTdtMin;
      tiSize++;
    }

    if (tiSize) {
      if (tiSize == 1) {
	if (DTiMatTmp(0,0) <= dt) dt= DTiMatTmp(0,0)* safetyFactorGapControl;
	else dt = DTiMatTmp(0,0)/1.9;
      }
      else {
	Mat DTiAndIndex = quickSortMedian(DTiMatTmp(0,0,tiSize-1,1),0);
	int iOpt = 0;
	double Grading = (ge(int(DTiAndIndex(0,1)))-g0(int(DTiAndIndex(0,1))))*DTiAndIndex(0,0)*(DTiAndIndex(1,0)-DTiAndIndex(0,0));
	for(int i=1; i<tiSize-1; i++) {
	  double GradingTMP = (ge(int(DTiAndIndex(i,1)))-g0(int(DTiAndIndex(i,1))))*DTiAndIndex(i,0)*(DTiAndIndex(i+1,0)-DTiAndIndex(i,0));
	  if (GradingTMP>Grading) {
	    Grading = GradingTMP;
	    iOpt = i;
	  }
	}
	dt = safetyFactorGapControl*DTiAndIndex(iOpt,0);
      }
    }
    // Prüfe ob gapTol eingehalten wurde
    bool gapsOK = true;
    if (gapTol) {
      z << z2e;
      q += system->deltaq(z,t+dtOld,dt);
      system->updateKinematics(t+dtOld+dt);
      system->updateLinksStage1(t+dtOld+dt);
      ge.resize() = system->getg()(gInd);
      for(int i=0; i<tiSize; i++) {
	if (ge(int(DTiMatTmp(i,1))) < -gapTol) {
	  gapsOK = false;
	  break;
	}
      }
      if (! gapsOK) {
	if (dt/2.0 <= dtMin) gapsOK=true;
	else dt=dt/2.0;
	if (dt<dtMin*0.9) dt = dtMin*0.9;
      }
    }
    return gapsOK;
  }

}
