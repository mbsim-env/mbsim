/* Copyright (C) 2004-2009  MBSim Development Team

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
 */

#include<config.h>
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/element.h"
#include "mbsim/link.h"
#include "time_stepping_ssc_integrator.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/stopwatch.h"
#include "mbsimtinyxml/tinyxml-src/tinynamespace.h"
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef NO_ISO_14882
using namespace std;
#endif


using namespace fmatvec;

namespace MBSim {


  TimeSteppingSSCIntegrator::TimeSteppingSSCIntegrator() : sysT1(NULL), sysT2(NULL), sysT3(NULL), dt(1e-6), dtOld(1e-6), dte(1e-6), dtMin(0), dtMax(1e-3), dt_SSC_vorGapControl(0), driftCompensation(false), t(0), tPlot(0), qSize(0), xSize(0), uSize(0),zSize(0), StepsWithUnchangedConstraints(-1), FlagErrorTest(2), FlagErrorTestAlwaysValid(true), aTol(1,INIT,1e-6), rTol(1,INIT,1e-4),FlagSSC(1), maxOrder(1), method(0), FlagGapControl(false), gapTol(1e-6), maxGainSSC(2.2), safetyFactorSSC(0.7), FlagPlotIntegrator(true), FlagPlotIntegrationSum(true), FlagCoutInfo(true), FlagPlotEveryStep(false), outputInterpolation(false), safetyFactorGapControl(-1), GapControlStrategy(1), numThreads(0), time(0.0), iter(0), iterA(0), iterB1(0), iterB2(0), iterC1(0), iterC2(0), iterC3(0), iterC4(0), iterB2RE(0), maxIterUsed(0), maxIter(0), sumIter(0), integrationSteps(0), integrationStepswithChange(0), refusedSteps(0), refusedStepsWithImpact(0), wrongAlertGapControl(0), stepsOkAfterGapControl(0), stepsRefusedAfterGapControl(0), statusGapControl(0), singleStepsT1(0), singleStepsT2(0), singleStepsT3(0), dtRelGapControl(1), qUncertaintyByExtrapolation(0), indexLSException(-1), Penetration(0), PenetrationCounter(0), PenetrationLog(0), PenetrationMin(0), PenetrationMax(0), maxdtUsed(0), mindtUsed(0), ChangeByGapControl(false), calcBlock2(0), IterConvergence(0), ConstraintsChanged(0), ConstraintsChangedBlock1(0), ConstraintsChangedBlock2(0), integrationStepsOrder1(0), integrationStepsOrder2(0), order(1), StepTrials(0), AnzahlAktiverKontakte(0), gNDurchschnittprostep(0) { 

    // Flags for Output 
    FlagPlotIntegrator     = true;
    FlagPlotIntegrationSum = true; 
    FlagCoutInfo           = true; 
    outputInterpolation    = false; 


    // SSC and GapControl
    FlagSSC = true;
    FlagErrorTest = 2;
    FlagErrorTestAlwaysValid = true;
    maxOrder = 1;
    method = 0;
    maxGainSSC = 2.2;
    safetyFactorSSC = 0.7;
    aTol(0) = 1e-6;
    rTol(0) = 1e-4;

    FlagGapControl = false;
    GapControlStrategy = 1;
    gapTol  = 1e-6;
    safetyFactorGapControl = -1;
    // integration
    numThreads = 1;
    driftCompensation = false;
    dtMin = 0;
    dtMax = 1e-3;

    // end options =================

    order = maxOrder;
  }

  TimeSteppingSSCIntegrator::~TimeSteppingSSCIntegrator() {
    SetValuedLinkListT1.clear();
    SetValuedLinkListT2.clear();
    SetValuedLinkListT3.clear();
  }

  void TimeSteppingSSCIntegrator::setFlagErrorTest(int Flag, bool alwaysValid) {
    FlagErrorTest = Flag;
    FlagErrorTestAlwaysValid = alwaysValid;
    assert(FlagErrorTest>=0);		// =0	control errors locally on all variables
    assert(FlagErrorTest<4);		// =2   u is scaled with stepsize
    assert(FlagErrorTest!=1);           // =3   exclude u from error test
  }

  void TimeSteppingSSCIntegrator::setMaxOrder(int order_, int method_) {
    maxOrder = order_;
    method   = method_;
    assert(method>=0);
    assert(method<3);
    assert(maxOrder>0);
    assert(maxOrder<5);
    order = maxOrder;
  }

  void TimeSteppingSSCIntegrator::integrate(DynamicSystemSolver& system_) {
    integrate(system_, system_, system_,1); 
  }

  void TimeSteppingSSCIntegrator::integrate(DynamicSystemSolver& systemT1_, DynamicSystemSolver& systemT2_, DynamicSystemSolver& systemT3_, int Threads) { 
    numThreads = Threads;
    preIntegrate(systemT1_, systemT2_, systemT3_);
    subIntegrate(systemT1_, tEnd);
    postIntegrate(systemT1_);
  }

  void TimeSteppingSSCIntegrator::preIntegrate(DynamicSystemSolver& system_) {
    preIntegrate(system_, system_, system_);
  }

  void TimeSteppingSSCIntegrator::preIntegrate(DynamicSystemSolver& systemT1_, DynamicSystemSolver& systemT2_, DynamicSystemSolver& systemT3_) {

    sysT1 = &systemT1_;
    sysT2 = &systemT2_;
    sysT3 = &systemT3_;

    t = tStart;

    if (dtMin<=0) {
      double eps1 = epsroot();
      dtMin = eps1;
      if (!(method==0 && maxOrder==1 && !FlagSSC)) dtMin =2.0*eps1;
      if (maxOrder>=2) dtMin = 4.0*eps1;
      if (maxOrder>=3) dtMin = 6.0*eps1;
      if (method==0 && maxOrder==3 && !FlagSSC) dtMin = 3.0*eps1;
      if (method==0 && maxOrder==2 && !FlagSSC) dtMin = 2.0*eps1;
    }


    if ( safetyFactorGapControl <0) {
      safetyFactorGapControl= 1.0 + nrmInf(rTol)*100;
      if (safetyFactorGapControl>1.001) safetyFactorGapControl=1.001;
    }
    if (FlagSSC && method) safetyFactorSSC = pow(0.3,1.0/(maxOrder+1));
    if (!FlagSSC) method=0;
    if ((!method) && (maxOrder>=3) && (FlagSSC)) maxOrder=3;
    if (!method && maxOrder<1) maxOrder=1;


    qSize = sysT1->getqSize(); // size of positions, velocities, state
    uSize = sysT1->getuSize();
    xSize = sysT1->getxSize();
    zSize = qSize + uSize + xSize;

    Index Iq(0,qSize-1);
    Index Iu(qSize, qSize+uSize-1);
    Index Ix(qSize+uSize, zSize-1);

    zT1.resize(zSize);
    if (sysT1==sysT2) zT2.resize()>>zT1; else zT2.resize(zSize);
    if (sysT1==sysT3) zT3.resize()>>zT1; else zT3.resize(zSize);

    qT1.resize() >> zT1(Iq);
    uT1.resize() >> zT1(Iu);
    xT1.resize() >> zT1(Ix);

    qT2.resize() >> zT2(Iq);
    uT2.resize() >> zT2(Iu);
    xT2.resize() >> zT2(Ix);

    qT3.resize() >> zT3(Iq);
    uT3.resize() >> zT3(Iu);
    xT3.resize() >> zT3(Ix);

    zi.resize(zSize,INIT,0.0); 	// starting value for ith step

    SetValuedLinkListT1.clear();
    SetValuedLinkListT2.clear();
    SetValuedLinkListT3.clear();

    sysT1->buildListOfSetValuedLinks(SetValuedLinkListT1,true);
    sysT2->buildListOfSetValuedLinks(SetValuedLinkListT2,true);
    sysT3->buildListOfSetValuedLinks(SetValuedLinkListT3,true);

    maxIter = sysT1->getMaxIter();
    iter= 0;
    integrationSteps= 0;
    integrationStepsOrder1= 0;
    integrationStepsOrder2= 0;
    integrationStepswithChange =0;
    refusedSteps = 0;
    wrongAlertGapControl =0;
    stepsOkAfterGapControl =0;
    Penetration=0;
    PenetrationLog=0;
    PenetrationCounter=0;
    PenetrationMax = 0;
    PenetrationMin = -1;
    stepsRefusedAfterGapControl =0;
    maxIterUsed = 0;
    sumIter = 0;
    StepTrials=0;
    singleStepsT1 = 0;
    singleStepsT2 = 0;
    singleStepsT3 = 0;
    maxdtUsed = dtMin;
    mindtUsed = dtMax;

    if (aTol.size() < zSize) {
      double aTol0=aTol(0);
      aTol.resize(zSize,INIT,aTol0);
    } 
    if (rTol.size() < zSize) {
      double rTol0=rTol(0);
      rTol.resize(zSize,INIT,rTol0);
    }
    assert(aTol.size() == zSize);
    assert(rTol.size() == zSize);

    if (dtPlot<=0) FlagPlotEveryStep = true;
    else FlagPlotEveryStep = false;

    cout.setf(ios::scientific, ios::floatfield);
    if (FlagPlotIntegrator) {
      integPlot.open((name + ".plt").c_str());
      integPlot << "#1 t [s]:" << endl; 
      integPlot << "#2 dt [s]:" << endl;
      integPlot << "#3 order :" << endl; 
      integPlot << "#4 iter  :" << endl;      
      integPlot << "#5 laSize:" << endl;
      integPlot << "#6 active contacts: " << endl;
      integPlot << "#7 calculation time [s]:" << endl;
    }
    
    if(z0.size()) zi = z0; 					// define initial state
    else sysT1->initz(zi); 

    sysT1->plot(zi,t,1.);

    tPlot = t;
  }

  void TimeSteppingSSCIntegrator::subIntegrate(DynamicSystemSolver& system, double tStop) { // system: only dummy!
    Timer.start();
    if (outputInterpolation) {
      getAllSetValuedla(la,laSizes,SetValuedLinkListT1);
      la.init(0.0);
    }

    qUncertaintyByExtrapolation=0;

    int StepFinished = 0;
    bool ExitIntegration = (t>=tStop);
    int UnchangedSteps =0;
    double dtHalf;
    double dtQuarter;
    double dtThird;
    double dtSixth;

    bool ConstraintsChangedA;
    bool ConstraintsChangedB;
    bool ConstraintsChangedC;
    bool ConstraintsChangedD;

    bool calcBlock2;
    bool Block2IsOptional = true;
    bool calcJobBT1 = false;                   // B: dt/2;  C: dt/4;  D: dt/6;  E: dt/3;
    bool calcJobBT2 = false;
    bool calcJobB2RET1 = false;
    bool calcJobB2RET2 = false;
    bool calcJobC = false;
    bool calcJobD = false;
    bool calcJobE1T1 = false;
    bool calcJobE23T1= false;
    bool calcJobE12T2= false;
    bool calcJobE3T2 = false;

    if (method == 0) {
      if (FlagSSC) {
        if (maxOrder==1) {Block2IsOptional= false; calcJobBT2 = true;}
        if (maxOrder==2) {
          calcJobBT1 = true;
          calcJobB2RET1 = true;
          calcJobC = true;
        }
        if (maxOrder==3) {
          calcJobBT1 = true;
          calcJobB2RET2 = true;
          calcJobC = true;
          calcJobD = true;
          calcJobE1T1 = true;
          calcJobE23T1= true;
        }
      }
      else {            // FlagSSC==false
        if (maxOrder==2) 
          calcJobBT2 = true;
        if (maxOrder==3) {
          calcJobBT1 = true;
          calcJobE12T2 = true;
          calcJobE3T2  = true;
        }
        if (maxOrder==4) {
          calcJobBT1 = true;
          calcJobC = true;
          calcJobD = true;
        }
      }
    }

    if (method>0) {
      if (maxOrder==1)  {calcJobBT2 = true; Block2IsOptional=false;}
      if (maxOrder > 1) {calcJobBT1 = true;  calcJobC = true;}
      if (maxOrder > 2) calcJobD = true;
      if (maxOrder > 3) {calcJobE1T1 = true; calcJobE23T1 = true; }
    } 

    if (FlagSSC==false) Block2IsOptional=false;

    if (numThreads ==0) {
      numThreads=2;
      if (calcJobD) numThreads++;
      if (method==0 && !FlagSSC && maxOrder==1) numThreads=1;
    }

    sysT1->getLinkStatus(LStmp,t);
    LS = LStmp;
    while(! ExitIntegration) 
    {
      while(StepFinished==0) 
      {
        dtHalf     = dt/2.0;
        dtQuarter  = dt/4.0;
        dtThird    = dt/3.0;
        dtSixth    = dt/6.0;
        ConstraintsChangedB = false;
        ConstraintsChangedC = false;
        ConstraintsChangedD = false;

        // Block 1 
#pragma omp parallel num_threads(numThreads)
        {
#pragma omp sections

          {
#pragma omp section           //thread 1
            {
              // one step integration (A)
              zT1 << zi;
              qT1 += sysT1->deltaq(zT1,t,dt);
              sysT1->update(zT1,t+dt,1);
              sysT1->getb().resize() = sysT1->getgd() + sysT1->getW().T()*slvLLFac(sysT1->getLLM(),sysT1->geth())*dt;
              iterA  = sysT1->solveImpacts(dt);
              getAllSetValuedla(la1d,la1dSizes,SetValuedLinkListT1);
              la1d/=dt;
              uT1 += sysT1->deltau(zT1,t+dt,dt);
              xT1 += sysT1->deltax(zT1,t+dt,dt);
              // wird jetzt von testTolerances() direkt aufgerufen; nach jedem erfolgreichen Schritt!
              //              if (FlagGapControl) {
              //                sysT1->updateStateDependentVariables(t+dt);
              //                getDataForGapControl(SetValuedLinkListT1);
              //              }
              sysT1->getLinkStatus(LStmp,t+dt);
              LSA = LStmp;
              ConstraintsChangedA = changedLinkStatus(LSA,LS,indexLSException);
              singleStepsT1++;
              z1d << zT1;

              // two step integration (first step) (B1) 
              if (calcJobBT1) {
                zT1  << zi;
                qT1 += sysT1->deltaq(zT1,t,dtHalf);
                sysT1->update(zT1,t+dtHalf,1);
                sysT1->getb().resize() = sysT1->getgd() + sysT1->getW().T()*slvLLFac(sysT1->getLLM(),sysT1->geth())*dtHalf;
                iterB1  = sysT1->solveImpacts(dtHalf);
                getAllSetValuedla(la2b,la2bSizes,SetValuedLinkListT1);
                la2b/=dtHalf;
                uT1 += sysT1->deltau(zT1,t+dtHalf,dtHalf);
                xT1 += sysT1->deltax(zT1,t+dtHalf,dtHalf);
                sysT1->getLinkStatus(LStmp,t+dtHalf);
                LSB1 = LStmp;
                ConstraintsChangedB = changedLinkStatus(LSB1,LS,indexLSException);
                singleStepsT1++;
                z2b << zT1;
              }
              // three step integration (first step) (E1) 
              if (calcJobE1T1) {

                zT1  << zi;
                qT1 += sysT1->deltaq(zT1,t,dtThird);
                sysT1->update(zT1,t+dtThird,1);
                sysT1->getb().resize() = sysT1->getgd() + sysT1->getW().T()*slvLLFac(sysT1->getLLM(),sysT1->geth())*dtThird;
                sysT1->solveImpacts(dtThird);
                uT1 += sysT1->deltau(zT1,t+dtThird,dtThird);
                xT1 += sysT1->deltax(zT1,t+dtThird,dtThird); 
                singleStepsT1++;
                z3b << zT1;
              }
            }  // close omp thread 1

#pragma omp section	        //thread 2
            {
              // two step integration (first step) (B1) 
              if (calcJobBT2) {
                zT2  << zi;
                qT2 += sysT2->deltaq(zT2,t,dtHalf);
                sysT2->update(zT2,t+dtHalf,1);
                sysT2->getb().resize() = sysT2->getgd() + sysT2->getW().T()*slvLLFac(sysT2->getLLM(),sysT2->geth())*dtHalf;
                iterB1  = sysT2->solveImpacts(dtHalf);
                getAllSetValuedla(la2b,la2bSizes,SetValuedLinkListT2);
                la2b/=dtHalf;
                uT2 += sysT2->deltau(zT2,t+dtHalf,dtHalf);
                xT2 += sysT2->deltax(zT2,t+dtHalf,dtHalf);
                sysT2->getLinkStatus(LStmp,t+dtHalf);
                LSB1 = LStmp;
                ConstraintsChangedB = changedLinkStatus(LSB1,LS,indexLSException);
                singleStepsT2++;
                z2b << zT2;
              }

              // four step integration (first two steps) (C12)
              if (calcJobC) {
                zT2  << zi;
                qT2 += sysT2->deltaq(zT2,t,dtQuarter);
                sysT2->update(zT2,t+dtQuarter,1);
                sysT2->getb().resize() = sysT2->getgd()+sysT2->getW().T()*slvLLFac(sysT2->getLLM(),sysT2->geth())*dtQuarter;
                iterC1 = sysT2->solveImpacts(dtQuarter);
                uT2 += sysT2->deltau(zT2,t+dtQuarter,dtQuarter);
                xT2 += sysT2->deltax(zT2,t+dtQuarter,dtQuarter);
                sysT2->getLinkStatus(LStmp,t+dtQuarter);
                LSC1 = LStmp;
                ConstraintsChangedC =  changedLinkStatus(LSC1,LS,indexLSException);

                qT2 += sysT2->deltaq(zT2,t+dtQuarter,dtQuarter);
                sysT2->update(zT2,t+dtHalf);
                sysT2->getb().resize() = sysT2->getgd()+sysT2->getW().T()*slvLLFac(sysT2->getLLM(),sysT2->geth())*dtQuarter;
                iterC2 = sysT2->solveImpacts(dtQuarter);
                uT2 += sysT2->deltau(zT2,t+dtHalf,dtQuarter);
                xT2 += sysT2->deltax(zT2,t+dtHalf,dtQuarter);
                sysT2->getLinkStatus(LStmp,t+dtHalf);
                LSC2 = LStmp;
                ConstraintsChangedC = ConstraintsChangedC || changedLinkStatus(LSC2,LSC1,indexLSException);
                singleStepsT2+=2;
                z4b << zT2;
              }
              // three step integration (first two steps ) (E12)
              if (calcJobE12T2) {
                zT2  << zi;
                qT2 += sysT2->deltaq(zT2,t,dtThird);
                sysT2->update(zT2,t+dtThird,1);
                sysT2->getb().resize() = sysT2->getgd() + sysT2->getW().T()*slvLLFac(sysT2->getLLM(),sysT2->geth())*dtThird;
                sysT2->solveImpacts(dtThird);
                uT2 += sysT2->deltau(zT2,t+dtThird,dtThird);
                xT2 += sysT2->deltax(zT2,t+dtThird,dtThird);

                qT2 += sysT2->deltaq(zT2,t+dtThird,dtThird);
                sysT2->update(zT2,t+2.0*dtThird);
                sysT2->getb().resize() = sysT2->getgd() + sysT2->getW().T()*slvLLFac(sysT2->getLLM(),sysT2->geth())*dtThird;
                sysT2->solveImpacts(dtThird);
                uT2 += sysT2->deltau(zT2,t+2.0*dtThird,dtThird);
                xT2 += sysT2->deltax(zT2,t+2.0*dtThird,dtThird);
                singleStepsT2+=2;
                z3b << zT2;
              }
            }  // close omp thread 2

#pragma omp section	        //thread 3
            {
              // six step integration (first three steps)   
              if(calcJobD) {
                zT3  << zi;
                qT3 += sysT3->deltaq(zT3,t,dtSixth);
                sysT3->update(zT3,t+dtSixth,1);
                sysT3->getb().resize() = sysT3->getgd() + sysT3->getW().T()*slvLLFac(sysT3->getLLM(),sysT3->geth())*dtSixth;
                sysT3->solveImpacts(dtSixth);
                uT3 += sysT3->deltau(zT3,t+dtSixth,dtSixth);
                xT3 += sysT3->deltax(zT3,t+dtSixth,dtSixth);
                sysT3->getLinkStatus(LStmp,t+dtSixth);
                LSD1 = LStmp;
                ConstraintsChangedD = changedLinkStatus(LSD1,LS,indexLSException);

                qT3 += sysT3->deltaq(zT3,t+dtSixth,dtSixth);
                sysT3->update(zT3,t+dtThird);
                sysT3->getb().resize() = sysT3->getgd() + sysT3->getW().T()*slvLLFac(sysT3->getLLM(),sysT3->geth())*dtSixth;
                sysT3->solveImpacts(dtSixth);
                uT3 += sysT3->deltau(zT3,t+dtThird,dtSixth);
                xT3 += sysT3->deltax(zT3,t+dtThird,dtSixth);
                sysT3->getLinkStatus(LStmp,t+dtThird);
                LSD2 = LStmp;
                ConstraintsChangedD = ConstraintsChangedD || changedLinkStatus(LSD2,LSD1,indexLSException);


                qT3 += sysT3->deltaq(zT3,t+dtThird,dtSixth);
                sysT3->update(zT3,t+dtHalf);
                sysT3->getb().resize() = sysT3->getgd() + sysT3->getW().T()*slvLLFac(sysT3->getLLM(),sysT3->geth())*dtSixth;
                sysT3->solveImpacts(dtSixth);
                uT3 += sysT3->deltau(zT3,t+dtHalf,dtSixth);
                xT3 += sysT3->deltax(zT3,t+dtHalf,dtSixth);
                sysT3->getLinkStatus(LStmp,t+dtHalf);
                LSD3 = LStmp;
                ConstraintsChangedD = ConstraintsChangedD || changedLinkStatus(LSD3,LSD2,indexLSException);
                singleStepsT3+=3;
                z6b << zT3;
              }
            }  // close omp thread 3

          }  // close omp sections (first sections)

#pragma omp single
          {
            ConstraintsChangedBlock1 = ConstraintsChangedB || ConstraintsChangedC || ConstraintsChangedD;
            ConstraintsChanged       = ConstraintsChangedA || ConstraintsChangedBlock1;
            calcBlock2 = (Block2IsOptional==false) || (ConstraintsChangedBlock1==false);
            ConstraintsChangedA = false;
            ConstraintsChangedB = false;
            ConstraintsChangedC = false;
            ConstraintsChangedD = false;
            ConstraintsChangedBlock2 = false;        

            if (FlagSSC && method==0 && maxOrder==3) zStern << 0.5*z2b - 4.0*z4b + 4.5*z6b;
            if (FlagSSC && method==0 && maxOrder==2) zStern << 2.0*z4b - z2b;
          }
          // Block 2
#pragma omp sections
          {
#pragma omp section                     // thread 1
            {
              // two step integration (B2)
              if (calcJobBT1 && calcBlock2) {
                zT1  << z2b;
                qT1 += sysT1->deltaq(zT1,t+dtHalf,dtHalf);
                sysT1->update(zT1,t+dt,1);
                sysT1->getb().resize() = sysT1->getgd() + sysT1->getW().T()*slvLLFac(sysT1->getLLM(),sysT1->geth())*dtHalf;
                iterB2  = sysT1->solveImpacts(dtHalf);
                uT1 += sysT1->deltau(zT1,t+dt,dtHalf);
                xT1 += sysT1->deltax(zT1,t+dt,dtHalf);
                sysT1->getLinkStatus(LStmp,t+dt);
                LSB2 = LStmp;
                ConstraintsChangedB = changedLinkStatus(LSB2,LSB1,indexLSException);
                singleStepsT1++;
                z2d << zT1;
              }
              if (calcJobB2RET1 && calcBlock2) { //B2RE
                zT1 << zStern;
                qT1 += sysT1->deltaq(zT1,t+dtHalf,dtHalf);
                sysT1->update(zT1,t+dt,1);
                sysT1->getb().resize() = sysT1->getgd() + sysT1->getW().T()*slvLLFac(sysT1->getLLM(),sysT1->geth())*dtHalf;
                iterB2RE  = sysT1->solveImpacts(dtHalf);
                uT1 += sysT1->deltau(zT1,t+dt,dtHalf);
                xT1 += sysT1->deltax(zT1,t+dt,dtHalf);
                singleStepsT1++;
                z2dRE << zT1;
              }
              // three step integration (E23) last two steps
              if (calcJobE23T1 && calcBlock2) {
                zT1  << z3b;
                qT1 += sysT1->deltaq(zT1,t+dtThird,dtThird);
                sysT1->update(zT1,t+2.0*dtThird,1);
                sysT1->getb().resize() = sysT1->getgd() + sysT1->getW().T()*slvLLFac(sysT1->getLLM(),sysT1->geth())*dtThird;
                sysT1->solveImpacts(dtThird);
                uT1 += sysT1->deltau(zT1,t+2.0*dtThird,dtThird);
                xT1 += sysT1->deltax(zT1,t+2.0*dtThird,dtThird);

                qT1 += sysT1->deltaq(zT1,t+2.0*dtThird,dtThird);
                sysT1->update(zT1,t+dt);
                sysT1->getb().resize() = sysT1->getgd() + sysT1->getW().T()*slvLLFac(sysT1->getLLM(),sysT1->geth())*dtThird;
                sysT1->solveImpacts(dtThird);
                uT1 += sysT1->deltau(zT1,t+dt,dtThird);
                xT1 += sysT1->deltax(zT1,t+dt,dtThird);
                singleStepsT1+=2;
                z3d << zT1;
              } 
            }  // close omp thread 1
#pragma omp section                   // thread 2
            { 
              // four step integration (C3 and C4) (last two steps )
              if (calcJobC && calcBlock2) {
                if (method) zT2 << z4b;
                else zT2 << zStern; 
                qT2 += sysT2->deltaq(zT2,t+dtHalf,dtQuarter);
                sysT2->update(zT2,t+dtHalf+dtQuarter,1);
                sysT2->getb().resize() = sysT2->getgd()+sysT2->getW().T()*slvLLFac(sysT2->getLLM(),sysT2->geth())*dtQuarter;
                iterC3  = sysT2->solveImpacts(dtQuarter);
                uT2 += sysT2->deltau(zT2,t+dtHalf+dtQuarter,dtQuarter);
                xT2 += sysT2->deltax(zT2,t+dtHalf+dtQuarter,dtQuarter);
                sysT2->getLinkStatus(LStmp,t+dtHalf+dtQuarter);
                LSC3 = LStmp;
                qT2 += sysT2->deltaq(zT2,t+dtHalf+dtQuarter,dtQuarter);
                sysT2->update(zT2,t+dt);
                sysT2->getb().resize() = sysT2->getgd()+sysT2->getW().T()*slvLLFac(sysT2->getLLM(),sysT2->geth())*dtQuarter;
                iterC4 = sysT2->solveImpacts(dtQuarter);
                uT2 += sysT2->deltau(zT2,t+dt,dtQuarter);
                xT2 += sysT2->deltax(zT2,t+dt,dtQuarter);
                sysT2->getLinkStatus(LStmp,t+dt);
                LSC4 = LStmp;
                ConstraintsChangedC = changedLinkStatus(LSC2,LSC3,indexLSException);
                ConstraintsChangedC = ConstraintsChangedC || changedLinkStatus(LSC3,LSC4,indexLSException);
                singleStepsT2+=2;
                z4d << zT2;
              }

              // two step integration (B2)
              if (calcJobBT2 && calcBlock2) {
                zT2  << z2b;
                qT2 += sysT2->deltaq(zT2,t+dtHalf,dtHalf);
                sysT2->update(zT2,t+dt,1);
                sysT2->getb().resize() = sysT2->getgd() + sysT2->getW().T()*slvLLFac(sysT2->getLLM(),sysT2->geth())*dtHalf;
                iterB2  = sysT2->solveImpacts(dtHalf);
                uT2 += sysT2->deltau(zT2,t+dt,dtHalf);
                xT2 += sysT2->deltax(zT2,t+dt,dtHalf);
                sysT2->getLinkStatus(LStmp,t+dt);
                LSB2 = LStmp;
                ConstraintsChangedB = changedLinkStatus(LSB2,LSB1,indexLSException);

                getAllSetValuedla(la2b,la2bSizes,SetValuedLinkListT2);
                la2b/=dtHalf;
                singleStepsT2++;
                z2d << zT2; 
              }
              if (calcJobB2RET2 && calcBlock2) { //B2RE
                zT2 << zStern;
                qT2 += sysT2->deltaq(zT2,t+dtHalf,dtHalf);
                sysT2->update(zT2,t+dt,1);
                sysT2->getb().resize() = sysT2->getgd() + sysT2->getW().T()*slvLLFac(sysT2->getLLM(),sysT2->geth())*dtHalf;
                iterB2RE  = sysT2->solveImpacts(dtHalf);
                uT2 += sysT2->deltau(zT2,t+dt,dtHalf);
                xT2 += sysT2->deltax(zT2,t+dt,dtHalf);
                singleStepsT2++;
                z2dRE << zT2;
              }
              // three step integration (E3) last step
              if (calcJobE3T2 && calcBlock2) {
                zT2  << z3b;
                qT2 += sysT2->deltaq(zT2,t+2.0*dtThird,dtThird);
                sysT2->update(zT2,t+dt,1);
                sysT2->getb().resize() = sysT2->getgd() + sysT2->getW().T()*slvLLFac(sysT2->getLLM(),sysT2->geth())*dtThird;
                sysT2->solveImpacts(dtThird);
                uT2 += sysT2->deltau(zT2,t+dt,dtThird);
                xT2 += sysT2->deltax(zT2,t+dt,dtThird);
                singleStepsT2++;
                z3d << zT2;
              }
            }  // close omp thread 2
#pragma omp section	        //thread 3
            {
              // six step integration (last three steps) 
              if (calcJobD && calcBlock2) {
                if (method) zT3  << z6b;
                else zT3<<zStern;
                qT3 += sysT3->deltaq(zT3,t+dtHalf,dtSixth);
                sysT3->update(zT3,t+4.0*dtSixth,1);
                sysT3->getb().resize() = sysT3->getgd() + sysT3->getW().T()*slvLLFac(sysT3->getLLM(),sysT3->geth())*dtSixth;
                sysT3->solveImpacts(dtSixth);
                uT3 += sysT3->deltau(zT3,t+4.0*dtSixth,dtSixth);
                xT3 += sysT3->deltax(zT3,t+4.0*dtSixth,dtSixth);
                sysT3->getLinkStatus(LStmp,t+4.0*dtSixth);
                LSD4 = LStmp;
                ConstraintsChangedD =  changedLinkStatus(LSD4,LSD3,indexLSException);

                qT3 += sysT3->deltaq(zT3,t+4.0*dtSixth,dtSixth);
                sysT3->update(zT3,t+5.0*dtSixth);
                sysT3->getb().resize() = sysT3->getgd() + sysT3->getW().T()*slvLLFac(sysT3->getLLM(),sysT3->geth())*dtSixth;
                sysT3->solveImpacts(dtSixth);
                uT3 += sysT3->deltau(zT3,t+5.0*dtSixth,dtSixth);
                xT3 += sysT3->deltax(zT3,t+5.0*dtSixth,dtSixth);
                sysT3->getLinkStatus(LStmp,t+5.0*dtSixth);
                LSD5 = LStmp;
                ConstraintsChangedD = ConstraintsChangedD || changedLinkStatus(LSD4,LSD5,indexLSException);

                qT3 += sysT3->deltaq(zT3,t+5.0*dtSixth,dtSixth);
                sysT3->update(zT3,t+dt);
                sysT3->getb().resize() = sysT3->getgd() + sysT3->getW().T()*slvLLFac(sysT3->getLLM(),sysT3->geth())*dtSixth;
                sysT3->solveImpacts(dtSixth);
                uT3 += sysT3->deltau(zT3,t+dt,dtSixth);
                xT3 += sysT3->deltax(zT3,t+dt,dtSixth);
                sysT3->getLinkStatus(LStmp,t+dt);
                LSD6 = LStmp;
                ConstraintsChangedD = ConstraintsChangedD || changedLinkStatus(LSD5,LSD6,indexLSException);

                singleStepsT3+=3;
                z6d << zT3;
              }
            }  // close omp thread 3



          }  // close omp sections (second sections)
        }  // close omp parallel

        ConstraintsChangedBlock2 = ConstraintsChangedB || ConstraintsChangedC || ConstraintsChangedD;
        ConstraintsChanged = ConstraintsChanged || ConstraintsChangedBlock2;
        if (integrationSteps>0 && ChangeByGapControl && !ConstraintsChanged) wrongAlertGapControl++;

        dtOld = dt;
        if (testTolerances()) {
          StepFinished = 1;
          sumIter += iter;
          if(iter > maxIterUsed) maxIterUsed = iter;
          if (maxdtUsed < dtOld) maxdtUsed = dtOld;
          if (mindtUsed > dtOld) mindtUsed = dtOld;
        }
        else {
          refusedSteps++;
          if (ConstraintsChanged) refusedStepsWithImpact++;
          StepTrials++;
          if (dtOld<=dtMin+macheps()) {
            StepFinished = -1;
            cerr << " TimeStepperSSC reached minimum stepsize dt= "<<dt<<" at t= "<<t<<endl;
            //exit(StepFinished);
          }
        }
      }

      StepTrials=0;
      StepFinished = 0;
      integrationSteps++; 
      if(order==1)integrationStepsOrder1++;
      if(order>=2)integrationStepsOrder2++;
      t += dte;
      plot();
      zi << ze;
      LS << LSe;
      if (outputInterpolation) {
        if (la.size() != lae.size()) { la.resize(); laSizes.resize();}
        la = lae;
        laSizes = laeSizes;
      }

      if(ConstraintsChanged) integrationStepswithChange++;
      if (t+t*10.0*macheps()>tStop) ExitIntegration = 1;
      else ExitIntegration=0;
      if (t+dt+dtMin>tStop) dt= tStop-t;
      if (StepsWithUnchangedConstraints>=0) {
        if (! ConstraintsChanged) UnchangedSteps++;
        else UnchangedSteps =0;
        if (UnchangedSteps >= StepsWithUnchangedConstraints) ExitIntegration = true;
      }
    }
  }

  void TimeSteppingSSCIntegrator::plot() {
    if ((FlagPlotEveryStep) || ((t>=tPlot)&&(outputInterpolation==false))) {
      tPlot+=dtPlot;
      zT1 << ze;
      setAllSetValuedla(lae,laeSizes,SetValuedLinkListT1);
      sysT1->plot(zT1,t,1);
    }
    if ((t>=tPlot) && outputInterpolation && !FlagPlotEveryStep) {

      // Dimension/size von la und lae synchronisieren
      Vec laSynchron;
      Vec laeSynchron;
      Vector<int> laSizesSynchron;
      if(t>tPlot) {
        int nLinks = laSizes.size();
        if (nLinks>laeSizes.size()) nLinks=laeSizes.size();
        int nlaAll=0;
        for(int i=0; i<nLinks; i++) {
          if (laSizes(i)>laeSizes(i)) nlaAll+=laeSizes(i);
          else nlaAll+=laSizes(i);
        }
        laSynchron.resize(nlaAll,NONINIT);
        laeSynchron.resize(nlaAll,NONINIT);
        laSizesSynchron.resize(nlaAll,NONINIT);
        int ila=0;
        int ilae=0;
        int iSynchron=0;
        for(int i=0; i<nLinks;i++) {
          laSizesSynchron(i) = laSizes(i);
          if (laSizesSynchron(i)>laeSizes(i)) laSizesSynchron(i) = laeSizes(i);
          laSynchron(iSynchron,iSynchron+laSizesSynchron(i)-1) = la(ila,ila+laSizesSynchron(i)-1);
          laeSynchron(iSynchron,iSynchron+laSizesSynchron(i)-1) = lae(ilae,ilae+laSizesSynchron(i)-1);
          ilae+=laeSizes(i);
          ila+=laSizes(i);
          iSynchron+=laSizesSynchron(i);
        }
      }
      while (t>tPlot) {
        double ratio = (tPlot -(t-dte))/dte;
        zT1 << zi + (ze-zi)*ratio;
        setAllSetValuedla(laSynchron+(laeSynchron-laSynchron)*ratio,laSizesSynchron,SetValuedLinkListT1);
        sysT1->plot(zT1,tPlot,1);
        tPlot += dtPlot;
      }
    }

    if (FlagPlotIntegrator) {
      time += Timer.stop();
      integPlot<< t << " " << dtOld << " " <<order << " " << iter << " " << sysT1->getlaSize()  << " "<<AnzahlAktiverKontakte<<" "<<time  <<endl;
    }
    if(output) cout << "   t = " <<  t << ",\tdt = "<< dtOld << ",\titer = "<<setw(5)<<setiosflags(ios::left) <<iter<<",\torder = "<<order << "\r"<<flush;
  }

  void TimeSteppingSSCIntegrator::postIntegrate(DynamicSystemSolver& system) {           // system: only dummy!
    time += Timer.stop();
    cout.unsetf(ios::scientific);
    if (FlagPlotIntegrator) integPlot.close();
    if (FlagPlotIntegrationSum) {
      int maxStepsPerThread = singleStepsT1;
      if (maxStepsPerThread<singleStepsT2) maxStepsPerThread = singleStepsT2;
      if (maxStepsPerThread<singleStepsT3) maxStepsPerThread = singleStepsT3;
      ofstream integSum((name + ".sum").c_str());
      integSum << "Integration time:   " << time << endl;
      integSum << "Integration steps:  " << integrationSteps << endl;
      integSum << "Evaluations MBS:    " << (singleStepsT1+singleStepsT2+singleStepsT3)<<"   (max/Thread: "<<maxStepsPerThread<<")"<<endl;
      integSum << "Steps with events: " << integrationStepswithChange<< endl;
      if (maxOrder>=2) {
        integSum<<"Integration steps order 1: "<<integrationStepsOrder1<<endl;
        integSum<<"Integration steps order "<<maxOrder<<": "<<integrationStepsOrder2<<endl;
      }
      if (FlagSSC){
        integSum << "Refused steps: " << refusedSteps << endl;
        integSum << "Refused steps with events: " << refusedStepsWithImpact << endl;
        integSum << "Maximum step size: " << maxdtUsed << endl;
        integSum << "Minimum step size: " << mindtUsed << endl;
        integSum << "Average step size: " << (t-tStart)/integrationSteps << endl;
      }
      if (FlagGapControl) {
        if (GapControlStrategy>0) {
          integSum << "Steps accepted after GapControl  :"<<stepsOkAfterGapControl<<endl;
          integSum << "Steps refused after GapControl   :"<<stepsRefusedAfterGapControl<<endl;
          integSum << "No impact after GapControl alert :"<<wrongAlertGapControl<<endl;
        }
        integSum << "Average Penetration  (arithm.)     :"<<Penetration/PenetrationCounter<<endl;
        integSum << "Average Penetration  (geom.)       :"<<exp(PenetrationLog/PenetrationCounter)<<endl;
        integSum << "PenetrationCounter "<<PenetrationCounter<<endl;
        integSum << "Penetration Min    "<<PenetrationMin<<endl;
        integSum << "Penetration Max    "<<PenetrationMax<<endl;
      }
      integSum << "Maximum number of iterations: " << maxIterUsed << endl;
      integSum << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;
      integSum.close();
    }   
    if (FlagCoutInfo) {
      if (output) cout << endl <<endl;
      int maxStepsPerThread = singleStepsT1;
      if (maxStepsPerThread<singleStepsT2) maxStepsPerThread = singleStepsT2;
      if (maxStepsPerThread<singleStepsT3) maxStepsPerThread = singleStepsT3;
      cout << "Summary Integration with TimeStepperSSC: "<<endl;
      cout << "Integration time:   " << time << endl;
      cout << "Integration steps:  " << integrationSteps << endl;
      cout << "Evaluations MBS:    " << (singleStepsT1+singleStepsT2+singleStepsT3)<<"   (max/Thread: "<<maxStepsPerThread<<")"<<endl;
      cout << "Steps with events: " << integrationStepswithChange<< endl;
      if (maxOrder>=2) {
        cout<<"Integration steps order 1: "<<integrationStepsOrder1<<endl;
        cout<<"Integration steps order "<<maxOrder<<": "<<integrationStepsOrder2<<endl;
      }
      if (FlagSSC) {
        cout << "Refused steps: " << refusedSteps << endl;
        cout << "Refused steps with events: " << refusedStepsWithImpact << endl;
        cout << "Maximum step size: " << maxdtUsed << endl;
        cout << "Minimum step size: " << mindtUsed << endl;
        cout << "Average step size: " << (t-tStart)/integrationSteps << endl;
      }
      if (FlagGapControl) {
        if (GapControlStrategy>0) {
          cout << "Steps accepted after GapControl  :"<<stepsOkAfterGapControl<<endl;
          cout << "Steps refused after GapControl   :"<<stepsRefusedAfterGapControl<<endl;
          cout << "No impact after GapControl alert :"<<wrongAlertGapControl<<endl;
        }
        cout << "Average Penetration  (arithm.)     :"<<Penetration/PenetrationCounter<<endl;
        cout << "Average Penetration  (geom.)       :"<<exp(PenetrationLog/PenetrationCounter)<<endl;
        cout << "PenetrationCounter "<<PenetrationCounter<<endl;
        cout << "Penetration Min    "<<PenetrationMin<<endl;
        cout << "Penetration Max    "<<PenetrationMax<<endl;
      }
      cout << "Maximum number of iterations: " << maxIterUsed << endl;
      cout << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;

    }
  }


  bool TimeSteppingSSCIntegrator::testTolerances() 
  {
    double dtNewRel = 1; 
    bool testOK = true;
    bool IterConvergenceBlock1=true;
    bool IterConvergenceBlock2=true;
    IterConvergence = (iterA<maxIter);
    iter =iterA;

    Vec EstErrorLocal;

    if (method==0) {
      if (!FlagSSC) {
        dte =dt;
        if (maxOrder==1) {LSe << LSA;     ze << z1d;          }
        if (maxOrder==2) {
          LSe << LSB2;    
          if (!ConstraintsChanged) {
            ze << 2.0*z2d - z1d; order=2;} 
          else {ze<<z2d; order=1;}
        }
        if (maxOrder==3) {
          LSe << LSB2;
          if (!ConstraintsChanged) {
            ze << 0.5*z1d - 4.0*z2d + 4.5*z3d; order=3;} 
          else {ze<<z3d; order=1;}
        }
        if (maxOrder==4) {
          LSe << LSD6;
          if (!ConstraintsChanged) {
            ze << -1.0/15.0*z1d + z2d + 27.0/5.0*z6d - 16.0/3.0*z4d; order=4;} 
          else {ze<<z6d; order=1;}
        }

      }
      else {
        if (maxOrder==1) {
          IterConvergenceBlock1 = (iterB1<maxIter);
          IterConvergenceBlock2 = (iterB2<maxIter);
          EstErrorLocal = z2d-z1d;
          dtNewRel = calculatedtNewRel(EstErrorLocal,dt);
          testOK = (dtNewRel>=1.0);
          dtNewRel = sqrt(dtNewRel);
          dte = dt;
          ze  << z2d;
          LSe << LSB2;
        }
        if (maxOrder>=2) {
          IterConvergenceBlock1 = (iterB1<maxIter) && (iterC1<maxIter) && (iterC2<maxIter);
          if (calcBlock2) IterConvergenceBlock2 = (iterB2<maxIter) && (iterB2RE<maxIter) && (iterC3<maxIter) && (iterC4<maxIter);
          if (ConstraintsChanged) {  // Prüfe auf Einhaltung der Toleranz im Block 1
            order=1;
            if (maxOrder==2) EstErrorLocal = z2b-z4b;
            if (maxOrder==3 && ConstraintsChangedBlock1) EstErrorLocal = 2.0*(z4b - z6b);
            if (maxOrder==3 && !ConstraintsChangedBlock1) EstErrorLocal = z2b -z4b;
            dtNewRel = calculatedtNewRel(EstErrorLocal,dt);
            testOK = (dtNewRel>=1.0);
            dtNewRel = sqrt(dtNewRel);
            dte= dt/2.0;
            if (ConstraintsChangedBlock1) {
              if (maxOrder==2) {ze << z4b;  LSe << LSC2;}
              if (maxOrder==3) {ze << z6b;  LSe << LSD3;}
            }
            else {
              ze << zStern; order=maxOrder;
              if (maxOrder==2) {LSe << LSC2;}
              if (maxOrder==3) {LSe << LSD3;}
            }
          }

          if (ConstraintsChanged && !ConstraintsChangedBlock1) {
            if (testOK) {
              if (maxOrder==2) EstErrorLocal = z2dRE-z4d;
              if (maxOrder==3) EstErrorLocal = 2.0*(z4d-z6d);
              double dtNewRelTmp = calculatedtNewRel(EstErrorLocal,dt);
              if (dtNewRelTmp>=1) {
                dtNewRel = sqrt(dtNewRelTmp);
                testOK = true;
                dte = dt;
                if (maxOrder==2) { ze << z4d; LSe << LSC4;}
                if (maxOrder==3) { ze << z6d; LSe << LSD6;}
                order =1;
              }
            }
          }

          if (!ConstraintsChanged) {
            order = maxOrder;
            if (maxOrder==2) EstErrorLocal = (2.0*z2d-z1d - 2.0*z4d+z2dRE)/3.0;
            if (maxOrder==3) EstErrorLocal = ((4.5*z3d+0.5*z1d-4.0*z2d) - (4.5*z6d+0.5*z2dRE-4.0*z4d))/5.0;
            dtNewRel = calculatedtNewRel(EstErrorLocal,dt);
            testOK = (dtNewRel>=1.0);
            dtNewRel = pow(dtNewRel,1.0/(maxOrder+1));  
            if (maxOrder==2) { ze << 2.0*z4d - z2dRE; LSe << LSC4;}
            if (maxOrder==3) { ze << 4.5*z6d+0.5*z2dRE-4.0*z4d; LSe << LSD6;}
            dte=dt;
            // order >=2 hat mehrfach versagt! Teste auf order 1
            if (!testOK && (StepTrials>2)) {
              double dtNewRelTmp1=0;
              double dtNewRelTmp2=0;
              if (maxOrder==2) { dtNewRelTmp1 = calculatedtNewRel(z2b - z4b, dt);
                dtNewRelTmp2 = calculatedtNewRel(z4d-z2dRE, dt);}
                if (maxOrder==3) { dtNewRelTmp1 = calculatedtNewRel(2.0*(z4b-z6b), dt);
                  dtNewRelTmp2 = calculatedtNewRel(2.0*(z4d-z6d), dt);}

                  if (dtNewRelTmp1>=1.0) {
                    order = 1;
                    testOK= true;
                    if (dtNewRelTmp2>=1.0) {
                      dtNewRel= sqrt(dtNewRelTmp2);
                      if (maxOrder==2) {ze << z4d; LSe<<LSC4;}
                      if (maxOrder==3) {ze << z6d; LSe<<LSD6;}
                    }
                    else {
                      dtNewRel= sqrt(dtNewRelTmp1);  
                      if (maxOrder==2) {ze << z4b; LSe<<LSC2;}
                      if (maxOrder==3) {ze << z6b; LSe<<LSD3;}
                      dte= dt/2.0;
                    }
                  }   
                  if (testOK) cerr<<"High order refused; order 1 accepted."<<endl; 
            }

          }     
        } //endif maxOrder>=2
      }   //endelse !FlagSSC
    } // endif method==0

    if (method) {
      order = maxOrder;

      if(maxOrder==1) {
        IterConvergenceBlock1 = (iterB1<maxIter);
        IterConvergenceBlock2 = (iterB2<maxIter);
        EstErrorLocal = z2d-z1d;
        if (!ConstraintsChanged) EstErrorLocal = EstErrorLocal*2.0;
        dtNewRel = calculatedtNewRel(EstErrorLocal,dt);
        testOK = (dtNewRel>=1.0);
        dte = dt;
        LSe = LSB2;
        if (ConstraintsChanged) ze << z2d;
        if (!ConstraintsChanged && method==1) ze<<z1d;
        if (!ConstraintsChanged && method==2) ze<<2.0*z2d - z1d;
      }

      if (maxOrder>1) {
        IterConvergenceBlock1 = (iterB1<maxIter) && (iterC1<maxIter) && (iterC2<maxIter);
        if(calcBlock2) IterConvergenceBlock2 = (iterB2<maxIter) && (iterC3<maxIter) && (iterC4<maxIter); 

        if(ConstraintsChanged) {  // Pruefe Block1
          order =1;
          if (maxOrder==2) EstErrorLocal = z2b-z4b;
          if (maxOrder>=3) EstErrorLocal = 2.0*(z4b - z6b);
          dtNewRel = calculatedtNewRel(EstErrorLocal,dt);
          testOK = (dtNewRel>=1.0);
          dtNewRel = sqrt(dtNewRel);
          dte= dt/2.0;
          if (maxOrder==2)  {ze << z4b;  LSe << LSC2;}
          if (maxOrder>=3)  {ze << z6b;  LSe << LSD3;}
        }

        if (ConstraintsChanged && !ConstraintsChangedBlock1) { // Pruefe Block2
          if (testOK) {
            if (maxOrder==2) EstErrorLocal = z2d-z4d;
            if (maxOrder>=3) EstErrorLocal = 2.0*(z4d-z6d);
            double dtNewRelTmp = calculatedtNewRel(EstErrorLocal,dt);
            if (dtNewRelTmp>=1) {
              dtNewRel = sqrt(dtNewRelTmp);
              testOK = true;
              dte = dt;
              order=1;
              if (maxOrder==2) { ze << z4d; LSe << LSC4;}
              if (maxOrder>=3) { ze << z6d; LSe << LSD6;}
            }
          }
        }
        if (!ConstraintsChanged) {
          Vec zOp, zOp1;
          if (maxOrder==2) { 
            zOp= 2.0*z2d - z1d;  
            zOp1= z1d/3.0 - 2.0*z2d + 8.0/3.0*z4d;
          }
          if (maxOrder==3) { 
            zOp1= -1.0/15.0*z1d + z2d + 27.0/5.0*z6d - 16.0/3.0*z4d; 
            zOp = z1d/3.0 - 2.0*z2d + 8.0/3.0*z4d;
          }
          if (maxOrder==4) { 
            zOp = -1.0/6.0*z1d + 4.0*z2d -27.0/2.0*z3d + 32.0/3.0*z4d;
            zOp1= 1.0/30.0*z1d - 2.0*z2d +27.0/2.0*z3d - 64.0/3.0*z4d  + 54.0/5.0*z6d; 
          }

          LSe << LSC4;
          dte=dt;
          if (method==1) ze << zOp;
          if (method==2) ze << zOp1;

          EstErrorLocal = zOp -zOp1;
          dtNewRel = calculatedtNewRel(EstErrorLocal,dt);
          testOK = (dtNewRel>=1.0);
          dtNewRel = pow(dtNewRel,1.0/(maxOrder+1)); 

          if (FlagSSC && !testOK && (StepTrials>2)) {
            double dtNewRelTmp1=0;
            double dtNewRelTmp2=0;
            if (maxOrder==2) { dtNewRelTmp1 = calculatedtNewRel(z2b - z4b, dt);
              dtNewRelTmp2 = calculatedtNewRel(z2d - z4d, dt);}
              if (maxOrder>=3) { dtNewRelTmp1 = calculatedtNewRel(2.0*(z4b-z6b), dt);
                dtNewRelTmp2 = calculatedtNewRel(2.0*(z4d-z6d), dt);}

                if (dtNewRelTmp1>=1.0) {
                  order = 1;
                  testOK= true;
                  if (dtNewRelTmp2>=1.0) {
                    dtNewRel= sqrt(dtNewRelTmp2);
                    if (maxOrder==2) {ze << z4d; LSe<<LSC4;}
                    if (maxOrder>=3) {ze << z6d; LSe<<LSD6;}
                  }
                  else {
                    dtNewRel= sqrt(dtNewRelTmp1);  
                    if (maxOrder==2) {ze << z4b; LSe<<LSC2;}
                    if (maxOrder>=3) {ze << z6b; LSe<<LSD3;}
                    dte= dt/2.0;
                  }
                }  
                if (testOK) cout<<"Hohe Ordnung abgeleht aber dafuer Order 1 akzeptiert!!!"<<endl; 
          }

        } //endif !ConstraintsChanged

      }

    } //endif method>0

    if (dte<dt) {		// nur halber Integrationsschritt wurde akzeptiert dte==dt/2
      lae.resize() = la2b;
      laeSizes.resize()  = la2bSizes;
      iter = iterB1;
      ConstraintsChanged = ConstraintsChangedBlock1;
      IterConvergenceBlock2 = true;
    }
    else {
      lae.resize() = la1d;
      laeSizes.resize()  = la1dSizes;        
      iter = iterA;
    }

    if (FlagSSC) { 
      IterConvergence = IterConvergence && IterConvergenceBlock1 && IterConvergenceBlock2;
      if ((! IterConvergence)&&(testOK)) {
        if (dt/2.0>dtMin) {
          testOK= false; 
          dt=dt/2.0;
          //cout<<"step size halved because of failed convergence"<<endl;
        }
        else {
          cout<<"Error: no convergence despite minimum stepsize("<<maxIter<<" iterations) Anyway, continuing integration..."<<endl;
        }
      }
      else {
        if(dtNewRel<0.5/safetyFactorSSC) dtNewRel = 0.5/safetyFactorSSC;
        double dtRelGap = (FlagGapControl && GapControlStrategy>0) ? dtRelGapControl*0.7 + 0.3 : 1.0;
        // erlaubt groeseres dtNewRel, falls vorher dt durch GapControl verkleinert wurde
        if(dtNewRel > maxGainSSC/dtRelGap) dtNewRel=maxGainSSC/dtRelGap;   //  0.6 oder 0.7 und 2.5 oder 2  (0.7/2.2 alt)
        dt = dt*dtNewRel*safetyFactorSSC;
      }  

      if (dt > dtMax) dt = dtMax;
      if (dt < dtMin) dt = dtMin;
      if (testOK && FlagGapControl) getDataForGapControl();  // verwendet wird sysT1, SetValuedLinkListT1, ze t, dte
      if (FlagGapControl && GapControlStrategy>0) {
        if (ChangeByGapControl && testOK) stepsOkAfterGapControl++;
        if (ChangeByGapControl && !testOK)stepsRefusedAfterGapControl++;
        if (testOK) { 
          // Mittelwert berechnen
          Vec Dq;
          Dq= z1d(0,qSize) - ze(0,qSize);
          double mDq=0;
          for(int i=0; i<qSize; i++) mDq +=fabs(Dq(i));
          mDq=mDq/qSize;
          // Standardabweichung
          double sDq=0;
          for(int i=0; i<qSize; i++) sDq +=(fabs(Dq(i))-mDq)*(fabs(Dq(i))-mDq);
          sDq=sqrt(sDq)/qSize;
          qUncertaintyByExtrapolation = 1.05*mDq+sDq;
        }
        ChangeByGapControl = GapControl(qUncertaintyByExtrapolation, testOK);
      }
    }
    else testOK=true;
    if (FlagGapControl && testOK) {
      for(int i=0; i<gUniActive.size(); i++){
        Penetration -= gUniActive(i);
        PenetrationLog += log(fabs(gUniActive(i)));
        PenetrationCounter+=1.0;
        if(gUniActive(i)>PenetrationMin) PenetrationMin=gUniActive(i);
        if(gUniActive(i)<PenetrationMax) PenetrationMax=gUniActive(i);
      }
    }
    if (testOK && FlagPlotIntegrator) {
      AnzahlAktiverKontakte=0;
      for(int i=0; i<LSe.size();i++) {
        if(LSe(i)>=2) AnzahlAktiverKontakte++;
      }
    }
    return testOK;
  }

  double TimeSteppingSSCIntegrator::calculatedtNewRel(const fmatvec::Vec &ErrorLocal, double H) {
    if (!FlagSSC) return 1.0;
    double dtNewRel=1.0e10; 
    double dtNewRel_i;
    double ResTol_i;
    double Hscale=1;
    bool includeVelocities = (FlagErrorTest!=3);
    if ((FlagErrorTest==3) && (!FlagErrorTestAlwaysValid) && (!ConstraintsChanged)) includeVelocities=true;
    if ((FlagErrorTest==2) && (FlagErrorTestAlwaysValid || ConstraintsChanged)) Hscale=H;

    for (int i=0; i< zSize; i++) {
      if((i<qSize) || (i>qSize+uSize) || includeVelocities) { 
        if ((i>=qSize)&&(i<qSize+uSize))
          ResTol_i = aTol(i)/Hscale +  rTol(i)*fabs(zi(i));
        else ResTol_i = aTol(i) + rTol(i)*fabs(zi(i));
        dtNewRel_i = ResTol_i / fabs(ErrorLocal(i));
        if (dtNewRel_i < dtNewRel) dtNewRel = dtNewRel_i;
      }
    }
    return dtNewRel;
  }

  bool TimeSteppingSSCIntegrator::GapControl(double qUnsafe, bool SSCTestOK) 
  {

    // Strategien unterscheiden sich, falls mehr als ein moeglicher stoss im Intervall [0 dt]

    // Strategie  1: Weiter hinter groessten Nullstelle, aber mindestens 0.3*dt_SSC (schnellster Integrationsfortschritt)
    // Strategie  2: Scoreverfahren: weiter hinter NS mit groesstem Score 
    // Strategie  3: Groesste NS so dass gapTol eingehalten wird
    // Strategie  4: Weiter hinter der kleinsten Nullstelle (geringste Penetration)
    // Strategie  5: event detection: 
    //               Integration bis kurz vor NS, dann mit kleiner dt ueber event hinweg und weiter mit alter dt
    // Strategie  0: GapControl wieder deaktiviert (z.B. um Penetration auszugeben)
    double dtOrg=dt;    

    if (GapControlStrategy==0) {dtRelGapControl = 1; return false;}
    if (statusGapControl>2) {statusGapControl=0; indexLSException=-1; }
    if (statusGapControl && !SSCTestOK) statusGapControl=0;
    if (statusGapControl==2) { 
      if (ConstraintsChanged && SSCTestOK) 
        dt= 0.75*dt_SSC_vorGapControl;
      else statusGapControl=0;
    }

    double NSi=-1;              // groesste Nullstelle im Bereich [0,dt]     (innen)
    double NSiMin=2*dt;         // kleinste Nullstelle im Bereich [dtmin; dt](innen)
    double NSa=-1;              // groesste Nullstelle im Bereich ]dt 1.5dt] (aussen) 
    double NS_=-1;              // zum Speichern der NS fuer z.B. Strategie 1 oder 2
    double NS;
    double Hold=dt;
    double ScoreMax = -1;
    double tTolMin = dt;
    int gInActiveSize = gInActive.size();
    int IndexNS=-1;
    int IndexNSMin=-1;

    for(int i=0; i<gInActiveSize; i++) {
      if (gdInActive(i)>0.0 || gdInActive(i)<0.0) NS = -gInActive(i)/gdInActive(i);
      else NS = -1;
      if (NS>0) {
        if (NS<=dt && NS>NSi) {NSi=NS;  IndexNS=i;}
        if (NS>dtMin && NS<=dt && NS<NSiMin) {NSiMin=NS; IndexNSMin=i;}
        if (NS>dt && NS<=1.5*dt && NS>NSa) NSa=NS;  

        if (GapControlStrategy==2 && NS<=dt) {
          double Score = fabs(gdInActive(i))*(dt-NS)*NS;
          if (Score>ScoreMax) {ScoreMax=Score; NS_= NS;}
        }
        if(GapControlStrategy==3 && NS<=dt) {
          double tTol = NS + fabs(gapTol/gdInActive(i));
          if (tTol<tTolMin) {tTolMin= tTol;  NS_=NS;}
        }
      }
    }

    if (statusGapControl==2 && ConstraintsChanged && SSCTestOK) {
      // identifiziere Eintrag in LS Vektor der zu diesem Stoss gehoert; falls sich nur ein Eintrag geaendert hat
      // wird maxOrder erzwungen dadurch dass dieser Eintrag im Vergleich der LS unberuecksichtigt bleibt
      statusGapControl=0;
      int n=0;
      int ni=-1;
      if(LS.size()==LSe.size()) {
        for(int i=0; i<LS.size(); i++) if (LSe(i)!=LS(i)) {n++; ni=i;}
      }
      if(n==1) indexLSException=ni;
      if (n==1 && NSi<=0) statusGapControl=3;
    }

    if (NSi>0){ 
      if (GapControlStrategy<=4 && GapControlStrategy) dt= safetyFactorGapControl*NSi+1e-15;
      if (GapControlStrategy==1 && dt/Hold<0.3) dt =0.3*Hold;
      if (GapControlStrategy==2 && NS_>0) dt= safetyFactorGapControl*NS_ +1e-15;
      if (GapControlStrategy==3 && NS_>0) dt= safetyFactorGapControl*NS_ +1e-15; 
      if (GapControlStrategy==4 && NSiMin<NSi) dt=safetyFactorGapControl*NSiMin+1e-15;
      if (GapControlStrategy==5) {
        if(NSi<NSiMin) {NSiMin=NSi; IndexNSMin=IndexNS;}
        NSi=NSiMin;
        double dtUnsafe = fabs(qUnsafe/gdInActive(IndexNSMin));
        dtUnsafe *= NSiMin/dte; //pow(NS/dte,order);
        if(dtUnsafe*10<dte) {  // Falls Unsicherheit zu gross macht gapContol keinen Sinn !
          if(dtUnsafe<0.4*dtMin) dtUnsafe=0.4*dtMin;
          if (statusGapControl==0) {
            dt_SSC_vorGapControl= dt;
            statusGapControl=1;
            dt = NSiMin- (safetyFactorGapControl-1)*NSiMin;
            if (dt<=dtMin) {dt=dtMin; statusGapControl=2;}
            else {
              dt = dt -dtUnsafe;
              if (dt<=dtMin) dt=dtMin;
            }
          }
          else {  //statusGapControl==1
            statusGapControl=2;
            dt = NSiMin*safetyFactorGapControl+dtUnsafe;
            if (dt<dtMin) dt=dtMin; 
          }
        } else statusGapControl=0;
      }
    }
    else {
      if(NSa>0 && GapControlStrategy && GapControlStrategy<=4){ 
        dt = NSa*0.5;}
      if (statusGapControl && statusGapControl<3) statusGapControl=0;
    }
    if (dt<0.75*dtMin) dt=0.75*dtMin;

    dtRelGapControl = dt/Hold;

    return (dt<dtOrg);
  }

    // SetValuedLinkLists are build up within preIntegrate 
    // The Lagragian Multiplier of all Links stored in SetValuedLinkList of the corresponding DynamicSystemSolver (sysT1)
    // are collecte and stored in Vec la
    // Vector<int> laSizes contains the singel la-size of each link

    void TimeSteppingSSCIntegrator::getAllSetValuedla(fmatvec::Vec& la_, fmatvec::Vector<int>& la_Sizes,vector<Link*> &SetValuedLinkList) {

      int SetValuedLaSize=0;
      int NumberOfSetValuedLinks = SetValuedLinkList.size();
      la_Sizes.resize(NumberOfSetValuedLinks,INIT,0);

      for(unsigned int i=0; i<SetValuedLinkList.size(); i++) {
        la_Sizes(i)=SetValuedLinkList[i]->getlaSize();
        SetValuedLaSize+=la_Sizes(i);
      }
      la_.resize(SetValuedLaSize,INIT,0.0);
      int j=0;
      int sizeLink;
      for(unsigned int i=0; i<SetValuedLinkList.size(); i++) {
        sizeLink = SetValuedLinkList[i]->getlaSize();
        if (SetValuedLinkList[i]->isActive())
          la_(j,j+sizeLink-1) = SetValuedLinkList[i]->getla();
        else 
          la_(j,j+sizeLink-1).init(0.0);
        j+=sizeLink;  
      }     
    } 

    void TimeSteppingSSCIntegrator::setAllSetValuedla(const fmatvec::Vec& la_, const fmatvec::Vector<int>& la_Sizes,vector<Link*> &SetValuedLinkList) {

      int j=0;
      int sizeLink;
      for(unsigned int i=0; i<SetValuedLinkList.size(); i++) {
        sizeLink = SetValuedLinkList[i]->getlaSize();
        if (! SetValuedLinkList[i]->isActive())
          SetValuedLinkList[i]->deletelaRef();
        if(sizeLink >= la_Sizes(i)) 
          SetValuedLinkList[i]->getla()(0,la_Sizes(i)-1) =  la_(j,j+la_Sizes(i)-1); 
        else
          SetValuedLinkList[i]->getla()(0,sizeLink-1) =  la_(j,j+sizeLink-1); 
        j+=la_Sizes(i);  
      }     
    }


    void TimeSteppingSSCIntegrator::getDataForGapControl() {
      int nInActive=0;
      int nActive=0;
      zT1<<ze;
      if((sysT1->getq())()!=zT1()) sysT1->updatezRef(zT1);
      sysT1->updateStateDependentVariables(t+dte);
      for(unsigned int i=0; i<SetValuedLinkListT1.size(); i++){ 
        SetValuedLinkListT1[i]->updateg(t+dte);
        SetValuedLinkListT1[i]->checkActiveg();
        SetValuedLinkListT1[i]->SizeLinearImpactEstimation(&nInActive, &nActive);
      }
      gInActive.resize(nInActive,NONINIT);
      gdInActive.resize(nInActive,NONINIT);
      gUniActive.resize(nActive,NONINIT);
      nInActive=0;
      nActive=0;
      for(unsigned int i=0; i<SetValuedLinkListT1.size(); i++) 
        SetValuedLinkListT1[i]->LinearImpactEstimation(gInActive,gdInActive,&nInActive,gUniActive,&nActive);
    }

    bool TimeSteppingSSCIntegrator::changedLinkStatus(const Vector<int> &L1, const Vector<int> &L2, int ex) {
      if (ex<0) return (L1!=L2);
      int n=L1.size();
      if (n!=L2.size()) return true;
      for(int i=0; i<n; i++) 
       if (i!=ex && L1(i)!=L2(i)) return true;
      return false;
    }

    void TimeSteppingSSCIntegrator::initializeUsingXML(TiXmlElement *element) {

      Integrator::initializeUsingXML(element);
      TiXmlElement *e;

      e=element->FirstChildElement(MBSIMINTNS"initialStepSize");
      if (e) setInitialStepSize(Element::getDouble(e));

      e=element->FirstChildElement(MBSIMINTNS"maximalStepSize");
      if (e) setStepSizeMax(Element::getDouble(e));

      e=element->FirstChildElement(MBSIMINTNS"minimalStepSize");
      if (e) setStepSizeMin(Element::getDouble(e));

      e=element->FirstChildElement(MBSIMINTNS"outputInterpolation");
      if (e) setOutputInterpolation(Element::getBool(e));

      e=element->FirstChildElement(MBSIMINTNS"gapControl");
      if (e) {
        TiXmlElement *ee;
        ee=e->FirstChildElement();
        if (ee->ValueStr()==MBSIMINTNS"withoutGapControl") setGapControl(-1);
        if (ee->ValueStr()==MBSIMINTNS"biggestRoot") setGapControl(1);
        if (ee->ValueStr()==MBSIMINTNS"scooring") setGapControl(2);
        if (ee->ValueStr()==MBSIMINTNS"gapTollerance") setGapControl(3);
        if (ee->ValueStr()==MBSIMINTNS"smallestRoot") setGapControl(4);
      }

      e=element->FirstChildElement(MBSIMINTNS"maximalOrder");
      if (e) {
        TiXmlElement *ee;
        ee=e->FirstChildElement(MBSIMINTNS"order");
        int orderXML=Element::getInt(ee);
        int methodXML=0; 
        ee=e->FirstChildElement(MBSIMINTNS"method");
        if(ee) {
          TiXmlElement *eee;
          eee=ee->FirstChildElement();
          if (eee->ValueStr()==MBSIMINTNS"extrapolation") methodXML=0;
          if (eee->ValueStr()==MBSIMINTNS"embedded") methodXML=1;
          if (eee->ValueStr()==MBSIMINTNS"embeddedHigherOrder") methodXML=2;
        }
        setMaxOrder(orderXML,methodXML); 
      }

      e=element->FirstChildElement(MBSIMINTNS"errorTest");
      if (e) {
        TiXmlElement *ee;
        ee=e->FirstChildElement();
        int FlagErrorTestXML=2;
        if(ee) {
          if (ee->ValueStr()==MBSIMINTNS"scale") FlagErrorTestXML=2;
          if (ee->ValueStr()==MBSIMINTNS"all") FlagErrorTestXML=0;
          if (ee->ValueStr()==MBSIMINTNS"exclude") FlagErrorTestXML=3;
        }
        setFlagErrorTest(FlagErrorTestXML); 
      }

      e=element->FirstChildElement(MBSIMINTNS"absoluteTolerance");
      if(e) setAbsoluteTolerance(Element::getVec(e));
      e=element->FirstChildElement(MBSIMINTNS"absoluteToleranceScalar");
      if(e) setAbsoluteTolerance(Element::getDouble(e));
      e=element->FirstChildElement(MBSIMINTNS"relativeTolerance");
      if(e) setRelativeTolerance(Element::getVec(e));
      e=element->FirstChildElement(MBSIMINTNS"relativeToleranceScalar");
      if(e) setRelativeTolerance(Element::getDouble(e));

      e=element->FirstChildElement(MBSIMINTNS"advancedOptions");
      if (e) {
        TiXmlElement *ee;
        ee=e->FirstChildElement(MBSIMINTNS"deactivateSSC");
        if (ee) deactivateSSC(!(Element::getBool(ee)));

        ee=e->FirstChildElement(MBSIMINTNS"gapTolerance");
        if (ee) setgapTolerance(Element::getDouble(ee));

        ee=e->FirstChildElement(MBSIMINTNS"maximalSSCGain");
        if (ee) setmaxGainSSC(Element::getDouble(ee));

        ee=e->FirstChildElement(MBSIMINTNS"safetyFactorSSC");
        if (ee) setSafetyFactorSSC(Element::getDouble(ee));
      }

    }

  }
