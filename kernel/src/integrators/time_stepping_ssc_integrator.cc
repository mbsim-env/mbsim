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
#include "stopwatch.h"
#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef NO_ISO_14882
using namespace std;

#endif


using namespace fmatvec;

namespace MBSim {


  TimeSteppingSSCIntegrator::TimeSteppingSSCIntegrator() : dt(1e-6), dtMin(0), dtMax(1e-4),driftCompensation(false), StepsWithUnchangedConstraints(-1),  FlagErrorTest(2), aTol(1,INIT,1e-6), rTol(1,INIT,1e-4),FlagSSC(1), maxOrder(1), gapTol(0), maxGainSSC(2.2), safetyFactorSSC(0.7), FlagPlotIntegrator(true), FlagPlotIntegrationSum(true), FlagCoutInfo(true), FlagPlotEveryStep(false), outputInterpolation(false), time(0.0), iter(0), maxIterUsed(0), sumIter(0), integrationSteps(0), refusedSteps(0), refusedStepsWithImpact(0)
  {
    order = maxOrder;
    optimisedtforgaps=false;
  }

  void TimeSteppingSSCIntegrator::setFlagErrorTest(int Flag) {
    FlagErrorTest = Flag;
    assert(FlagErrorTest>=0);		// =0	control errors locally on all variables
    assert(FlagErrorTest<4);		// =2   u is scaled with stepsize
    assert(FlagErrorTest!=1);           // =3   exclude u from error test
  }

  void TimeSteppingSSCIntegrator::setMaxOrder(int order_) {
    maxOrder = order_;
    if (maxOrder>2) maxOrder=2;
    if (maxOrder<1) maxOrder=1;
    assert(maxOrder>0);
    assert(maxOrder<3);
    order = maxOrder;
  }

  void TimeSteppingSSCIntegrator::integrate(MultiBodySystem& system_) {
    integrate(system_, system_, system_); 
  }

  void TimeSteppingSSCIntegrator::integrate(MultiBodySystem& systemT1_, MultiBodySystem& systemT2_, MultiBodySystem& systemT3_) { 
    initIntegrator(systemT1_, systemT2_, systemT3_);
    IntegrationStep();
    closeIntegrator();
  }

  void TimeSteppingSSCIntegrator::initIntegrator(MultiBodySystem& system_) {
    initIntegrator(system_,system_,system_);
  }

  void TimeSteppingSSCIntegrator::initIntegrator(MultiBodySystem& systemT1_, MultiBodySystem& systemT2_, MultiBodySystem& systemT3_) {

    systemT1 = &systemT1_;
    systemT2 = &systemT2_;
    systemT3 = &systemT3_;

    if (dtMin==0) dtMin = epsroot()*maxOrder*2.0;


    if (FlagSSC) safetyFactorGapControl = 1.0 + nrmInf(rTol)*10.0;
    else safetyFactorGapControl = 1.001;

    qSize = systemT1->getqSize(); // size of positions, velocities, state
    int uSize = systemT1->getuSize();
    xSize = systemT1->getxSize();
    zSize = qSize + uSize + xSize;

    Index Iq(0,qSize-1);
    Index Ix(qSize,qSize+xSize-1);
    Index Iu(xSize+qSize, zSize-1);

    zT1.resize(zSize);
    qT1.resize() >> zT1(Iq);
    xT1.resize() >> zT1(Ix);
    uT1.resize() >> zT1(Iu);

    zT2.resize(zSize);
    qT2.resize() >> zT2(Iq);
    xT2.resize() >> zT2(Ix);
    uT2.resize() >> zT2(Iu);

    zT3.resize(zSize);
    qT3.resize() >> zT3(Iq);
    xT3.resize() >> zT3(Ix);
    uT3.resize() >> zT3(Iu);

    zi.resize(zSize,INIT,0.0); 	// starting value for ith step

    maxIter = systemT1->getMaxIter();
    iter= 0;
    integrationSteps= 0;
    integrationStepsOrder1= 0;
    integrationStepsOrder2= 0;
    integrationStepswithChange =0;
    refusedSteps = 0;
    maxIterUsed = 0;
    sumIter = 0;
    StepTrials=0;
    maxdtUsed = dtMin;
    mindtUsed = dtMax;

    if (aTol.size() < zSize) aTol.resize(zSize,INIT,aTol(0));
    if (rTol.size() < zSize) rTol.resize(zSize,INIT,rTol(0));
    assert(aTol.size() == zSize);
    assert(rTol.size() == zSize);

    if (dtPlot==0) FlagPlotEveryStep = true;
    else FlagPlotEveryStep = false;

    cout.setf(ios::scientific, ios::floatfield);
    if (FlagPlotIntegrator) {
      integPlot.open((systemT1->getDirectoryName() + name + ".plt").c_str());
      integPlot << "#1 t [s]:" << endl; 
      integPlot << "#2 dt [s]:" << endl;
      integPlot << "#3 order :" << endl; 
      integPlot << "#4 iter  :" << endl;      
      integPlot << "#5 laSize:" << endl;
      integPlot << "#6 calculation time [s]:" << endl;
    }
  }

  void TimeSteppingSSCIntegrator::IntegrationStep() {
    Timer.start();
    t = tStart;
    tPlot = tStart;
    if(z0.size()) zi = z0; 					// define initial state
    else systemT1->initz(zi); 
    if (outputInterpolation) {
      laBi.resize((systemT1->getAllBilateralla()).size(),INIT,0.0);		// la = LA/dt [N]
      laUni.resize((systemT1->getAllUnilateralla()).size(),INIT,0.0);
      laBi.init(0.0);
      laUni.init(0.0);
    }

    int StepFinished = 0;
    bool ExitIntegration = (t>=tEnd);
    int UnchangedSteps =0;
    double dtHalf;
    double dtQuarter;
    bool ConstraintsChangedA;
    bool ConstraintsChangedB;
    bool ConstraintsChangedC;

    bool calcJobB1 = FlagSSC || (maxOrder>1);
    bool calcJobC12  = (FlagSSC && (maxOrder>1)) || (maxOrder>2); 
    bool calcJobB2;
    bool calcJobB2RE;
    bool calcJobC34;

    LS.resize() = systemT1->getLinkStatus();

    while(! ExitIntegration) 
    {
      while(StepFinished==0) 
      {
        dtHalf     = dt/2.0;
        dtQuarter  = dt/4.0;
        ConstraintsChangedB = false;
        ConstraintsChangedC = false;
        // Block 1 

#pragma omp parallel num_threads(2)
        {
#pragma omp sections
          {
#pragma omp section           //thread 1
            {
              // one step integration (A)
              zT1 << zi;
              systemT1->setActiveConstraintsChanged(true);
              qT1 += systemT1->deltaq(zT1,t,dt);
              systemT1->update(zT1,t+dt);
              iterA  = systemT1->solve(dt);
              la1dBi.resize()  = systemT1->getAllBilateralla()/dt;
              la1dUni.resize() = systemT1->getAllUnilateralla()/dt;
              uT1 += systemT1->deltau(zT1,t+dt,dt);
              xT1 += systemT1->deltax(zT1,t+dt,dt);
              systemT1->updateLinkStatus();
              LSA = systemT1->getLinkStatus();
              ConstraintsChangedA = (LSA != LS);
              z1d << zT1;

              // two step integration (first step) (B1) (Thread 1 if dtQuarter steps are performed; else ->Thread 2)
              if (calcJobB1 && calcJobC12) {
                zT1  << zi;
                systemT1->setActiveConstraintsChanged(true);
                qT1 += systemT1->deltaq(zT1,t,dtHalf);
                systemT1->update(zT1,t+dtHalf);
                iterB1  = systemT1->solve(dtHalf);
                la2bBi.resize()  = systemT1->getAllBilateralla()/dtHalf;
                la2bUni.resize() = systemT1->getAllUnilateralla()/dtHalf;
                uT1 += systemT1->deltau(zT1,t+dtHalf,dtHalf);
                xT1 += systemT1->deltax(zT1,t+dtHalf,dtHalf); 
                systemT1->updateLinkStatus();
                LSB1=systemT1->getLinkStatus() ;
                ConstraintsChangedB = (LSB1 != LS);
                z2b << zT1;
              }
            }  // close omp thread 1

#pragma omp section	        //thread 2
            {
              // two step integration (first step) (B1) (Thread 1 if dtQuarter steps are performed; else ->Thread 2)
              if (calcJobB1 && !calcJobC12) {
                zT2  << zi;
                systemT2->setActiveConstraintsChanged(true);
                qT2 += systemT2->deltaq(zT2,t,dtHalf);
                systemT2->update(zT2,t+dtHalf);
                iterB1  = systemT2->solve(dtHalf);
                la2bBi.resize()  = systemT2->getAllBilateralla()/dtHalf;
                la2bUni.resize() = systemT2->getAllUnilateralla()/dtHalf;
                uT2 += systemT2->deltau(zT2,t+dtHalf,dtHalf);
                xT2 += systemT2->deltax(zT2,t+dtHalf,dtHalf); 
                systemT2->updateLinkStatus();
                LSB1=systemT2->getLinkStatus() ;
                ConstraintsChangedB = (LSB1 != LS);
                z2b << zT2;
              }

              // four step integration (first two steps)
              if (calcJobC12) {
                zT2  << zi;
                systemT2->setActiveConstraintsChanged(true);
                qT2 += systemT2->deltaq(zT2,t,dtQuarter);
                systemT2->update(zT2,t+dtQuarter);
                iterC1 = systemT2->solve(dtQuarter);
                uT2 += systemT2->deltau(zT2,t+dtQuarter,dtQuarter);
                xT2 += systemT2->deltax(zT2,t+dtQuarter,dtQuarter);
                systemT2->updateLinkStatus();
                LSC1 = systemT2->getLinkStatus();
                ConstraintsChangedC =  (LSC1 != LS);

                qT2 += systemT2->deltaq(zT2,t+dtQuarter,dtQuarter);
                systemT2->update(zT2,t+dtHalf);
                iterC2 = systemT2->solve(dtQuarter);
                uT2 += systemT2->deltau(zT2,t+dtHalf,dtQuarter);
                xT2 += systemT2->deltax(zT2,t+dtHalf,dtQuarter);
                systemT2->updateLinkStatus();
                LSC2 = systemT2->getLinkStatus();
                ConstraintsChangedC = ConstraintsChangedC || (LSC2!=LSC1);
                z4b << zT2;
              }
            }  // close omp thread 2
          }  // close omp sections (first sections)

#pragma omp single
          {
            ConstraintsChangedBlock1 = ConstraintsChangedB || ConstraintsChangedC;
            ConstraintsChanged       = ConstraintsChangedA || ConstraintsChangedBlock1;

            ConstraintsChangedA = false;
            ConstraintsChangedB = false;
            ConstraintsChangedC = false;
            ConstraintsChangedBlock2 = false;        

            calcJobB2 =  FlagSSC && ((maxOrder==1) || (maxOrder>1 && !ConstraintsChangedBlock1));
            calcJobB2 = calcJobB2 || (!FlagSSC && (maxOrder==2));
            calcJobB2RE = FlagSSC && (maxOrder==2) && !ConstraintsChangedBlock1; 
            calcJobC34  = (FlagSSC && (maxOrder>1)) || (maxOrder>2) && !ConstraintsChangedBlock1;
                
          }
          // Block 2
#pragma omp sections
          {
#pragma omp section                     // thread 1
            {
              // two step integration (B2 and B2RE) (starting with z2b or 2*z4b-z2b)
              if (calcJobB2) {
                zT1  << z2b;
                systemT1->setActiveConstraintsChanged(true);
                qT1 += systemT1->deltaq(zT1,t+dtHalf,dtHalf);
                systemT1->update(zT1,t+dt);
                iterB2  = systemT1->solve(dtHalf);
                uT1 += systemT1->deltau(zT1,t+dt,dtHalf);
                xT1 += systemT1->deltax(zT1,t+dt,dtHalf);
                systemT1->updateLinkStatus();
                LSB2 = systemT1->getLinkStatus();
                ConstraintsChangedB = (LSB2!=LSB1);
                z2d << zT1; 
              }
              if (calcJobB2RE) { //B2RE
                zT1 << 2.0*z4b - z2b;
                systemT1->setActiveConstraintsChanged(true);
                qT1 += systemT1->deltaq(zT1,t+dtHalf,dtHalf);
                systemT1->update(zT1,t+dt);
                iterB2RE  = systemT1->solve(dtHalf);
                uT1 += systemT1->deltau(zT1,t+dt,dtHalf);
                xT1 += systemT1->deltax(zT1,t+dt,dtHalf);
                z2dRE << zT1;
              }
            }  // close omp thread 1

            // four step integration (C3 and C4) (last two steps )
#pragma omp section                   // thread 2
            {
              if (calcJobC34) { 
                zT2 <<  2.0*z4b - z2b;
                systemT2->setActiveConstraintsChanged(true);
                qT2 += systemT2->deltaq(zT2,t+dtHalf,dtQuarter);
                systemT2->update(zT2,t+dtHalf+dtQuarter);
                iterC3  = systemT2->solve(dtQuarter);
                uT2 += systemT2->deltau(zT2,t+dtHalf+dtQuarter,dtQuarter);
                xT2 += systemT2->deltax(zT2,t+dtHalf+dtQuarter,dtQuarter);
                systemT2->updateLinkStatus();
                LSC3 = systemT2->getLinkStatus();

                qT2 += systemT2->deltaq(zT2,t+dtHalf+dtQuarter,dtQuarter);
                systemT2->update(zT2,t+dt);
                iterC4 = systemT2->solve(dtQuarter);
                uT2 += systemT2->deltau(zT2,t+dt,dtQuarter);
                xT2 += systemT2->deltax(zT2,t+dt,dtQuarter);
                systemT2->updateLinkStatus();
                LSC4 = systemT2->getLinkStatus();
                ConstraintsChangedC = (LSC2 !=LSC3) || (LSC3 !=LSC4);
                z4dRE << zT2;
              }
            }  // close omp thread 2
          }  // close omp sections (second sections)
      }  // close omp parallel


      ConstraintsChangedBlock2 = ConstraintsChangedB || ConstraintsChangedC;
      ConstraintsChanged = ConstraintsChanged || ConstraintsChangedBlock2;

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
        if (dtOld == dtMin) {
          StepFinished = -1;
          cout << " TimeStepperSSC reached minimum stepsize dt= "<<dt<<" at t= "<<t<<endl;
          //exit(StepFinished);
        }
      }
    }

    StepTrials=0;
    StepFinished = 0;
    integrationSteps++; 
    if(order==1)integrationStepsOrder1++;
    if(order==2)integrationStepsOrder2++;
    t += dte;
    plot();

    zi << ze;
    LS << LSe;
    if (outputInterpolation) {
      if (laBi.size() != laeBi.size()) laBi.resize();
      laBi = laeBi;
      if (laUni.size() != laeUni.size()) laUni.resize();
      laUni = laeUni;
    }

    if(ConstraintsChanged) integrationStepswithChange++;
    ExitIntegration = (t>=tEnd);

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
    systemT1->setAllBilateralla(laBi);
    systemT1->setAllUnilateralla(laUni);
    systemT1->plot(zT1,t,1);
  }
  if ((t>=tPlot) && outputInterpolation && !FlagPlotEveryStep) {
    while (t>tPlot) {
      if (laBi.size()  > laeBi.size())  laBi.resize()   = laBi(0,laeBi.size()-1);
      if (laBi.size()  < laeBi.size())  laeBi.resize()  = laeBi(0,laBi.size()-1);
      if (laUni.size() > laeUni.size()) laUni.resize()  = laUni(0,laeUni.size()-1);
      if (laUni.size() < laeUni.size()) laeUni.resize() = laeUni(0,laUni.size()-1);
      double ratio = (tPlot -(t-dte))/dte;
      zT1 << zi + (ze-zi)*ratio;
      systemT1->setAllBilateralla(laBi+(laeBi-laBi)*ratio);
      systemT1->setAllUnilateralla(laUni+(laeUni-laUni)*ratio);
      systemT1->plot(zT1,tPlot,1);
      tPlot += dtPlot;
    }
  }

  if (FlagPlotIntegrator) {
    time += Timer.stop();
    integPlot<< t << " " << dtOld << " " <<order << " " << iter << " " << systemT1->getlaSize()  << " "<< time  <<endl;
  }
  if(output) cout << "   t = " <<  t << ",\tdt = "<< dtOld << ",\titer = "<<setw(5)<<setiosflags(ios::left) <<iter<<",\torder = "<<order << "\r"<<flush;
}

void TimeSteppingSSCIntegrator::closeIntegrator() {
  time += Timer.stop();
  cout.unsetf(ios::scientific);
  if (FlagPlotIntegrator) integPlot.close();
  if (FlagPlotIntegrationSum) {
    ofstream integSum((systemT1->getDirectoryName() + name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum << "Steps with impacts: " << integrationStepswithChange<< endl;
    if (maxOrder==2) {
      integSum<<"Integration steps order 1: "<<integrationStepsOrder1<<endl;
      integSum<<"Integration steps order 2: "<<integrationStepsOrder2<<endl;
    }
    if (FlagSSC){
      integSum << "Refused steps: " << refusedSteps << endl;
      integSum << "Refused steps with impact: " << refusedStepsWithImpact << endl;
      integSum << "Maximum step size: " << maxdtUsed << endl;
      integSum << "Minimum step size: " << mindtUsed << endl;
      integSum << "Average step size: " << (tEnd-tStart)/integrationSteps << endl;
    }
    integSum << "Maximum number of iterations: " << maxIterUsed << endl;
    integSum << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;
    integSum.close();
  }   
  if (FlagCoutInfo) {
    if (output) cout << endl <<endl;
    cout << "Summary Integration with TimeStepperSSC: "<<endl;
    cout << "Integration time: " << time << endl;
    cout << "Integration steps: " << integrationSteps << endl;
    cout << "Steps with impacts: " << integrationStepswithChange<< endl;
    if (maxOrder==2) {
      cout<<"Integration steps order 1: "<<integrationStepsOrder1<<endl;
      cout<<"Integration steps order 2: "<<integrationStepsOrder2<<endl;
    }
    if (FlagSSC) {
      cout << "Refused steps: " << refusedSteps << endl;
      cout << "Refused steps with impact: " << refusedStepsWithImpact << endl;
      cout << "Maximum step size: " << maxdtUsed << endl;
      cout << "Minimum step size: " << mindtUsed << endl;
      cout << "Average step size: " << (tEnd-tStart)/integrationSteps << endl;
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

  if (!FlagSSC) {
    if (maxOrder==1) {LSe << LSA;     ze << z1d;          }
    if (maxOrder==2) {
      LSe << LSB2;    
      if (!ConstraintsChanged) {
        ze << 2.0*z2d - z1d; order=2;} 
      else {ze<<z2d; order=1;}
    }
    dte = dt;
    return true;
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
    if (maxOrder==2) {
      IterConvergenceBlock1 = (iterB1<maxIter) && (iterC1<maxIter) && (iterC2<maxIter);
      if (ConstraintsChangedBlock1) {
        IterConvergence=true;
        order=1;
        EstErrorLocal = z2b-z4b;
        dtNewRel = calculatedtNewRel(EstErrorLocal,dt);
        testOK = (dtNewRel>=1.0);
        dtNewRel = sqrt(dtNewRel);
        dte= dt/2.0;
        ze  << z4b;
        LSe << LSC2;
      }
      else {
        IterConvergenceBlock2 = (iterB2<maxIter) && (iterB2RE<maxIter) && (iterC3<maxIter) && (iterC4<maxIter);
        if (ConstraintsChanged) {
          EstErrorLocal = z2b-z4b;			// Pruefe auf Einhaltung der Toleranz im Block1 (kein Stoss)
          dtNewRel = calculatedtNewRel(EstErrorLocal,dt);
          testOK = (dtNewRel>=1);
          dtNewRel = sqrt(dtNewRel);
          dte = dt/2.0;
          ze  << 2.0*z4b-z2b;
          LSe << LSC2;
          order = 2;
          if (testOK) {
            EstErrorLocal = z2dRE-z4dRE;
            double dtNewRelTmp = calculatedtNewRel(EstErrorLocal,dt);
            if (dtNewRelTmp>=1) {
              dtNewRel = sqrt(dtNewRelTmp);
              testOK = true;
              dte = dt;
              ze  << z4dRE;
              LSe << LSC4;
              order =1;
            }
            else {IterConvergence=true; IterConvergenceBlock2=true;}
          }
        }
        else {
          order = 2;
          EstErrorLocal = (2.0*z2d-z1d - 2.0*z4dRE+z2dRE)/3.0;
          dtNewRel = calculatedtNewRel(EstErrorLocal,dt);
          testOK = (dtNewRel>=1.0);
          dtNewRel = pow(dtNewRel,1.0/3.0);  
          ze  << 2.0*z4dRE - z2dRE;
          LSe << LSC4;
          dte=dt;
          // order 2 hat mehrfach versagt! Teste auf order 1
          if (!testOK && (StepTrials>2)) {
            double dtNewRelTmp1 = calculatedtNewRel(z2b - z4b, dt);
            double dtNewRelTmp2 = calculatedtNewRel(z4dRE-z2dRE, dt);
            if (dtNewRelTmp1>=1.0) {
              order = 1;
              testOK= true;
              if (dtNewRelTmp2>=1.0) {
                dtNewRel= sqrt(dtNewRelTmp2);
                ze << z4dRE;
              }
              else {
                dtNewRel= sqrt(dtNewRelTmp1);  
                ze << z2b; 
                dte= dt/2.0;
              }
            }
          }
        }
      }
    }
  }

  if (dte<dt) {		// nur halber Integrationsschritt wurde akzeptiert dte==dt/2
    laeUni.resize() = la2bUni;
    laeBi.resize()  = la2bBi;
    iter = iterB1;
    ConstraintsChanged=ConstraintsChangedBlock1;
  }
  else {
    laeUni.resize() = la1dUni;
    laeBi.resize()  = la1dBi;        
    iter = iterA;
  }

  IterConvergence = IterConvergence && IterConvergenceBlock1 && IterConvergenceBlock2;
  if ((! IterConvergence)&&(testOK)) {
    if (dt/2.0>dtMin) {
      testOK= false; 
      dt=dt/2.0;
    }
    else {
      cout<<"Error: no convergence despite minimum stepsize("<<maxIter<<" iterations) Anyway, continuing integration..."<<endl;
    }
  }
  else {
    if(dtNewRel<0.5/safetyFactorSSC) dtNewRel = 0.5/safetyFactorSSC;
    if(dtNewRel > maxGainSSC) dtNewRel=maxGainSSC;   //  0.6 oder 0.7 und 2.5 oder 2  (0.7/2.2 in ...SSC3.h)
    dt = dt*dtNewRel*safetyFactorSSC;
  }  

  if (dt > dtMax) dt = dtMax;
  if (dt < dtMin) dt = dtMin;

  return testOK;
}

double TimeSteppingSSCIntegrator::calculatedtNewRel(const fmatvec::Vec &ErrorLocal, double H) {
  double dtNewRel=1.0e10; 
  double dtNewRel_i;
  double ResTol_i;
  int ErrorSize = ErrorLocal.size();
  if (FlagErrorTest==3) ErrorSize = qSize+xSize;

  for (int i=0; i< ErrorSize; i++) {
    if ((i>=qSize+xSize)&&(FlagErrorTest==2))
      ResTol_i = aTol(i)/H+  rTol(i)*fabs(zi(i));
    else ResTol_i = aTol(i) + rTol(i)*fabs(zi(i));
    dtNewRel_i = ResTol_i / fabs(ErrorLocal(i));
    if (dtNewRel_i < dtNewRel) dtNewRel = dtNewRel_i;
  }
  return dtNewRel;
}

}
