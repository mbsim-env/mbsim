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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _TIME_STEPPING_SSC_INTEGRATOR_H_ 
#define _TIME_STEPPING_SSC_INTEGRATOR_H_

#include "integrator.h"
#include "mbsim/utils/stopwatch.h"

namespace MBSim {
  class Link;
}

namespace MBSimIntegrator {

  /** \brief Half-explicit time-stepping integrator of first or higer order with StepSize Control (SSC)
   *  important options / settings :      
   *                      
   *  a) setMaxOrder(int order, int method=0)
   *                    order:   maximum order of integration scheme 
   *                              1 to 3 with SSC by extrapolation (method=0) (order=4 without SSC)
   *                              1 to 4 with embedded SSC (1=1(2), ... 4=4(5)) (method=1 or 2)
   *                    method:  method used for error estimation and step size control
   *                              0: step size control by extrapolation (steps wit dt and dt/2 are compared)  DEFAULT
   *                              1: embedded method (compare maxOrder maxOrder+1); proceed with maxOrder (recommended)
   *                              2: embedded method with local extrapolation (integration is continued with maxOrder+1)
   *
   * b) setFlagErrorTest(int Flag, bool alwaysValid= true)
   *                    Flag:    for scaling variables for the purpose of error estimation  
   *                              0: include velocities u for error test
   *                              2: scale velocities u with stepsize for error test
   *                              3: exclude velocities u for error test
   *            alwaysValid:  true : u is scaled resp. exluded during smooth and nonsmooth steps
   *            alwaysValid: false : u is scaled resp. exluded only during nonsmooth steps
   *
   * c) deactivateSSC(bool flag=false) : maximum order: 1 to 4
   *
   * d) setGapControl(bool FlagGapControl=true, int GapControlStrategy=0)                  
   *        FlagGapControl       activate / deactivate gap control
   *        GapControlStrategy   choose strategy for Gap Control (1:maximal Stepsize to  4:minimal Penetration) 
   *                              1: uses biggest root (maximal dt)
   *                              2: score for all roots are evaluated
   *                              3: gapTol is used
   *                              4: uses smallest root (minimal penetration)
   *                              0: gap control deactivated
   *
   *  \author Robert Huber
   *  \date 2009-09-07 modifications for mbsim_dev
   */
  class TimeSteppingSSCIntegrator : public Integrator { 

    public:
      enum Method {
        extrapolation=0,
        embedded,
        embeddedHigherOrder,
        unknownMethod
      };

      enum GapControl {
        noGapControl=0,
        biggestRoot,
        scoring,
        gapTolerance,
        smallestRoot,
        unknownGapControl
      };

      enum ErrorTest {
        all=0,
        scale=2,
        exclude=3,
        unknownErrorTest
      };

    protected:
      void resize(MBSim::DynamicSystemSolver *system);

      MBSim::DynamicSystemSolver* sysT1{NULL};
      MBSim::DynamicSystemSolver* sysT2{NULL};
      MBSim::DynamicSystemSolver* sysT3{NULL};

      double dt{1e-6}, dtOld{1e-6}, dte{1e-6};
      double dtMin{0}, dtMax{1e-3};
      double dt_SSC_vorGapControl{0};
      bool driftCompensation{false};
      double t{0}, tPlot{0};
      int qSize{0}, xSize{0}, uSize{0}, zSize{0};
      fmatvec::Vec ze, zi, z1d, z2d, z2dRE, z3d, z4d, z6d, z2b, z3b, z4b, z6b, zStern;
      fmatvec::VecInt LS, LSe, LStmp_T1, LStmp_T2, LStmp_T3, LSA, LSB1, LSB2, LSC1, LSC2, LSC3, LSC4, LSD1, LSD2, LSD3, LSD4, LSD5, LSD6;
      fmatvec::Vec la, lae, la1d, la2b;
      fmatvec::VecInt laSizes, laeSizes, la1dSizes, la2bSizes;
      fmatvec::Vec gInActive, gdInActive;
      std::vector<MBSim::Link*> SetValuedLinkListT1;
      std::vector<MBSim::Link*> SetValuedLinkListT2;
      std::vector<MBSim::Link*> SetValuedLinkListT3;

      int StepsWithUnchangedConstraints{-1};

      /** include (0) or exclude (3) variable u or scale (2) with stepsize for error test*/
      int FlagErrorTest{2};
      /** FlagErrorTest is always valid or only during nonsmooth steps */
      bool FlagErrorTestAlwaysValid{true};
      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** activate step size control*/
      bool FlagSSC{true};
      /** maximum order of integration scheme 
      *   SSC by extrapolation (steps with differnt dt are compared): 1, 2, 3  
      *   SSC with embedded methods (differnt orders are compared): 1 to 4  [ 1= 1(2), ... 4= 4(5) ] maxOrder(maxOrder+1) */
      int maxOrder{1};
      /** method for intrgration and step size control 
      *   method=extrapolation   step size control by extrapolation (steps wit dt and dt/2 are compared)  DEFAULT
      *   method=embedded   embedded method for step size control (maxOrder and maxOrder+1 are compared);  proceed with maxOrder (recommended)
      *   method=embeddedHigherOrder   embedded method for SSC with -local extrapolation- (integration is continued with maxOrder+1) */
      Method method{extrapolation};
      /* Flag for Gap Control */
      bool FlagGapControl{false};
      /** Toleranz for closing gaps */
      double gapTol{1e-6};
      /** maximal gain factor for increasing dt by stepsize control (default 2.5; maxGain * safetyFactor must be GT 1)*/
      double maxGainSSC{2.2};
      /** safety factor for stepsize estimation: dt = dt_estimate * safetyFactorSSC (]0;1]; default 0.6)*/
      double safetyFactorSSC{0.7};
      /** filestream for integrator info at each step */
      std::ofstream integPlot;
      /** Flag: write output info to cout only for plot-time-instances (default false)*/
      bool FlagOutputOnlyAtTPlot{false};
      /** every successful integration step is ploted (set dtPlot=0 to activate FlagPlotEveryStep) (default false)*/
      bool FlagPlotEveryStep{false};
      /** Flag interpolate z and la for plotting (default false)*/
      bool outputInterpolation{false};
      /** Safety factor for GapControl (dtNew = dt_gapcontrol*safetyFactor; default min(1+RTol,1.001) */
      double safetyFactorGapControl{-1};
      /** choose strategy for Gap Control (1:maximal Stepsize to  4:minimal Penetration) 
       *   1: uses biggest root (maximal dt)
       *   2: score for all roots are evaluated
       *   3: gapTol is used
       *   4: uses smallest root (minimal penetration)
       *   0: gap control deactivated
       */
      GapControl GapControlStrategy{biggestRoot};
      /** Number of Threads */
      int numThreads{1};
      /** computaional time*/
      double time{0};
      /** for internal use (start clock, integration info ...) */
      MBSim::StopWatch Timer;
      int iter{0}, iterA{0}, iterB1{0}, iterB2{0}, iterC1{0}, iterC2{0}, iterC3{0}, iterC4{0}, iterB2RE{0}, maxIterUsed{0}, maxIter{0}, sumIter{0};
      int integrationSteps{0}, integrationStepswithChange{0}, refusedSteps{0}, refusedStepsWithImpact{0};
      int wrongAlertGapControl{0}, stepsOkAfterGapControl{0}, stepsRefusedAfterGapControl{0}, statusGapControl{0};
      int singleStepsT1{0}, singleStepsT2{0}, singleStepsT3{0};
      double dtRelGapControl{1}, qUncertaintyByExtrapolation{0};
      int indexLSException{-1};
      fmatvec::Vec gUniActive;
      double Penetration{0}, PenetrationCounter{0}, PenetrationLog{0}, PenetrationMin{0}, PenetrationMax{0};
      double maxdtUsed{0}, mindtUsed{0};
      bool ChangeByGapControl{false};
      bool calcBlock2{0};
      bool IterConvergence{0};
      bool ConstraintsChanged{0}, ConstraintsChangedBlock1{0}, ConstraintsChangedBlock2{0};
      int integrationStepsOrder1{0};
      int integrationStepsOrder2{0};
      int order{1};
      int StepTrials{0};
      int AnzahlAktiverKontakte{0};
      double gNDurchschnittprostep{0};

      fmatvec::Vec bi;

    public:
      /*! Destructor */
      ~TimeSteppingSSCIntegrator() override;
      /*! Set initial step size */
      void setInitialStepSize(double dt_) { dt = dt_; }
      /*! Set maximal step size */
      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }
      /*! Set minimal step size (default 2*maxOrder*epsroot */
      void setMinimumStepSize(double dtMin_) { dtMin = dtMin_; }
      /*! Set maximal gain for increasing dt by stepsize control */
      void setMaximumGain(double maxGain) { maxGainSSC = maxGain; }
      /*! safety factor for stepsize estimation: dt = dt_estimate * safetyFactorSSC (]0;1]; default 0.6)*/
      void setSafetyFactor(double sfactor) { safetyFactorSSC=sfactor; }
      void setSafetyFactorForGapControl(double s) { safetyFactorGapControl = s; }
      /*! Set Flag for output interpolation */
      void setOutputInterpolation(bool flag) { outputInterpolation = flag; }
      /*! set Flag for writing output only at tPlot-Time instances or not (default false)*/
      void setFlagOutputOnlyAtTPlot(bool flag) { FlagOutputOnlyAtTPlot = flag; }
/*! Set Flag to optimise dt for minmal penetration of unilateral links;
       *  choose strategy for Gap Control (1:maximal Stepsize to  4:minimal Penetration) 
       *   1: uses biggest root (maximal dt)
       *   2: score for all roots are evaluated
       *   3: gapTol is used
       *   4: uses smallest root (minimal penetration)
       *   0: gap control deactivated with statistic calculations
       *   -1: gap control deactivated without statistic calculations
       */
      void setGapControl(GapControl gapControl) { GapControlStrategy = gapControl; }
      /*! Set drift compensation */
      void setDriftCompensation(bool dc) { driftCompensation = dc; }
      /*! set maximum order (1,2,3 (method=extrapolation) or 1 to 4 (method=embedded,embeddedHigherOrder) and
       *      without SSC maximum order from 1 to 4 is possible
       */
      void setMaximumOrder(int maxOrder_) { maxOrder = maxOrder_; }
      /*! set method
       *    extrapolation: SSC by extrapolation (recommended!!);
       *    embedded,embeddedHigherOrder: embedded SSC; proceed with maxOrder [1] (recommended if you don't want to use 0) or with maxOrder+1 [2]
       */
      void setMethod(Method method_) { method = method_; }
      /* deactivate step size control */
      void setStepSizeControl(bool flag) { FlagSSC = flag; }
      /*! Set error test (default 0: all variables ae tested;  2: u is scaled with dt;  3: exclude u
       */
      void setErrorTest(ErrorTest errorTest) { FlagErrorTest = errorTest; }
      /*! alwaysValid = true  : u is scaled resp. exluded during the smooth and nonsmooth part
       *  alwaysValid = false : u is scaled resp. exluded only during nonsmooth steps
       */
      void setAlwaysValid(bool alwaysValid) { FlagErrorTestAlwaysValid = alwaysValid; }

      /*! Start the integration */
      using Integrator::integrate;
      void integrate() override;
      /*! Threads: Number of Threads (0,1,2 or 3) 0: auto (number of threads depends on order and SSC)*/ 
      void integrate(MBSim::DynamicSystemSolver& systemT1_, MBSim::DynamicSystemSolver& systemT2_, MBSim::DynamicSystemSolver& systemT3_, int Threads=0);

      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) { aTol = aTol_; }
      void setAbsoluteTolerance(double aTol_) { aTol = fmatvec::Vec(1,fmatvec::INIT,aTol_); }
      void setRelativeTolerance(const fmatvec::Vec &rTol_) { rTol = rTol_; }
      void setRelativeTolerance(double rTol_) { rTol = fmatvec::Vec(1,fmatvec::INIT,rTol_); }
      void setgapTolerance(double gTol) { gapTol = gTol; }

      /** subroutines for integrate function */

      void preIntegrate();
      void subIntegrate(double tStop);
      void postIntegrate();
      void preIntegrate(MBSim::DynamicSystemSolver& systemT1_, MBSim::DynamicSystemSolver& systemT2_, MBSim::DynamicSystemSolver& systemT3_);
       
      /** internal subroutines */
      void getDataForGapControl();
      bool testTolerances();
      bool GapControl(double qUnsafe, bool SSCTestOK); 
      bool changedLinkStatus(const fmatvec::VecInt &L1, const fmatvec::VecInt &L2, int ex);
      double calculatedtNewRel(const fmatvec::Vec &ErrorLocal, double H);
      void plot();

      virtual void initializeUsingXML(xercesc::DOMElement *element);

      void updatebi();
      void updatela();
      void updatezd();
  };

}

#endif 
