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
 *   jan.p.clauberg@googlemail.com
 *   huber@amm.mw.tum.de
 */

#ifndef _AUTO_TIME_STEPPING_SSC_INTEGRATOR_H_ 
#define _AUTO_TIME_STEPPING_SSC_INTEGRATOR_H_

#include<fmatvec.h>
#include "integrator.h"
#include "mbsim/utils/stopwatch.h"

namespace MBSim {
  
  class Link;

  /** \brief Explicit and Implicit time-stepping integrator of first or higer order with StepSize Control (SSC)
   *  StepSizeControl, GapControl, Extrapolation und concept analogue to TimeSteppingSSCIntegrator
   *
   *  important options / settings :      
   *                      
   *  a) setMaxOrder(int order, int method=0)
   *                    order:   maximum order of integration scheme 
   *                              1 to 3 with SSC by extrapolation (method=0) (order=4 without SSC)
   *                              1 to 4 with embedded SSC (1=1(2), ... 4=4(5)) (method=1 or 2)
   *                    method:  method used for error estimation and Stepsize control
   *                              0: step size control by extrapolation (steps wit dt and dt/2 are compared)  DEFAULT
   *                              1: embedded method (compare maxOrder maxOrder+1); proceed with maxOrder (recommended)
   *                              2: embedded method with local extrapolation (integration is continued with maxOrder+1)
   *
   *  b) setFlagErrorTest(int Flag, bool alwaysValid= true)
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
   * e) Basic Implicit Integration
   *      Implicit Integration only possible with:
   *        - StepSizeControl method 0 (step size control by extrapolation)
   *        - GapControl deactivated (GapControl should work, but isn't tested in detail)
   *
   * f) setInexactJac(bool inexactJac_=false)
   *      inexactJac_: true:  - Linear Implicit Integration: Jacobis are updated if time step size changes,
   *                          - Full Implicit Integration:   Jacobis are updated of Newton-Iterations doesn't converge
   *
   * e) Parallel Computation:
   *      The concept allows the following parallel computation methods:
   *        - parallel computation of different time step sizes dt, dt/2, etc.
   *        - parallel computation of Jacobi matrices (dhdu, dhdq)
   *        - parallel plotting of system during the next integration step
   *
   *      Usage:
   *        - integrate(sys):                     sequential computation
   *        - integrate(sys, systems):            sequential integration, parallel computation of Jacobis
   *        - integrate(sys,sys,sys,sys):         parallel integration, parallel plotting, sequential computation of Jacobis
   *        - integrate(sys,sys,sys,sys,systems): parallel integration, parallel plotting, parallel computations of Jacobis 
   *
   *
   *
   *
   *  \author Jan Clauberg
   */
  class AutoTimeSteppingSSCIntegrator : public Integrator { 

    protected:
      DynamicSystemSolver* sysT1;
      DynamicSystemSolver* sysT2;
      DynamicSystemSolver* sysT3;
      DynamicSystemSolver* sysTP;

      double dt, dtOld, dte;
      double dtMin, dtMax;
      double dt_SSC_vorGapControl;
      bool driftCompensation;
      double t, tPlot, tPlotP;
      int qSize, xSize, uSize, zSize;
      fmatvec::Vec ze, zeP, zi, ziP, zT1, zT2, zT3, zTP, z1d, z2d, z2dRE, z3d, z4d, z6d, z2b, z3b, z4b, z6b, zStern;
      
      fmatvec::VecInt LS, LSe, LSA, LSB1, LSB2, LSC1, LSC2, LSC3, LSC4, LSD1, LSD2, LSD3, LSD4, LSD5, LSD6;
      fmatvec::VecInt LS_Reg, LSe_Reg, LSA_Reg, LSB1_Reg, LSB2_Reg, LSC1_Reg, LSC2_Reg, LSC3_Reg, LSC4_Reg, LSD1_Reg, LSD2_Reg, LSD3_Reg, LSD4_Reg, LSD5_Reg, LSD6_Reg;
      fmatvec::VecInt LS_Reg_T1, LS_Reg_T2, LS_Reg_T3;
      fmatvec::VecInt LS_Reg_z2b, S_Reg_z1d, LS_Reg_z4b, LS_Reg_z6b, LS_Reg_z1d, LS_Reg_z3b, LSB1_2_Reg;
      fmatvec::VecInt LS_tmp;

      fmatvec::Vec la, laP, lae, laeP, la1d, la2b;
      fmatvec::VecInt laSizes, laSizesP, laeSizes, laeSizesP, la1dSizes, la2bSizes;
      fmatvec::Vec qT1, qT2, qT3, qTP;
      fmatvec::Vec uT1, uT2, uT3, uTP;
      fmatvec::Vec xT1, xT2, xT3, xTP;
      fmatvec::Vec gInActive, gdInActive;
      std::vector<MBSim::Link*> SetValuedLinkListT1;
      std::vector<MBSim::Link*> SetValuedLinkListT2;
      std::vector<MBSim::Link*> SetValuedLinkListT3;
      std::vector<MBSim::Link*> SetValuedLinkListTP;

      int StepsWithUnchangedConstraints;

      /** Include (0) or Exclude (3) Variable u or Scale (2) with Stepsize for Error Test*/
      int FlagErrorTest;
      
      /** FlagErrorTest is Always Valid or Only During Nonsmooth Steps */
      bool FlagErrorTestAlwaysValid;
      
      /** Absolute Tolerance */
      fmatvec::Vec aTol;
      
      /** Relative Tolerance */
      fmatvec::Vec rTol;
      
      /** Stepsize Control activated or Deactivated */
      bool FlagSSC;
      
      /** Maximum order of Integration Scheme 
      *   SSC by extrapolation (steps with differnt dt are compared): 1, 2, 3  
      *   SSC with embedded methods (differnt orders are compared): 1 to 4  [ 1= 1(2), ... 4= 4(5) ] maxOrder(maxOrder+1) - not available for implicit integration */
      int maxOrder;
      
      /** method for intrgration and step size control 
      *   method=0   step size control by extrapolation (steps wit dt and dt/2 are compared)  DEFAULT
      *   method=1   embedded method for step size control (maxOrder and maxOrder+1 are compared);  proceed with maxOrder (recommended)
      *   method=2   embedded method for SSC with -local extrapolation- (integration is continued with maxOrder+1) */
      int method;
      
      /* Flag for Gap Control - Gap Control not available for implicit integration*/
      bool FlagGapControl;
      
      /** Toleranz for closing gaps */
      double gapTol;
      
      /** maximal gain factor for increasing dt by stepsize control (default 2.5; maxGain * safetyFactor must be GT 1)*/
      double maxGainSSC;
      
      /** safety factor for stepsize estimation: dt = dt_estimate * safetyFactorSSC (]0;1]; default 0.6)*/
      double safetyFactorSSC; 
      
      /** filestream for integrator info at each step */
      std::ofstream integPlot;
      
      /** Flag: write integrator info at each step to a file (default true)*/
      bool FlagPlotIntegrator;
      
      /** Flag: write integration summary to a file (default true)*/
      bool FlagPlotIntegrationSum;
      
      /** Flag: write integration info (integ. summary) to cout  (default true)*/
      bool FlagCoutInfo;
      
      /** every successful integration step is ploted (set dtPlot=0 to activate FlagPlotEveryStep) (default false)*/
      bool FlagPlotEveryStep;
      
      /** Flag interpolate z and la for plotting (default true)*/
      bool outputInterpolation;
      
      /** Safety factor for GapControl (dtNew = dt_gapcontrol*safetyFactor; default min(1+RTol,1.001) */
      double safetyFactorGapControl;
      
      /** choose strategy for Gap Control (1:maximal Stepsize to  4:minimal Penetration) 
       *   1: uses biggest root (maximal dt)
       *   2: score for all roots are evaluated
       *   3: gapTol is used
       *   4: uses smallest root (minimal penetration)
       *   0: gap control deactivated
       */
      int GapControlStrategy;
      
      /** Number of Threads */
      int numThreads, numThreadsB1;
      
      /** computaional time*/
      double time, dhdztime, timeB1, timeB2, timePlot, timePlotPar;
      
      /** for internal use (start clock, integration info ...) */
      StopWatch Timer, dhdzTimer, TimerB1, TimerB2, TimerPlot, TimerPlotPar;

      /** Iterations for solving set-valued laws of different systems */
      int iter, iterA, iterB1, iterB2, iterC1, iterC2, iterC3, iterC4, iterB2RE, maxIterUsed, maxIter, sumIter;

      /** Counter for Integrations steps */
      int integrationSteps, integrationStepswithChange, refusedSteps, refusedStepsWithImpact;
      
      /** Counter for Jacobi updates */
      int JacUpdateT1, JacUpdateT2, JacUpdateT3;

      /** Variables for GapControl */
      int wrongAlertGapControl, stepsOkAfterGapControl, stepsRefusedAfterGapControl, statusGapControl;
      
      /** Single steps of different systems */
      int singleStepsT1, singleStepsT2, singleStepsT3;

      /** Other internal variables */
      double dtRelGapControl, qUncertaintyByExtrapolation;
      int indexLSException;
      fmatvec::Vec gUniActive;
      double Penetration, PenetrationCounter, PenetrationLog, PenetrationMin, PenetrationMax;
      double maxdtUsed, mindtUsed;
      bool ChangeByGapControl;
      bool calcBlock2;
      bool IterConvergence;
      bool ConstraintsChanged, ConstraintsChangedBlock1, ConstraintsChangedBlock2, ConstraintsChanged_B1_A;
      int integrationStepsOrder1, integrationStepsOrder2;
      int order;
      int StepTrials;
      int AnzahlAktiverKontakte;
      double gNDurchschnittprostep;

      /** iteration counters */
      int iter_T1, iter_T2, iter_T3, step;
      
      fmatvec::Index Iq, Iu, Ix;
      
      /** Flag for Explicit or Implicit Integration */
      bool expInt;

      /** Number of Iterations for Full-Implicit Integration */
      int it_T1, it_T2, it_T3;
      
      /** Flag if gActiveChanged (set-valued) */
      bool gAC_T1, gAC_T2, gAC_T3, gAC_reg_T1, gAC_reg_T2, gAC_reg_T3;
      
      /** Flag if System has already been updated */
      bool upgedated_T1, upgedated_T2, upgedated_T3;

      /** Flag if Jabobis of different Systems were updated in B1 or B2 */
      bool JacUpdate_B1_T1, JacUpdate_B2_T1, JacUpdate_B1_T2, JacUpdate_B2_T2, JacUpdate_T1, JacUpdate_T2;
      
      /** Maximal number of Iterations for Full-Implicit Integration */
      int itMax;
      
      /** Tolerance for Full-Implicit Integration */
      double itTol;
      
      /** Jacobi-Matrices for implicit integration */
      fmatvec::Mat dhdq_n_T1, dhdu_n_T1, dhdq_n_T2, dhdu_n_T2, dhdq_n_T3, dhdu_n_T3, dhdq_T1, dhdu_T1, dhdq_T2, dhdu_T2, dhdq_T3, dhdu_T3;
      fmatvec::Mat dhdq_z1d, dhdu_z1d, dhdq_z2b, dhdq_z3b, dhdq_z4b, dhdq_z6b, dhdu_z2b, dhdu_z3b, dhdu_z4b, dhdu_z6b, dhdq_end, dhdu_end, dhdq_z2d, dhdu_z2d, dhdq_z2dRE, dhdu_z2dRE, dhdq_z3d, dhdu_z3d, dhdq_z4d, dhdu_z4d, dhdq_z6d, dhdu_z6d;
      
      /** combination parameter between explicit (0) and implicit (1) Euler scheme */
      double theta;

      bool parJac;
      bool parInt;

      std::vector<DynamicSystemSolver*> *psystems;

      /** Flag of Exact or Inexact Jacobians should be uses */
      bool inexactJac;

      int maxImpIter;
      int JacConstSteps;

      /** Set-valued force laws with explicit or implicit discretization */
      bool SetValuedForceLawsExplicit;

      /** Flag if Debug-Output should be displayed */
      bool debugOutput;

      /** Flag if Plotting should be done in Parallel */
      bool plotParallel;

      // TODO delme
      fmatvec::Mat JacVgl, JacVgl2, JacVgl3;

      /** Counter for all Jacobian Updates */
      int JacCounter;

    public:
      
      /*! Constructor with \default dt(1e-5), \default driftCompensation(false) */
      AutoTimeSteppingSSCIntegrator();
      
      /*! Destructor */
      ~AutoTimeSteppingSSCIntegrator();
      
      /*! Set initial step size */
      void setInitialStepSize(double dt_) {dt = dt_;}
      
      /*! Set maximal step size */
      void setStepSizeMax(double dtMax_) {dtMax = dtMax_;}
      
      /*! Set minimal step size (default 2*maxOrder*epsroot() */
      void setStepSizeMin(double dtMin_) {dtMin = dtMin_;}
      
      /*! Set maximal gain for increasing dt by stepsize control */
      void setmaxGainSSC(double maxGain) {maxGainSSC = maxGain;}
      
      /*! safety factor for stepsize estimation: dt = dt_estimate * safetyFactorSSC (]0;1]; default 0.6)*/
      void setSafetyFactorSSC(double sfactor) {safetyFactorSSC=sfactor;}
      void setSafetyFactorGapControl(double s){safetyFactorGapControl=s;}
      
      /*! Set Flag for output interpolation */
      void setOutputInterpolation(bool flag=true) {outputInterpolation = flag;}
      
      /*! set Flag for  writing integrator info at each step to a file (default true)*/
      void setFlagPlotIntegrator(bool flag=true) {FlagPlotIntegrator = flag;}
      
      /*! Set Flag to optimise dt for minmal penetration of unilateral links;
       *  choose strategy for Gap Control (1:maximal Stepsize to  4:minimal Penetration) 
       *   1: uses biggest root (maximal dt)
       *   2: score for all roots are evaluated
       *   3: gapTol is used
       *   4: uses smallest root (minimal penetration)
       *   0: gap control deactivated with statistic calculations
       *   -1: gap control deactivated without statistic calculations
       */
      void setGapControl(int strategy=1) {FlagGapControl=(strategy>=0); GapControlStrategy=(strategy<0)?0:strategy; }
      
      /*! Set drift compensation */
      void setDriftCompensation(bool dc) {driftCompensation = dc;}
      
      /*! set maximum order (1,2,3 (method=0) or 1 to 4 (method=1,2) and
       *      method 0: SSC by extrapolation (recommended!!);  
       *      1,2: embedded SSC; proceed with maxOrder [1] (recommended if you don't want to use 0) or with maxOrder+1 [2]
       *      without SSC maximum order from 1 to 4 is possible*/  
      void setMaxOrder(int order_, int method_=0);
      
      /* deactivate step size control */
      void deactivateSSC(bool flag=false) {FlagSSC=flag;}
      
      /*! Set Flag vor ErrorTest (default 0: all variables ae tested;  2: u is scaled with dt;  3: exclude u
       *      alwaysValid = true  : u is scaled resp. exluded during the smooth and nonsmooth part
       *      alwaysValid = false : u is scaled resp. exluded only during nonsmooth steps
       */
      void setFlagErrorTest(int Flag, bool alwaysValid=true);

      /*! Start the integration */
      void integrate(DynamicSystemSolver& system_);
      void integrate(DynamicSystemSolver& system_, std::vector<DynamicSystemSolver*> systems);

      void integrate(DynamicSystemSolver& systemT1_, DynamicSystemSolver& systemT2_, DynamicSystemSolver& systemT3_, DynamicSystemSolver& systemTP_, int Threads=0);
      void integrate(DynamicSystemSolver& systemT1_, DynamicSystemSolver& systemT2_, DynamicSystemSolver& systemT3_, DynamicSystemSolver& systemTP_, std::vector<DynamicSystemSolver*> systems, int Threads=0);
      
      /** Tolerances for Integrator */
      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) {aTol.resize() = aTol_;}
      void setAbsoluteTolerance(double aTol_) {aTol.resize() = fmatvec::Vec(1,fmatvec::INIT,aTol_);}
      void setRelativeTolerance(const fmatvec::Vec &rTol_) {rTol.resize() = rTol_;}
      void setRelativeTolerance(double rTol_) {rTol.resize() = fmatvec::Vec(1,fmatvec::INIT,rTol_);}
      void setgapTolerance(double gTol) {gapTol = gTol;}

      /** subroutines for integrate function */
      void preIntegrate(DynamicSystemSolver& system);
      void subIntegrate(DynamicSystemSolver& system, double tStop);
      void postIntegrate(DynamicSystemSolver& system);
      void preIntegrate(DynamicSystemSolver& systemT1_, DynamicSystemSolver& systemT2_, DynamicSystemSolver& systemT3_, DynamicSystemSolver& systemTP_);
       
      /** internal subroutines */
      void getAllSetValuedla(fmatvec::Vec& la_,fmatvec::VecInt& la_Sizes,std::vector<MBSim::Link*> &SetValuedLinkList);
      void setAllSetValuedla(const fmatvec::Vec& la_,const fmatvec::VecInt& la_Sizes,std::vector<MBSim::Link*> &SetValuedLinkList);
      void getDataForGapControl();
      bool testTolerances();
      bool GapControl(double qUnsafe, bool SSCTestOK); 
      bool changedLinkStatus(const fmatvec::VecInt &L1, const fmatvec::VecInt &L2, int ex);
      double calculatedtNewRel(const fmatvec::Vec &ErrorLocal, double H);
      void plot();
      void plotPar();
      void doIntegPlot();

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element); // TODO
      
      /**
       * \brief special update of dynamic system for theta time stepping integrator
       * \param dynamic system
       * \param state vector
       * \param time
       * \param System number
       */
      void update(DynamicSystemSolver& system, const fmatvec::Vec& z, double t, int nrSys_=1);
      
      void doStep(DynamicSystemSolver& system_, fmatvec::Vec& z_, int nrSys_, double t_, double dt_, bool exp_);
      
      /** 
       * \brief Do One Explicit Time Integration Step
       * \param dynamic system
       * \param Vector z
       * \param Number of Dynamic System
       * \param Current Time
       * \param Current TimeStepSize
       */
      void doExpStep(DynamicSystemSolver& system_, fmatvec::Vec& z_, int nrSys_, double t_, double dt_);
      
      /** 
       * \brief Do One Implicit Time Integration Step
       * \param dynamic system
       * \param Vector z
       * \param Number of Dynamic System
       * \param Current Time
       * \param Current TimeStepSize
       */
      void doImpStep(DynamicSystemSolver& system_, fmatvec::Vec& z_, int nrSys_, double t_, double dt_);
     
      /** 
       * \brief Do One Linear Implicit Time Integration Step
       * \param dynamic system
       * \param Vector z
       * \param Number of Dynamic System
       * \param Current Time
       * \param Current TimeStepSize
       */
      void doLinImpStep(DynamicSystemSolver& system_, fmatvec::Vec& z_, int nrSys_, double t_, double dt_);

      /** set combination parameter between explicit (0) and implicit (1) Euler scheme */
      void setTheta(double theta_ ) { theta  = theta_; }

      /** get Jabobians by parallel calculation */
      void getdhdqdhdu(fmatvec::Mat& dhdq_, fmatvec::Mat& dhdu_, const fmatvec::Vec z_, const double t_, const int nsys_);

      /** set maximum number of full implicit iterations */
      void setItMax(int itMax_) {itMax = itMax_;}
      
      /** set of exact or inexact Jabians should be used */
      void setInexactJac(bool inexactJac_) {inexactJac = inexactJac_;}
      
      /** Tolerance for Newton Iterations of full implicit iterations */
      void setItTol(double itTol_) {itTol = itTol_;}

      /** Set-valued force laws with explicit or implicit discretization */
      void setSetValuedForceLawsExplicit(bool SetValuedForceLawsExplicit_) {SetValuedForceLawsExplicit = SetValuedForceLawsExplicit_;}

      /** Flag if Debug-Output should be displayed */
      void setDebugOutput(bool debugOutput_) {debugOutput = debugOutput_;}

      /** Flag if Plotting should be done in Parallel */
      void setPlotParallel(bool plotParallel_) {plotParallel = plotParallel_;}
  };

}

#endif 
