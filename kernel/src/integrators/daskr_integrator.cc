/* Copyright (C) 2007  Robert Huber

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
 *   rhuber.berlios.de
 *
 */

#include<config.h>
#include<ctime>
#include<fstream>
#include "multi_body_system.h"
#include "fortran_wrapper.h"
#include "daskr_integrator.h"

#ifndef NO_ISO_14882
using namespace std;
#endif

namespace MBSim {

  DASKRIntegrator::DASKRIntegrator() : aTol(1,INIT,1e-6), rTol(1,INIT,1e-4), dt0(0), maxSteps(100000), dtMax(0), DAEIndex(2), MaxOrder(5), FlagErrorTest(1), useExternalJac(0), info(20,INIT,0), time(0), FlagPlotIntegrator(true), FlagPlotIntegrationSum(true), FlagCoutInfo(true),IntSteps(0), RefusedSteps(0), FuncEvals(0), JacEval(0), RootEvals(0) {
    name = "DASKR";
  }

  void DASKRIntegrator::setDAEIndex(int index) {	// Index 2 or Index 2 with Gear Gupta Leimkuhler Stabilisation (21) or 1
    DAEIndex = index;					// Index 1: DASKR is used as ODE Integrator (la are calculated by mbsim)
    if (!(DAEIndex == 21)) {
      assert(DAEIndex<3);
      assert(DAEIndex>0);
    }
  }

  void DASKRIntegrator::setMaxOrder(int order_) {
    MaxOrder = order_;
    assert(MaxOrder<6);
    assert(MaxOrder>0);
  }

  void DASKRIntegrator::residuum(double* t, double* Y_, double* Ydot_, double* CJ, double* res_, int* ires, double* rpar, int* ipar) {
    Vec Y(*(ipar+2), Y_);
    Vec Ydot(*(ipar+2), Ydot_);
    Vec res(*(ipar+2), res_);
    system->F_DAE(Y, res, *t, *ipar);
    res(0,*(ipar+1)-1) -= Ydot(0,*(ipar+1)-1);
    //cout << "t="<<*t<<"  "<<res<<endl;
  }

  void DASKRIntegrator::jac(double* t, double* Y_, double* Ydot_, double* PD, double* CJ, double* rpar, int* ipar) {
    Vec Y(*(ipar+2), Y_);
    Mat Jacobian(*(ipar+2), *(ipar+2), PD);
    system->JacF_DAE(*t, Y, Jacobian, *ipar);
    Jacobian(0,0,*(ipar+1)-1,*(ipar+1)-1) -= *CJ * Mat(*(ipar+1),*(ipar+1),EYE);
  }

  void DASKRIntegrator::rt(int* neq, double* t, double* Y_, double* Ydot_, int* nrt, double *rval_, double* rpar, int* ipar) {
    Vec Y(*(ipar+2), Y_);
    Vec rval(*nrt,rval_);
    system->root_DAE(Y,rval,*t);
  }

  void DASKRIntegrator::setFlagErrorTest(int Flag) {
    FlagErrorTest = Flag;
    assert(FlagErrorTest>=0);		// =0	control errors locally on all variables (diff. and algebraic)
    assert(FlagErrorTest<3);		// =1   exclude algebraic variables from error test
  }					// =2   variables are scaled with stepsize (index 2) or stepsize**2 (index 3)


  void DASKRIntegrator::initIntegrator(MultiBodySystem &system_) {
    system = &system_;
    refreshIntegratorSetups();
    if (FlagCoutInfo) {
      cout << "DAE index ";
      if (DAEIndex==21) cout << "2 (GGL formulation)";
      else cout << DAEIndex;
      cout << " with ";
      if (useExternalJac) cout << "external jacobian." << endl;
      else cout << "internally computed jacobian." << endl;
    }

    cout.setf(ios::scientific, ios::floatfield);

    if (FlagPlotIntegrator) {
      integPlot.open((system->getDirectoryName() + name + ".plt").c_str());
      integPlot << "#1 t [s]:" << endl; 
      integPlot << "#2 dt [s]:" << endl; 
      integPlot << "#3 order :"<< endl;
      integPlot << "#4 idid : "<< endl;
      integPlot << "#5 calculation time [s]:" << endl;
    }
  }

  void DASKRIntegrator::refreshIntegratorSetups() {
    int zSize= system->getzSize();
    system->saveUnilaterLinkStatus();
    int laBiSize = system->getlaBilateralSize();
    int laUniSize= system->getSizeUnilateralConstraints();
    YSize    = zSize + laBiSize + laUniSize;
    nrt = system->getsvSize();
    if (DAEIndex==21) YSize += laBiSize;		//TODO
    if (DAEIndex==1)  YSize = zSize;			//TODO

    Y.resize(YSize,INIT,0.0);
    Ydot.resize(YSize,INIT, 0.0);

    t = tStart;
    z.resize();
    z>> Y(0,zSize-1);

    if(z0.size())
      Y(0,zSize-1) = z0;
    else
      system->initz(z);

    assert(aTol.size() == rTol.size());
    int iTol;
    if(aTol.size() == 1) iTol = 0; // Skalar
    else {
      iTol = 1; // Vektor
      assert (aTol.size() >= YSize);
    }
    
    info(0) = 0;			// initialisation of integrator
    info(1) = iTol;			// scalar or vector as tolerances    
    info(2) = 0; 			// intermediate output mode
    if(useExternalJac)info(4) = 1;	// info(4)=1: use external function for jac; 
    if(dtMax) info(6) = 1;		// max stepsize
    if(dt0)   info(7) = 1;		// initial stepsize
    if((MaxOrder)&&(! MaxOrder==5))     // max Order (default for info(8)=0 Order 5)
      info(8)=1;    
    info(13)= 1;			// stop after initial conditions calculation
    info(15)= FlagErrorTest;		// error test (0: include algebraic variables; 1: exclude algebraic variables)
                                        //             2: ATol scaled with dt(index 2 variables) ord dt^2(index 3 var.))

    LRW = 60 + 9*YSize + YSize*YSize + 3*nrt;
    if (info(15)) LRW+= YSize;
    LIW = 50 + YSize + YSize;
    int LID = 50;

    jroot.resize(nrt,INIT,0); 

    rwork.resize(LRW,INIT,0.0);
    iwork.resize(LIW,INIT,0);

    for (int i=0; i<zSize; i++)     iwork(i+LID) = 1;		// differential variable
    for (int i=zSize; i<YSize; i++) iwork(i+LID) =-1;		// algebraic variable
    if(info(6)) rwork(1) = dtMax;				// max stepsize
    if(info(7)) rwork(2) = dt0;					// initial stepsize
    if(info(8)) iwork(2) = MaxOrder;				// maximum order
    iwork(44) = system->getqSize() + system->getxSize(); 	// Dimension of Index 1 Variables 
    iwork(45) = system->getuSize();				// Dimension of Index 2 Variables
    iwork(46) = laBiSize+laUniSize;				// Dimension of Index 3 Variables

    rPar.resize(1);
    iPar.resize(3,INIT,0);
    iPar(0) = DAEIndex;    
    iPar(1) = zSize;
    iPar(2) = YSize;
  }


  int DASKRIntegrator::computeInitialConditions(bool FlagPlot0, bool FlagCoutInfo_) {
    info(10)= 1; 			// calculate consistent initial values
    // calculate initial conditions
    DDASKR(residuum, &YSize, &t, Y(), Ydot(), &tEnd,
	info(), rTol(), aTol(), &idid, rwork(), &LRW, iwork(), &LIW, rPar(), iPar(), jac, 0, rt, &nrt, jroot());
    if (idid == 4) {
      if (FlagCoutInfo_) cout << "daskr initial condition calculation was successfull."<<endl;
      info(10) = 0; 			// switch off calculation of initial values
      if (FlagPlot0) system->plotDAE(Y,t,DAEIndex);
    }
    return idid;
  }


  void DASKRIntegrator::IntegrationStep() 
  {
    s0 = clock();
    double tOut = t+dtPlot;
    if (tOut>tEnd) tOut = tEnd;
    bool ExitIntegration = (t>=tEnd);
    
    while(! ExitIntegration) {
      DDASKR(residuum, &YSize, &t, Y(), Ydot(), &tOut,
	  info(), rTol(), aTol(), &idid, rwork(), &LRW, iwork(), &LIW, rPar(), iPar(), jac, 0, rt, &nrt, jroot());

      if (idid>0) {
	ExitIntegration = (t>=tEnd);
	if (idid==5) {
	  ExitIntegration=true;
	}
	if(idid==2 || idid==3) {
	  tOut += dtPlot;
	  if (tOut>tEnd) tOut = tEnd;
	  if ((tEnd-tOut)<dtPlot) tOut = tEnd;

	  system->plotDAE(Y,t,DAEIndex);
	}
	if (output) 
	  cout << "   t = " <<  t << ",\tdt = "<< rwork(6) << ",\torder = "<<iwork(7)<<"\r"<<flush;
	if (FlagPlotIntegrator) {
	  double s1 = clock();
	  time += (s1-s0)/CLOCKS_PER_SEC;
	  s0 = s1; 
	  integPlot<< t << " " << rwork(6) << " "<<iwork(7)<< " " << idid << " "<< time << endl;
	}
      }
      else {
	if (idid==-1) {
	  if (iwork(10)<maxSteps) info(0)=1;
	  else exit(idid);
	}
	else  exit(idid);
      }

    } 
   IntSteps     += iwork(10); 
   RefusedSteps += iwork(13);
   FuncEvals    += iwork(11);
   JacEval      += iwork(12);
   RootEvals    += iwork(35);
  }


  void DASKRIntegrator::closeIntegrator() {
    cout.unsetf (ios::scientific);
    double s1 = clock();
    time += (s1-s0)/CLOCKS_PER_SEC;
    if (FlagPlotIntegrator) integPlot.close();
    if (FlagPlotIntegrationSum) {
      ofstream integSum((system->getDirectoryName() + name + ".sum").c_str());
      integSum << "Integration time     : " << time         << endl;
      integSum << "Integration steps    : " << IntSteps     << endl;
      integSum << "Refused steps        : " << RefusedSteps << endl;
      integSum << "Function calls       : " << FuncEvals    << endl;
      integSum << "Jacobian evaluations : " << JacEval      << endl;
      integSum << "Root function calls  : " << RootEvals    << endl; 
      integSum.close();
    }
    if (FlagCoutInfo) {
      if (output) cout << endl<<endl;
      cout << "Summary Integration with DASKR : "<< endl;
      cout << "Integration time     : " << time         << endl;
      cout << "Integration steps    : " << IntSteps     << endl;
      cout << "Refused steps        : " << RefusedSteps << endl;
      cout << "Function calls       : " << FuncEvals    << endl;
      cout << "Jacobian evaluations : " << JacEval      << endl;
      cout << "Root function calls  : " << RootEvals    << endl; 
    }
  }

  void DASKRIntegrator::integrate(MultiBodySystem& system_) {
    initIntegrator(system_);
    int errorCodeIniCond = computeInitialConditions(true,FlagCoutInfo);
    if (errorCodeIniCond>0) IntegrationStep();
    closeIntegrator();
  }



}

