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
 */

#include <config.h>
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/element.h"
#include "mbsim/link.h"
#include "auto_time_stepping_ssc_integrator.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/stopwatch.h"
#include "mbxmlutilstinyxml/tinynamespace.h"
#include "mbxmlutilstinyxml/tinyxml.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef NO_ISO_14882
using namespace std;
#endif


using namespace fmatvec;
using namespace MBXMLUtils;

namespace MBSim {


  AutoTimeSteppingSSCIntegrator::AutoTimeSteppingSSCIntegrator() : sysT1(NULL), sysT2(NULL), sysT3(NULL), sysTP(NULL), dt(1e-6), dtOld(1e-6), dte(1e-6), dtMin(0), dtMax(1e-3), dt_SSC_vorGapControl(0), driftCompensation(false), t(0), tPlot(0), tPlotP(0), qSize(0), xSize(0), uSize(0),zSize(0), StepsWithUnchangedConstraints(-1), FlagErrorTest(2), FlagErrorTestAlwaysValid(true), aTol(1,INIT,1e-6), rTol(1,INIT,1e-4),FlagSSC(1), maxOrder(1), method(0), FlagGapControl(false), gapTol(1e-6), maxGainSSC(2.2), safetyFactorSSC(0.7), FlagPlotIntegrator(true), FlagPlotIntegrationSum(true), FlagCoutInfo(true), FlagPlotEveryStep(false), outputInterpolation(false), safetyFactorGapControl(-1), GapControlStrategy(1), numThreads(0), time(0.0), dhdztime(0.0), iter(0), iterA(0), iterB1(0), iterB2(0), iterC1(0), iterC2(0), iterC3(0), iterC4(0), iterB2RE(0), maxIterUsed(0), maxIter(0), sumIter(0), integrationSteps(0), integrationStepswithChange(0), refusedSteps(0), refusedStepsWithImpact(0), wrongAlertGapControl(0), stepsOkAfterGapControl(0), stepsRefusedAfterGapControl(0), statusGapControl(0), singleStepsT1(0), singleStepsT2(0), singleStepsT3(0), dtRelGapControl(1), qUncertaintyByExtrapolation(0), indexLSException(-1), Penetration(0), PenetrationCounter(0), PenetrationLog(0), PenetrationMin(0), PenetrationMax(0), maxdtUsed(0), mindtUsed(0), ChangeByGapControl(false), calcBlock2(0), IterConvergence(0), ConstraintsChanged(0), ConstraintsChangedBlock1(0), ConstraintsChangedBlock2(0), integrationStepsOrder1(0), integrationStepsOrder2(0), order(1), StepTrials(0), AnzahlAktiverKontakte(0), gNDurchschnittprostep(0), iter_T1(0), iter_T2(0), iter_T3(0), step(0), expInt(true), it_T1(0), it_T2(0), it_T3(0), gAC_T1(true), gAC_T2(true), gAC_T3(true), gAC_reg_T1(true), gAC_reg_T2(true), gAC_reg_T3(true), upgedated_T1(false), upgedated_T2(false), upgedated_T3(false), JacUpdate_B1_T1(false), JacUpdate_B2_T1(false), JacUpdate_B1_T2(false), JacUpdate_B2_T2(false), JacUpdate_T1(false), JacUpdate_T2(false), itMax(1), itTol(1e-8), theta(0.), parJac(false), parInt(true), psystems(NULL), inexactJac(false), maxImpIter(0), SetValuedForceLawsExplicit(false), debugOutput(false), plotParallel(false), JacCounter(0) {

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
    expInt = true;

    // end options =================

    order = maxOrder;
  }

  AutoTimeSteppingSSCIntegrator::~AutoTimeSteppingSSCIntegrator() {
    SetValuedLinkListT1.clear();
    SetValuedLinkListT2.clear();
    SetValuedLinkListT3.clear();
    SetValuedLinkListTP.clear();
  }

  void AutoTimeSteppingSSCIntegrator::setFlagErrorTest(int Flag, bool alwaysValid) {
    FlagErrorTest = Flag;
    FlagErrorTestAlwaysValid = alwaysValid;
    assert(FlagErrorTest>=0);		// =0	control errors locally on all variables
    assert(FlagErrorTest<4);		// =2   u is scaled with stepsize
    assert(FlagErrorTest!=1);   // =3   exclude u from error test
  }

  void AutoTimeSteppingSSCIntegrator::setMaxOrder(int order_, int method_) {
    maxOrder = order_;
    method   = method_;
    assert(method>=0);
    assert(method<3);
    assert(maxOrder>0);
    assert(maxOrder<5);
    order = maxOrder;
  }

  void AutoTimeSteppingSSCIntegrator::getdhdqdhdu(Mat& dhdq_, Mat& dhdu_, const fmatvec::Vec z_, const double t_, const int nsys_) {

    vector<fmatvec::Vec> zsys;
    for(unsigned i=0; i<(*psystems).size(); i++) {
      zsys.push_back(z_.copy());
    }

    int nq = qSize;
    int nu = uSize;

    Mat dhdqtmp[200];
    Mat dhdutmp[200];

    int nthr = 0; 
    // Anzahl Threads fuer dhdq und dhdu
    if (parJac && !parInt) {
      nthr = (*psystems).size()/2; // nthr Anzahl Threads pro dhdq / dhdu
    }
    else if (parJac && parInt) {
      nthr = (*psystems).size()/6; // wegen 3 Systemen
    }
    else {
      throw MBSimError("Number of Systems for parallel calculation of Jaocobis wrong!");
    }

    int nthr_q=0;
    int nthr_u=0;

    (nq<=nthr) ? nthr_q=nq : nthr_q=nthr;
    (nu<=nthr) ? nthr_u=nu : nthr_u=nthr;

    Vec Verteilung_q(nthr_q,INIT,0.0), Verteilung_u(nthr_u,INIT,0.0);
    Vec Grenzen_q(nthr_q,INIT,0.0), Grenzen_u(nthr_u,INIT,0.0);
    int iq=0, iu=0;

    do {
      for(int i=0; i<nthr_q; i++) {
        if(iq<nq) {
          Verteilung_q(i)++;
          iq++;
        }
      }
    } while(iq<nq);

    do {
      for(int i=0; i<nthr_u; i++) {
        if(iu<nu) {
          Verteilung_u(i)++;
          iu++;
        }
      }
    } while(iu<nu);

    for(int i=0; i<nthr_q; i++) {
      for(int j=0; j<nthr_q; j++) {
        if(j<=i)
          Grenzen_q(i)+=Verteilung_q(j);
      }
    }

    for(int i=0; i<nthr_u; i++) {
      for(int j=0; j<nthr_u; j++) {
        if(j<=i)
          Grenzen_u(i)+=Verteilung_u(j);
      }
    }

    int nsysMax=0;
    (parInt==true) ? nsysMax=3 : nsysMax=1;

#pragma omp parallel num_threads(nthr_q+nthr_u)
    {
#pragma omp single 
      {
        for(int i=0; i<nthr_q; i++) {
#pragma omp task
          {
            if(i==0) {
              int index = (nsys_-1)*(nthr)+i;
              if((*psystems)[index]->getq()() != zsys[index]()) (*psystems)[index]->updatezRef(zsys[index]);
              dhdqtmp[i]=(*psystems)[index]->dhdq(t,0,static_cast<int>(Grenzen_q(i)));
            }
            else {
              int index = (nsys_-1)*(nthr)+i;
              if((*psystems)[index]->getq()() != zsys[index]()) (*psystems)[index]->updatezRef(zsys[index]);
              dhdqtmp[i]=(*psystems)[index]->dhdq(t,static_cast<int>(Grenzen_q(i-1)),static_cast<int>(Grenzen_q(i)));
            }
          }
        }

        for(int i=0; i<nthr_u; i++) {
#pragma omp task
          {
            if(i==0) {
              int index = (nsysMax*nthr)+(nsys_-1)*(nthr)+i;
              if((*psystems)[index]->getq()() != zsys[index]()) (*psystems)[index]->updatezRef(zsys[index]);
              dhdutmp[i]=(*psystems)[index]->dhdu(t,0,static_cast<int>(Grenzen_u(i)));
            }
            else {
              int index = (nsysMax*nthr)+(nsys_-1)*(nthr)+i;
              if((*psystems)[index]->getq()() != zsys[index]()) (*psystems)[index]->updatezRef(zsys[index]);
              dhdutmp[i]=(*psystems)[index]->dhdu(t,static_cast<int>(Grenzen_u(i-1)),static_cast<int>(Grenzen_u(i)));
            }
          }
        }
      }
#pragma omp barrier     
    }

    // Matrizen aufsumieren;
    for(int i=1; i<nthr_q; i++) { 
      dhdqtmp[0]+=dhdqtmp[i];
    }

    // Matrizen aufsumieren;
    for(int i=1; i<nthr_u; i++) { 
      dhdutmp[0]+=dhdutmp[i];
    }

    dhdq_=dhdqtmp[0].copy();
    dhdu_=dhdutmp[0].copy();
  }

  void AutoTimeSteppingSSCIntegrator::update(DynamicSystemSolver& system, const Vec& z, double t, int nrSys_) {
    if(system.getq()()!=z()) system.updatezRef(z);

    fmatvec::VecInt *LS_Reg_tmp_before=NULL;
    fmatvec::VecInt LS_Reg_tmp_after;

    if (nrSys_==1) LS_Reg_tmp_before=&LS_Reg_T1;
    if (nrSys_==2) LS_Reg_tmp_before=&LS_Reg_T2;
    if (nrSys_==3) LS_Reg_tmp_before=&LS_Reg_T3;   

    system.updateStateDependentVariables(t);
    system.updateg(t);
    system.checkActive(1);
    system.checkActiveReg(1);
    //system.setUpActiveLinks(); // in DynamicSystemSolver::update auskommentiert

    if(system.gActiveChanged()) {
      if (nrSys_==1) gAC_T1=true;
      if (nrSys_==2) gAC_T2=true;
      if (nrSys_==3) gAC_T3=true;

      system.calcgdSize(3);
      system.calclaSize(3);
      system.calcrFactorSize(3);

      system.updateWRef(system.getWParent()(Index(0,system.getuSize()-1),Index(0,system.getlaSize()-1)));
      system.updateVRef(system.getVParent()(Index(0,system.getuSize()-1),Index(0,system.getlaSize()-1)));
      system.updatelaRef(system.getlaParent()(0,system.getlaSize()-1));
      system.updategdRef(system.getgdParent()(0,system.getgdSize()-1));
      if(system.getImpactSolver() == RootFinding) system.updateresRef(system.getresParent()(0,system.getlaSize()-1));
      system.updaterFactorRef(system.getrFactorParent()(0,system.getrFactorSize()-1));
    }
    system.updategd(t);
    system.updateT(t); 
    system.updateJacobians(t);
    system.updateh(t);
    system.updateM(t);
    if (SetValuedForceLawsExplicit) system.facLLM();
    system.updateW(t); 
    system.updateV(t);

    system.getLinkStatusReg(LS_Reg_tmp_after,t);

    if (*LS_Reg_tmp_before!=LS_Reg_tmp_after) { 
      if (nrSys_==1) gAC_reg_T1=true;
      if (nrSys_==2) gAC_reg_T2=true;
      if (nrSys_==3) gAC_reg_T3=true;
    }

    if (nrSys_==1) LS_Reg_T1=LS_Reg_tmp_after;
    if (nrSys_==2) LS_Reg_T2=LS_Reg_tmp_after;
    if (nrSys_==3) LS_Reg_T3=LS_Reg_tmp_after;

  }  

  void AutoTimeSteppingSSCIntegrator::doStep(DynamicSystemSolver& system_, Vec& z_, int nrSys_, double t_, double dt_, bool exp_) {
    if (exp_==true) doExpStep(system_, z_, nrSys_, t_, dt_);
    else if (exp_==false && itMax!=1) doImpStep(system_, z_, nrSys_, t_, dt_);
    else if (exp_==false && itMax==1) doLinImpStep(system_, z_, nrSys_, t_, dt_);
  }

  void AutoTimeSteppingSSCIntegrator::doExpStep(DynamicSystemSolver& system_, Vec& z_, int nrSys_, double t_, double dt_) {

    int *piter=NULL;
    Vec z_l, q_l, u_l, x_l;

    if(nrSys_==1) {
      piter = &iter_T1;
    }
    else if(nrSys_==2) {
      piter = &iter_T2;
    }
    else if(nrSys_==3) {
      piter = &iter_T3;
    }   

    *piter = 0;

    z_l >> z_;
    q_l >> z_(Iq);
    u_l >> z_(Iu);
    x_l >> z_(Ix);

    q_l += system_.deltaq(z_l,t_,dt_);
    system_.update(z_,t_+dt_,1);
    system_.getb().resize() = system_.getgd() + system_.getW().T()*slvLLFac(system_.getLLM(),system_.geth())*dt_;
    *piter  = system_.solveImpacts(dt_);
    u_l += system_.deltau(z_,t_+dt_,dt_);
    x_l += system_.deltax(z_,t_+dt_,dt_);
  }

  void AutoTimeSteppingSSCIntegrator::doImpStep(DynamicSystemSolver& system_, Vec& z_, int nrSys_, double t_, double dt_) {

    int *pit=NULL, *piter=NULL;
    bool *pgAC=NULL;
    bool *pgAC_reg=NULL, *pupgedated=NULL;
    Vec z_l, q_l, u_l, x_l;
    Mat dhdq_n, dhdu_n;
    double maxdevi_u;
    int JacAttempts=0;
    bool *JacUpdate=NULL;

    if(nrSys_==1) {
      pit = &it_T1;
      piter = &iter_T1;
      pgAC = &gAC_T1;
      pgAC_reg = &gAC_reg_T1;
      pupgedated = &upgedated_T1;
      dhdq_n << dhdq_n_T1;
      dhdu_n << dhdu_n_T1;
      JacUpdate = &JacUpdate_T1;
    }
    else if(nrSys_==2) {
      pit = &it_T2;
      piter = &iter_T2;
      pgAC = &gAC_T2;
      pgAC_reg = &gAC_reg_T2;
      pupgedated = &upgedated_T2;
      dhdq_n << dhdq_n_T2;
      dhdu_n << dhdu_n_T2;
      JacUpdate = &JacUpdate_T2;
    }
    else if(nrSys_==3) {
      pit = &it_T3;
      piter = &iter_T3;
      pgAC = &gAC_T3;
      pgAC_reg = &gAC_reg_T3;
      pupgedated = &upgedated_T3;
      dhdq_n << dhdq_n_T3;
      dhdu_n << dhdu_n_T3;
    }

    *pit = 0;
    *piter = 0;
    *pgAC = false;
    *pgAC_reg = false;

    z_l >> z_;
    q_l >> z_(Iq);
    u_l >> z_(Iu);
    x_l >> z_(Ix);

    update(system_,z_l,t_,nrSys_);  // hier wird z_l dem System system_ zugewiesen
    *pgAC = false;                  // ersten Schritt auf jeden Fall ausfuehren

    if (*pgAC_reg==true) {
      *pupgedated=false;
    }

    Vec h_l = system_.geth().copy();
    Vec la_l = system_.getla();

    Vec heff_n;
    SqrMat luMeff_n;
    Vec z_n, q_n, u_n, x_n, h_n;
    z_n.resize() = z_l.copy();
    q_n >> z_n(Iq);
    u_n >> z_n(Iu);
    x_n >> z_n(Ix);     

    bool saveJac=false;

    do {

      if (*pit>0) {
        update(system_,z_n,t_,nrSys_);
        if (*pgAC_reg==true) {
          *pupgedated=false;
        }
      }

      if (inexactJac==false) {
        if (parJac && !parInt) {
          JacCounter++;
          getdhdqdhdu(dhdq_n,dhdu_n,z_l,t_,1);
          saveJac=true;
        }
        else if (parJac && parInt) {
          JacCounter++;
          getdhdqdhdu(dhdq_n,dhdu_n,z_l,t_,nrSys_);
          saveJac=true;
        }
        else {
          JacCounter++;
          if (debugOutput) cout << "Update Jakobis seq." << endl;
          dhdq_n = system_.dhdq(t_);
          dhdu_n = system_.dhdu(t_);
          saveJac=true;
        } 
      }
      else if (inexactJac==true && *pupgedated==false) {
        *JacUpdate=true;

        if (parJac && !parInt) {
          JacCounter++;
          cout << "update Jacobis!" << endl;
          getdhdqdhdu(dhdq_n,dhdu_n,z_n,t_,1);
          *pupgedated=true;
          saveJac=true;
        }
        else if (parJac && parInt) {
          JacCounter++;
          getdhdqdhdu(dhdq_n,dhdu_n,z_n,t_,nrSys_);
          *pupgedated=true;
          saveJac=true;
        }
        else {
          JacCounter++;
          if (debugOutput) cout << endl << "update Jacobis! System= " << nrSys_ << endl;
          cout << endl << "update Jacobis! System= " << nrSys_ << endl;
          dhdq_n = system_.dhdq(t_);
          dhdu_n = system_.dhdu(t_);
          *pupgedated=true;
          saveJac=true;
        }    
      }  


      if (saveJac) {
        if(nrSys_==1) {
          dhdq_n_T1 << dhdq_n;
          dhdu_n_T1 << dhdu_n;
        }
        else if(nrSys_==2) {
          dhdq_n_T2 << dhdq_n;
          dhdu_n_T2 << dhdu_n;
        }
        else if(nrSys_==3) {
          dhdq_n_T3 << dhdq_n;
          dhdu_n_T3 << dhdu_n;
        }
        saveJac=false;
      }

      if (*pgAC==true) {  // Integration nach erster Iteration abbrechen falls Stoß aufgetreten
        *pgAC=false;
        break;
      }

      h_n = system_.geth().copy();
      Mat W_n = system_.getW().copy();
      Mat V_n = system_.getV().copy();
      Mat T_n = system_.getT().copy();
      SymMat M_n = system_.getM().copy();

      VecInt ipiv(M_n.size());
      luMeff_n = SqrMat(facLU(M_n - theta*dt_*dhdu_n - theta*theta*dt_*dt_*dhdq_n*T_n,ipiv));

      heff_n = -M_n/dt_*u_n + M_n/dt_*u_l + theta*theta*dhdq_n*dt_*T_n*u_n - theta*theta*dhdq_n*dt_*T_n*u_l - theta*dhdq_n*q_n + theta*dhdq_n*q_l - theta*h_l + theta*h_n + h_l + theta*dhdq_n*T_n*dt_*u_l;

      if (!SetValuedForceLawsExplicit) {
        system_.getG().resize() = SqrMat(W_n.T()*slvLUFac(luMeff_n,V_n,ipiv));
        system_.getGs().resize() << system_.getG();
        system_.getb().resize() = system_.getgd() + W_n.T()*slvLUFac(luMeff_n,heff_n,ipiv)*dt_;
      }

      if (SetValuedForceLawsExplicit) {
        SymMat M_LLM = system_.getLLM();
        system_.getG().resize() = SqrMat(W_n.T()*slvLLFac(M_LLM,V_n));
        system_.getGs().resize() << system_.getG();
        system_.getb().resize() = system_.getgd() + W_n.T()*slvLLFac(M_LLM,h_n)*dt_;      
      }

      *piter = system_.solveImpacts(dt_);

      Vec du_n = slvLUFac(luMeff_n,heff_n*dt_ + W_n*system_.getla(),ipiv);
      u_n += du_n;

      Vec dq_n = (1.-theta)*T_n*u_l*dt_ + theta*T_n*u_n*dt_;
      q_n = q_l + dq_n;

      x_n += system_.deltax(z_n,t_,dt_);       

      Vec dev_u = M_n*u_n - M_n*u_l - (1.-theta)*h_l*dt_ - theta*h_n*dt_ - W_n*system_.getla();
      maxdevi_u = 1e-18;

      for(int i=0; i<dev_u.size(); i++) {
        if(abs(dev_u(i))>maxdevi_u) {
          maxdevi_u = abs(dev_u(i)); 
        }       
      }
      
      (*pit)++;

      // Jacobis updaten und aktuelle Iteration von vorne beginnen, wenn keine Konvergenz erreicht wird
      if (maxdevi_u>itTol && *pit==itMax-1 && JacAttempts<2) {
        *pupgedated=false;
        *pit=0;
        z_n << z_l;
        JacAttempts++;
      }
      else if (*pit==itMax && JacAttempts>=1) {
        //cout << "Jacobi-Update didn't help to converge... continuing anyway!" << endl;
      }

    } while(*pit<itMax && maxdevi_u>itTol);

    z_l = z_n.copy();
  }

  void AutoTimeSteppingSSCIntegrator::doLinImpStep(DynamicSystemSolver& system_, Vec& z_, int nrSys_, double t_, double dt_) {

    Vec z_l, q_l, u_l, x_l;
    Mat dhdq_n, dhdu_n;
    bool *pupgedated=NULL;
    int *piter=NULL;

    if(nrSys_==1) {
      pupgedated = &upgedated_T1;
      dhdq_n << dhdq_n_T1;
      dhdu_n << dhdu_n_T1;
      piter = &iter_T1;
    }
    else if(nrSys_==2) {
      pupgedated = &upgedated_T2;
      dhdq_n << dhdq_n_T2;
      dhdu_n << dhdu_n_T2;
      piter = &iter_T2;
    }
    else if(nrSys_==3) {
      pupgedated = &upgedated_T3;
      dhdq_n << dhdq_n_T3;
      dhdu_n << dhdu_n_T3;
      piter = &iter_T3;
    }

    z_l >> z_;
    q_l >> z_(Iq);
    u_l >> z_(Iu);
    x_l >> z_(Ix);

    double t = t_;
    double dt = dt_;
    bool saveJac=false;

    t += theta*dt;

    update(system_,z_l,t,nrSys_);  // hier wird z_l dem System system_ zugewiesen
    update(system_,z_l,t,nrSys_);  // hier wird z_l dem System system_ zugewiesen
    update(system_,z_l,t,nrSys_);  // hier wird z_l dem System system_ zugewiesen
    update(system_,z_l,t,nrSys_);  // hier wird z_l dem System system_ zugewiesen

    Mat T = system_.getT().copy();
    SymMat M = system_.getM().copy();
    Vec h = system_.geth().copy();
    Mat W = system_.getW().copy();
    Mat V = system_.getV().copy();
    
    if (inexactJac==false) {
      if (parJac && !parInt) {
        JacCounter++;
        dhdzTimer.start();
        getdhdqdhdu(dhdq_n,dhdu_n,z_l,t_,1);
        saveJac=true;
        dhdztime+=dhdzTimer.stop();      
      }
      else if (parJac && parInt) {
        JacCounter++;
        getdhdqdhdu(dhdq_n,dhdu_n,z_l,t_,nrSys_);
        saveJac=true;
      }
      else {
        JacCounter++;
        if (debugOutput) cout << "Update Jakobis seq." << endl;
        dhdzTimer.start();
        dhdq_n = system_.dhdq(t_);
        dhdu_n = system_.dhdu(t_);
        dhdztime+=dhdzTimer.stop();
        saveJac=true;
      } 
    }
    else if (inexactJac==true && *pupgedated==false) {
      if (parJac && !parInt) {
        JacCounter++;
        getdhdqdhdu(dhdq_n,dhdu_n,z_l,t_,1);
        saveJac=true;
        *pupgedated=true;
      }
      else if (parJac && parInt) {
        JacCounter++;
        getdhdqdhdu(dhdq_n,dhdu_n,z_l,t_,nrSys_);
        saveJac=true;
        *pupgedated=true;
      }
      else {
        JacCounter++;
        dhdq_n = system_.dhdq(t_);
        dhdu_n = system_.dhdu(t_);
        saveJac=true;
        *pupgedated=true;
        if (debugOutput) cout << "Update Jakobis seq." << endl;
      }
    }

    if (saveJac) {
      if(nrSys_==1) {
        dhdq_n_T1 << dhdq_n;
        dhdu_n_T1 << dhdu_n;
      }
      else if(nrSys_==2) {
        dhdq_n_T2 << dhdq_n;
        dhdu_n_T2 << dhdu_n;
      }
      else if(nrSys_==3) {
        dhdq_n_T3 << dhdq_n;
        dhdu_n_T3 << dhdu_n;
      }
      saveJac=false;
    }

    VecInt ipiv(M.size());
    SqrMat luMeff = SqrMat(facLU(M - theta*dt*dhdu_n - theta*theta*dt*dt*dhdq_n*T,ipiv));
    Vec heff = h+theta*dhdq_n*T*u_l*dt;
    system_.getG().resize() = SqrMat(W.T()*slvLUFac(luMeff,V,ipiv));
    system_.getGs().resize() << system_.getG();
    system_.getb().resize() = system_.getgd() + W.T()*slvLUFac(luMeff,heff,ipiv)*dt;

    *piter = system_.solveImpacts(dt);

    Vec du = slvLUFac(luMeff,heff*dt + V*system_.getla(),ipiv);

    q_l += T*(u_l+theta*du)*dt;
    u_l += du;
    x_l += system_.deltax(z_l,t,dt);
  }

  void AutoTimeSteppingSSCIntegrator::integrate(DynamicSystemSolver& system_) {
    if (theta>epsroot()) expInt=false;
    else expInt=true;
    parInt=false;
    parJac=false;
    integrate(system_, system_, system_, system_, 1); 
  }

  void AutoTimeSteppingSSCIntegrator::integrate(DynamicSystemSolver& system_, vector<DynamicSystemSolver*> systems) {
    if (theta>epsroot()) expInt=false;
    else expInt=true;
    parJac=true;
    parInt=false;
    psystems = &systems;
    integrate(system_, system_, system_, system_, 1); 
  }

  void AutoTimeSteppingSSCIntegrator::integrate(DynamicSystemSolver& systemT1_, DynamicSystemSolver& systemT2_, DynamicSystemSolver& systemT3_, DynamicSystemSolver& systemTP_, int Threads) { 
    if (theta>epsroot()) expInt=false;
    else expInt=true;
    numThreads = Threads;
    preIntegrate(systemT1_, systemT2_, systemT3_, systemTP_);
    subIntegrate(systemT1_, tEnd);
    postIntegrate(systemT1_);
  }

  void AutoTimeSteppingSSCIntegrator::integrate(DynamicSystemSolver& systemT1_, DynamicSystemSolver& systemT2_, DynamicSystemSolver& systemT3_, DynamicSystemSolver& systemTP_, vector<DynamicSystemSolver*> systems, int Threads) { 
    if (theta>epsroot()) expInt=false;
    else expInt=true;
    parJac=true;
    parInt=true;
    psystems = &systems;
    numThreads = Threads;
    preIntegrate(systemT1_, systemT2_, systemT3_, systemTP_);
    subIntegrate(systemT1_, tEnd);
    postIntegrate(systemT1_);
  }

  void AutoTimeSteppingSSCIntegrator::preIntegrate(DynamicSystemSolver& system_) {
    preIntegrate(system_, system_, system_, system_);
  }

  void AutoTimeSteppingSSCIntegrator::preIntegrate(DynamicSystemSolver& systemT1_, DynamicSystemSolver& systemT2_, DynamicSystemSolver& systemT3_, DynamicSystemSolver& systemTP_) {

    sysT1 = &systemT1_;
    sysT2 = &systemT2_;
    sysT3 = &systemT3_;
    sysTP = &systemTP_;

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

    Iq = Index(0,qSize-1);
    Iu = Index(qSize, qSize+uSize-1);
    Ix = Index(qSize+uSize, zSize-1);

    zT1.resize(zSize);
    if (sysT1==sysT2) zT2.resize()>>zT1; else zT2.resize(zSize);
    if (sysT1==sysT3) zT3.resize()>>zT1; else zT3.resize(zSize);
    if (sysT1==sysTP) zTP.resize()>>zT1; else zTP.resize(zSize);

    qT1.resize() >> zT1(Iq);
    uT1.resize() >> zT1(Iu);
    xT1.resize() >> zT1(Ix);

    qT2.resize() >> zT2(Iq);
    uT2.resize() >> zT2(Iu);
    xT2.resize() >> zT2(Ix);

    qT3.resize() >> zT3(Iq);
    uT3.resize() >> zT3(Iu);
    xT3.resize() >> zT3(Ix);

    qTP.resize() >> zTP(Iq);
    uTP.resize() >> zTP(Iu);
    xTP.resize() >> zTP(Ix);

    zi.resize(zSize,INIT,0.0); 	// starting value for ith step

    SetValuedLinkListT1.clear();
    SetValuedLinkListT2.clear();
    SetValuedLinkListT3.clear();
    SetValuedLinkListTP.clear();

    sysT1->buildListOfSetValuedLinks(SetValuedLinkListT1,true);
    sysT2->buildListOfSetValuedLinks(SetValuedLinkListT2,true);
    sysT3->buildListOfSetValuedLinks(SetValuedLinkListT3,true);
    sysTP->buildListOfSetValuedLinks(SetValuedLinkListTP,true);

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
      integPlot << "#1 t [s]  :" << endl; 
      integPlot << "#2 dt [s] :" << endl;
      integPlot << "#3 order  :" << endl; 
      integPlot << "#4 iter   :" << endl;      
      integPlot << "#5 laSize :" << endl;
      integPlot << "#6 active contacts: " << endl;
      integPlot << "#7 calculation time [s]:" << endl;
      integPlot << "#8 it_T1  :" << endl;
      integPlot << "#9 it_T2  :" << endl;
      integPlot << "#10 it_T3 :" << endl;
      integPlot << "#11 JacUpdate_B1_T1 :" << endl;
      integPlot << "#12 JacUpdate_B1_T2 :" << endl;
      integPlot << "#13 JacUpdate_B2_T1 :" << endl;
      integPlot << "#14 JacUpdate_B2_T2 :" << endl;
      integPlot << "#15 ConstraintsChanged_B1_A :" << endl;
    }

    if(z0.size()) zi = z0; 					// define initial state
    else sysT1->initz(zi); 

    sysTP->plot2(zi,t,1.);

    tPlot = t;
  }

  void AutoTimeSteppingSSCIntegrator::subIntegrate(DynamicSystemSolver& system, double tStop) { // system: only dummy!
    Timer.start();
    timeB1=0;
    timeB2=0;

    if (outputInterpolation) {
      getAllSetValuedla(la,laSizes,SetValuedLinkListT1);
      la.init(0.0);
    }

    if (theta>epsroot() && (method || FlagGapControl)) { // SSC-Methode und Gap-Control fuer implizite Integration pruefen
      throw MBSimError("Using implicit integration only method=0 and FlagGapControl=0 allowed!");
    }

    if (theta>epsroot() && (inexactJac && itMax==1 && !FlagSSC)) { // linear implizite Integration mit inexakter Jakobi nur mit SSC (Kopplung des Updatens an SSC)
      throw MBSimError("Linear implicit Integration with inexact Jabobians only allowed with SSC!");
    }

    if (theta>epsroot() && inexactJac && itMax>1 && !((FlagSSC && maxOrder==1) || (!FlagSSC && maxOrder==1) || (!FlagSSC && maxOrder==2))) { // schließe noch nicht getestete Versionen aus
      throw MBSimError("Implicit Integration with inexact Jabobians only tested with (FlagSSC && maxOrder==1) || (!FlagSSC && maxOrder==1) || (!FlagSSC && maxOrder==2)!");
    }    

    if (!FlagSSC) dt=dtMin;

#ifdef _OPENMP
    omp_set_nested(1);
#endif

    qUncertaintyByExtrapolation=0;

    JacConstSteps = 0;

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

    bool plotten=false;

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

    if (plotParallel) {
      if (!(!FlagSSC && maxOrder==2 && method==0) && !(FlagSSC && maxOrder==1 && method==0)) numThreads=numThreads+1;
    }

    if (sysT1==sysT2) {
      numThreads=1;
    }
    
    //if (theta > epsroot() && inexactJac==true) {
      sysT1->update(zi,t);
      sysT2->update(zi,t);
      sysT3->update(zi,t);
      sysTP->update(zi,t);
    //}
    
    sysT1->getLinkStatus(LS,t);
    sysT1->getLinkStatusReg(LS_Reg,t);

    dhdq_T1 << dhdq_n_T1; dhdu_T1 << dhdu_n_T1;
    dhdq_T2 << dhdq_n_T2; dhdu_T2 << dhdu_n_T2;
    dhdq_T3 << dhdq_n_T3; dhdu_T3 << dhdu_n_T3;
    
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
        JacUpdate_T1=false;
        JacUpdate_T2=false;
        JacUpdate_B1_T1=false;
        JacUpdate_B2_T1=false;
        JacUpdate_B1_T2=false;
        JacUpdate_B2_T2=false;

#pragma omp parallel num_threads(numThreads)
        {  
#pragma omp single
          {
            TimerB1.start();
#pragma omp task
            {
              if (plotten && plotParallel && (!FlagSSC && maxOrder==1)) {
                plotPar();
                plotten=false;
              }
            }
#pragma omp task
            {
              // one step integration (A)
              zT1 << zi;

              if (inexactJac) {
                LS_Reg_T1 = LS_Reg;
                dhdq_n_T1 << dhdq_T1; dhdu_n_T1 << dhdu_T1;
                //dhdq_n_T1 << dhdq_z1d; dhdu_n_T1 << dhdu_z1d;
              }

              doStep(*sysT1,zT1,1,t,dt,expInt);

              iterA = iter_T1;

              getAllSetValuedla(la1d,la1dSizes,SetValuedLinkListT1);
              la1d/=dt;

              sysT1->getLinkStatus(LSA,t+dt);
              if (inexactJac) sysT1->getLinkStatusReg(LSA_Reg,t+dt);

              ConstraintsChangedA = changedLinkStatus(LSA,LS,indexLSException);
              singleStepsT1++;

              z1d << zT1;

              if (inexactJac) {
                LS_Reg_z1d << LSA_Reg;
                dhdq_z1d << dhdq_n_T1; dhdu_z1d << dhdu_n_T1;
              }

              // two step integration (first step) (B1) 
              if (calcJobBT1) {
                zT1  << zi;

                if (inexactJac) {
                  LS_Reg_T1 = LS_Reg;
                  dhdq_n_T1 << dhdq_T1; dhdu_n_T1 << dhdu_T1;
                }

                doStep(*sysT1,zT1,1,t,dtHalf,expInt);
                iterB1  = iter_T1;

                getAllSetValuedla(la2b,la2bSizes,SetValuedLinkListT1);
                la2b/=dtHalf;          

                sysT1->getLinkStatus(LSB1,t+dtHalf);
                if (inexactJac) sysT1->getLinkStatusReg(LSB1_Reg,t+dtHalf);

                ConstraintsChangedB = changedLinkStatus(LSB1,LS,indexLSException);
                singleStepsT1++;

                z2b << zT1;

                if (inexactJac) {
                  LS_Reg_z2b << LSB1_Reg;
                  dhdq_z2b << dhdq_n_T1; dhdu_z2b << dhdu_n_T1;
                }
              }

              // three step integration (first step) (E1) 
              if (calcJobE1T1) {
                zT1  << zi;

                if (inexactJac) {
                  LS_Reg_T1 << LS_Reg;
                  dhdq_n_T1 << dhdq_T1; dhdu_n_T1 << dhdu_T1;
                }

                doStep(*sysT1,zT1,1,t,dtThird,expInt);
                singleStepsT1++;

                if (inexactJac) sysT1->getLinkStatusReg(LSB1_2_Reg,t+dtThird);

                z3b << zT1;

                if (inexactJac) {
                  LS_Reg_z3b << LSB1_2_Reg;
                  dhdq_z3b << dhdq_n_T1; dhdu_z3b << dhdu_n_T1;
                }
              }
            }
#pragma omp task
            {
              // two step integration (first step) (B1) 
              if (calcJobBT2) {
                zT2  << zi;

                if (inexactJac) {
                  LS_Reg_T2 = LS_Reg;
                  dhdq_n_T2 << dhdq_T2; dhdu_n_T2 << dhdu_T2;
                }

                doStep(*sysT2,zT2,2,t,dtHalf,expInt);
                iterB1 = iter_T2;

                getAllSetValuedla(la2b,la2bSizes,SetValuedLinkListT2);
                la2b/=dtHalf;

                sysT2->getLinkStatus(LSB1,t+dtHalf);
                if (inexactJac) sysT2->getLinkStatusReg(LSB1_Reg,t+dtHalf);

                ConstraintsChangedB = changedLinkStatus(LSB1,LS,indexLSException);
                singleStepsT2++;

                z2b << zT2;

                if (inexactJac) {
                  LS_Reg_z2b << LSB1_Reg;
                  dhdq_z2b << dhdq_n_T2; dhdu_z2b << dhdu_n_T2;
                }
              }

              // four step integration (first two steps) (C12)
              if (calcJobC) {
                zT2  << zi;

                if (inexactJac) {
                  LS_Reg_T2 = LS_Reg;
                  dhdq_n_T2 << dhdq_T2; dhdu_n_T2 << dhdu_T2;
                }

                doStep(*sysT2,zT2,2,t,dtQuarter,expInt);
                iterC1 = iter_T2;

                sysT2->getLinkStatus(LSC1,t+dtQuarter);
                if (inexactJac) sysT2->getLinkStatusReg(LSC1_Reg,t+dtQuarter);

                ConstraintsChangedC =  changedLinkStatus(LSC1,LS,indexLSException);

                doStep(*sysT2,zT2,2,t+dtQuarter,dtQuarter,expInt);
                iterC2 = iter_T2;

                sysT2->getLinkStatus(LSC2,t+dtHalf);
                if (inexactJac) sysT2->getLinkStatusReg(LSC2_Reg,t+dtHalf);

                ConstraintsChangedC = ConstraintsChangedC || changedLinkStatus(LSC2,LSC1,indexLSException);
                singleStepsT2+=2;

                z4b << zT2;

                if (inexactJac) {
                  LS_Reg_z4b << LSC2_Reg;
                  dhdq_z4b << dhdq_n_T2; dhdu_z4b << dhdu_n_T2;
                }
              }

              // three step integration (first two steps ) (E12)
              if (calcJobE12T2) {
                zT2  << zi;

                if (inexactJac) {
                  LS_Reg_T2 = LS_Reg;
                  dhdq_n_T2 << dhdq_T2; dhdu_n_T2 << dhdu_T2;
                }

                doStep(*sysT2,zT2,2,t,dtThird,expInt);

                doStep(*sysT2,zT2,2,t+dtThird,dtThird,expInt);
                singleStepsT2+=2;

                if (inexactJac) sysT2->getLinkStatusReg(LS_tmp,t+dtThird*2.);

                z3b << zT2;

                if (inexactJac) {
                  LS_Reg_z3b << LS_tmp;
                  dhdq_z3b << dhdq_n_T2; dhdu_z3b << dhdu_n_T2;
                }
              }
            }
#pragma omp task
            {
              // six step integration (first three steps)   
              if(calcJobD) {
                zT3  << zi;

                if (inexactJac) {
                  LS_Reg_T3 = LS_Reg;
                  dhdq_n_T3 << dhdq_T3; dhdu_n_T3 << dhdu_T3;
                }

                doStep(*sysT3,zT3,3,t,dtSixth,expInt);

                sysT3->getLinkStatus(LSD1,t+dtSixth);
                if (inexactJac) sysT3->getLinkStatusReg(LSD1_Reg,t+dtSixth);

                ConstraintsChangedD = changedLinkStatus(LSD1,LS,indexLSException);

                doStep(*sysT3,zT3,3,t+dtSixth,dtSixth,expInt);

                sysT3->getLinkStatus(LSD2,t+dtThird);
                if (inexactJac) sysT3->getLinkStatusReg(LSD2_Reg,t+dtThird);

                ConstraintsChangedD = ConstraintsChangedD || changedLinkStatus(LSD2,LSD1,indexLSException);

                doStep(*sysT3,zT3,3,t+2.*dtSixth,dtSixth,expInt);

                sysT3->getLinkStatus(LSD3,t+dtHalf);
                if (inexactJac) sysT3->getLinkStatusReg(LSD3_Reg,t+dtHalf);

                ConstraintsChangedD = ConstraintsChangedD || changedLinkStatus(LSD3,LSD2,indexLSException);
                singleStepsT3+=3;

                z6b << zT3;

                if (inexactJac) {
                  LS_Reg_z6b << LSD3_Reg;
                  dhdq_z6b << dhdq_n_T3; dhdu_z6b << dhdu_n_T3;
                }
              }
            }
          }
#pragma omp single
          timeB1 += TimerB1.stop();
#pragma omp single
          {
            JacUpdate_B1_T1=JacUpdate_T1;
            JacUpdate_B1_T2=JacUpdate_T2;
            JacUpdate_T1=false;
            JacUpdate_T2=false;

            ConstraintsChanged_B1_A = ConstraintsChangedA;
            
            ConstraintsChangedBlock1 = ConstraintsChangedB || ConstraintsChangedC || ConstraintsChangedD;
            ConstraintsChanged       = ConstraintsChangedA || ConstraintsChangedBlock1;
            calcBlock2 = (Block2IsOptional==false) || (ConstraintsChangedBlock1==false);
            ConstraintsChangedA = false;
            ConstraintsChangedB = false;
            ConstraintsChangedC = false;
            ConstraintsChangedD = false;
            ConstraintsChangedBlock2 = false;        

            if (FlagSSC && maxOrder==3) zStern << 0.5*z2b - 4.0*z4b + 4.5*z6b;
            if (FlagSSC && maxOrder==2) zStern << 2.0*z4b - z2b;
          }

#pragma omp single
          {
            TimerB2.start();
#pragma omp task
            {
              if (plotten && plotParallel && !(!FlagSSC && maxOrder==1)) {
                plotPar();
                plotten=false;
              }
            }
#pragma omp task
            {
              // two step integration (B2)
              if (calcJobBT1 && calcBlock2) {
                zT1  << z2b;

                if (inexactJac) {
                  LS_Reg_T1 << LS_Reg_z2b;
                  dhdq_n_T1 << dhdq_z2b; dhdu_n_T1 << dhdu_z2b;
                }

                doStep(*sysT1,zT1,1,t+dtHalf,dtHalf,expInt);

                sysT1->getLinkStatus(LSB2,t+dt);
                if (inexactJac) sysT1->getLinkStatusReg(LSB2_Reg,t+dt);

                ConstraintsChangedB = changedLinkStatus(LSB2,LSB1,indexLSException);
                singleStepsT1++;

                z2d << zT1;

                if (inexactJac) {
                  dhdq_z2d << dhdq_n_T1; dhdu_z2d << dhdu_n_T1;
                }
              }
              if (calcJobB2RET1 && calcBlock2) { //B2RE
                zT1 << zStern;

                if (inexactJac) {
                  if (maxOrder==2) LS_Reg_T1 << LS_Reg_z4b; dhdq_n_T1 << dhdq_z4b; dhdu_n_T1 << dhdu_z4b;
                }

                doStep(*sysT1,zT1,1,t+dtHalf,dtHalf,expInt);
                iterB2RE = iter_T1;
                singleStepsT1++;

                z2dRE << zT1;
                
                if (inexactJac) {
                  dhdq_z2dRE << dhdq_n_T1; dhdu_z2dRE << dhdu_n_T1;
                }

              }
              // three step integration (E23) last two steps
              if (calcJobE23T1 && calcBlock2) {
                zT1  << z3b;

                if (inexactJac) {
                  LS_Reg_T1 << LS_Reg_z3b;
                  dhdq_n_T1 << dhdq_z3b; dhdu_n_T1 << dhdu_z3b;
                }

                doStep(*sysT1,zT1,1,t+dtThird,dtThird,expInt);

                doStep(*sysT1,zT1,1,t+2.0*dtThird,dtThird,expInt);
                singleStepsT1+=2;

                z3d << zT1;

                if (inexactJac) {
                  dhdq_z3d << dhdq_n_T1; dhdu_z3d << dhdu_n_T1;
                }

              } 
            }
#pragma omp task
            {
              // four step integration (C3 and C4) (last two steps )
              if (calcJobC && calcBlock2) {
                zT2 << zStern;

                if (inexactJac) {
                  if (maxOrder==2) LS_Reg_T2 << LS_Reg_z4b; dhdq_n_T2 << dhdq_z4b; dhdu_n_T2 << dhdu_z4b;
                  if (maxOrder==3) LS_Reg_T2 << LS_Reg_z4b; dhdq_n_T2 << dhdq_z4b; dhdu_n_T2 << dhdu_z4b;
                }

                doStep(*sysT2,zT2,2,t+dtHalf,dtQuarter,expInt);
                iterC3 = iter_T2;

                sysT2->getLinkStatus(LSC3,t+dtHalf+dtQuarter);
                if (inexactJac) sysT2->getLinkStatusReg(LSC3_Reg,t+dtHalf+dtQuarter);

                doStep(*sysT2,zT2,2,t+dtHalf+dtQuarter,dtQuarter,expInt);
                iterC4 = iter_T2;

                sysT2->getLinkStatus(LSC4,t+dt);
                if (inexactJac) sysT2->getLinkStatusReg(LSC4_Reg,t+dt);

                ConstraintsChangedC = changedLinkStatus(LSC2,LSC3,indexLSException);
                ConstraintsChangedC = ConstraintsChangedC || changedLinkStatus(LSC3,LSC4,indexLSException);

                singleStepsT2+=2;
                z4d << zT2;

                if (inexactJac) {
                  dhdq_z4d << dhdq_n_T2; dhdu_z4d << dhdu_n_T2;
                }

              }

              // two step integration (B2)
              if (calcJobBT2 && calcBlock2) {
                zT2  << z2b;

                if (inexactJac) {
                  LS_Reg_T2 << LS_Reg_z2b;
                  dhdq_n_T2 << dhdq_z2b; dhdu_n_T2 << dhdu_z2b;
                }

                doStep(*sysT2,zT2,2,t+dtHalf,dtHalf,expInt);
                iterB2 = iter_T2;

                sysT2->getLinkStatus(LSB2,t+dt);
                if (inexactJac) sysT2->getLinkStatusReg(LSB2_Reg,t+dt);

                ConstraintsChangedB = changedLinkStatus(LSB2,LSB1,indexLSException);

                getAllSetValuedla(la2b,la2bSizes,SetValuedLinkListT2);
                la2b/=dtHalf;
                singleStepsT2++;

                z2d << zT2;

                if (inexactJac) {
                  dhdq_z2d << dhdq_n_T2; dhdu_z2d << dhdu_n_T2;
                }

              }
              if (calcJobB2RET2 && calcBlock2) { //B2RE
                zT2 << zStern;

                if (inexactJac) {
                  if (maxOrder==3) LS_Reg_T2 << LS_Reg_z6b; dhdq_n_T2 << dhdq_z6b; dhdu_n_T2 << dhdu_z6b;
                }

                doStep(*sysT2,zT2,2,t+dtHalf,dtHalf,expInt);
                iterB2RE = iter_T2;
                singleStepsT2++;

                z2dRE << zT2;

                if (inexactJac) {
                  dhdq_z2dRE << dhdq_n_T2; dhdu_z2dRE << dhdu_n_T2;
                }

              }
              // three step integration (E3) last step
              if (calcJobE3T2 && calcBlock2) {
                zT2  << z3b;

                if (inexactJac) {
                  LS_Reg_T2 << LS_Reg_z3b;
                  dhdq_n_T2 << dhdq_z3b; dhdu_n_T2 << dhdu_z3b;
                }

                doStep(*sysT2,zT2,2,t+2.0*dtThird,dtThird,expInt);
                singleStepsT2++;
                z3d << zT2;

                if (inexactJac) {
                  dhdq_z3d << dhdq_n_T2; dhdu_z3d << dhdu_n_T2;
                }

              }
            }
#pragma omp task
            {
              // six step integration (last three steps) 
              if (calcJobD && calcBlock2) {
                zT3 << zStern;

                if (inexactJac) {
                  LS_Reg_T3 << LS_Reg_z6b;
                  dhdq_n_T3 << dhdq_z6b; dhdu_n_T3 << dhdu_z6b;
                }

                doStep(*sysT3,zT3,3,t+dtHalf,dtSixth,expInt);

                sysT3->getLinkStatus(LSD4,t+4.0*dtSixth);
                if (inexactJac) sysT3->getLinkStatusReg(LSD4_Reg,t+4.0*dtSixth);

                ConstraintsChangedD =  changedLinkStatus(LSD4,LSD3,indexLSException);

                doStep(*sysT3,zT3,3,t+4.0*dtSixth,dtSixth,expInt);

                sysT3->getLinkStatus(LSD5,t+5.0*dtSixth);
                if (inexactJac) sysT3->getLinkStatusReg(LSD5_Reg,t+5.0*dtSixth);

                ConstraintsChangedD = ConstraintsChangedD || changedLinkStatus(LSD4,LSD5,indexLSException);

                doStep(*sysT3,zT3,3,t+5.0*dtSixth,dtSixth,expInt);

                sysT3->getLinkStatus(LSD6,t+dt);
                if (inexactJac) sysT3->getLinkStatusReg(LSD6_Reg,t+dt);

                ConstraintsChangedD = ConstraintsChangedD || changedLinkStatus(LSD5,LSD6,indexLSException);

                singleStepsT3+=3;
                z6d << zT3;

                if (inexactJac) {
                  dhdq_z6d << dhdq_n_T3; dhdu_z6d << dhdu_n_T3;
                }

              }
            }
          }
#pragma omp single
          timeB2 += TimerB2.stop();
      }
      
      JacUpdate_B2_T1=JacUpdate_T1;
      JacUpdate_B2_T2=JacUpdate_T2;
      JacUpdate_T1=false;
      JacUpdate_T2=false;
      
      maxImpIter=0;
      if (it_T1>=maxImpIter) maxImpIter=it_T1;      
      if (it_T2>=maxImpIter) maxImpIter=it_T2;      
      if (it_T3>=maxImpIter) maxImpIter=it_T3;      

      ConstraintsChangedBlock2 = ConstraintsChangedB || ConstraintsChangedC || ConstraintsChangedD;
      ConstraintsChanged = ConstraintsChanged || ConstraintsChangedBlock2;

      dtOld = dt;
      if (testTolerances()) {
        StepFinished = 1;
        sumIter += iter;
        if (iter > maxIterUsed) maxIterUsed = iter;
        if (maxdtUsed < dtOld) maxdtUsed = dtOld;
        if (mindtUsed > dtOld) mindtUsed = dtOld;
      }
      else {
        refusedSteps++;
        if (ConstraintsChanged) refusedStepsWithImpact++;
        StepTrials++;
        if (dtOld<=dtMin+macheps()) {
          StepFinished = -1;
          //cerr << " TimeStepperSSC reached minimum stepsize dt= "<<dt<<" at t= "<<t<<endl;
          //exit(StepFinished);
        }
      }
      if (abs(dt-dtOld)>epsroot() && itMax==1 && expInt==false) {
        upgedated_T1=false;
        upgedated_T2=false;
        upgedated_T2=false;
      }
      }
      plotten=true;
      if (FlagPlotIntegrator) doIntegPlot();

      StepTrials=0;
      StepFinished = 0;
      integrationSteps++; 
      if(order==1)integrationStepsOrder1++;
      if(order>=2)integrationStepsOrder2++;
      t += dte;

      // Save values for plotting
      ziP.resize(); ziP = zi;
      zeP.resize(); zeP = ze;
      laSizesP.resize(); laSizesP = laSizes;
      laeSizesP.resize(); laeSizesP = laeSizes;
      laP.resize(); laP = la;
      laeP.resize(); laeP = lae;

      if (!plotParallel) plot();

      zi << ze;
      LS << LSe;
      LS_Reg << LSe_Reg;

      if (inexactJac) {
        //dhdq_T1 = dhdq_end; dhdu_T1 = dhdu_end;
        //dhdq_T2 = dhdq_end; dhdu_T2 = dhdu_end;
        //dhdq_T3 = dhdq_end; dhdu_T3 = dhdu_end;
      }

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

      if (ExitIntegration && plotParallel) {
        plotPar();
      }
    }

    cout << endl << endl << "Time for dhdz= " << dhdztime << endl;
    cout << "Time for B1= " << timeB1 << endl;
    cout << "Time for B2= " << timeB2 << endl;
    cout << "Time for Plot= " << timePlot << endl;
    cout << "Time for PlotPar= " << timePlotPar << endl;
  }

  void AutoTimeSteppingSSCIntegrator::plot() {
    TimerPlot.start();
    if ((FlagPlotEveryStep) || ((t>=tPlot)&&(outputInterpolation==false))) {
      tPlot+=dtPlot;
      zT1 << ze;
      setAllSetValuedla(lae,laeSizes,SetValuedLinkListT1);
      sysTP->plot2(zT1,t,1);
    }
    if ((t>=tPlot) && outputInterpolation && !FlagPlotEveryStep) {

      // Dimension/size von la und lae synchronisieren
      Vec laSynchron;
      Vec laeSynchron;
      VecInt laSizesSynchron;
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
        sysTP->plot2(zT1,tPlot,1);
        tPlot += dtPlot;
      }
    }
    if((output && expInt==true) || (output && expInt==false && itMax==1)) cout << "   t = " <<  t << ",\tdt = "<< dtOld << ",\titer = "<<setw(5)<<setiosflags(ios::left) <<iter<<",\torder = "<<order << "\r"<<flush;
    if(output && expInt==false && itMax>1) cout << "   t = " <<  t << ",\tdt = "<< dtOld << ",\titer = "<<setw(5)<<setiosflags(ios::left) <<iter<<",\torder = "<<order << ",\timpiter = "<<setw(5)<<setiosflags(ios::left) << maxImpIter << "\r"<<flush;
    timePlotPar += TimerPlotPar.stop();
    //if (FlagPlotIntegrator) {
    //  time += Timer.stop();
    //  integPlot<< t << " " << dtOld << " " <<order << " " << iter << " " << sysTP->getlaSize()  << " "<<AnzahlAktiverKontakte<<" "<<time  <<endl;
    //}
    //if(output) cout << "   t = " <<  t << ",\tdt = "<< dtOld << ",\titer = "<<setw(5)<<setiosflags(ios::left) <<iter<<",\torder = "<<order << "\r"<<flush;
    //timePlot += TimerPlot.stop();
  }

  void AutoTimeSteppingSSCIntegrator::plotPar() {
    TimerPlotPar.start();
    if ((FlagPlotEveryStep) || ((t>=tPlotP)&&(outputInterpolation==false))) {
      tPlotP+=dtPlot;
      zTP << zeP;
      setAllSetValuedla(laeP,laeSizesP,SetValuedLinkListTP);
      sysTP->plot2(zTP,t,1);
    }
    if ((t>=tPlotP) && outputInterpolation && !FlagPlotEveryStep) {
      // Dimension/size von la und lae synchronisieren
      Vec laSynchron;
      Vec laeSynchron;
      VecInt laSizesSynchron;
      if(t>tPlotP) {
        int nLinks = laSizesP.size();
        if (nLinks>laeSizesP.size()) nLinks=laeSizesP.size();
        int nlaAll=0;
        for(int i=0; i<nLinks; i++) {
          if (laSizesP(i)>laeSizesP(i)) nlaAll+=laeSizesP(i);
          else nlaAll+=laSizesP(i);
        }
        laSynchron.resize(nlaAll,NONINIT);
        laeSynchron.resize(nlaAll,NONINIT);
        laSizesSynchron.resize(nlaAll,NONINIT);
        int ila=0;
        int ilae=0;
        int iSynchron=0;
        for(int i=0; i<nLinks;i++) {
          laSizesSynchron(i) = laSizesP(i);
          if (laSizesSynchron(i)>laeSizesP(i)) laSizesSynchron(i) = laeSizesP(i);
          laSynchron(iSynchron,iSynchron+laSizesSynchron(i)-1) = laP(ila,ila+laSizesSynchron(i)-1);
          laeSynchron(iSynchron,iSynchron+laSizesSynchron(i)-1) = laeP(ilae,ilae+laSizesSynchron(i)-1);
          ilae+=laeSizesP(i);
          ila+=laSizesP(i);
          iSynchron+=laSizesSynchron(i);
        }
      }
      while (t>tPlotP) {
        double ratio = (tPlotP -(t-dte))/dte;
        zTP << ziP + (zeP-ziP)*ratio;
        setAllSetValuedla(laSynchron+(laeSynchron-laSynchron)*ratio,laSizesSynchron,SetValuedLinkListTP);
        sysTP->plot2(zTP,tPlotP,1);
        tPlotP += dtPlot;
      }
    }

    if((output && expInt==true) || (output && expInt==false && itMax==1)) cout << "   t = " <<  t << ",\tdt = "<< dtOld << ",\titer = "<<setw(5)<<setiosflags(ios::left) <<iter<<",\torder = "<<order << "\r"<<flush;
    if(output && expInt==false && itMax>1) cout << "   t = " <<  t << ",\tdt = "<< dtOld << ",\titer = "<<setw(5)<<setiosflags(ios::left) <<iter<<",\torder = "<<order << ",\timpiter = "<<setw(5)<<setiosflags(ios::left) << maxImpIter << "\r"<<flush;
    timePlotPar += TimerPlotPar.stop();
  }

  void AutoTimeSteppingSSCIntegrator::doIntegPlot() {
      time += Timer.stop();
      integPlot << t << " " << dtOld << " " << order << " " << iter << " " << sysTP->getlaSize()  << " " << AnzahlAktiverKontakte << " " << time << " " << it_T1 << " " << it_T2 << " " << it_T3 << " " << JacUpdate_B1_T1 << " " << JacUpdate_B1_T2 << " " << JacUpdate_B2_T1 << " " << JacUpdate_B2_T2 << " " << ConstraintsChanged_B1_A << endl;
  }

  void AutoTimeSteppingSSCIntegrator::postIntegrate(DynamicSystemSolver& system) {           // system: only dummy!
    time += Timer.stop();
    cout.unsetf(ios::scientific);
    if (FlagPlotIntegrator) {
      integPlot.close();
    }
    if (FlagPlotIntegrationSum) {
      int maxStepsPerThread = singleStepsT1;
      if (maxStepsPerThread<singleStepsT2) maxStepsPerThread = singleStepsT2;
      if (maxStepsPerThread<singleStepsT3) maxStepsPerThread = singleStepsT3;
      ofstream integSum((name + ".sum").c_str());
      integSum << "Integration time:   " << time << endl;
      integSum << "Integration steps:  " << integrationSteps << endl;
      integSum << "Evaluations MBS:    " << (singleStepsT1+singleStepsT2+singleStepsT3)<<"   (max/Thread: "<<maxStepsPerThread<<")"<<endl;
      integSum << "Steps with events: " << integrationStepswithChange<< endl;
      integSum << "Jacobian Updates: " << JacCounter << endl;
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
      cout << "Summary Integration with AutoTimeStepperSSC: "<<endl;
      cout << "Integration time:   " << time << endl;
      cout << "Integration steps:  " << integrationSteps << endl;
      cout << "Evaluations MBS:    " << (singleStepsT1+singleStepsT2+singleStepsT3)<<"   (max/Thread: "<<maxStepsPerThread<<")"<<endl;
      cout << "Steps with events: " << integrationStepswithChange<< endl;
      cout << "Jacobian Updates: " << JacCounter << endl;
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


  bool AutoTimeSteppingSSCIntegrator::testTolerances() {

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
        if (maxOrder==1) {
          LSe << LSA;
          ze << z1d;
          if (inexactJac) {
            LSe_Reg << LSA_Reg;
            dhdq_end << dhdq_n_T1; dhdu_end << dhdu_n_T1;
          }
        }
        if (maxOrder==2) {
          LSe << LSB2;
          if (inexactJac) {
            LSe_Reg << LSB2_Reg;
            dhdq_end << dhdq_n_T2; dhdu_end << dhdu_n_T2;
            dhdq_T1 << dhdq_z1d; dhdu_T1 << dhdu_z1d;
            dhdq_T2 << dhdq_z2d; dhdu_T2 << dhdu_z2d;
          }
          ze<<z1d; order=1;
          if (!JacUpdate_B1_T1 && !JacUpdate_B1_T2 && !JacUpdate_B2_T1 && !JacUpdate_B2_T2) {
            if (!ConstraintsChanged) {
              ze << 2.0*z2d - z1d; order=2;} 
            else {
              ze<<z2d; order=1;
            }
          }
          else if (JacUpdate_B1_T1) {
            ze<<z1d; order=1;
          }
          else if (JacUpdate_B1_T2 || JacUpdate_B2_T2) {
            ze<<z2d; order=1;
          }
        }
        if (maxOrder==3) {
          LSe << LSB2;
          if (inexactJac) {
            LSe_Reg << LSB2_Reg;
            dhdq_end << dhdq_n_T1; dhdu_end << dhdu_n_T1;
          }
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
          if (inexactJac) {
            LSe_Reg << LSB2_Reg;
            dhdq_end << dhdq_n_T2; dhdu_end << dhdu_n_T2;
            dhdq_T1 << dhdq_z1d; dhdu_T1 << dhdu_z1d;
            dhdq_T2 << dhdq_z2d; dhdu_T2 << dhdu_z2d;
          }
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
            if (ConstraintsChangedBlock1) {   // falls sich constraints im ersten Block geaendert haben
              if (maxOrder==2) {
                ze << z4b;
                LSe << LSC2;
                if (inexactJac) {
                  LSe_Reg << LSC2_Reg;
                  dhdq_end << dhdq_z4b; dhdu_end << dhdu_z4b;
                }
              }
              if (maxOrder==3) {
                ze << z6b;
                LSe << LSD3;
                if (inexactJac) {
                  LSe_Reg << LSD3_Reg;
                  dhdq_end << dhdq_z6b; dhdu_end << dhdu_z6b;
                }
              }
            }
            else {  // falls sich constraints im zweiten Block geaendert haben
              ze << zStern; order=maxOrder;
              if (maxOrder==2) {
                LSe << LSC2;
                if (inexactJac) {
                  LSe_Reg << LSC2_Reg;
                  dhdq_end << dhdq_z4b; dhdu_end << dhdu_z4b;
                }
              }
              if (maxOrder==3) {
                LSe << LSD3;
                if (inexactJac) {
                  LSe_Reg << LSD3_Reg;
                  dhdq_end << dhdq_z6b; dhdu_end << dhdu_z6b;
                }
              }
            }
          }

          if (ConstraintsChanged && !ConstraintsChangedBlock1) {  // constraints haben sich im zweiten Block geaendert
            if (testOK) {
              if (maxOrder==2) EstErrorLocal = z2dRE-z4d;
              if (maxOrder==3) EstErrorLocal = 2.0*(z4d-z6d);
              double dtNewRelTmp = calculatedtNewRel(EstErrorLocal,dt);
              if (dtNewRelTmp>=1) {
                dtNewRel = sqrt(dtNewRelTmp);
                testOK = true;
                dte = dt;
                if (maxOrder==2) {
                  ze << z4d;
                  LSe << LSC4;
                  if (inexactJac) {
                    LSe_Reg << LSC4_Reg;
                    dhdq_end << dhdq_n_T2; dhdu_end << dhdu_n_T2;
                  }
                }
                if (maxOrder==3) {
                  ze << z6d;
                  LSe << LSD6;
                  if (inexactJac) {
                    LSe_Reg << LSD6_Reg;
                    dhdq_end << dhdq_n_T3; dhdu_end << dhdu_n_T3;
                  } 
                }
                order = 1;
              }
            }
          }

          if (!ConstraintsChanged) {  // falls sich constraints nicht geaendert haben
            order = maxOrder;
            if (maxOrder==2) EstErrorLocal = (2.0*z2d-z1d - 2.0*z4d+z2dRE)/3.0;
            if (maxOrder==3) EstErrorLocal = ((4.5*z3d+0.5*z1d-4.0*z2d) - (4.5*z6d+0.5*z2dRE-4.0*z4d))/5.0;
            dtNewRel = calculatedtNewRel(EstErrorLocal,dt);
            testOK = (dtNewRel>=1.0);
            dtNewRel = pow(dtNewRel,1.0/(maxOrder+1));  
            if (maxOrder==2) { 
              ze << 2.0*z4d - z2dRE;
              LSe << LSC4;
              if (inexactJac) {
                LSe_Reg << LSC4_Reg;
                dhdq_end << dhdq_n_T2; dhdu_end << dhdu_n_T2;
              } 
            }
            if (maxOrder==3) { 
              ze << 4.5*z6d+0.5*z2dRE-4.0*z4d;
              LSe << LSD6;
              if (inexactJac) {
                LSe_Reg << LSD6_Reg;
                dhdq_end << dhdq_n_T3; dhdu_end << dhdu_n_T3;
              }
            }
            dte=dt;
            // order >=2 hat mehrfach versagt! Teste auf order 1
            if (!testOK && (StepTrials>2)) {
              double dtNewRelTmp1=0;
              double dtNewRelTmp2=0;
              if (maxOrder==2) {
                dtNewRelTmp1 = calculatedtNewRel(z2b - z4b, dt);
                dtNewRelTmp2 = calculatedtNewRel(z4d-z2dRE, dt);
              }
              if (maxOrder==3) {
                dtNewRelTmp1 = calculatedtNewRel(2.0*(z4b-z6b), dt);
                dtNewRelTmp2 = calculatedtNewRel(2.0*(z4d-z6d), dt);
              }

              if (dtNewRelTmp1>=1.0) {
                order = 1;
                testOK= true;
                if (dtNewRelTmp2>=1.0) {
                  dtNewRel= sqrt(dtNewRelTmp2);
                  if (maxOrder==2) {
                    ze << z4d;
                    LSe << LSC4;
                    if (inexactJac) {
                      LSe_Reg << LSC4_Reg;
                      dhdq_end << dhdq_n_T2; dhdu_end << dhdu_n_T2;
                    } 
                  }
                  if (maxOrder==3) {
                    ze << z6d;
                    LSe << LSD6;
                    if (inexactJac) {
                      LSe_Reg << LSD6_Reg;
                      dhdq_end << dhdq_n_T3; dhdu_end << dhdu_n_T3;
                    }
                  }
                }
                else {
                  dtNewRel= sqrt(dtNewRelTmp1);  
                  if (maxOrder==2) {
                    ze << z4b;
                    LSe << LSC2;
                    if (inexactJac) {
                      LSe_Reg << LSC2_Reg;
                      dhdq_end << dhdq_z4b; dhdu_end << dhdu_z4b;
                    } 
                  }
                  if (maxOrder==3) {
                    ze << z6b;
                    LSe << LSD3;
                    if (inexactJac) {
                      LSe_Reg << LSD3_Reg;
                      dhdq_end << dhdq_z6b; dhdu_end << dhdu_z6b;
                    }
                  }
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
                if (testOK) cerr<<"Hohe Ordnung abgeleht aber dafuer Order 1 akzeptiert!!!"<<endl; 
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
          cerr<<"Error: no convergence despite minimum stepsize("<<maxIter<<" iterations) Anyway, continuing integration..."<<endl;
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

  double AutoTimeSteppingSSCIntegrator::calculatedtNewRel(const fmatvec::Vec &ErrorLocal, double H) {
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

  bool AutoTimeSteppingSSCIntegrator::GapControl(double qUnsafe, bool SSCTestOK) 
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
  // VecInt laSizes contains the singel la-size of each link

  void AutoTimeSteppingSSCIntegrator::getAllSetValuedla(fmatvec::Vec& la_, fmatvec::VecInt& la_Sizes,vector<Link*> &SetValuedLinkList) {

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

  void AutoTimeSteppingSSCIntegrator::setAllSetValuedla(const fmatvec::Vec& la_, const fmatvec::VecInt& la_Sizes,vector<Link*> &SetValuedLinkList) {

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


  void AutoTimeSteppingSSCIntegrator::getDataForGapControl() {
    int nInActive=0;
    int nActive=0;
    zT1<<ze;
    if((sysT1->getq())()!=zT1()) sysT1->updatezRef(zT1);
    sysT1->updateStateDependentVariables(t+dte);
    for(unsigned int i=0; i<SetValuedLinkListT1.size(); i++){ 
      SetValuedLinkListT1[i]->updateg(t+dte);
      SetValuedLinkListT1[i]->checkActive(1);
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

  bool AutoTimeSteppingSSCIntegrator::changedLinkStatus(const VecInt &L1, const VecInt &L2, int ex) {
    if (ex<0) return (L1!=L2);
    int n=L1.size();
    if (n!=L2.size()) return true;
    for(int i=0; i<n; i++) 
      if (i!=ex && L1(i)!=L2(i)) return true;
    return false;
  }

  void AutoTimeSteppingSSCIntegrator::initializeUsingXML(TiXmlElement *element) {

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
