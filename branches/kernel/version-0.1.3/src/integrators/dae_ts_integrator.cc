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
#include<ctime>
#include "multi_body_system.h"
#include "dae_ts_integrator.h"
#include "integrators.h"



#ifndef NO_ISO_14882
using namespace std;

#endif


using namespace fmatvec;

namespace MBSim {


  DAETSIntegrator::DAETSIntegrator() : outputRoots(true)
  {
      name = "DAETSIntegrator";
      TSIntegrator = new TimeSteppingSSCIntegrator();
      DAEIntegrator= new DASKRIntegrator();
      TSIntegrator->deactivateSSC(false);
      TSIntegrator->setOutputInterpolation(true);
      TSIntegrator->optimiseDtForGaps(false);
      TSIntegrator->setDriftCompensation(false);
      TSIntegrator->StepsWithUnchangedConstraints = 4;
      DAEIntegrator->DAEIndex = 2;
      DAEIntegrator->FlagErrorTest = 2;
      DAEIntegrator->useExternalJac = false;
  }

  DAETSIntegrator::~DAETSIntegrator() {
    delete DAEIntegrator;
    delete TSIntegrator;
 }

  void DAETSIntegrator::settEnd(double tEnd_) {
    tEnd = tEnd_;
    TSIntegrator->tEnd = tEnd_;
    DAEIntegrator->tEnd= tEnd_;
  }

  void DAETSIntegrator::settStart(double tStart_){
    tStart = tStart_;
    TSIntegrator->tStart = tStart_;
    DAEIntegrator->tStart= tStart_;
  }

  void DAETSIntegrator::setdtPlot(double dtPlot_) {
    dtPlot = dtPlot_;
    TSIntegrator->dtPlot = dtPlot_;
    DAEIntegrator->dtPlot= dtPlot_;
  }

  void DAETSIntegrator::setz0(const fmatvec::Vec &z0_) {
    z0 = z0_;      
    TSIntegrator->z0 = z0_;
    DAEIntegrator->z0= z0_;
  }

  void DAETSIntegrator::setOutput(bool flag) {
    output = flag;
    TSIntegrator->output = flag;
    DAEIntegrator->output= flag;
  }

  void DAETSIntegrator::setaTol(const fmatvec::Vec &aTol_) {
    TSIntegrator->aTol.resize() = aTol_;
    DAEIntegrator->aTol.resize()= aTol_;
  }

  void DAETSIntegrator::setaTol(double aTol_) {
    TSIntegrator->aTol.resize() = Vec(1,INIT,aTol_);
    DAEIntegrator->aTol.resize()= Vec(1,INIT,aTol_);
  }

  void DAETSIntegrator::setrTol(const fmatvec::Vec &rTol_) {
    TSIntegrator->rTol.resize() = rTol_;
    DAEIntegrator->rTol.resize()= rTol_;
  }

  void DAETSIntegrator::setrTol(double rTol_) {
    TSIntegrator->rTol.resize() = Vec(1,INIT,rTol_);
    DAEIntegrator->rTol.resize()= Vec(1,INIT,rTol_);
  }

  void DAETSIntegrator::integrate(MultiBodySystem& system_) { 
    initIntegrator(system_);
    IntegrationStep();
    closeIntegrator();
  }

  void DAETSIntegrator::initIntegrator(MultiBodySystem &system_) {
    system = &system_;
    Timer.start();
    TSIntegrator->initIntegrator(*system);
    DAEIntegrator->initIntegrator(*system);
  }

  void DAETSIntegrator::closeIntegrator() {
    time = Timer.stop();
    TSIntegrator->closeIntegrator();
    DAEIntegrator->closeIntegrator();
    cout << endl<< endl<< "Integration time = " << time << endl;
  }
  
 void DAETSIntegrator::IntegrationStep() {
    double t=tStart;
    bool ExitIntegration = (t>=tEnd);
    while (! ExitIntegration) {
      DAEIntegrator->tStart = t;
      DAEIntegrator->z0 = z0;
      system->saveUnilaterLinkStatus();
      DAEIntegrator->refreshIntegratorSetups();
      int errorCodeInitCond = DAEIntegrator->computeInitialConditions(false,false);
      if (errorCodeInitCond>0) {
       DAEIntegrator->IntegrationStep();
        t = DAEIntegrator->t;
        z0= DAEIntegrator->z;
        ExitIntegration = (t>=tEnd);
        if ((DAEIntegrator->idid == 5)&& (outputRoots)) cout << endl << "root at t= "<<t<<endl;
      }
      else ExitIntegration = true;

      system->deleteUnilaterLinkStatus();

      if (! ExitIntegration) {
        TSIntegrator->tStart= t;
        TSIntegrator->z0 = z0;
        TSIntegrator->IntegrationStep();
        t = TSIntegrator->t;  
        z0= TSIntegrator->zi;
        ExitIntegration = (t>=tEnd);
      }
   }
 }
}

