/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#include <config.h>
#include "hets2_integrator.h"
#include <mbsim/dynamic_system_solver.h>
#include <ctime>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, HETS2Integrator)

  void HETS2Integrator::preIntegrate() {
    debugInit();

    // set the time
    assert(dtPlot >= dt);
    system->setTime(tStart);

    // define initial state
    if(z0.size()) {
      if(z0.size() != system->getzSize())
        throwError("(HETS2Integrator::integrate): size of z0 does not match, must be " + to_string(system->getzSize()));
      system->setState(z0);
    }
    else
      system->evalz0();

    system->setUseOldla(false);
    system->setGeneralizedForceTolerance(1e-10/dt); // adaptation from impulse
    system->setGeneralizedRelativeAccelerationTolerance(1e-10/dt); // as we use local velocities to express accelerations within solveConstraints

    assert(fabs(((int) (dtPlot/dt + 0.5))*dt - dtPlot) < dt*dt);

    // start timing
    s0 = clock();
  }

  void HETS2Integrator::subIntegrate(double tStop) {

    while(system->getTime()<tStop) { // time loop

      // increase integration step counter
      integrationSteps++;

      /* LEFT INTERVAL END EVALUATIONS = ZERO STAGE EVALUATIONS */
      // update until the Jacobian matrices, especially also the active set
      evaluateStage();

      // save values
      Vec q_n = system->getq().copy();
      Vec u_n = system->getu().copy();
      Mat T_n = system->evalT().copy();
      SymMat LLM_1 = system->evalLLM().copy();
      Vec h_1 = system->evalh().copy();
      Mat W_1 = system->evalW().copy();
      Mat V_1 = system->evalV().copy();

      // plot
      if(system->getTime() >= tPlot) {
        system->setUpdatela(false);
        system->setUpdateLa(false);
        system->plot();
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        if(msgAct(Status)) msg(Status) << "   t = " << system->getTime() << ",\tdt = "<< dtInfo << ",\titer = " << setw(5) << setiosflags(ios::left) << system->getIterC() << flush;
        tPlot += dtPlot;
      }
      /*****************************************/

      Vec q_p = q_n + T_n*u_n*dt;
      system->setq(q_p);
      system->getTime() += dt;

      system->resetUpToDate();

        // increase integration step counter for constraints
        integrationStepsConstraint++;

        // adapt last time step-size
        dtInfo = dt;

        system->getbc(false) << system->evalW().T()*(u_n/dt + slvLLFac(LLM_1,h_1));
        system->setUpdatebc(false);

        Vec la_1 = system->evalla().copy();
        Vec r_1 = V_1*la_1;

        Vec u_1 = u_n + slvLLFac(LLM_1,h_1+r_1)*dt;
        system->setu(u_1);
        /*****************************************/

        Vec q_1 = q_n + (T_n*u_n+system->evalT()*u_1)*dt*0.5; // T-matrix in the sense of Brasey1994a, velocity is unknown and has to be calculated with constraint forces
        system->setq(q_1);

        if(system->getIterC()>maxIter) maxIter = system->getIterC();
        sumIter += system->getIterC();

        system->resetUpToDate();

        bool impact = evaluateStage();

        SymMat LLM_2 = system->evalLLM().copy();
        Vec h_2 = system->evalh().copy();
        Mat W_2 = system->evalW().copy();
        Mat V_2 = system->evalV().copy();

        // update until the Jacobian matrices, especially also the active set
        if(impact) {
          u_1 = u_n + (slvLLFac(LLM_1,h_1) + slvLLFac(LLM_2,h_2))*dt*0.5;
          system->evalgd(); // TODO this equals W_2.T()*u_1, should be W_2.T()*u_n
          system->getbi(false) << W_2.T()*u_1;
          system->setUpdatebi(false);

          Vec La_2 = system->evalLa().copy();
          Vec rdt_2 = V_2*La_2;
          system->setu(u_1 + slvLLFac(LLM_2,rdt_2));

          system->resetUpToDate();
        }
        else {
          system->getbc(false) << W_2.T()*(u_n*2./dt + (slvLLFac(LLM_1,h_1+r_1) + slvLLFac(LLM_2,h_2)));
          system->setUpdatebc(false);

          Vec la_2 = system->evalla().copy();
          Vec r_2 = V_2*la_2;

          system->setu(u_n + (slvLLFac(LLM_1,h_1+r_1) + slvLLFac(LLM_2,h_2+r_2))*dt*0.5);

          if(system->getIterC()>maxIter) maxIter = system->getIterC();
          sumIter += system->getIterC();

          system->resetUpToDate();
      }
    }
  }

  void HETS2Integrator::postIntegrate() {
    msg(Info) << endl << endl << "******************************" << endl;
    msg(Info) << "INTEGRATION SUMMARY: " << endl;
    msg(Info) << "End time [s]: " << tEnd << endl;
    msg(Info) << "Integration time [s]: " << time << endl;
    msg(Info) << "Integration steps: " << integrationSteps << endl;
    msg(Info) << "Fraction of impulsive integration steps: " << double(integrationStepsImpact)/integrationSteps << endl;
    msg(Info) << "Maximum number of iterations: " << maxIter << endl;
    msg(Info) << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;
    msg(Info) << "******************************" << endl;
    msg(Info).flush();
  }

  void HETS2Integrator::integrate() {
    system->setUseConstraintSolverForPlot(true);
    preIntegrate();
    subIntegrate(tEnd);
    postIntegrate();
  }

  void HETS2Integrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stepSize");
    if(e) setStepSize(E(e)->getText<double>());
  }

  bool HETS2Integrator::evaluateStage() {
    system->checkActive(1);

    bool impact = system->detectImpact();

    // adapt size of constraint system on velocity level
    if (system->gActiveChanged()) {
      system->calcgdSize(2); // contacts which stay closed
      system->calclaSize(2); // contacts which stay closed
      system->calcrFactorSize(2); // contacts which stay closed

      system->updateWRef(system->getWParent(0)(RangeV(0, system->getuSize() - 1), RangeV(0, system->getlaSize() - 1)));
      system->updateVRef(system->getVParent(0)(RangeV(0, system->getuSize() - 1), RangeV(0, system->getlaSize() - 1)));
      system->updatelaRef(system->getlaParent()(0, system->getlaSize() - 1));
      system->updateLaRef(system->getLaParent()(0, system->getlaSize() - 1));
      system->updategdRef(system->getgdParent()(0, system->getgdSize() - 1));
      system->updaterFactorRef(system->getrFactorParent()(0, system->getrFactorSize() - 1));
    }

    return impact; 
  }

}
