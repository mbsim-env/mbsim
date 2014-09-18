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

#include<config.h>
#include<mbsim/dynamic_system_solver.h>
#include "hets2_integrator.h"

#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  HETS2Integrator::HETS2Integrator() : dt(1e-3), 
  t(0.), 
  tPlot(0.),
  iter(0), 
  step(0), 
  integrationSteps(0), 
  maxIter(0), 
  sumIter(0), 
  s0(0.), 
  time(0.), 
  stepPlot(0),
  z(),
  q(),
  u(),
  x(),
  integPlot() {}

  void HETS2Integrator::preIntegrate(DynamicSystemSolver& system) {

    // set the time
    assert(dtPlot >= dt);
    t = tStart;

    // size of positions, velocities, state
    int nq = system.getqSize();
    int nu = system.getuSize();
    int nx = system.getxSize();
    int n = nq + nu + nx;

    // referencing the dynamic system
    Index Iq(0,nq-1);
    Index Iu(nq,nq+nu-1);
    Index Ix(nq+nu,n-1);
    z.resize(n);
    q>>z(Iq);
    u>>z(Iu);
    x>>z(Ix);

    // define initial state
    if(z0.size()) z = z0;
    else system.initz(z);

    // prepare plotting
    integPlot.open((name + ".plt").c_str());
    integPlot << "time" << " " << "time step-size" << " " <<  "constraint iterations" << " " << "calculation time" << " " << "size of constraint system" << endl;
    cout.setf(ios::scientific, ios::floatfield);

    stepPlot =(int) (dtPlot/dt + 0.5);
    assert(fabs(stepPlot*dt - dtPlot) < dt*dt);

    // start timing
    s0 = clock();
  }

  void HETS2Integrator::subIntegrate(DynamicSystemSolver& system, double tStop) {

    while(t<tStop) { // time loop

      // increase integration step counter
      integrationSteps++;

      /* LEFT INTERVAL END EVALUATIONS = ZERO STAGE EVALUATIONS */
      // update until the Jacobian matrices, especially also the active set
      evaluateStage(system);

      // update forces
      system.updateh(t);

      // update mass matrix and compute LU factorization
      system.updateM(t);
      system.facLLM();      

      // update matrix of generalized constraint directions
      system.updateW(t);

      // update matrix of generalized constraint directions with the possibility to project sliding contacts
      system.updateV(t);

      // save values
      Vec qStage0 = system.getq().copy();
      Vec uStage0 = system.getu().copy();
      Mat TStage0 = system.getT().copy();
      SymMat LLMStage0 = system.getLLM().copy();
      Vec hStage0 = system.geth().copy();
      Mat VStage0 = system.getV().copy();

      // plot
      if((step*stepPlot - integrationSteps) < 0) {
        step++;
        system.plot(t,dt);
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        integPlot << t << " " << dt << " " <<  iter << " " << time << " "<< system.getlaSize() << endl;
        if(output) cout << "   t = " <<  t << ",\tdt = "<< dt << ",\titer = " << setw(5) << setiosflags(ios::left) << iter << "\r" << flush;
        tPlot += dtPlot;
      }
      /*****************************************/

      /* CHECK IMPACTS */
      // first stage position and time update (velocity is unknown and has to be calculated with constraints/impacts)
      q += system.getT()*u*dt;
      t += dt;

      // update until the Jacobian matrices, especially also the active set
      bool impact = evaluateStage(system); // TODO: this also updates the active set!

      /* CALCULATE CONSTRAINT FORCES ON VELOCITY LEVEL AND FIRST STAGE */
      if (not impact) {
        // update matrix of generalized constraint directions
        system.updateW(t);

        // update mass action matrix of constraint equation system
        system.updateG(t); // G=WStage1^T*MStage0^{-1}*VStage0

        // update right hand side of constraint equation system
        system.getb() << system.getgd()/dt + system.getW().T()*slvLLFac(LLMStage0,hStage0);

        // solve the constraint equation system
        if (system.getla().size() not_eq 0) {
          iter = system.solveConstraints();
        }

        if(iter>maxIter) {
          maxIter = iter;
        }
        sumIter += iter;

        // save values
        Vec laStage0 = system.getla().copy();

        // first stage velocity update
        u += slvLLFac(LLMStage0,hStage0+VStage0*laStage0)*dt;
        /*****************************************/

        /* CALCULATE CONSTRAINT FORCES ON VELOCITY LEVEL AND OUTPUT STAGE */
        // output stage position update
        q = qStage0 + (system.getT()*u+TStage0*uStage0)*dt*0.5; // T-matrix in the sense of Brasey1994a, velocity is unknown and has to be calculated with constraint forces

        // update until the Jacobian matrices, especially also the active set
        evaluateStage(system);

        // update forces
        system.updateh(t);

        // update mass matrix and compute LU factorization
        system.updateM(t);
        system.facLLM();

        // update matrix of generalized constraint directions
        system.updateW(t);

        // update matrix of generalized constraint directions with the possibility to project sliding contacts
        system.updateV(t);

        // update mass action matrix of constraint equation system
        system.updateG(t);

        // output stage velocity update without constraint force
        u = uStage0 + (slvLLFac(LLMStage0,hStage0+VStage0*laStage0) + slvLLFac(system.getLLM(),system.geth()))*dt*0.5;

        // update until the Jacobian matrices, especially also the active set
        evaluateStage(system);

        // update right hand side of constraint equation system
        system.getb() << 2.*system.getgd()/dt;

        // solve the constraint equation system
        if (system.getla().size() not_eq 0) {
          iter = system.solveConstraints();
        }

        if(iter>maxIter) {
          maxIter = iter;
        }
        sumIter += iter;

        // output stage velocity update
        u += slvLLFac(system.getLLM(),system.getV()*system.getla())*dt*0.5;
      }
      /*****************************************/

      /* CALCULATE IMPACTS ON VELOCITY LEVEL AND OUTPUT STAGE */
      else {
        // first stage velocity update
        u += slvLLFac(LLMStage0,hStage0)*dt;

        // output stage position update
        q = qStage0 + (system.getT()*u+TStage0*uStage0)*dt*0.5; // T-matrix in the sense of Brasey1994a, velocity is unknown and has to be calculated with impacts 

        // update until the Jacobian matrices, especially also the active set
        evaluateStage(system);

        // update forces
        system.updateh(t);

        // update mass matrix and compute LU factorization
        system.updateM(t);
        system.facLLM();

        // update matrix of generalized constraint directions
        system.updateW(t);

        // update matrix of generalized constraint directions with the possibility to project sliding contacts
        system.updateV(t);

        // update mass action matrix of constraint equation system
        system.updateG(t);

        // output stage velocity update without impact
        u = uStage0 + (slvLLFac(LLMStage0,hStage0) + slvLLFac(system.getLLM(),system.geth()))*dt*0.5;

        //// update until the Jacobian matrices, especially also the active set
        evaluateStage(system);

        // update right hand side of impact equation system
        system.getb() << system.getgd();

        // solve the impact equation system
        if (system.getla().size() not_eq 0) {
          system.getla() = system.getla().copy()*dt;
          iter = system.solveImpacts();
        }

        if(iter>maxIter) {
          maxIter = iter;
        }
        sumIter += iter;

        // output stage velocity update
        u += slvLLFac(system.getLLM(),system.getV()*system.getla());

        system.getla() /= dt; 
      }
      /*****************************************/

      //x += system.deltax(z,t,dt);
    }
  }

  void HETS2Integrator::postIntegrate(DynamicSystemSolver& system) {
    integPlot.close();

    ofstream integSum((name + ".sum").c_str());
    integSum << "Integration time: " << time << endl;
    integSum << "Integration steps: " << integrationSteps << endl;
    integSum << "Maximum number of iterations: " << maxIter << endl;
    integSum << "Average number of iterations: " << double(sumIter)/integrationSteps << endl;
    integSum.close();

    cout.unsetf(ios::scientific);
    cout << endl;
  }

  void HETS2Integrator::integrate(DynamicSystemSolver& system) {
    debugInit();
    preIntegrate(system);
    subIntegrate(system, tEnd);
    postIntegrate(system);
  }

  void HETS2Integrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stepSize");
    setStepSize(Element::getDouble(e));
  }

  bool HETS2Integrator::evaluateStage(DynamicSystemSolver& system) {
    if (system.getq()() != z())
      system.updatezRef(z);

    // update variables which depend on the state, e.g. Cartesian descriptions
    system.updateStateDependentVariables(t);

    // update gaps and observe their activity
    system.updateg(t);
    system.checkActive(1);

    bool impact = system.detectImpact();

    // adapt size of constraint system on velocity level
    if (system.gActiveChanged()) {
      system.calcgdSize(2); // contacts which stay closed
      system.calclaSize(2); // contacts which stay closed
      system.calcrFactorSize(2); // contacts which stay closed

      system.updateWRef(system.getWParent(0)(Index(0, system.getuSize() - 1), Index(0, system.getlaSize() - 1)));
      system.updateVRef(system.getVParent(0)(Index(0, system.getuSize() - 1), Index(0, system.getlaSize() - 1)));
      system.updatelaRef(system.getlaParent()(0, system.getlaSize() - 1));
      system.updategdRef(system.getgdParent()(0, system.getgdSize() - 1));
      system.updaterFactorRef(system.getrFactorParent()(0, system.getrFactorSize() - 1));
    }

    // update gap velocities
    system.updategd(t);

    // update transformation matrix for derived generalized positions and generalized velocities
    system.updateT(t);

    // update Jacobians for projection of Cartesian contact couplings in the direction of generalized velocities
    system.updateJacobians(t);

    return impact; 
  }

}

