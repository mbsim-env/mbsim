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
    assert(dtPlot >= dt);

    t = tStart;

    int nq = system.getqSize(); // size of positions, velocities, state
    int nu = system.getuSize();
    int nx = system.getxSize();
    int n = nq + nu + nx;

    Index Iq(0,nq-1);
    Index Iu(nq,nq+nu-1);
    Index Ix(nq+nu,n-1);
    z.resize(n);
    q>>z(Iq);
    u>>z(Iu);
    x>>z(Ix);

    if(z0.size()) z = z0; // define initial state
    else system.initz(z);

    integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);

    stepPlot =(int) (dtPlot/dt + 0.5);
    assert(fabs(stepPlot*dt - dtPlot) < dt*dt);

    s0 = clock();
  }

  void HETS2Integrator::subIntegrate(DynamicSystemSolver& system, double tStop) {
    while(t<tStop) { // time loop

      integrationSteps++;

      if((step*stepPlot - integrationSteps) < 0) {
        step++;
        system.plot2(z,t,dt);
        double s1 = clock();
        time += (s1-s0)/CLOCKS_PER_SEC;
        s0 = s1; 
        integPlot<< t << " " << dt << " " <<  iter << " " << time << " "<<system.getlaSize() <<endl;
        if(output) cout << "   t = " <<  t << ",\tdt = "<< dt << ",\titer = "<<setw(5)<<setiosflags(ios::left) << iter <<  "\r"<<flush;
        tPlot += dtPlot;
      }

      /* LEFT INTERVAL END EVALUATIONS = FIRST STAGE EVALUATIONS */
      // update until the Jacobian matrices, especially also the active set
      evaluateStage(system);

      // update forces
      system.updateh(t);

      // update mass matrix and compute LU factorization
      system.updateM(t);
      system.facLLM();      
      
      // save values
      Vec qOld = system.getq().copy();
      Mat TOld = system.getT().copy();
      SymMat LLMOld = system.getLLM().copy();
      Vec hOld = system.geth().copy();
      Vec gdOld = system.getgd().copy();
      /*****************************************/

      /* CALCULATE CONSTRAINT FORCES ON VELOCITY LEVEL */
      q += system.getT()*u*dt; // prediction stage (position and time, no velocity)
      t += dt;
      evaluateStage(system); // TODO: this also updates the activ set?

      // update matrix of generalized constraint directions
      system.updateW(t);
      Mat WOld = system.getW().copy();

      // update matrix of generalized constraint directions with the possibility to project sliding contacts
      system.updateV(t);

      // update mass action matrix of constraint equation system
      system.getLLM() = LLMOld.copy();
      system.updateG(t); // TODO: normally G is set up as a mixture
      
      // update right hand side of constraint equation system
      system.geth() = hOld.copy();
      //system.getb() << gdOld + system.getW().T()*slvLLFac(LLMOld,hOld)*dt; // TODO: normally we have not gdOld here
      
      // solve the constraint equation system
      if (system.getla().size() not_eq 0)
        iter = system.solveConstraintsIndex2LinearEquations(dt);
      //iter = system.solveImpacts(dt);

      if(iter>maxIter)
        maxIter = iter;
      sumIter += iter;
      
      // save values
      Vec uOld = system.getu().copy();
      Vec laOld = system.getla().copy();
      /*****************************************/

      /* CALCULATE SECOND STAGE */
      u += slvLLFac(LLMOld,hOld+WOld*laOld)*dt;
      q = qOld + TOld*(u+uOld)*dt*0.5; // TODO: T-matrix for new stage is implicitely defined!
      /*****************************************/

      /* RIGHT INTERVAL END EVALUATIONS = SECOND STAGE EVALUATIONS */
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
      /*****************************************/

      if(true) {
      /* CALCULATE CONSTRAINT FORCES ON VELOCITY LEVEL */
      /*****************************************/
      system.getG() *= dt;
      
      // update right hand side of constraint equation system
      system.getb() << gdOld + system.getW().T()*slvLLFac(LLMOld,hOld+WOld*laOld)*dt*0.5; // TODO: normally we have not gdOld here
      } 
      else {
      /* CALCULATE IMPULSIVE FORCES ON VELOCITY LEVEL */
      /*****************************************/
      }

      /* CALCULATE OUTPUT STAGE */
      u = uOld + slvLLFac(LLMOld,hOld)*dt*0.5 + slvLLFac(system.getLLM(),system.geth())*dt*0.5;
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

  void HETS2Integrator::evaluateStage(DynamicSystemSolver& system) {
    if (system.getq()() != z())
      system.updatezRef(z);

    // update variables which depend on the state, e.g. Cartesian descriptions
    system.updateStateDependentVariables(t);

    // update gaps and observe their activity
    system.updateg(t);
    system.checkActive(1);

    // adapt size of constraint system on velocity level
    if (system.gActiveChanged()) {
      system.calcgdSize(2); // contacts which stay closed
      system.calclaSize(2); // contacts which stay closed
      system.calcrFactorSize(2); // contacts which stay closed

      system.updateWRef(system.getWParent(0)(Index(0, system.getuSize() - 1), Index(0, system.getlaSize() - 1)));
      system.updateVRef(system.getVParent(0)(Index(0, system.getuSize() - 1), Index(0, system.getlaSize() - 1)));
      system.updatelaRef(system.getlaParent()(0, system.getlaSize() - 1));
      system.updategdRef(system.getgdParent()(0, system.getgdSize() - 1));
      //if (impactSolver == RootFinding)
      //  updateresRef(resParent(0, laSize - 1));
      //updaterFactorRef(rFactorParent(0, rFactorSize - 1));
    }

    // update gap velocities
    system.updategd(t);

    // update transformation matrix for derived generalized positions and generalized velocities
    system.updateT(t);

    // update Jacobians for projection of Cartesian contact couplings in the direction of generalized velocities
    system.updateJacobians(t);  
  }

}

