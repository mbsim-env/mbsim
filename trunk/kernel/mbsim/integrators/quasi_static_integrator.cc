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
 * Contact: martin.o.foerg@googlemail.com
 */

#include<config.h>
#include<mbsim/dynamic_system_solver.h>
#include "quasi_static_integrator.h"
#include <mbsim/utils/nonlinear_algebra.h>

#include <time.h>
#include <boost/iostreams/tee.hpp>
#include <boost/iostreams/stream.hpp>

#ifndef NO_ISO_14882
using namespace std;
#endif

namespace bio = boost::iostreams;
using bio::tee_device;
using bio::stream;

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(QuasiStaticIntegrator, MBSIMINT % "QuasiStaticIntegrator")

  QuasiStaticIntegrator::QuasiStaticIntegrator() :
      dt(1e-3), t(0.), tPlot(0.), iter(0), step(0), integrationSteps(0), maxIter(0), sumIter(0), s0(0.), time(0.), stepPlot(0), driftCompensation(false) {
  }

  void QuasiStaticIntegrator::preIntegrate(DynamicSystemSolver& system) {
    // initialisation
    assert(dtPlot >= dt);

    t = tStart;

    int nq = system.getqSize(); // size of positions, velocities, state
    int nu = system.getuSize();
    int nx = system.getxSize();
    int n = nq + nu + nx;

    Index Iq(0, nq - 1);
    Index Iu(nq, nq + nu - 1);
    Index Ix(nq + nu, n - 1);
    z.resize(n);
    q >> z(Iq);
    u >> z(Iu);
    x >> z(Ix);

    if (z0.size())
      z = z0; // define initial state
    else
      system.initz(z);

    integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);

    stepPlot = (int) (dtPlot / dt + 0.5);
    if (fabs(stepPlot * dt - dtPlot) > dt * dt) {
      cout << "WARNING: Due to the plot-Step settings it is not possible to plot exactly at the correct times." << endl;
    }

    s0 = clock();
  }

  void QuasiStaticIntegrator::subIntegrate(DynamicSystemSolver& system, double tStop) {
    while (t < tStop) { // time loop
      integrationSteps++;
      if ((step * stepPlot - integrationSteps) < 0) {
        step++;
        if (driftCompensation)
          system.projectGeneralizedPositions(t, 0);
        system.plot2(z, t, dt);
        double s1 = clock();
        time += (s1 - s0) / CLOCKS_PER_SEC;
        s0 = s1;
        integPlot << t << " " << dt << " " << iter << " " << time << " " << system.getlaSize() << endl;
        if (output)
          cout << "   t = " << t << ",\tdt = " << dt << ",\titer = " << setw(5) << setiosflags(ios::left) << iter << "\r" << flush;
        tPlot += dtPlot;
      }

      t += dt;   // step 0: update time, go into new time step.

      // as the time changes, update boundary conditions, external forces......
      // step 1 and 2: update state dependent variables and check the gap distance of contact in order to get the active index.
      // update the g, gd, Jacobin, W, h, M, LLCM, G....
//		system.update(z, t);

      hFun fun_h(&system, t, z);
      //      jacFun fun_jac();
      //      MultiDimNewtonMethod slv_h(&fun_h, &fun_jac); //todo: add this later
      MultiDimNewtonMethod slv_h(&fun_h);

      q = slv_h.solve(q); // use the q from the previous timestep as the initial guess.
      if (slv_h.getInfo() != 0)
        throw("ERROR (QuasiStaticIntegrator::subIntegrate): No convergence of Newton method for the new time step");

      iter = system.solveImpacts(dt);  // todo: which states does this change?

      if (iter > maxIter)
        maxIter = iter;
      sumIter += iter;

      //      u += system.deltau(z,t,dt);  // todo: do we need to update them here?
      x += system.deltax(z, t, dt);
    }
  }

//  void QuasiStaticIntegrator::subIntegrate(DynamicSystemSolver& system, double tStop) {
//  while (t < tStop) { // time loop
//		integrationSteps++;
//		if ((step * stepPlot - integrationSteps) < 0) {
//			step++;
//			if (driftCompensation)
//				system.projectGeneralizedPositions(t, 0);
//			system.plot2(z, t, dt);
//			double s1 = clock();
//			time += (s1 - s0) / CLOCKS_PER_SEC;
//			s0 = s1;
//			integPlot << t << " " << dt << " " << iter << " " << time << " "
//					<< system.getlaSize() << endl;
//			if (output)
//				cout << "   t = " << t << ",\tdt = " << dt << ",\titer = "
//						<< setw(5) << setiosflags(ios::left) << iter << "\r"
//						<< flush;
//			tPlot += dtPlot;
//		}
//
//  //      q += system.deltaq(z,t,dt); // step 4 : compute the new generalized position, order 1: q = q + Y * u * dt
//
//		t += dt;   // step 0: update time, go into new time step.
//
//		// as the time changes, update boundary conditions, external forces......
//		// step 1 and 2: update state dependent variables and check the gap distance of contact in order to get the active index.
//		// update the g, gd, Jacobin, W, h, M, LLCM, G....
//		system.update(z, t);
//
//  //      system.getb() << system.getgd() + system.getW().T()*slvLLFac(system.getLLM(),system.geth())*dt;
//
//		// get the the necessary components from system
//  //      Mat T = system.getT().copy();
//  //      SymMat M = system.getM().copy();
//  //      Mat W = system.getW().copy();
//  //      Mat V = system.getV().copy();
//  //		Vec h = system.geth().copy();
//  //		Mat dhdq = system.dhdq(t);
//  //		Mat dhdu = system.dhdu(t);
//  //		Mat dhdx = system.dhdx(t); // todo: check whether need dhdx
//
//		// construct Jacobian matrix
//		// todo: offer different possible strategy for Jacobian, maybe read the paper from simpack.
//		// fully update, partially update, update in every n time steps.........
//
//		hFun fun_h(&system, t, z);
//  //      jacFun fun_jac();
//  //      MultiDimNewtonMethod slv_h(&fun_h, &fun_jac); //todo: add this later
//
////		fun_h.updateTime(t);
//
//		MultiDimNewtonMethod slv_h(&fun_h);
//
//		q = slv_h.solve(q); // use the q from the previous timestep as the initial guess.
//		if (slv_h.getInfo() != 0)
//			throw("ERROR (QuasiStaticIntegrator::subIntegrate): No convergence of Newton method for the new time step");
//
//		iter = system.solveImpacts(dt);  // todo: which states does this change?
//
//		if (iter > maxIter)
//			maxIter = iter;
//		sumIter += iter;
//
//  //      u += system.deltau(z,t,dt);  // todo: do we need to update them here?
//  //      x += system.deltax(z,t,dt);
//	}
//  }

  void QuasiStaticIntegrator::postIntegrate(DynamicSystemSolver& system) {
    integPlot.close();

    typedef tee_device<ostream, ofstream> TeeDevice;
    typedef stream<TeeDevice> TeeStream;
    ofstream integSum((name + ".sum").c_str());
    TeeDevice ts_tee(cout, integSum);
    TeeStream ts_split(ts_tee);

    ts_split << endl << endl << "******************************" << endl;
    ts_split << "INTEGRATION SUMMARY: " << endl;
    ts_split << "End time [s]: " << tEnd << endl;
    ts_split << "Integration time [s]: " << time << endl;
    ts_split << "Integration steps: " << integrationSteps << endl;
    ts_split << "Maximum number of iterations: " << maxIter << endl;
    ts_split << "Average number of iterations: " << double(sumIter) / integrationSteps << endl;
    ts_split << "******************************" << endl;
    ts_split.flush();
    ts_split.close();

    cout.unsetf(ios::scientific);
    cout << endl;
  }

  void QuasiStaticIntegrator::integrate(DynamicSystemSolver& system) {
    debugInit();
    preIntegrate(system);
    subIntegrate(system, tEnd);
    postIntegrate(system);
  }

  void QuasiStaticIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIMINT % "stepSize");
    setStepSize(Element::getDouble(e));
  }

  fmatvec::Vec hFun::operator()(const fmatvec::Vec& q) {
    // update h according the new q and system boundary condition

    // set the q of system to be the input q
    z(Index(0, sys->getqSize() - 1)) << q;
    // update the system by calling system.update(z,t);  inside this function the h vector is updated.
    sys->update(z, t);

    // get the new h vector
    return sys->geth().copy();

//	// todo: may get the reference and return a reference
//	// the return type has to be changed into Vec&
//	return sys->geth();

  }
}

