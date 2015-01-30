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
#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h>

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
      dt(1e-3), t(0.), tPlot(0.), gTol(1e-3), hTol(1), iter(0), step(0), integrationSteps(0), maxIter(0), sumIter(0), s0(0.), time(0.), stepPlot(0), driftCompensation(false) {
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

      //		system.update(z, t);
      // as the time changes, update boundary conditions, external forces......
      // step 1 and 2: update state dependent variables and check the gap distance of contact in order to get the active index.
      // update the g, gd, Jacobin, W, h, M, LLCM, G....
//		system.update(z, t);

      /* FIND EQUILIBRIUM*/
      hgFun fun_hg(&system, t, z);

      Vec qla;
      qla.resize(q.size() + system.getla().size());

      qla(0, q.size() - 1) = q;
      qla(q.size(), qla.size() - 1) = system.getla();

//			jacFun fun_jac(&system, t, z);  // dh/dq
//			MultiDimNewtonMethod slv_h(&fun_h, &fun_jac);  // this fun_jac dh/dq does not consider the la force

      if (0) {
        MultiDimNewtonMethod slv_qla(&fun_hg);
        slv_qla.setLinearAlgebra(1);
        slv_qla.setTolerance(gTol);
        qla = slv_qla.solve(qla); // use the qla from the previous timestep as the initial guess.
        if (slv_qla.getInfo() != 0)
          throw MBSimError("ERROR (QuasiStaticIntegrator::subIntegrate): No convergence of Newton method for the new time step");
        iter = slv_qla.getNumberOfIterations();
      }
      else {
        NewtonJacobianFunction * jac = new NumericalNewtonJacobianFunction();
        MultiDimensionalNewtonMethod newton;
        map<Index, double> tolerances;
        tolerances.insert(pair<Index, double>(Index(0, q.size() - 1), hTol));
        tolerances.insert(pair<Index, double>(Index(q.size(), qla.size() - 1), gTol));
        LocalResidualCriteriaFunction cfunc(tolerances);
        GlobalResidualCriteriaFunction cfuncGlob(gTol);
        StandardDampingFunction dfunc(30);

        newton.setFunction(&fun_hg);
        newton.setJacobianFunction(jac);
        newton.setCriteriaFunction(&cfunc);
        newton.setDampingFunction(&dfunc);
        newton.setLinearAlgebra(1); // as system is possible underdetermined
        qla = newton.solve(qla);
        if (newton.getInfo() != 0)
          throw MBSimError("ERROR (QuasiStaticIntegrator::subIntegrate): No convergence of Newton method for the new time step");
        iter = newton.getNumberOfIterations();
      }

      q = qla(0, q.size() - 1);
      system.setLa(qla(q.size(), qla.size() - 1));

      if ((step * stepPlot - integrationSteps) < 0) {
        /* WRITE OUTPUT */
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

      /* UPDATE SYSTEM FOR NEXT STEP*/
      t += dt;   // step 0: update time, go into new time step.

      system.update(z, t);

      //in timestepping, u is updated according to 14a.
      // should we consider the contributation part from the contact? or just calculate u according (q^(l+1) -q^(l))/dt
      // then u is also introduced into the slv_h.solve(q) by consider it as constant.
//		u += system.deltau(z,t,dt);
//		u = (q - qOld) / dt;
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

  fmatvec::Vec hgFun::operator()(const fmatvec::Vec& qla) {
    // backup the original q of the dynamical system sys.
//    Vec qlaOld;
    int qSize = sys->getqSize();
    int laSize = sys->getlaSize();
    int qlaSize = qSize + laSize;
//    qlaOld.resize(qlaSize);

//	qlaOld(0, qSize - 1) = sys->getq();
//	qlaOld(qSize, qlaSize - 1) = sys->getla();

    // set the q of system to be the input q
    sys->setq(qla(0, qSize - 1));
    sys->setLa(qla(qSize, qlaSize - 1));
    // update the system by calling system.update(z,t);  inside this function the h vector is updated.
    sys->update(z, t);
    // get the new h vector
    Vec hg;
    hg.resize(qla.size());

    hg(0, qSize - 1) = sys->geth() + sys->getW() * sys->getla();
    hg(qSize, qlaSize - 1) = sys->getg().copy();

//    cout << "t = "  << t << "\n";
//    cout << "sys.geth() = "  <<  sys->geth() << "\n";
//    cout << "w * la = "  << sys->getW() * sys->getla()  << "\n \n";
//    cout << "hg = "  << hg  << "\n \n";

    // recover the old sys state
//	sys->setq(qlaOld(0, qSize - 1));
//	sys->setLa(qlaOld(qSize, qlaSize - 1));
//	sys->update(z, t);

    return hg;

//	// todo: may get the reference and return a reference
//	// the return type has to be changed into Vec&
//	return sys->geth();

  }

  fmatvec::SqrMat jacFun::operator()(const fmatvec::Vec& q) {
    // backup the original q of the dynamical system sys.
    Vec qOld;
    qOld << sys->getq();

    // set the q of system to be the input q
    sys->setq(q);

    SqrMat jac;
    jac << sys->dhdq(t);

    // recover the old sys state
    sys->setq(qOld);
    sys->update(z, t);

    return jac;
  }
}

