/* Copyright (C) 2004-2025  Martin Förg
 
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
 *   martin.o.foerg@googlemail.com
 *
 */

#include <config.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/utils/eps.h>
#include "dae_integrator.h"

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace xercesc;
using namespace MBXMLUtils;

namespace MBSim {

  // DAE1
  void DAEIntegrator::par_ud_xd_gdd_par_q_u(Mat &J, const Vec &ud_) {
    const Vec &zd = system->getzd(false);
    const Vec &ud = ud_()?ud_:system->getud(false);
    for(int j=0; j<system->getqSize()+system->getuSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updateud();
      system->updatexd();
      const Vec &gdd1 = system->evalW().T()*ud + system->evalwb();
      for(int i=system->getqSize(); i<system->getzSize(); i++)
        J(i-qMove,j)=(zd(i)-zd0(i))/delt;
      for(int i=0; i<system->getgdSize(); i++)
        J(i+laInd,j)=(gdd1(i)-gd0(i))/delt; // we use gd0 for gdd0 here
      system->getState()(j)=zSave;
    }
  }

  // DAE1
  void DAEIntegrator::par_zd_gdd_par_q_u(Mat &J, const Vec &ud_) {
    const Vec &zd = system->getzd(false);
    const Vec &ud = ud_()?ud_:system->getud(false);
    for(int j=0; j<system->getqSize()+system->getuSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updatezd();
      const Vec &gdd1 = system->evalW().T()*ud + system->evalwb();
      for(int i=0; i<system->getzSize(); i++)
        J(i,j)=(zd(i)-zd0(i))/delt;
      for(int i=0; i<system->getgdSize(); i++)
        J(i+laInd,j)=(gdd1(i)-gd0(i))/delt; // we use gd0 for gdd0 here
      system->getState()(j)=zSave;
    }
  }

  // DAE1
  void DAEIntegrator::par_ud_xd_par_x(Mat &J) {
    const Vec &zd = system->getzd(false);
    for(int j=system->getqSize()+system->getuSize(); j<system->getzSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updateud();
      system->updatexd();
      for(int i=system->getqSize(); i<system->getzSize(); i++)
        J(i-qMove,j)=(zd(i)-zd0(i))/delt;
      system->getState()(j)=zSave;
    }
  }

  // DAE2
  void DAEIntegrator::par_ud_xd_gd_par_q(Mat &J) {
    const Vec &zd = system->getzd(false);
    const Vec &gd = system->getgd(false);
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updateud();
      system->updatexd();
      system->updategd();
      for(int i=system->getqSize(); i<system->getzSize(); i++)
        J(i-qMove,j)=(zd(i)-zd0(i))/delt;
      for(int i=0; i<system->getgdSize(); i++)
        J(i+laInd,j)=(gd(i)-gd0(i))/delt;
      system->getState()(j)=zSave;
    }
  }

  // DAE2
  void DAEIntegrator::par_zd_gd_par_q(Mat &J) {
    const Vec &zd = system->getzd(false);
    const Vec &gd = system->getgd(false);
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updatezd();
      system->updategd();
      for(int i=0; i<system->getzSize(); i++)
        J(i,j)=(zd(i)-zd0(i))/delt;
      for(int i=0; i<system->getgdSize(); i++)
        J(i+laInd,j)=(gd(i)-gd0(i))/delt;
      system->getState()(j)=zSave;
    }
  }

  // DAE3
  void DAEIntegrator::par_ud_xd_g_par_q(Mat &J) {
    const Vec &zd = system->getzd(false);
    const Vec &g = system->getg(false);
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updateud();
      system->updatexd();
      system->updateg();
      for(int i=system->getqSize(); i<system->getzSize(); i++)
        J(i-qMove,j)=(zd(i)-zd0(i))/delt;
      for(int i=0; i<system->getgSize(); i++)
        J(i+laInd,j)=(g(i)-gd0(i))/delt; // we use gd0 for g0 here
      system->getState()(j)=zSave;
    }
  }

  // DAE3
  void DAEIntegrator::par_zd_g_par_q(Mat &J) {
    const Vec &zd = system->getzd(false);
    const Vec &g = system->getg(false);
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updatezd();
      system->updateg();
      for(int i=0; i<system->getzSize(); i++)
        J(i,j)=(zd(i)-zd0(i))/delt;
      for(int i=0; i<system->getgSize(); i++)
        J(i+laInd,j)=(g(i)-gd0(i))/delt; // we use gd0 for g0 here
      system->getState()(j)=zSave;
    }
  }

  // GGL
  void DAEIntegrator::par_ud_xd_gd_g_par_q(Mat &J) {
    const Vec &zd = system->getzd(false);
    const Vec &gd = system->getgd(false);
    const Vec &g = system->getg(false);
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updateud();
      system->updatexd();
      system->updategd();
      system->updateg();
      for(int i=system->getqSize(); i<system->getzSize(); i++)
        J(i,j)=(zd(i)-zd0(i))/delt;
      for(int i=0; i<system->getgdSize(); i++)
        J(i+laInd,j)=(gd(i)-gd0(i))/delt;
      for(int i=0; i<system->getgSize(); i++)
        J(i+lInd,j)=(g(i)-g0(i))/delt;
      system->getState()(j)=zSave;
    }
  }

  // GGL
  void DAEIntegrator::par_zd_gd_g_par_q(Mat &J) {
    const Vec &zd = system->getzd(false);
    const Vec &gd = system->getgd(false);
    const Vec &g = system->getg(false);
    for(int j=0; j<system->getqSize(); j++) {
      double zSave=system->getState()(j);
      double delt=delta(j,zSave);
      system->getState()(j)=zSave+delt;
      system->resetUpToDate();
      system->setUpdatela(false);
      system->updatezd();
      system->updategd();
      system->updateg();
      for(int i=0; i<system->getzSize(); i++)
        J(i,j)=(zd(i)-zd0(i))/delt;
      for(int i=0; i<system->getgdSize(); i++)
        J(i+laInd,j)=(gd(i)-gd0(i))/delt;
      for(int i=0; i<system->getgSize(); i++)
        J(i+lInd,j)=(g(i)-g0(i))/delt;
      system->getState()(j)=zSave;
    }
  }

  void DAEIntegrator::calcSize() {
    if(formalism==DAE1 or formalism==DAE2)
      neq = system->getzSize()+system->getlaSize();
    else if(formalism==DAE3)
      neq = system->getzSize()+system->getgSize();
    else if(formalism==GGL)
      neq = system->getzSize()+system->getgdSize()+system->getgSize();
    else
      neq = system->getzSize();
  }

  void DAEIntegrator::init() {
    ImplicitIntegrator::init();

    if(!aTolPos) {
      // tolerance given by a single scalar or vector for aTol/rTol -> update tolerance vector at init (the maximal possible size is used in init)

      calcSize();

      // no tolerance given -> use default tolerance
      if(aTol.size() == 0 && !aTolScalar)
        aTol.resize(neq,INIT,1e-6);
      if(rTol.size() == 0 && !rTolScalar)
        rTol.resize(neq,INIT,1e-6);

      // scalar tolerance given -> use the scalar for all entries in the tolerance vector
      if(aTolScalar)
        aTol.resize(neq,INIT,aTolScalar.value());
      if(rTolScalar)
        rTol.resize(neq,INIT,rTolScalar.value());

      // check tolerance vector size
      if(aTol.size() != neq)
        throwError("(DAEIntegrator::integrate): size of aTol/rTol must be " + to_string(neq));
      if(rTol.size() != aTol.size())
        throwError("(DAEIntegrator::integrate): size of rTol does not match aTol, must be " + to_string(aTol.size()));
    }
  }

  void DAEIntegrator::reinit() {
    calcSize();
    if(formalism>0) {
      laInd = system->getzSize()-qMove;
      Rla = RangeV(system->getzSize(), system->getzSize()+system->getlaSize()-1);
      RlaMove = RangeV(Rla.start()-qMove, Rla.end()-qMove);
      if(formalism==GGL) {
        lInd = laInd+system->getlaSize();
        Rl = RangeV(system->getzSize()+system->getlaSize(), neq-1);
      }
    }

    if(aTolPos) {
      // tolerance given by a single scalar for position, velocity, 1st order and force states -> update tolerance vector at reinit

      aTol.resize(neq);
      rTol.resize(neq);
      aTol.set(Rq , VecV(Rq.size(), INIT, aTolPos.value()));
      rTol.set(Rq , VecV(Rq.size(), INIT, rTolPos.value()));
      aTol.set(Ru , VecV(Ru.size(), INIT, aTolVel.value()));
      rTol.set(Ru , VecV(Ru.size(), INIT, rTolVel.value()));
      aTol.set(Rx , VecV(Rx.size(), INIT, aTol1st.value()));
      rTol.set(Rx , VecV(Rx.size(), INIT, rTol1st.value()));
      if(formalism>0) {
        aTol.set(Rla, VecV(Rla.size(), INIT, aTolForce.value()));
        rTol.set(Rla, VecV(Rla.size(), INIT, rTolForce.value()));
      }
      if(formalism==GGL) {
        aTol.set(Rl, VecV(Rl.size(), INIT, aTolForce.value()));
        rTol.set(Rl, VecV(Rl.size(), INIT, rTolForce.value()));
      }
    }
  }

  void DAEIntegrator::initializeUsingXML(DOMElement *element) {
    ImplicitIntegrator::initializeUsingXML(element);
    DOMElement *e;

    e=E(element)->getFirstElementChildNamed(MBSIM%"absoluteTolerance");
    if(e) setAbsoluteTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"absoluteToleranceScalar");
    if(e) setAbsoluteTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"relativeTolerance");
    if(e) setRelativeTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"relativeToleranceScalar");
    if(e) setRelativeTolerance(E(e)->getText<double>());

    e=E(element)->getFirstElementChildNamed(MBSIM%"absolutePositionTolerance");
    if(e) {
      setAbsolutePositionTolerance(E(e)->getText<double>());
      e=E(element)->getFirstElementChildNamed(MBSIM%"absoluteVelocityTolerance");
      setAbsoluteVelocityTolerance(E(e)->getText<double>());
      e=E(element)->getFirstElementChildNamed(MBSIM%"absoluteFirstOrderTolerance");
      setAbsoluteFirstOrderTolerance(E(e)->getText<double>());
      e=E(element)->getFirstElementChildNamed(MBSIM%"absoluteForceTolerance");
      setAbsoluteForceTolerance(E(e)->getText<double>());
      e=E(element)->getFirstElementChildNamed(MBSIM%"relativePositionTolerance");
      setRelativePositionTolerance(E(e)->getText<double>());
      e=E(element)->getFirstElementChildNamed(MBSIM%"relativeVelocityTolerance");
      setRelativeVelocityTolerance(E(e)->getText<double>());
      e=E(element)->getFirstElementChildNamed(MBSIM%"relativeFirstOrderTolerance");
      setRelativeFirstOrderTolerance(E(e)->getText<double>());
      e=E(element)->getFirstElementChildNamed(MBSIM%"relativeForceTolerance");
      setRelativeForceTolerance(E(e)->getText<double>());
    }
  }
}
