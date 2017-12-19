/* Copyright (C) 2017 Markus Friedrich

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
 */

#ifndef _BOOST_ODEINT_INTEGRATOR_H_
#define _BOOST_ODEINT_INTEGRATOR_H_

#include "integrator.h"
#include <mbsim/dynamic_system_solver.h>
#include <boost/numeric/odeint/stepper/generation/make_dense_output.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/fusion/container/set.hpp>

namespace MBSimIntegrator {

  using namespace std;
  using namespace std::placeholders;
  using namespace MBSim;
  using namespace fmatvec;
  using namespace xercesc;
  using namespace MBXMLUtils;
  using namespace boost::numeric;

  // Convert a MBSim state (fmatvec::Vec) to the boost odeint state type.
  // Most boost odeint integrators can handle arbitariy state type (see above how fmatvec::Vec is enabled for boost odeint).
  // In this case the following function are defined as noop.
  // If boost odeint requires a special state type then the state is copied.
  inline void assign(Vec &d, const Vec &s) {
    d=s;
  }
  template<typename Dst, typename Src>
  inline void assign(Dst &d, const Src &s) {
    if(static_cast<size_t>(d.size())!=static_cast<size_t>(s.size()))
      d.resize(s.size());
    copy(s.begin(), s.end(), d.begin());
  }

  namespace BoostOdeintHelper {
    // type to define a boost odeint system concept of type "system"
    struct SystemTag {};
    // type to define a boost odeint system concept of type "implicit system"
    struct ImplicitSystemTag {};

    // Helper class to return a object of the boost odeint system concept of type SystemType
    // giving functions for zd and jacobian.
    // Declaration.
    template<class SystemType, class ZdFunc, class JacFunc>
    class BoostOdeintSystem;
    // Explicit spezialization for boost odeint system concept of type "system"
    template<class ZdFunc, class JacFunc>
    class BoostOdeintSystem<BoostOdeintHelper::SystemTag, ZdFunc, JacFunc> {
      public:
        BoostOdeintSystem(const ZdFunc& zdFunc__, const JacFunc&) : zdFunc_(zdFunc__) {}
        const ZdFunc& operator()() const { return zdFunc_; }
      private:
        const ZdFunc& zdFunc_;
    };
    // Explicit spezialization for boost odeint system concept of type "implicit system"
    template<class ZdFunc, class JacFunc>
    class BoostOdeintSystem<BoostOdeintHelper::ImplicitSystemTag, ZdFunc, JacFunc> {
      public:
        BoostOdeintSystem(const ZdFunc& zdFunc__, const JacFunc& jacFunc__) : implFunc_(make_pair(zdFunc__, jacFunc__)) {}
        const pair<ZdFunc, JacFunc>& operator()() const { return implFunc_; }
      private:
        const pair<ZdFunc, JacFunc> implFunc_;
    };
  }

  //! Boost odeint integrator with dense output of type Stepper.
  //! SystemType must define the type of the system concept of Stepper,
  //! which may currently be SystemTag or ImplicitSystemTag.
  template<typename Stepper, typename SystemType>
  class BoostOdeintDOS : public Integrator {
    public:
      void integrate() override;
      void preIntegrate() override;
      void subIntegrate(double tStepEnd) override;
      void postIntegrate() override;

      //! Set initial step size.
      void setInitialStepSize(double dt0_) { dt0=dt0_; }
      //! Set absolute tolerance.
      void setAbsoluteTolerance(double aTol_) { aTol=aTol_; }
      //! Set relative tolerance.
      void setRelativeTolerance(double rTol_) { rTol=rTol_; }
      //! Set maximal step size.
      void setMaximalStepSize(double dtMax_) { dtMax=dtMax_; }
      //! Define wether to trigger a plot before and after each found root.
      void setPlotOnRoot(double plotOnRoot_) { plotOnRoot=plotOnRoot_; }
      //! Set the maximal allowed position drift.
      void setMaximalPositionDrift(double gMax_) { gMax=gMax_; }
      //! Set the maximal allowed velocity drift.
      void setMaximalVelocityDrift(double gdMax_) { gdMax=gdMax_; }

      void initializeUsingXML(DOMElement *element) override;
    protected:
      // Helper function to check if svLast and svStepEnd has a sign change in any element.
      inline bool signChangedWRTsvLast(const Vec &svStepEnd) const;

      // boost odeint style member function calculating zd
      void zd(const typename Stepper::state_type &z, typename Stepper::state_type &zd, const double t);
      // boost odeint style member function calculating the jacobian
      void jac(const typename Stepper::state_type &z, ublas::matrix<double> &jac, const double t, typename Stepper::state_type &ft);
      // boost odeint style std::function's calculating zd
      const function<void(const typename Stepper::state_type&, typename Stepper::state_type&, const double)> zdFunc
        {bind(&BoostOdeintDOS<Stepper, SystemType>::zd, this, _1, _2, _3)};
      // boost odeint style std::function's calculating the jacobian
      const function<void(const typename Stepper::state_type&, ublas::matrix<double>&c, const double, typename Stepper::state_type&)> jacFunc
        {bind(&BoostOdeintDOS<Stepper, SystemType>::jac, this, _1, _2, _3, _4)};
      // Helper member which returns a object of the boost odeint system concept of type SystemType
      // giving functions for zd and jacobian.
      const BoostOdeintHelper::BoostOdeintSystem<SystemType, decltype(zdFunc), decltype(jacFunc)> boostOdeintSystem{zdFunc, jacFunc};

      // typedef of a boost odeint dense output stepper of type Stepper.
      typedef typename odeint::result_of::make_dense_output<Stepper>::type DOSType;
      // the boost odeint dense output stepper.
      unique_ptr<DOSType> dos;

      // variables with corresponding setter functions
      double dt0{1e-10};
      double aTol{1e-6};
      double rTol{1e-6};
      double dtMax{0.1};
      bool plotOnRoot{false};
      double gMax{1e-6};
      double gdMax{1e-6};

      // internal variables
      double tPlot;
      Vec svLast;
      typename Stepper::state_type zTemp;

      // internal variables required for the numerical jacobian calculation
      Vec zDisturbed;
      Vec zd0;
      static const double delta;
      static const double deltaInv;

      // internal variables used for integrator statistics
      size_t nrSteps;
      size_t nrRHS;
      size_t nrJacs;
      size_t nrPlots;
      size_t nrSVs;
      size_t nrRoots;
      size_t nrDriftCorr;
  };

  // initialize static members
  template<typename Stepper, typename SystemType>
  const double BoostOdeintDOS<Stepper, SystemType>::delta=numeric_limits<double>::epsilon();
  template<typename Stepper, typename SystemType>
  const double BoostOdeintDOS<Stepper, SystemType>::deltaInv=1.0/delta;

  // implementation

  template<typename Stepper, typename SystemType>
  void BoostOdeintDOS<Stepper, SystemType>::zd(const typename Stepper::state_type &z, typename Stepper::state_type &zd, const double t) {
    nrRHS++;
    // RHS
    system->setTime(t);
    assign(system->getState(), z);
    system->resetUpToDate();
    assign(zd, system->evalzd());
  }

  template<typename Stepper, typename SystemType>
  void BoostOdeintDOS<Stepper, SystemType>::jac(const typename Stepper::state_type &z, ublas::matrix<double> &jac, const double t, typename Stepper::state_type &ft) {
    nrJacs++;
    // RHS jacobian
    if(static_cast<size_t>(z.size())!=static_cast<size_t>(jac.size1()) ||
       static_cast<size_t>(z.size())!=static_cast<size_t>(jac.size2()))
      jac.resize(z.size(), z.size());
    system->setTime(t);
    assign(system->getState(), z);
    system->resetUpToDate();
    assign(zDisturbed, z);
    zd0=system->evalzd();

    for(size_t i=0; i<static_cast<size_t>(z.size()); ++i) {
      zDisturbed(i)+=delta;
      system->setState(zDisturbed);
      system->resetUpToDate();
      auto col=ublas::column(jac, i);
      auto zd=(system->evalzd()-zd0)*deltaInv;
      copy(zd.begin(), zd.end(), col.begin());
      zDisturbed(i)-=delta;
    }

    system->setTime(t+delta);
    system->setState(zDisturbed);
    system->resetUpToDate();
    assign(ft, (system->evalzd()-zd0)*deltaInv);
  }

  template<typename Stepper, typename SystemType>
  void BoostOdeintDOS<Stepper, SystemType>::integrate() {
    preIntegrate();
    subIntegrate(tEnd);
    postIntegrate();
  }

  template<typename Stepper, typename SystemType>
  void BoostOdeintDOS<Stepper, SystemType>::preIntegrate() {
    debugInit();

    nrSteps=0;
    nrRHS=0;
    nrJacs=0;
    nrPlots=0;
    nrSVs=0;
    nrRoots=0;
    nrDriftCorr=0;

    system->setTime(tStart);

    // get initial state
    if(z0.size()) {
      if(z0.size()!=system->getzSize())
        throw MBSimError("BoostOdeintDOS: size of z0 does not match");
      assign(zTemp, z0);
    }
    else
      assign(zTemp, system->evalz0());

    system->computeInitialCondition();
    nrPlots++;
    system->plot();
    tPlot=tStart+dtPlot;
    nrSVs++;
    svLast=system->evalsv();

    // initialize odeint
    dos.reset(new DOSType(make_dense_output(aTol, rTol, dtMax, Stepper())));
    dos->initialize(zTemp, tStart, dt0);
  }

  template<typename Stepper, typename SystemType>
  void BoostOdeintDOS<Stepper, SystemType>::subIntegrate(double tStepEnd) {
    // loop until at least tStepEnd is reached
    while(dos->current_time()<tStepEnd) {
      // make one step with odeint
      nrSteps++;
      auto step=dos->do_step(boostOdeintSystem());

      // check if a root exists in the current step
      double curTimeAndState=dos->current_time(); // save current time/state as double: just used to avoid unnessesary system updates
      system->setTime(dos->current_time());
      assign(system->getState(), dos->current_state());
      system->resetUpToDate();
      nrSVs++;
      auto shift=signChangedWRTsvLast(system->evalsv());
      // if a root exists in the current step ...
      if(shift) {
        // ... search the first root and set step.second to this time
        double dt=step.second-step.first;
        while(dt>1e-10) {
          dt/=2;
          double tRoot=step.second-dt;
          dos->calc_state(tRoot, zTemp);
          curTimeAndState=tRoot;
          system->setTime(tRoot);
          assign(system->getState(), zTemp);
          system->resetUpToDate();
          nrSVs++;
          if(signChangedWRTsvLast(system->evalsv()))
            step.second=tRoot;
        }
        // root found increase time by dt and set jsv
        step.second+=dt;
        dos->calc_state(step.second, zTemp);
        curTimeAndState=step.second;
        system->setTime(step.second);
        assign(system->getState(), zTemp);
        system->resetUpToDate();
        nrSVs++;
        auto &sv=system->evalsv();
        auto &jsv=system->getjsv();
        for(int i=0; i<sv.size(); ++i)
          jsv(i)=svLast(i)*sv(i)<0;
      }

      // plot every dtPlot up to the end of the current step (the current step end may already be shorted by roots)
      while(tPlot<=step.second && tPlot<tStepEnd) {
        dos->calc_state(tPlot, zTemp);
        if(curTimeAndState!=tPlot) {
          curTimeAndState=tPlot;
          system->setTime(tPlot);
          assign(system->getState(), zTemp);
          system->resetUpToDate();
        }
        nrPlots++;
        system->plot();
        if(output)
          msg(Info)<<"t = "<<tPlot<<", dt="<<dos->current_time_step()<<"\r"<<flush;
        tPlot+=dtPlot;
      }

      if(shift) {
        // shift the system
        dos->calc_state(step.second, zTemp);
        if(curTimeAndState!=step.second) {
          curTimeAndState=step.second;
          system->setTime(step.second);
          assign(system->getState(), zTemp);
          system->resetUpToDate();
        }
        if(plotOnRoot) {
          nrPlots++;
          system->plot();
        }
        nrRoots++;
        system->resetUpToDate();
        system->shift();
        if(plotOnRoot) {
          nrPlots++;
          system->plot();
        }
        nrSVs++;
        svLast=system->evalsv();
        // reinit odeint with new state
        assign(zTemp, system->getState());
        dos->initialize(zTemp, step.second, dos->current_time_step());
      }
      else {
        // check if projection is needed (if a root was found projection is done anyway by shift())
        bool reinitNeeded=false;
        if(system->positionDriftCompensationNeeded(gMax)) {
          system->projectGeneralizedPositions(3);
          system->projectGeneralizedVelocities(3);
          reinitNeeded=true;
        }
        else if(system->velocityDriftCompensationNeeded(gdMax)) {
          system->projectGeneralizedVelocities(3);
          reinitNeeded=true;
        }
        if(reinitNeeded) {
          nrDriftCorr++;
          assign(zTemp, system->getState());
          dos->initialize(zTemp, step.second, dos->current_time_step());
        }
      }
    }
  }

  template<typename Stepper, typename SystemType>
  void BoostOdeintDOS<Stepper, SystemType>::postIntegrate() {
    msg(Info)<<endl;
    msg(Info)<<"Integration statistics:"<<endl;
    msg(Info)<<"nrSteps     = "<<nrSteps<<endl;
    msg(Info)<<"nrRHS       = "<<nrRHS<<endl;
    msg(Info)<<"nrJacs      = "<<nrJacs<<endl;
    msg(Info)<<"nrPlots     = "<<nrPlots<<endl;
    msg(Info)<<"nrSVs       = "<<nrSVs<<endl;
    msg(Info)<<"nrRoots     = "<<nrRoots<<endl;
    msg(Info)<<"nrDriftCorr = "<<nrDriftCorr<<endl;
  }

  template<typename Stepper, typename SystemType>
  bool BoostOdeintDOS<Stepper, SystemType>::signChangedWRTsvLast(const Vec &svStepEnd) const {
    for(int i=0; i<svStepEnd.size(); i++)
      if(svLast(i)*svStepEnd(i)<0)
        return true;
    return false;
  }

  template<typename Stepper, typename SystemType>
  void BoostOdeintDOS<Stepper, SystemType>::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteToleranceScalar");
    if(e) setAbsoluteTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeToleranceScalar");
    if(e) setRelativeTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialStepSize");
    if(e) setInitialStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"maximalStepSize");
    if(e) setMaximalStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"plotOnRoot");
    if(e) setPlotOnRoot(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"maximalPositionDrift");
    if(e) setMaximalPositionDrift(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"maximalVelocityDrift");
    if(e) setMaximalVelocityDrift(E(e)->getText<double>());
  }

}

#endif
