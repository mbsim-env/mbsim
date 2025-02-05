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

#include "root_finding_integrator.h"
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/utils/eps.h>
#include <boost/numeric/odeint/util/is_resizeable.hpp>
#include <boost/numeric/odeint/stepper/stepper_categories.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

namespace boost {
  namespace numeric {
    namespace odeint {
      // Enable fmatvec::Vec, the state type of MBSim, as a boost odeint state type.
      template<>
      struct is_resizeable<fmatvec::Vec> {
        typedef boost::true_type type;
        static const bool value=true;
      };
    }
  }

  // Enable fmatvec::Vec, the state type of MBSim, as a boost odeint state type.
  inline int size(const fmatvec::Vec& v) {
    return v.size();
  }
}

namespace MBSim {

  namespace BoostOdeintHelper {

    // Convert a MBSim state (fmatvec::Vec) to the boost odeint state type.
    // Most boost odeint integrators can handle arbitariy state type (see above how fmatvec::Vec is enabled for boost odeint).
    // In this case the following function are defined as noop.
    // If boost odeint requires a special state type then the state is copied.
    inline void assign(fmatvec::Vec &d, const fmatvec::Vec &s) {
      if(d.size()!=s.size()) d.resize(s.size(),fmatvec::NONINIT);
      d=s;
    }
    template<typename Dst, typename Src>
    inline void assign(Dst &d, const Src &s) {
      if(static_cast<size_t>(d.size())!=static_cast<size_t>(s.size()))
        d.resize(s.size());
      std::copy(s.begin(), s.end(), d.begin());
    }

    // type to define a boost odeint system concept of type "system"
    struct ExplicitSystemTag {};
    // type to define a boost odeint system concept of type "implicit system"
    struct ImplicitSystemTag {};

    // Helper class to return a object of the boost odeint system concept of type SystemCategory
    // giving functions for zd and jacobian.
    // Declaration.
    template<class SystemCategory, class ZdFunc, class JacFunc>
    class BoostOdeintSystem;
    // Explicit spezialization for boost odeint system concept of type "system"
    template<class ZdFunc, class JacFunc>
    class BoostOdeintSystem<ExplicitSystemTag, ZdFunc, JacFunc> {
      public:
        BoostOdeintSystem(const ZdFunc& zdFunc__, const JacFunc&) : zdFunc_(zdFunc__) {}
        const ZdFunc& operator()() const { return zdFunc_; }
      private:
        const ZdFunc& zdFunc_;
    };
    // Explicit spezialization for boost odeint system concept of type "implicit system"
    template<class ZdFunc, class JacFunc>
    class BoostOdeintSystem<ImplicitSystemTag, ZdFunc, JacFunc> {
      public:
        BoostOdeintSystem(const ZdFunc& zdFunc__, const JacFunc& jacFunc__) : implFunc_(make_pair(zdFunc__, jacFunc__)) {}
        const std::pair<ZdFunc, JacFunc>& operator()() const { return implFunc_; }
      private:
        const std::pair<ZdFunc, JacFunc> implFunc_;
    };
  }

  //! Integrator based on any Boost odeint dense output stepper.
  //! DOS must conform to the following concept:
  //! - must be a valid boost odeint dense output stepper
  //! - SystemCategory must be a typedef of either ExplicitSystemTag or ImplicitSystemTag
  //! - UnderlayingStepperCategory must be a typedef of the underlaying stepper category
  //! - DOS(double aTol, double rTol, double dtMax) must be a valid constructor
  template<typename DOS>
  class BoostOdeintDOS : public RootFindingIntegrator {
    protected:
      // flag if the underlaying stepper is controlled
      static constexpr bool isControlled{std::is_base_of<boost::numeric::odeint::controlled_stepper_tag,
                                                         typename DOS::UnderlayingStepperCategory>::value};
      // helper typedef to enable member functions if the underlaying stepper is controlled
      template<typename H>
      using EnableIfControlled=typename std::enable_if<std::is_base_of<boost::numeric::odeint::controlled_stepper_tag,
                                                                       typename H::UnderlayingStepperCategory>::value>::type;

    public:
      void integrate() override;
      void preIntegrate() override;
      void subIntegrate(double tSamplePoint) override;
      void postIntegrate() override;

      //! Set initial step size.
      void setInitialStepSize(double dt0_) { dt0=dt0_; }

      //! Set absolute tolerance.
      template<typename H=DOS, typename=EnableIfControlled<H>>
      void setAbsoluteTolerance(double aTol_) { aTol=aTol_; }

      //! Set relative tolerance.
      template<typename H=DOS, typename=EnableIfControlled<H>>
      void setRelativeTolerance(double rTol_) { rTol=rTol_; }

      //! Set maximum step size.
      template<typename H=DOS, typename=EnableIfControlled<H>>
      void setMaximumStepSize(double dtMax_) { dtMax=dtMax_; }

      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      // boost odeint style member function calculating zd
      void zd(const typename DOS::state_type &z, typename DOS::state_type &zd, const double t);
      // boost odeint style member function calculating the jacobian
      void jac(const typename DOS::state_type &z, boost::numeric::ublas::matrix<double> &jac, const double t,
               typename DOS::state_type &ft);
      // boost odeint style std::function's calculating zd
      const std::function<void(const typename DOS::state_type&, typename DOS::state_type&, const double)> zdFunc
        {bind(&BoostOdeintDOS<DOS>::zd, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)};
      // boost odeint style std::function's calculating the jacobian
      const std::function<void(const typename DOS::state_type&, boost::numeric::ublas::matrix<double>&c, const double,
                               typename DOS::state_type&)> jacFunc
        {bind(&BoostOdeintDOS<DOS>::jac, this, std::placeholders::_1, std::placeholders::_2,
              std::placeholders::_3, std::placeholders::_4)};
      // Helper member which returns a object of the boost odeint system concept of type SystemCategory
      // giving functions for zd and jacobian.
      const BoostOdeintHelper::BoostOdeintSystem<typename DOS::SystemCategory, decltype(zdFunc), decltype(jacFunc)>
        boostOdeintSystem{zdFunc, jacFunc};

      // the boost odeint dense output stepper.
      std::unique_ptr<DOS> dos;

      // variables with corresponding setter functions
      double dt0{1e-10};
      double aTol{1e-6};
      double rTol{1e-6};
      double dtMax{0.1};

      // internal variables
      double tPlot;
      size_t plotSample;
      typename DOS::state_type zTemp;

      // internal variables required for the numerical jacobian calculation
      fmatvec::Vec zDisturbed;

      // internal variables used for integrator statistics
      size_t nrSteps;
      size_t nrRHS;
      size_t nrJacs;
      size_t nrPlots;
      size_t nrSVs;
      size_t nrRoots;
      size_t nrDriftCorr;
  };

  // implementation

  template<typename DOS>
  void BoostOdeintDOS<DOS>::zd(const typename DOS::state_type &z, typename DOS::state_type &zd, const double t) {
    nrRHS++;
    // RHS
    system->setTime(t);
    BoostOdeintHelper::assign(system->getState(), z);
    system->resetUpToDate();
    BoostOdeintHelper::assign(zd, system->evalzd());
    if(dtPlot<0)
      system->plot();
  }

  template<typename DOS>
  void BoostOdeintDOS<DOS>::jac(const typename DOS::state_type &z, boost::numeric::ublas::matrix<double> &jac, const double t,
                                typename DOS::state_type &ft) {
    nrJacs++;
    // RHS jacobian
    if(static_cast<size_t>(z.size())!=static_cast<size_t>(jac.size1()) ||
       static_cast<size_t>(z.size())!=static_cast<size_t>(jac.size2()))
      jac.resize(z.size(), z.size());
    system->setTime(t);
    BoostOdeintHelper::assign(system->getState(), z);
    system->resetUpToDate();
    BoostOdeintHelper::assign(zDisturbed, z);
    fmatvec::Vec zd0=system->evalzd();

    for(size_t i=0; i<static_cast<size_t>(z.size()); ++i) {
      double delta=sqrt(macheps*std::max(1.e-5,abs(zDisturbed(i))));
      double zDisturbediSave=zDisturbed(i);
      zDisturbed(i)+=delta;
      system->setState(zDisturbed);
      system->resetUpToDate();
      auto col=boost::numeric::ublas::column(jac, i);
      auto zd=(system->evalzd()-zd0)/delta;
      std::copy(zd.begin(), zd.end(), col.begin());
      zDisturbed(i)=zDisturbediSave;
    }

    system->setTime(t+MBSim::epsroot);
    system->setState(zDisturbed);
    system->resetUpToDate();
    BoostOdeintHelper::assign(ft, (system->evalzd()-zd0)*MBSim::epsrootInv);
  }

  template<typename DOS>
  void BoostOdeintDOS<DOS>::integrate() {
    preIntegrate();
    subIntegrate(tEnd);
    postIntegrate();
  }

  template<typename DOS>
  void BoostOdeintDOS<DOS>::preIntegrate() {
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
      if(z0.size()!=system->getzSize()+system->getisSize())
        throwError("BoostOdeintDOS:: size of z0 does not match, must be " + std::to_string(system->getzSize()+system->getisSize()));
      BoostOdeintHelper::assign(zTemp, z0(fmatvec::RangeV(0,system->getzSize()-1)));
      system->setInternalState(z0(fmatvec::RangeV(system->getzSize(),z0.size()-1)));
    }
    else
      BoostOdeintHelper::assign(zTemp, system->evalz0());

    BoostOdeintHelper::assign(system->getState(), zTemp);
    system->resetUpToDate();
    system->computeInitialCondition();
    nrPlots++;
    system->plot();
    plotSample=1;
    tPlot=tStart+plotSample*dtPlot;
    nrSVs++;
    svLast <<= system->evalsv();
    BoostOdeintHelper::assign(zTemp, system->getState()); // needed, as computeInitialCondition may change the state

    // initialize odeint
    dos.reset(new DOS(aTol, rTol, dtMax));
    dos->initialize(zTemp, tStart, dt0);
  }

  template<typename DOS>
  void BoostOdeintDOS<DOS>::subIntegrate(double tSamplePoint) {
    // loop until at least tSamplePoint is reached
    static constexpr double oneMinusEps = 1 - std::numeric_limits<double>::epsilon();
    while(dos->current_time()<tSamplePoint*oneMinusEps) {
      // make one step with odeint
      nrSteps++;
#if BOOST_VERSION >= 107100
      // do not make a step which exceeds tSamplePoint
      dos->setDtMax(std::min(dtMax, tSamplePoint-dos->current_time()));
#endif
      auto step=dos->do_step(boostOdeintSystem());

      // check if a root exists in the current step
      double curTimeAndState=dos->current_time(); // save current time/state as double: just used to avoid unnessesary system updates
      system->setTime(dos->current_time());
      BoostOdeintHelper::assign(system->getState(), dos->current_state());
      system->resetUpToDate();
      nrSVs++;
      shift=signChangedWRTsvLast(system->evalsv());
      // if a root exists in the current step ...
      if(shift) {
        // ... search the first root and set step.second to this time
        double dt=step.second-step.first;
        while(dt>dtRoot) {
          dt/=2;
          double tRoot=step.second-dt;
          dos->calc_state(tRoot, zTemp);
          curTimeAndState=tRoot;
          system->setTime(tRoot);
          BoostOdeintHelper::assign(system->getState(), zTemp);
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
        BoostOdeintHelper::assign(system->getState(), zTemp);
        system->resetUpToDate();
        nrSVs++;
        auto &sv=system->evalsv();
        auto &jsv=system->getjsv();
        for(int i=0; i<sv.size(); ++i)
          jsv(i)=svLast(i)*sv(i)<0;
      }

      if(dtPlot>0) {
        // plot every dtPlot up to the end of the current step (the current step end may already be shorted by roots)
        static constexpr double onePlusEps = 1 + std::numeric_limits<double>::epsilon();
        while(tPlot<=step.second*onePlusEps
#if BOOST_VERSION < 107100
              && tPlot<=tSamplePoint*onePlusEps
#endif
             ) {
          dos->calc_state(tPlot, zTemp);
          if(curTimeAndState!=tPlot) {
            curTimeAndState=tPlot;
            system->setTime(tPlot);
            BoostOdeintHelper::assign(system->getState(), zTemp);
            system->resetUpToDate();
          }
          nrPlots++;
          system->plot();
          if(msgAct(Status))
            msg(Status)<<"t = "<<tPlot<<", dt="<<dos->current_time_step()<<"                    "<<std::flush;
          system->updateInternalState();
          plotSample++;
          tPlot=tStart+plotSample*dtPlot;
        }
      }
      else {
        // for dtPlot<=0 plot every integrator step
        nrPlots++;
        system->plot();
        if(msgAct(Status))
          msg(Status)<<"t = "<<step.second<<", dt="<<dos->current_time_step()<<"                    "<<std::flush;
      }

#if BOOST_VERSION < 107100
      if(step.second<tSamplePoint) {
#endif
        if(shift) {
          // shift the system
          dos->calc_state(step.second, zTemp);
          if(curTimeAndState!=step.second) {
            curTimeAndState=step.second;
            system->setTime(step.second);
            BoostOdeintHelper::assign(system->getState(), zTemp);
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
          BoostOdeintHelper::assign(zTemp, system->getState());
          dos->initialize(zTemp, step.second, dos->current_time_step());
        }
        else {
          // curTimeAndState may not be a step.second due to plotting -> reset DSS to this time/state if so
          // (note that due to root finding curTimeAndSTate can also be different to step.second but in this case this code is not reached)
          if((gMax>=0 || gdMax>=0) && curTimeAndState!=step.second) {
            curTimeAndState=step.second;
            system->setTime(step.second);
            BoostOdeintHelper::assign(system->getState(), dos->current_state());
            system->resetUpToDate();
          }
          // check if projection is needed (if a root was found projection is done anyway by shift())
          bool reinitNeeded=false;
          if(gMax>=0 and system->positionDriftCompensationNeeded(gMax)) {
            system->projectGeneralizedPositions(3);
            system->projectGeneralizedVelocities(3);
            reinitNeeded=true;
          }
          else if(gdMax>=0 and system->velocityDriftCompensationNeeded(gdMax)) {
            system->projectGeneralizedVelocities(3);
            reinitNeeded=true;
          }
          if(reinitNeeded) {
            nrDriftCorr++;
            BoostOdeintHelper::assign(zTemp, system->getState());
            dos->initialize(zTemp, step.second, dos->current_time_step());
          }
          system->updateStopVectorParameters();
        }
#if BOOST_VERSION < 107100
      }
      else {
        // TODO: this code block (espezially the dos->initialize) should be avoided since initializting the integrator
        // is expensive and not needed at macro sample points or updates of discrete states.
        // For this the do_step call at the top must be restricted to reach at most tSamplePoint.
        // However, this seem currently not possible with the boost odeint integrators.
        // That's why we use this workaround here which has not the optimal performance.
        dos->calc_state(tSamplePoint, zTemp);
        curTimeAndState=tSamplePoint;
        system->setTime(tSamplePoint);
        BoostOdeintHelper::assign(system->getState(), zTemp);
        system->resetUpToDate();
        dos->initialize(zTemp, tSamplePoint, dos->current_time_step());
        system->updateStopVectorParameters();
      }
#endif

#if BOOST_VERSION >= 107100
      // if the system time or state was changed after do_step (for plotting, root-finding, ...)
      // we need to reset it to the integrator state
      if(curTimeAndState!=dos->current_time()) {
        system->setTime(dos->current_time());
        BoostOdeintHelper::assign(system->getState(), dos->current_state());
        system->resetUpToDate();
      }
#endif
      system->updateInternalState();
    }
  }

  template<typename DOS>
  void BoostOdeintDOS<DOS>::postIntegrate() {
    msg(Info)<<std::endl;
    msg(Info)<<"Integration statistics:"<<std::endl;
    msg(Info)<<"nrSteps     = "<<nrSteps<<std::endl;
    msg(Info)<<"nrRHS       = "<<nrRHS<<std::endl;
    msg(Info)<<"nrJacs      = "<<nrJacs<<std::endl;
    msg(Info)<<"nrPlots     = "<<nrPlots<<std::endl;
    msg(Info)<<"nrSVs       = "<<nrSVs<<std::endl;
    msg(Info)<<"nrRoots     = "<<nrRoots<<std::endl;
    msg(Info)<<"nrDriftCorr = "<<nrDriftCorr<<std::endl;
  }

  namespace {
    // declaration
    template<bool, typename Self>
    struct InitializeUsingXMLControlled;

    // initializeUsingXML only used if isControlled == true
    template<typename Self>
    struct InitializeUsingXMLControlled<true, Self> {
      static void call(Self self, xercesc::DOMElement* element) {
        xercesc::DOMElement *e;

        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"absoluteToleranceScalar");
        if(e) self->setAbsoluteTolerance(MBXMLUtils::E(e)->getText<double>());

        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"relativeToleranceScalar");
        if(e) self->setRelativeTolerance(MBXMLUtils::E(e)->getText<double>());

        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"maximumStepSize");
        if(e) self->setMaximumStepSize(MBXMLUtils::E(e)->getText<double>());
      }
    };

    // initializeUsingXML only used if isControlled == false
    template<typename Self>
    struct InitializeUsingXMLControlled<false, Self> {
      static void call(Self self, xercesc::DOMElement* element) {
        xercesc::DOMElement *e;

        // we are not checking in the XML schema for isControlled -> do it here at runtime

        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"absoluteToleranceScalar");
        if(e) self->throwError("absoluteToleranceScalar element used for an fixed step-size stepper.");

        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"relativeToleranceScalar");
        if(e) self->throwError("relativeToleranceScalar element used for an fixed step-size stepper.");

        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"maximumStepSize");
        if(e) self->throwError("maximumStepSize element used for an fixed step-size stepper.");
      }
    };
  }

  template<typename DOS>
  void BoostOdeintDOS<DOS>::initializeUsingXML(xercesc::DOMElement *element) {
    RootFindingIntegrator::initializeUsingXML(element);
    xercesc::DOMElement *e;

    InitializeUsingXMLControlled<isControlled, decltype(this)>::call(this, element);

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"initialStepSize");
    if(e) setInitialStepSize(MBXMLUtils::E(e)->getText<double>());
  }

}

#endif
