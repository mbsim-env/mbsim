/* Copyright (C) 2017 Markus Friedrich
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
 */

#include "boost_odeint_integrator.h"

// runge_kutta_dopri5<Vec>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/stepper/dense_output_runge_kutta.hpp>

// bulirsch_stoer<Vec>
#include <boost/numeric/odeint/stepper/bulirsch_stoer_dense_out.hpp>

// euler<Vec>
#include <boost/numeric/odeint/stepper/euler.hpp>

// rosenbrock4<double>
#include <boost/numeric/odeint/stepper/rosenbrock4_dense_output.hpp>

namespace MBSim {

  namespace BoostOdeintHelper {

    // Definition of concepts of DOS. (see header file BoostOdeintDOS)



    // runge_kutta_dopri5<Vec>
  
    // type definitions
    typedef boost::numeric::odeint::runge_kutta_dopri5<fmatvec::Vec> RKDOPRI5Stepper;
    typedef boost::numeric::odeint::default_error_checker<double, boost::numeric::odeint::range_algebra,
                                                          boost::numeric::odeint::default_operations> RKDOPRI5Checker;
    class RKDOPRI5Adjuster : public boost::numeric::odeint::default_step_adjuster<double, double> {
      public:
        RKDOPRI5Adjuster(double dtMax) : boost::numeric::odeint::default_step_adjuster<double, double>(dtMax) {}
#if BOOST_VERSION >= 107100
        void setDtMax(double dtMax) { m_max_dt = dtMax; }
#endif
    };
#if BOOST_VERSION >= 107100
    typedef boost::numeric::odeint::controlled_runge_kutta<RKDOPRI5Stepper, RKDOPRI5Checker,
                                                           boost::reference_wrapper<RKDOPRI5Adjuster>> ControlledRK;
#else
    typedef boost::numeric::odeint::controlled_runge_kutta<RKDOPRI5Stepper, RKDOPRI5Checker,
                                                           RKDOPRI5Adjuster> ControlledRK;
#endif
    typedef boost::numeric::odeint::dense_output_runge_kutta<ControlledRK> DOSRK;

    // DOS concept for the boost odeint runge_kutta_dopri5<Vec> stepper
    class RKDOPRI5 : public DOSRK {
      public:
        typedef ExplicitSystemTag SystemCategory;
        typedef typename ControlledRK::stepper_category UnderlayingStepperCategory;
        RKDOPRI5(double aTol, double rTol, double dtMax) :
#if BOOST_VERSION >= 107100
          DOSRK(ControlledRK(RKDOPRI5Checker(aTol, rTol), boost::ref(rkdopri5Adjuster), RKDOPRI5Stepper())), rkdopri5Adjuster(dtMax) {}
#else
          DOSRK(ControlledRK(RKDOPRI5Checker(aTol, rTol), RKDOPRI5Adjuster(dtMax), RKDOPRI5Stepper())) {}
#endif
#if BOOST_VERSION >= 107100
        void setDtMax(double dtMax) { rkdopri5Adjuster.setDtMax(dtMax); }
      private:
        RKDOPRI5Adjuster rkdopri5Adjuster;
#endif
    };



    // bulirsch_stoer<Vec>

    // type definitions
    typedef boost::numeric::odeint::bulirsch_stoer_dense_out<fmatvec::Vec> DOSBS;

    // DOS concept for the boost odeint bulirsch_stoer_dense_out<Vec> stepper.
    class BulirschStoer : public DOSBS {
      public:
        typedef ExplicitSystemTag SystemCategory;
        typedef boost::numeric::odeint::controlled_stepper_tag UnderlayingStepperCategory;
        BulirschStoer(double aTol, double rTol, double dtMax) : DOSBS(aTol, rTol, 1.0, 1.0, dtMax) {}
#if BOOST_VERSION >= 107100
        void setDtMax(double dtMax) { m_max_dt = dtMax; }
#endif
    };



    // euler<Vec>

    // type definitions
    typedef boost::numeric::odeint::euler<fmatvec::Vec> EulerStepper;
    typedef boost::numeric::odeint::dense_output_runge_kutta<EulerStepper> DOSEuler;

    // DOS concept for the boost odeint euler<Vec> stepper
    class Euler : public DOSEuler {
      public:
        typedef ExplicitSystemTag SystemCategory;
        typedef typename EulerStepper::stepper_category UnderlayingStepperCategory;
        Euler(double aTol, double rTol, double dtMax) : DOSEuler() {}
#if BOOST_VERSION >= 107100
        void setDtMax(double) {}
#endif
    };



    // rosenbrock4<double>

    // type definitions
    typedef boost::numeric::odeint::rosenbrock4<double> RB4;
    class ControlledRB4 : public boost::numeric::odeint::rosenbrock4_controller<RB4> {
      public:
        ControlledRB4(double aTol, double rTol, double dtMax) : boost::numeric::odeint::rosenbrock4_controller<RB4>(aTol, rTol, dtMax) {}
#if BOOST_VERSION >= 107100
        void setDtMax(double dtMax) { m_max_dt = dtMax; }
#endif
    };
#if BOOST_VERSION >= 107100
    typedef boost::numeric::odeint::rosenbrock4_dense_output<boost::reference_wrapper<ControlledRB4>> DOSRB4;
#else
    typedef boost::numeric::odeint::rosenbrock4_dense_output<ControlledRB4> DOSRB4;
#endif
  
    // DOS concept for the boost odeint rosenbrock4<double> stepper
    class Rosenbrock4 : public DOSRB4 {
      public:
        typedef ImplicitSystemTag SystemCategory;
        typedef typename ControlledRB4::stepper_category UnderlayingStepperCategory;
#if BOOST_VERSION >= 107100
        Rosenbrock4(double aTol, double rTol, double dtMax) : DOSRB4(boost::ref(controlledRB4)), controlledRB4(aTol, rTol, dtMax) {}
#else
        Rosenbrock4(double aTol, double rTol, double dtMax) : DOSRB4(ControlledRB4(aTol, rTol, dtMax)) {}
#endif
#if BOOST_VERSION >= 107100
        void setDtMax(double dtMax) { controlledRB4.setDtMax(dtMax); }
      private:
        ControlledRB4 controlledRB4;
#endif
    };

  }

  // explicit integrators
  typedef BoostOdeintDOS<BoostOdeintHelper::RKDOPRI5     > BoostOdeintDOS_RKDOPRI5;
  typedef BoostOdeintDOS<BoostOdeintHelper::BulirschStoer> BoostOdeintDOS_BulirschStoer;
  typedef BoostOdeintDOS<BoostOdeintHelper::Euler        > BoostOdeintDOS_Euler;
  // implicit integrators
  typedef BoostOdeintDOS<BoostOdeintHelper::Rosenbrock4  > BoostOdeintDOS_Rosenbrock4;

}
