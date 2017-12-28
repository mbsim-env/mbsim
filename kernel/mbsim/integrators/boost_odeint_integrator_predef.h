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

// rosenbrock4<double>
#include <boost/numeric/odeint/stepper/rosenbrock4_dense_output.hpp>

namespace MBSimIntegrator {

  namespace BoostOdeintHelper {

    // Definition of concepts of DOS. (see header file BoostOdeintDOS)



    // runge_kutta_dopri5<Vec>
  
    // type definitions
    typedef boost::numeric::odeint::runge_kutta_dopri5<fmatvec::Vec> RKDOPRI5Stepper;
    typedef boost::numeric::odeint::controlled_runge_kutta<RKDOPRI5Stepper> ControlledRK;
    typedef boost::numeric::odeint::dense_output_runge_kutta<ControlledRK> DOSRK;

    // DOS concept for the boost odeint runge_kutta_dopri5<Vec> stepper
    class RKDOPRI5 : public DOSRK {
      public:
        typedef ExplicitSystemTag SystemCategory;
#if BOOST_VERSION >= 106000
        RKDOPRI5(double aTol, double rTol, double dtMax) :
          DOSRK(ControlledRK(ControlledRK::error_checker_type(aTol, rTol), ControlledRK::step_adjuster_type(dtMax), RKDOPRI5Stepper())) {}
#else // boost odeint < 1.60 does not support dtMax
        RKDOPRI5(double aTol, double rTol) :
          DOSRK(ControlledRK(ControlledRK::error_checker_type(aTol, rTol), RKDOPRI5Stepper())) {}
#endif
    };



    // bulirsch_stoer<Vec>

    // type definitions
    typedef boost::numeric::odeint::bulirsch_stoer_dense_out<fmatvec::Vec> DOSBS;

    // DOS concept for the boost odeint bulirsch_stoer_dense_out<Vec> stepper.
    class BulirschStoer : public DOSBS {
      public:
        typedef ExplicitSystemTag SystemCategory;
#if BOOST_VERSION >= 106000
        BulirschStoer(double aTol, double rTol, double dtMax) : DOSBS(aTol, rTol, 1.0, 1.0, dtMax) {}
#endif
        BulirschStoer(double aTol, double rTol) : DOSBS(aTol, rTol) {}
    };



    // rosenbrock4<double>

    // type definitions
    typedef boost::numeric::odeint::rosenbrock4<double> RB4;
    typedef boost::numeric::odeint::rosenbrock4_controller<RB4> ControlledRB4;
    typedef boost::numeric::odeint::rosenbrock4_dense_output<ControlledRB4> DOSRB4;
  
    // DOS concept for the boost odeint rosenbrock4<double> stepper
    class Rosenbrock4 : public DOSRB4 {
      public:
        typedef ImplicitSystemTag SystemCategory;
#if BOOST_VERSION >= 106000
        Rosenbrock4(double aTol, double rTol, double dtMax) : DOSRB4(ControlledRB4(aTol, rTol, dtMax)) {}
#endif
        Rosenbrock4(double aTol, double rTol) : DOSRB4(ControlledRB4(aTol, rTol)) {}
    };

  }

  typedef BoostOdeintDOS<BoostOdeintHelper::RKDOPRI5     > BoostOdeintDOS_RKDOPRI5;
  typedef BoostOdeintDOS<BoostOdeintHelper::BulirschStoer> BoostOdeintDOS_BulirschStoer;
  typedef BoostOdeintDOS<BoostOdeintHelper::Rosenbrock4  > BoostOdeintDOS_Rosenbrock4;

}
