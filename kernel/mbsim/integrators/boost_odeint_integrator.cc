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

#include <config.h>
#include "boost_odeint_integrator.h"
#include "../objectfactory.h"

#include <boost/numeric/odeint.hpp>
#include <boost/preprocessor/punctuation/comma.hpp>

using namespace std;
using namespace boost::numeric::odeint;

namespace boost {
  namespace numeric {
    namespace odeint {

      // Enable bulirsch_stoer as make_dense_output generator.
      template<class State>
      struct get_dense_output<bulirsch_stoer<State>>
      {
        typedef bulirsch_stoer_dense_out<State> type;
      };
      template<class State>
      struct dense_output_factory<bulirsch_stoer<State>, bulirsch_stoer_dense_out<State>> {
        bulirsch_stoer_dense_out<State> operator()(double aTol, double rTol, const bulirsch_stoer<State>&) const {
          return bulirsch_stoer_dense_out<State>(aTol, rTol);
        }
        bulirsch_stoer_dense_out<State> operator()(double aTol, double rTol, double dtMax, const bulirsch_stoer<State>&) const {
          return bulirsch_stoer_dense_out<State>(aTol, rTol, 1.0, 1.0, dtMax);
        }
      };

    }
  }
}

namespace MBSimIntegrator {

// explicit integrators
MBSIM_OBJECTFACTORY_REGISTERCLASSWITHTEMPLATENAME_AND_INSTANTIATE(
  MBSIMINT, BoostOdeintDOS<runge_kutta_dopri5<fmatvec::Vec> BOOST_PP_COMMA() BoostOdeintHelper::SystemTag>, RKDOPRI5     )
MBSIM_OBJECTFACTORY_REGISTERCLASSWITHTEMPLATENAME_AND_INSTANTIATE(
  MBSIMINT, BoostOdeintDOS<bulirsch_stoer<fmatvec::Vec> BOOST_PP_COMMA() BoostOdeintHelper::SystemTag>, BulirschStoer)
// implicit integrators
MBSIM_OBJECTFACTORY_REGISTERCLASSWITHTEMPLATENAME_AND_INSTANTIATE(
  MBSIMINT, BoostOdeintDOS<rosenbrock4<double> BOOST_PP_COMMA() BoostOdeintHelper::ImplicitSystemTag>, Rosenbrock4)

}
