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
#include "boost_odeint_integrator_predef.h"

using namespace fmatvec;
using namespace boost::numeric::odeint;
using namespace MBSimIntegrator::BoostOdeintHelper;

namespace MBSimIntegrator {

// explicit integrators
MBSIM_OBJECTFACTORY_REGISTERCLASSWITHTEMPLATENAME_AND_INSTANTIATE(MBSIMINT, BoostOdeintDOS<RKDOPRI5     >, RKDOPRI5     )
MBSIM_OBJECTFACTORY_REGISTERCLASSWITHTEMPLATENAME_AND_INSTANTIATE(MBSIMINT, BoostOdeintDOS<BulirschStoer>, BulirschStoer)
// not working due to but in boost odeint, see https://github.com/boostorg/odeint/pull/27
// MBSIM_OBJECTFACTORY_REGISTERCLASSWITHTEMPLATENAME_AND_INSTANTIATE(MBSIMINT, BoostOdeintDOS<Euler        >, Euler        )
// implicit integrators
MBSIM_OBJECTFACTORY_REGISTERCLASSWITHTEMPLATENAME_AND_INSTANTIATE(MBSIMINT, BoostOdeintDOS<Rosenbrock4  >, Rosenbrock4  )

}
