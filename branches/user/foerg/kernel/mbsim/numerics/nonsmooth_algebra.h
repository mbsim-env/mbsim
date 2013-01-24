/* Copyright (C) 2004-2012 MBSim Development Team
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

#ifndef NUMERICS_NONSMOOTH_ALGEBRA_H_
#define NUMERICS_NONSMOOTH_ALGEBRA_H_

#include <fmatvec.h>

namespace MBSim {

  /**
   * \brief computes the prox-function for the set of real numbers that has a lower boundary : )boundary , infinity]
   * \param the argument of the prox-function
   * \param the lower boundary (=0, for no value)
   */
  double proxCN(const double arg, const double boundary=0);

  double proxCT2D(const double arg, const double LaNmue);

  fmatvec::Vec proxCT3D(const fmatvec::Vec& arg, const double laNmue);

}

#endif
