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


#include <config.h>
#include "nonsmooth_algebra.h"

using namespace fmatvec;

namespace MBSim {

  double proxCN(const double arg, const double boundary) {
    return ((arg < boundary) ? boundary : arg);
  }

  double proxCT2D(const double arg, const double laNmue) {
    if(fabs(arg)<=laNmue)
      return arg;
    else
      return (arg>=0) ? laNmue : -laNmue;
  }

  Vec proxCT3D(const Vec& arg, const double laNmue) {
    const double fabsLaT = nrm2(arg);
    Vec prox(2,NONINIT);
    if(fabsLaT <=  laNmue)
      prox = arg;
    else
      prox = (laNmue/fabsLaT)*arg;
    return prox;
  }

}
