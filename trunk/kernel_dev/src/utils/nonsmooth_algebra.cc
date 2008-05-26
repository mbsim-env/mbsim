/* Copyright (C) 2004-2006  Martin Förg
 
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
 *   mfoerg@users.berlios.de
 *
 */
#include <config.h>
#include "nonsmooth_algebra.h"

namespace MBSim {

  double proxCN(double arg) {
    return (arg < 0) ? 0 : arg;
  }

  double proxCT2D(double arg, double laNmue) {
    if(fabs(arg)<=laNmue)
      return arg;
    else 
      return (arg>=0) ? laNmue : -laNmue;
  }

  Vec proxCT3D(const Vec& arg, double laNmue) {
    Vec prox(2,NONINIT);
    double fabsLaT = nrm2(arg);
    if(fabsLaT <=  laNmue) 
      prox = arg;
    else 
      prox = (laNmue/fabsLaT)*arg;
    return prox;
  }

  double proxCN(double arg,double boundary) {
    return ((arg < boundary) ? boundary : arg);
  }

}
