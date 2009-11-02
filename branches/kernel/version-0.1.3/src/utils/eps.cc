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
#include "eps.h"
#include <cmath>

namespace MBSim {

  double macheps() {

    static double eps = 0.;
    static bool alreadyCalculated = false;

    if (!alreadyCalculated) {
      for (eps = 1.; (1.0 + eps)>1.0; )
	eps *= 0.5;
      eps *= 2.0;
      alreadyCalculated = true;
    }
    return eps;
  }

  double epsroot() { 

    static double epsroot = 0.;
    static bool alreadyCalculated = false;

    if (!alreadyCalculated) {
      epsroot = sqrt(macheps());
      alreadyCalculated = true;
    }
    return epsroot;
  }

}
