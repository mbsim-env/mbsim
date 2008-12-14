/* Copyright (C) 2006  Roland Zander
 
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
 *   rzander@users.berlios.de
 *
 */

#include <config.h>
#include "contact_utils.h"

namespace MBSim {

  double computeAngleOnUnitCircle(const Vec& r) {
    return r(1)>=0 ? acos(r(0)) : 2*M_PI-acos(r(0));
  }

  Vec computeAnglesOnUnitSphere(const Vec& r) {
    Vec zeta(2,NONINIT);
    double l = sqrt(r(0)*r(0) + r(1)*r(1));
    zeta(0)= r(1)>=0 ? acos(r(0)/l) : 2*M_PI-acos(r(0)/l);
    zeta(1)= asin(r(2));
    return zeta;
  }

}
