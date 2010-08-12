/* Copyright (C) 2004-2010 MBSim Development Team
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
 * Contact: thschindler@users.berlios.de
 */

#include <config.h>

#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  void FiniteElement2s13MFRMindlin::computeM_RR() {

    M_RR    = new SymMat(3,INIT,0.);

    const double &r1 = NodeCoordinates(0);
    const double &phi1 = NodeCoordinates(1);
    const double &r2 = NodeCoordinates(2);
    const double &phi2 = NodeCoordinates(3);

    (*M_RR)(0,0) = rho * d2 * phi2 * pow(r2, 0.4e1) / 0.4e1 - rho * d2 * phi2 * pow(r1, 0.4e1) / 0.4e1 - rho * d2 * phi1 * pow(r2, 0.4e1) / 0.4e1 + rho * d2 * phi1 * pow(r1, 0.4e1) / 0.4e1 + rho * d1 * phi2 * pow(r2, 0.3e1) / 0.3e1 - rho * d1 * phi2 * pow(r1, 0.3e1) / 0.3e1 - rho * d1 * phi1 * pow(r2, 0.3e1) / 0.3e1 + rho * d1 * phi1 * pow(r1, 0.3e1) / 0.3e1 + rho * d0 * phi2 * r2 * r2 / 0.2e1 - rho * d0 * phi2 * r1 * r1 / 0.2e1 - rho * d0 * phi1 * r2 * r2 / 0.2e1 + rho * d0 * phi1 * r1 * r1 / 0.2e1;
    (*M_RR)(1,1) = rho * d2 * phi2 * pow(r2, 0.4e1) / 0.4e1 - rho * d2 * phi2 * pow(r1, 0.4e1) / 0.4e1 - rho * d2 * phi1 * pow(r2, 0.4e1) / 0.4e1 + rho * d2 * phi1 * pow(r1, 0.4e1) / 0.4e1 + rho * d1 * phi2 * pow(r2, 0.3e1) / 0.3e1 - rho * d1 * phi2 * pow(r1, 0.3e1) / 0.3e1 - rho * d1 * phi1 * pow(r2, 0.3e1) / 0.3e1 + rho * d1 * phi1 * pow(r1, 0.3e1) / 0.3e1 + rho * d0 * phi2 * r2 * r2 / 0.2e1 - rho * d0 * phi2 * r1 * r1 / 0.2e1 - rho * d0 * phi1 * r2 * r2 / 0.2e1 + rho * d0 * phi1 * r1 * r1 / 0.2e1;
    (*M_RR)(2,2) = rho * d2 * phi2 * pow(r2, 0.4e1) / 0.4e1 - rho * d2 * phi2 * pow(r1, 0.4e1) / 0.4e1 - rho * d2 * phi1 * pow(r2, 0.4e1) / 0.4e1 + rho * d2 * phi1 * pow(r1, 0.4e1) / 0.4e1 + rho * d1 * phi2 * pow(r2, 0.3e1) / 0.3e1 - rho * d1 * phi2 * pow(r1, 0.3e1) / 0.3e1 - rho * d1 * phi1 * pow(r2, 0.3e1) / 0.3e1 + rho * d1 * phi1 * pow(r1, 0.3e1) / 0.3e1 + rho * d0 * phi2 * r2 * r2 / 0.2e1 - rho * d0 * phi2 * r1 * r1 / 0.2e1 - rho * d0 * phi1 * r2 * r2 / 0.2e1 + rho * d0 * phi1 * r1 * r1 / 0.2e1;

  }

}

