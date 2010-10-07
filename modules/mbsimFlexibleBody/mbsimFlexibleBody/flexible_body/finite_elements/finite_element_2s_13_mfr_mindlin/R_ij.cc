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

  void FiniteElement2s13MFRMindlin::computeR_ij() {

    R_ij = new SymMat(3,INIT,0.);

    const double &r1 = NodeCoordinates(0);
    const double &phi1 = NodeCoordinates(1);
    const double &r2 = NodeCoordinates(2);
    const double &phi2 = NodeCoordinates(3);

    (*R_ij)(0,0) = rho * (-0.10e2 * d2 * cos(phi1) * sin(phi1) * pow(r2, 0.6e1) + 0.10e2 * d2 * cos(phi1) * sin(phi1) * pow(r1, 0.6e1) - 0.10e2 * d2 * phi1 * pow(r2, 0.6e1) + 0.10e2 * d2 * phi1 * pow(r1, 0.6e1) + 0.10e2 * d2 * cos(phi2) * sin(phi2) * pow(r2, 0.6e1) - 0.10e2 * d2 * cos(phi2) * sin(phi2) * pow(r1, 0.6e1) + 0.10e2 * d2 * phi2 * pow(r2, 0.6e1) - 0.10e2 * d2 * phi2 * pow(r1, 0.6e1) + 0.12e2 * d1 * cos(phi2) * sin(phi2) * pow(r2, 0.5e1) - 0.12e2 * d1 * cos(phi2) * sin(phi2) * pow(r1, 0.5e1) + 0.12e2 * d1 * phi2 * pow(r2, 0.5e1) - 0.12e2 * d1 * phi2 * pow(r1, 0.5e1) - 0.12e2 * d1 * cos(phi1) * sin(phi1) * pow(r2, 0.5e1) + 0.12e2 * d1 * cos(phi1) * sin(phi1) * pow(r1, 0.5e1) - 0.12e2 * phi1 * d1 * pow(r2, 0.5e1) + 0.12e2 * phi1 * d1 * pow(r1, 0.5e1) - 0.15e2 * d0 * cos(phi1) * sin(phi1) * pow(r2, 0.4e1) + 0.15e2 * d0 * cos(phi1) * sin(phi1) * pow(r1, 0.4e1) - 0.15e2 * phi1 * d0 * pow(r2, 0.4e1) + 0.15e2 * phi1 * d0 * pow(r1, 0.4e1) + 0.15e2 * d0 * cos(phi2) * sin(phi2) * pow(r2, 0.4e1) - 0.15e2 * d0 * cos(phi2) * sin(phi2) * pow(r1, 0.4e1) + 0.15e2 * d0 * phi2 * pow(r2, 0.4e1) - 0.15e2 * d0 * phi2 * pow(r1, 0.4e1)) / 0.120e3;
    (*R_ij)(0,1) = rho * (0.10e2 * pow(cos(phi1), 0.2e1) * d2 * pow(r2, 0.6e1) - 0.10e2 * pow(cos(phi1), 0.2e1) * d2 * pow(r1, 0.6e1) - 0.10e2 * pow(cos(phi2), 0.2e1) * d2 * pow(r2, 0.6e1) + 0.10e2 * pow(cos(phi2), 0.2e1) * d2 * pow(r1, 0.6e1) + 0.12e2 * pow(cos(phi1), 0.2e1) * d1 * pow(r2, 0.5e1) - 0.12e2 * pow(cos(phi1), 0.2e1) * d1 * pow(r1, 0.5e1) - 0.12e2 * pow(cos(phi2), 0.2e1) * d1 * pow(r2, 0.5e1) + 0.12e2 * pow(cos(phi2), 0.2e1) * d1 * pow(r1, 0.5e1) + 0.15e2 * pow(cos(phi1), 0.2e1) * d0 * pow(r2, 0.4e1) - 0.15e2 * pow(cos(phi1), 0.2e1) * d0 * pow(r1, 0.4e1) - 0.15e2 * pow(cos(phi2), 0.2e1) * d0 * pow(r2, 0.4e1) + 0.15e2 * pow(cos(phi2), 0.2e1) * d0 * pow(r1, 0.4e1)) / 0.120e3;
    (*R_ij)(1,1) = -rho * (-0.10e2 * d2 * cos(phi1) * sin(phi1) * pow(r2, 0.6e1) + 0.10e2 * d2 * cos(phi1) * sin(phi1) * pow(r1, 0.6e1) + 0.10e2 * d2 * phi1 * pow(r2, 0.6e1) - 0.10e2 * d2 * phi1 * pow(r1, 0.6e1) + 0.10e2 * d2 * cos(phi2) * sin(phi2) * pow(r2, 0.6e1) - 0.10e2 * d2 * cos(phi2) * sin(phi2) * pow(r1, 0.6e1) - 0.10e2 * d2 * phi2 * pow(r2, 0.6e1) + 0.10e2 * d2 * phi2 * pow(r1, 0.6e1) + 0.12e2 * d1 * cos(phi2) * sin(phi2) * pow(r2, 0.5e1) - 0.12e2 * d1 * cos(phi2) * sin(phi2) * pow(r1, 0.5e1) - 0.12e2 * d1 * phi2 * pow(r2, 0.5e1) + 0.12e2 * d1 * phi2 * pow(r1, 0.5e1) - 0.12e2 * d1 * cos(phi1) * sin(phi1) * pow(r2, 0.5e1) + 0.12e2 * d1 * cos(phi1) * sin(phi1) * pow(r1, 0.5e1) + 0.12e2 * phi1 * d1 * pow(r2, 0.5e1) - 0.12e2 * phi1 * d1 * pow(r1, 0.5e1) - 0.15e2 * d0 * cos(phi1) * sin(phi1) * pow(r2, 0.4e1) + 0.15e2 * d0 * cos(phi1) * sin(phi1) * pow(r1, 0.4e1) + 0.15e2 * phi1 * d0 * pow(r2, 0.4e1) - 0.15e2 * phi1 * d0 * pow(r1, 0.4e1) + 0.15e2 * d0 * cos(phi2) * sin(phi2) * pow(r2, 0.4e1) - 0.15e2 * d0 * cos(phi2) * sin(phi2) * pow(r1, 0.4e1) - 0.15e2 * d0 * phi2 * pow(r2, 0.4e1) + 0.15e2 * d0 * phi2 * pow(r1, 0.4e1)) / 0.120e3;

  }

}

