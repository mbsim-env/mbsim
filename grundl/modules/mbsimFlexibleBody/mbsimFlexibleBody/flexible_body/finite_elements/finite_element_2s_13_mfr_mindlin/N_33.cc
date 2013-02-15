/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#include <config.h>

#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  void FiniteElement2s13MFRMindlin::computeN_33() {

    const double &r1 = NodeCoordinates(0);
    const double &phi1 = NodeCoordinates(1);
    const double &r2 = NodeCoordinates(2);
    const double &phi2 = NodeCoordinates(3);

    (*(N_ij[2][2]))(0,0) = (phi1 - phi2) * rho * (0.10e2 * d2 * pow(r1, 0.4e1) + 0.12e2 * pow(r1, 0.3e1) * d1 - 0.4e1 * pow(r1, 0.3e1) * r2 * d2 + 0.15e2 * r1 * r1 * d0 - 0.3e1 * r1 * r1 * r2 * r2 * d2 - 0.6e1 * r1 * r1 * r2 * d1 - 0.4e1 * r1 * r2 * r2 * d1 - 0.10e2 * r1 * r2 * d0 - 0.2e1 * r1 * pow(r2, 0.3e1) * d2 - 0.5e1 * r2 * r2 * d0 - 0.2e1 * pow(r2, 0.3e1) * d1 - pow(r2, 0.4e1) * d2) / 0.180e3;
    (*(N_ij[2][2]))(0,3) = (phi1 - phi2) * rho * (0.2e1 * d2 * pow(r1, 0.4e1) - r1 * r2 * r2 * d1 + 0.5e1 * r1 * r1 * d0 + 0.3e1 * pow(r1, 0.3e1) * d1 + pow(r1, 0.3e1) * r2 * d2 - 0.5e1 * r2 * r2 * d0 + r1 * r1 * r2 * d1 - 0.3e1 * pow(r2, 0.3e1) * d1 - 0.2e1 * pow(r2, 0.4e1) * d2 - r1 * pow(r2, 0.3e1) * d2) / 0.180e3;
    (*(N_ij[2][2]))(0,6) = rho * (0.2e1 * phi1 * d2 * pow(r1, 0.4e1) - phi1 * r1 * r2 * r2 * d1 + 0.5e1 * phi1 * r1 * r1 * d0 + 0.3e1 * phi1 * pow(r1, 0.3e1) * d1 + phi1 * pow(r1, 0.3e1) * r2 * d2 - 0.5e1 * phi1 * r2 * r2 * d0 + phi1 * r1 * r1 * r2 * d1 - 0.3e1 * phi1 * pow(r2, 0.3e1) * d1 - 0.2e1 * phi1 * pow(r2, 0.4e1) * d2 - phi1 * r1 * pow(r2, 0.3e1) * d2 - 0.2e1 * pow(r1, 0.4e1) * d2 * phi2 + r1 * phi2 * r2 * r2 * d1 - 0.5e1 * r1 * r1 * phi2 * d0 - 0.3e1 * pow(r1, 0.3e1) * d1 * phi2 - pow(r1, 0.3e1) * d2 * r2 * phi2 + 0.5e1 * phi2 * r2 * r2 * d0 - r1 * r1 * phi2 * r2 * d1 + 0.3e1 * phi2 * pow(r2, 0.3e1) * d1 + 0.2e1 * d2 * pow(r2, 0.4e1) * phi2 + r1 * d2 * pow(r2, 0.3e1) * phi2) / 0.360e3;
    (*(N_ij[2][2]))(0,9) = (phi1 - phi2) * rho * (0.10e2 * d2 * pow(r1, 0.4e1) + 0.12e2 * pow(r1, 0.3e1) * d1 - 0.4e1 * pow(r1, 0.3e1) * r2 * d2 + 0.15e2 * r1 * r1 * d0 - 0.3e1 * r1 * r1 * r2 * r2 * d2 - 0.6e1 * r1 * r1 * r2 * d1 - 0.4e1 * r1 * r2 * r2 * d1 - 0.10e2 * r1 * r2 * d0 - 0.2e1 * r1 * pow(r2, 0.3e1) * d2 - 0.5e1 * r2 * r2 * d0 - 0.2e1 * pow(r2, 0.3e1) * d1 - pow(r2, 0.4e1) * d2) / 0.360e3;
    (*(N_ij[2][2]))(3,0) = (phi1 - phi2) * rho * (0.2e1 * d2 * pow(r1, 0.4e1) - r1 * r2 * r2 * d1 + 0.5e1 * r1 * r1 * d0 + 0.3e1 * pow(r1, 0.3e1) * d1 + pow(r1, 0.3e1) * r2 * d2 - 0.5e1 * r2 * r2 * d0 + r1 * r1 * r2 * d1 - 0.3e1 * pow(r2, 0.3e1) * d1 - 0.2e1 * pow(r2, 0.4e1) * d2 - r1 * pow(r2, 0.3e1) * d2) / 0.180e3;
    (*(N_ij[2][2]))(3,3) = (phi1 - phi2) * rho * (d2 * pow(r1, 0.4e1) + 0.2e1 * pow(r1, 0.3e1) * d1 + 0.2e1 * pow(r1, 0.3e1) * r2 * d2 + 0.5e1 * r1 * r1 * d0 + 0.4e1 * r1 * r1 * r2 * d1 + 0.3e1 * r1 * r1 * r2 * r2 * d2 + 0.6e1 * r1 * r2 * r2 * d1 + 0.10e2 * r1 * r2 * d0 + 0.4e1 * r1 * pow(r2, 0.3e1) * d2 - 0.10e2 * pow(r2, 0.4e1) * d2 - 0.12e2 * pow(r2, 0.3e1) * d1 - 0.15e2 * r2 * r2 * d0) / 0.180e3;
    (*(N_ij[2][2]))(3,6) = (phi1 - phi2) * rho * (d2 * pow(r1, 0.4e1) + 0.2e1 * pow(r1, 0.3e1) * d1 + 0.2e1 * pow(r1, 0.3e1) * r2 * d2 + 0.5e1 * r1 * r1 * d0 + 0.4e1 * r1 * r1 * r2 * d1 + 0.3e1 * r1 * r1 * r2 * r2 * d2 + 0.6e1 * r1 * r2 * r2 * d1 + 0.10e2 * r1 * r2 * d0 + 0.4e1 * r1 * pow(r2, 0.3e1) * d2 - 0.10e2 * pow(r2, 0.4e1) * d2 - 0.12e2 * pow(r2, 0.3e1) * d1 - 0.15e2 * r2 * r2 * d0) / 0.360e3;
    (*(N_ij[2][2]))(3,9) = rho * (0.2e1 * phi1 * d2 * pow(r1, 0.4e1) - phi1 * r1 * r2 * r2 * d1 + 0.5e1 * phi1 * r1 * r1 * d0 + 0.3e1 * phi1 * pow(r1, 0.3e1) * d1 + phi1 * pow(r1, 0.3e1) * r2 * d2 - 0.5e1 * phi1 * r2 * r2 * d0 + phi1 * r1 * r1 * r2 * d1 - 0.3e1 * phi1 * pow(r2, 0.3e1) * d1 - 0.2e1 * phi1 * pow(r2, 0.4e1) * d2 - phi1 * r1 * pow(r2, 0.3e1) * d2 - 0.2e1 * pow(r1, 0.4e1) * d2 * phi2 + r1 * phi2 * r2 * r2 * d1 - 0.5e1 * r1 * r1 * phi2 * d0 - 0.3e1 * pow(r1, 0.3e1) * d1 * phi2 - pow(r1, 0.3e1) * d2 * r2 * phi2 + 0.5e1 * phi2 * r2 * r2 * d0 - r1 * r1 * phi2 * r2 * d1 + 0.3e1 * phi2 * pow(r2, 0.3e1) * d1 + 0.2e1 * d2 * pow(r2, 0.4e1) * phi2 + r1 * d2 * pow(r2, 0.3e1) * phi2) / 0.360e3;
    (*(N_ij[2][2]))(6,0) = rho * (0.2e1 * phi1 * d2 * pow(r1, 0.4e1) - phi1 * r1 * r2 * r2 * d1 + 0.5e1 * phi1 * r1 * r1 * d0 + 0.3e1 * phi1 * pow(r1, 0.3e1) * d1 + phi1 * pow(r1, 0.3e1) * r2 * d2 - 0.5e1 * phi1 * r2 * r2 * d0 + phi1 * r1 * r1 * r2 * d1 - 0.3e1 * phi1 * pow(r2, 0.3e1) * d1 - 0.2e1 * phi1 * pow(r2, 0.4e1) * d2 - phi1 * r1 * pow(r2, 0.3e1) * d2 - 0.2e1 * pow(r1, 0.4e1) * d2 * phi2 + r1 * phi2 * r2 * r2 * d1 - 0.5e1 * r1 * r1 * phi2 * d0 - 0.3e1 * pow(r1, 0.3e1) * d1 * phi2 - pow(r1, 0.3e1) * d2 * r2 * phi2 + 0.5e1 * phi2 * r2 * r2 * d0 - r1 * r1 * phi2 * r2 * d1 + 0.3e1 * phi2 * pow(r2, 0.3e1) * d1 + 0.2e1 * d2 * pow(r2, 0.4e1) * phi2 + r1 * d2 * pow(r2, 0.3e1) * phi2) / 0.360e3;
    (*(N_ij[2][2]))(6,3) = (phi1 - phi2) * rho * (d2 * pow(r1, 0.4e1) + 0.2e1 * pow(r1, 0.3e1) * d1 + 0.2e1 * pow(r1, 0.3e1) * r2 * d2 + 0.5e1 * r1 * r1 * d0 + 0.4e1 * r1 * r1 * r2 * d1 + 0.3e1 * r1 * r1 * r2 * r2 * d2 + 0.6e1 * r1 * r2 * r2 * d1 + 0.10e2 * r1 * r2 * d0 + 0.4e1 * r1 * pow(r2, 0.3e1) * d2 - 0.10e2 * pow(r2, 0.4e1) * d2 - 0.12e2 * pow(r2, 0.3e1) * d1 - 0.15e2 * r2 * r2 * d0) / 0.360e3;
    (*(N_ij[2][2]))(6,6) = (phi1 - phi2) * rho * (d2 * pow(r1, 0.4e1) + 0.2e1 * pow(r1, 0.3e1) * d1 + 0.2e1 * pow(r1, 0.3e1) * r2 * d2 + 0.5e1 * r1 * r1 * d0 + 0.4e1 * r1 * r1 * r2 * d1 + 0.3e1 * r1 * r1 * r2 * r2 * d2 + 0.6e1 * r1 * r2 * r2 * d1 + 0.10e2 * r1 * r2 * d0 + 0.4e1 * r1 * pow(r2, 0.3e1) * d2 - 0.10e2 * pow(r2, 0.4e1) * d2 - 0.12e2 * pow(r2, 0.3e1) * d1 - 0.15e2 * r2 * r2 * d0) / 0.180e3;
    (*(N_ij[2][2]))(6,9) = (phi1 - phi2) * rho * (0.2e1 * d2 * pow(r1, 0.4e1) - r1 * r2 * r2 * d1 + 0.5e1 * r1 * r1 * d0 + 0.3e1 * pow(r1, 0.3e1) * d1 + pow(r1, 0.3e1) * r2 * d2 - 0.5e1 * r2 * r2 * d0 + r1 * r1 * r2 * d1 - 0.3e1 * pow(r2, 0.3e1) * d1 - 0.2e1 * pow(r2, 0.4e1) * d2 - r1 * pow(r2, 0.3e1) * d2) / 0.180e3;
    (*(N_ij[2][2]))(9,0) = (phi1 - phi2) * rho * (0.10e2 * d2 * pow(r1, 0.4e1) + 0.12e2 * pow(r1, 0.3e1) * d1 - 0.4e1 * pow(r1, 0.3e1) * r2 * d2 + 0.15e2 * r1 * r1 * d0 - 0.3e1 * r1 * r1 * r2 * r2 * d2 - 0.6e1 * r1 * r1 * r2 * d1 - 0.4e1 * r1 * r2 * r2 * d1 - 0.10e2 * r1 * r2 * d0 - 0.2e1 * r1 * pow(r2, 0.3e1) * d2 - 0.5e1 * r2 * r2 * d0 - 0.2e1 * pow(r2, 0.3e1) * d1 - pow(r2, 0.4e1) * d2) / 0.360e3;
    (*(N_ij[2][2]))(9,3) = rho * (0.2e1 * phi1 * d2 * pow(r1, 0.4e1) - phi1 * r1 * r2 * r2 * d1 + 0.5e1 * phi1 * r1 * r1 * d0 + 0.3e1 * phi1 * pow(r1, 0.3e1) * d1 + phi1 * pow(r1, 0.3e1) * r2 * d2 - 0.5e1 * phi1 * r2 * r2 * d0 + phi1 * r1 * r1 * r2 * d1 - 0.3e1 * phi1 * pow(r2, 0.3e1) * d1 - 0.2e1 * phi1 * pow(r2, 0.4e1) * d2 - phi1 * r1 * pow(r2, 0.3e1) * d2 - 0.2e1 * pow(r1, 0.4e1) * d2 * phi2 + r1 * phi2 * r2 * r2 * d1 - 0.5e1 * r1 * r1 * phi2 * d0 - 0.3e1 * pow(r1, 0.3e1) * d1 * phi2 - pow(r1, 0.3e1) * d2 * r2 * phi2 + 0.5e1 * phi2 * r2 * r2 * d0 - r1 * r1 * phi2 * r2 * d1 + 0.3e1 * phi2 * pow(r2, 0.3e1) * d1 + 0.2e1 * d2 * pow(r2, 0.4e1) * phi2 + r1 * d2 * pow(r2, 0.3e1) * phi2) / 0.360e3;
    (*(N_ij[2][2]))(9,6) = (phi1 - phi2) * rho * (0.2e1 * d2 * pow(r1, 0.4e1) - r1 * r2 * r2 * d1 + 0.5e1 * r1 * r1 * d0 + 0.3e1 * pow(r1, 0.3e1) * d1 + pow(r1, 0.3e1) * r2 * d2 - 0.5e1 * r2 * r2 * d0 + r1 * r1 * r2 * d1 - 0.3e1 * pow(r2, 0.3e1) * d1 - 0.2e1 * pow(r2, 0.4e1) * d2 - r1 * pow(r2, 0.3e1) * d2) / 0.180e3;
    (*(N_ij[2][2]))(9,9) = (phi1 - phi2) * rho * (0.10e2 * d2 * pow(r1, 0.4e1) + 0.12e2 * pow(r1, 0.3e1) * d1 - 0.4e1 * pow(r1, 0.3e1) * r2 * d2 + 0.15e2 * r1 * r1 * d0 - 0.3e1 * r1 * r1 * r2 * r2 * d2 - 0.6e1 * r1 * r1 * r2 * d1 - 0.4e1 * r1 * r2 * r2 * d1 - 0.10e2 * r1 * r2 * d0 - 0.2e1 * r1 * pow(r2, 0.3e1) * d2 - 0.5e1 * r2 * r2 * d0 - 0.2e1 * pow(r2, 0.3e1) * d1 - pow(r2, 0.4e1) * d2) / 0.180e3;

  }

}

