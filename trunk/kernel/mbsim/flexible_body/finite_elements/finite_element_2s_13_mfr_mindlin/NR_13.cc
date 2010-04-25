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

#include "mbsim/flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  void FiniteElement2s13MFRMindlin::computeNR_13(const Vec &NodeCoordinates) {

    const double &r1 = NodeCoordinates(0);
    const double &phi1 = NodeCoordinates(1);
    const double &r2 = NodeCoordinates(2);
    const double &phi2 = NodeCoordinates(3);

    NR_ij[0][2](0) = -(0.10e2 * d2 * pow(r1, 0.5e1) + 0.12e2 * pow(r1, 0.4e1) * d1 - 0.2e1 * pow(r1, 0.4e1) * d2 * r2 - 0.3e1 * pow(r1, 0.3e1) * d1 * r2 + 0.15e2 * pow(r1, 0.3e1) * d0 - 0.2e1 * pow(r1, 0.3e1) * d2 * r2 * r2 - 0.5e1 * r1 * r1 * d0 * r2 - 0.3e1 * r1 * r1 * d1 * r2 * r2 - 0.2e1 * r1 * r1 * d2 * pow(r2, 0.3e1) - 0.5e1 * r1 * d0 * r2 * r2 - 0.3e1 * r1 * d1 * pow(r2, 0.3e1) - 0.2e1 * r1 * d2 * pow(r2, 0.4e1) - 0.5e1 * d0 * pow(r2, 0.3e1) - 0.3e1 * d1 * pow(r2, 0.4e1) - 0.2e1 * d2 * pow(r2, 0.5e1)) * rho * (phi2 * sin(phi1) - cos(phi1) - sin(phi1) * phi1 + cos(phi2)) / (phi1 - phi2) / 0.60e2;
    NR_ij[0][2](1) = -(0.10e2 * d2 * pow(r1, 0.6e1) - 0.4e1 * d2 * r2 * pow(r1, 0.5e1) + 0.14e2 * d1 * pow(r1, 0.5e1) - 0.7e1 * d1 * r2 * pow(r1, 0.4e1) + 0.21e2 * d0 * pow(r1, 0.4e1) - 0.4e1 * pow(r1, 0.4e1) * d2 * r2 * r2 - 0.14e2 * d0 * r2 * pow(r1, 0.3e1) - 0.7e1 * pow(r1, 0.3e1) * d1 * r2 * r2 - 0.4e1 * pow(r1, 0.3e1) * d2 * pow(r2, 0.3e1) - 0.14e2 * r1 * r1 * d0 * r2 * r2 - 0.7e1 * r1 * r1 * d1 * pow(r2, 0.3e1) - 0.4e1 * r1 * r1 * d2 * pow(r2, 0.4e1) - 0.14e2 * d0 * r1 * pow(r2, 0.3e1) - 0.7e1 * d1 * r1 * pow(r2, 0.4e1) - 0.4e1 * d2 * r1 * pow(r2, 0.5e1) + 0.10e2 * d2 * pow(r2, 0.6e1) + 0.14e2 * d1 * pow(r2, 0.5e1) + 0.21e2 * d0 * pow(r2, 0.4e1)) * rho * (phi2 * sin(phi1) - cos(phi1) - sin(phi1) * phi1 + cos(phi2)) / (phi1 - phi2) / 0.840e3;
    NR_ij[0][2](2) = -(0.10e2 * d2 * pow(r1, 0.5e1) + 0.12e2 * pow(r1, 0.4e1) * d1 - 0.2e1 * pow(r1, 0.4e1) * d2 * r2 - 0.3e1 * pow(r1, 0.3e1) * d1 * r2 + 0.15e2 * pow(r1, 0.3e1) * d0 - 0.2e1 * pow(r1, 0.3e1) * d2 * r2 * r2 - 0.5e1 * r1 * r1 * d0 * r2 - 0.3e1 * r1 * r1 * d1 * r2 * r2 - 0.2e1 * r1 * r1 * d2 * pow(r2, 0.3e1) - 0.5e1 * r1 * d0 * r2 * r2 - 0.3e1 * r1 * d1 * pow(r2, 0.3e1) - 0.2e1 * r1 * d2 * pow(r2, 0.4e1) - 0.5e1 * d0 * pow(r2, 0.3e1) - 0.3e1 * d1 * pow(r2, 0.4e1) - 0.2e1 * d2 * pow(r2, 0.5e1)) * rho * (-phi2 * cos(phi1) + cos(phi1) * phi1 - 0.2e1 * sin(phi1) - cos(phi2) * phi2 + phi1 * cos(phi2) + 0.2e1 * sin(phi2)) / (phi1 - phi2) / 0.120e3;
    NR_ij[0][2](3) = -(0.2e1 * d2 * pow(r1, 0.5e1) + 0.3e1 * pow(r1, 0.4e1) * d1 + 0.2e1 * pow(r1, 0.4e1) * d2 * r2 + 0.5e1 * pow(r1, 0.3e1) * d0 + 0.3e1 * pow(r1, 0.3e1) * d1 * r2 + 0.2e1 * pow(r1, 0.3e1) * d2 * r2 * r2 + 0.5e1 * r1 * r1 * d0 * r2 + 0.3e1 * r1 * r1 * d1 * r2 * r2 + 0.2e1 * r1 * r1 * d2 * pow(r2, 0.3e1) + 0.5e1 * r1 * d0 * r2 * r2 + 0.3e1 * r1 * d1 * pow(r2, 0.3e1) + 0.2e1 * r1 * d2 * pow(r2, 0.4e1) - 0.12e2 * d1 * pow(r2, 0.4e1) - 0.10e2 * d2 * pow(r2, 0.5e1) - 0.15e2 * d0 * pow(r2, 0.3e1)) * rho * (phi2 * sin(phi1) - cos(phi1) - sin(phi1) * phi1 + cos(phi2)) / (phi1 - phi2) / 0.60e2;
    NR_ij[0][2](4) = (0.10e2 * d2 * pow(r1, 0.6e1) - 0.4e1 * d2 * r2 * pow(r1, 0.5e1) + 0.14e2 * d1 * pow(r1, 0.5e1) - 0.7e1 * d1 * r2 * pow(r1, 0.4e1) + 0.21e2 * d0 * pow(r1, 0.4e1) - 0.4e1 * pow(r1, 0.4e1) * d2 * r2 * r2 - 0.14e2 * d0 * r2 * pow(r1, 0.3e1) - 0.7e1 * pow(r1, 0.3e1) * d1 * r2 * r2 - 0.4e1 * pow(r1, 0.3e1) * d2 * pow(r2, 0.3e1) - 0.14e2 * r1 * r1 * d0 * r2 * r2 - 0.7e1 * r1 * r1 * d1 * pow(r2, 0.3e1) - 0.4e1 * r1 * r1 * d2 * pow(r2, 0.4e1) - 0.14e2 * d0 * r1 * pow(r2, 0.3e1) - 0.7e1 * d1 * r1 * pow(r2, 0.4e1) - 0.4e1 * d2 * r1 * pow(r2, 0.5e1) + 0.10e2 * d2 * pow(r2, 0.6e1) + 0.14e2 * d1 * pow(r2, 0.5e1) + 0.21e2 * d0 * pow(r2, 0.4e1)) * rho * (phi2 * sin(phi1) - cos(phi1) - sin(phi1) * phi1 + cos(phi2)) / (phi1 - phi2) / 0.840e3;
    NR_ij[0][2](5) = -(0.2e1 * d2 * pow(r1, 0.5e1) + 0.3e1 * pow(r1, 0.4e1) * d1 + 0.2e1 * pow(r1, 0.4e1) * d2 * r2 + 0.5e1 * pow(r1, 0.3e1) * d0 + 0.3e1 * pow(r1, 0.3e1) * d1 * r2 + 0.2e1 * pow(r1, 0.3e1) * d2 * r2 * r2 + 0.5e1 * r1 * r1 * d0 * r2 + 0.3e1 * r1 * r1 * d1 * r2 * r2 + 0.2e1 * r1 * r1 * d2 * pow(r2, 0.3e1) + 0.5e1 * r1 * d0 * r2 * r2 + 0.3e1 * r1 * d1 * pow(r2, 0.3e1) + 0.2e1 * r1 * d2 * pow(r2, 0.4e1) - 0.12e2 * d1 * pow(r2, 0.4e1) - 0.10e2 * d2 * pow(r2, 0.5e1) - 0.15e2 * d0 * pow(r2, 0.3e1)) * rho * (-phi2 * cos(phi1) + cos(phi1) * phi1 - 0.2e1 * sin(phi1) - cos(phi2) * phi2 + phi1 * cos(phi2) + 0.2e1 * sin(phi2)) / (phi1 - phi2) / 0.120e3;
    NR_ij[0][2](6) = -(0.2e1 * d2 * pow(r1, 0.5e1) + 0.3e1 * pow(r1, 0.4e1) * d1 + 0.2e1 * pow(r1, 0.4e1) * d2 * r2 + 0.5e1 * pow(r1, 0.3e1) * d0 + 0.3e1 * pow(r1, 0.3e1) * d1 * r2 + 0.2e1 * pow(r1, 0.3e1) * d2 * r2 * r2 + 0.5e1 * r1 * r1 * d0 * r2 + 0.3e1 * r1 * r1 * d1 * r2 * r2 + 0.2e1 * r1 * r1 * d2 * pow(r2, 0.3e1) + 0.5e1 * r1 * d0 * r2 * r2 + 0.3e1 * r1 * d1 * pow(r2, 0.3e1) + 0.2e1 * r1 * d2 * pow(r2, 0.4e1) - 0.12e2 * d1 * pow(r2, 0.4e1) - 0.10e2 * d2 * pow(r2, 0.5e1) - 0.15e2 * d0 * pow(r2, 0.3e1)) * rho * (cos(phi1) + phi1 * sin(phi2) - cos(phi2) - sin(phi2) * phi2) / (phi1 - phi2) / 0.60e2;
    NR_ij[0][2](7) = (0.10e2 * d2 * pow(r1, 0.6e1) - 0.4e1 * d2 * r2 * pow(r1, 0.5e1) + 0.14e2 * d1 * pow(r1, 0.5e1) - 0.7e1 * d1 * r2 * pow(r1, 0.4e1) + 0.21e2 * d0 * pow(r1, 0.4e1) - 0.4e1 * pow(r1, 0.4e1) * d2 * r2 * r2 - 0.14e2 * d0 * r2 * pow(r1, 0.3e1) - 0.7e1 * pow(r1, 0.3e1) * d1 * r2 * r2 - 0.4e1 * pow(r1, 0.3e1) * d2 * pow(r2, 0.3e1) - 0.14e2 * r1 * r1 * d0 * r2 * r2 - 0.7e1 * r1 * r1 * d1 * pow(r2, 0.3e1) - 0.4e1 * r1 * r1 * d2 * pow(r2, 0.4e1) - 0.14e2 * d0 * r1 * pow(r2, 0.3e1) - 0.7e1 * d1 * r1 * pow(r2, 0.4e1) - 0.4e1 * d2 * r1 * pow(r2, 0.5e1) + 0.10e2 * d2 * pow(r2, 0.6e1) + 0.14e2 * d1 * pow(r2, 0.5e1) + 0.21e2 * d0 * pow(r2, 0.4e1)) * rho * (cos(phi1) + phi1 * sin(phi2) - cos(phi2) - sin(phi2) * phi2) / (phi1 - phi2) / 0.840e3;
    NR_ij[0][2](8) = (0.2e1 * d2 * pow(r1, 0.5e1) + 0.3e1 * pow(r1, 0.4e1) * d1 + 0.2e1 * pow(r1, 0.4e1) * d2 * r2 + 0.5e1 * pow(r1, 0.3e1) * d0 + 0.3e1 * pow(r1, 0.3e1) * d1 * r2 + 0.2e1 * pow(r1, 0.3e1) * d2 * r2 * r2 + 0.5e1 * r1 * r1 * d0 * r2 + 0.3e1 * r1 * r1 * d1 * r2 * r2 + 0.2e1 * r1 * r1 * d2 * pow(r2, 0.3e1) + 0.5e1 * r1 * d0 * r2 * r2 + 0.3e1 * r1 * d1 * pow(r2, 0.3e1) + 0.2e1 * r1 * d2 * pow(r2, 0.4e1) - 0.12e2 * d1 * pow(r2, 0.4e1) - 0.10e2 * d2 * pow(r2, 0.5e1) - 0.15e2 * d0 * pow(r2, 0.3e1)) * rho * (-phi2 * cos(phi1) + cos(phi1) * phi1 - 0.2e1 * sin(phi1) - cos(phi2) * phi2 + phi1 * cos(phi2) + 0.2e1 * sin(phi2)) / (phi1 - phi2) / 0.120e3;
    NR_ij[0][2](9) = -(0.10e2 * d2 * pow(r1, 0.5e1) + 0.12e2 * pow(r1, 0.4e1) * d1 - 0.2e1 * pow(r1, 0.4e1) * d2 * r2 - 0.3e1 * pow(r1, 0.3e1) * d1 * r2 + 0.15e2 * pow(r1, 0.3e1) * d0 - 0.2e1 * pow(r1, 0.3e1) * d2 * r2 * r2 - 0.5e1 * r1 * r1 * d0 * r2 - 0.3e1 * r1 * r1 * d1 * r2 * r2 - 0.2e1 * r1 * r1 * d2 * pow(r2, 0.3e1) - 0.5e1 * r1 * d0 * r2 * r2 - 0.3e1 * r1 * d1 * pow(r2, 0.3e1) - 0.2e1 * r1 * d2 * pow(r2, 0.4e1) - 0.5e1 * d0 * pow(r2, 0.3e1) - 0.3e1 * d1 * pow(r2, 0.4e1) - 0.2e1 * d2 * pow(r2, 0.5e1)) * rho * (cos(phi1) + phi1 * sin(phi2) - cos(phi2) - sin(phi2) * phi2) / (phi1 - phi2) / 0.60e2;
    NR_ij[0][2](10) = -(0.10e2 * d2 * pow(r1, 0.6e1) - 0.4e1 * d2 * r2 * pow(r1, 0.5e1) + 0.14e2 * d1 * pow(r1, 0.5e1) - 0.7e1 * d1 * r2 * pow(r1, 0.4e1) + 0.21e2 * d0 * pow(r1, 0.4e1) - 0.4e1 * pow(r1, 0.4e1) * d2 * r2 * r2 - 0.14e2 * d0 * r2 * pow(r1, 0.3e1) - 0.7e1 * pow(r1, 0.3e1) * d1 * r2 * r2 - 0.4e1 * pow(r1, 0.3e1) * d2 * pow(r2, 0.3e1) - 0.14e2 * r1 * r1 * d0 * r2 * r2 - 0.7e1 * r1 * r1 * d1 * pow(r2, 0.3e1) - 0.4e1 * r1 * r1 * d2 * pow(r2, 0.4e1) - 0.14e2 * d0 * r1 * pow(r2, 0.3e1) - 0.7e1 * d1 * r1 * pow(r2, 0.4e1) - 0.4e1 * d2 * r1 * pow(r2, 0.5e1) + 0.10e2 * d2 * pow(r2, 0.6e1) + 0.14e2 * d1 * pow(r2, 0.5e1) + 0.21e2 * d0 * pow(r2, 0.4e1)) * rho * (cos(phi1) + phi1 * sin(phi2) - cos(phi2) - sin(phi2) * phi2) / (phi1 - phi2) / 0.840e3;
    NR_ij[0][2](11) = (0.10e2 * d2 * pow(r1, 0.5e1) + 0.12e2 * pow(r1, 0.4e1) * d1 - 0.2e1 * pow(r1, 0.4e1) * d2 * r2 - 0.3e1 * pow(r1, 0.3e1) * d1 * r2 + 0.15e2 * pow(r1, 0.3e1) * d0 - 0.2e1 * pow(r1, 0.3e1) * d2 * r2 * r2 - 0.5e1 * r1 * r1 * d0 * r2 - 0.3e1 * r1 * r1 * d1 * r2 * r2 - 0.2e1 * r1 * r1 * d2 * pow(r2, 0.3e1) - 0.5e1 * r1 * d0 * r2 * r2 - 0.3e1 * r1 * d1 * pow(r2, 0.3e1) - 0.2e1 * r1 * d2 * pow(r2, 0.4e1) - 0.5e1 * d0 * pow(r2, 0.3e1) - 0.3e1 * d1 * pow(r2, 0.4e1) - 0.2e1 * d2 * pow(r2, 0.5e1)) * rho * (-phi2 * cos(phi1) + cos(phi1) * phi1 - 0.2e1 * sin(phi1) - cos(phi2) * phi2 + phi1 * cos(phi2) + 0.2e1 * sin(phi2)) / (phi1 - phi2) / 0.120e3;

  }

}

