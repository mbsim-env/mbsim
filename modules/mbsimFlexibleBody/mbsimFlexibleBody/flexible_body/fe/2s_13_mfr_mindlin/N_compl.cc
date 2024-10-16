/* Copyright (C) 2004-2015 MBSim Development Team
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

#include "mbsimFlexibleBody/flexible_body/fe/2s_13_mfr_mindlin.h"

using namespace std;
using namespace fmatvec;

namespace MBSimFlexibleBody {

  void FiniteElement2s13MFRMindlin::computeN_compl() {

    N_compl = new Mat(3,NodeDofs*Nodes,INIT,0.);

    const double &r1 = NodeCoordinates(0);
    const double &phi1 = NodeCoordinates(1);
    const double &r2 = NodeCoordinates(2);
    const double &phi2 = NodeCoordinates(3);

    (*N_compl)(2,0) = (phi1 - phi2) * rho * (0.12e2 * d2 * pow(r1, 0.4e1) - 0.3e1 * pow(r1, 0.3e1) * d2 * r2 + 0.15e2 * pow(r1, 0.3e1) * d1 - 0.5e1 * r1 * r1 * r2 * d1 + 0.20e2 * r1 * r1 * d0 - 0.3e1 * r1 * r1 * d2 * r2 * r2 - 0.10e2 * r1 * r2 * d0 - 0.5e1 * r1 * r2 * r2 * d1 - 0.3e1 * r1 * d2 * pow(r2, 0.3e1) - 0.10e2 * d0 * r2 * r2 - 0.5e1 * d1 * pow(r2, 0.3e1) - 0.3e1 * d2 * pow(r2, 0.4e1)) / 0.120e3;
    (*N_compl)(2,3) = (phi1 - phi2) * rho * (0.3e1 * d2 * pow(r1, 0.4e1) + 0.5e1 * pow(r1, 0.3e1) * d1 + 0.3e1 * pow(r1, 0.3e1) * d2 * r2 + 0.10e2 * r1 * r1 * d0 + 0.5e1 * r1 * r1 * r2 * d1 + 0.3e1 * r1 * r1 * d2 * r2 * r2 + 0.10e2 * r1 * r2 * d0 + 0.5e1 * r1 * r2 * r2 * d1 + 0.3e1 * r1 * d2 * pow(r2, 0.3e1) - 0.15e2 * d1 * pow(r2, 0.3e1) - 0.20e2 * d0 * r2 * r2 - 0.12e2 * d2 * pow(r2, 0.4e1)) / 0.120e3;
    (*N_compl)(2,6) = (phi1 - phi2) * rho * (0.3e1 * d2 * pow(r1, 0.4e1) + 0.5e1 * pow(r1, 0.3e1) * d1 + 0.3e1 * pow(r1, 0.3e1) * d2 * r2 + 0.10e2 * r1 * r1 * d0 + 0.5e1 * r1 * r1 * r2 * d1 + 0.3e1 * r1 * r1 * d2 * r2 * r2 + 0.10e2 * r1 * r2 * d0 + 0.5e1 * r1 * r2 * r2 * d1 + 0.3e1 * r1 * d2 * pow(r2, 0.3e1) - 0.15e2 * d1 * pow(r2, 0.3e1) - 0.20e2 * d0 * r2 * r2 - 0.12e2 * d2 * pow(r2, 0.4e1)) / 0.120e3;
    (*N_compl)(2,9) = (phi1 - phi2) * rho * (0.12e2 * d2 * pow(r1, 0.4e1) - 0.3e1 * pow(r1, 0.3e1) * d2 * r2 + 0.15e2 * pow(r1, 0.3e1) * d1 - 0.5e1 * r1 * r1 * r2 * d1 + 0.20e2 * r1 * r1 * d0 - 0.3e1 * r1 * r1 * d2 * r2 * r2 - 0.10e2 * r1 * r2 * d0 - 0.5e1 * r1 * r2 * r2 * d1 - 0.3e1 * r1 * d2 * pow(r2, 0.3e1) - 0.10e2 * d0 * r2 * r2 - 0.5e1 * d1 * pow(r2, 0.3e1) - 0.3e1 * d2 * pow(r2, 0.4e1)) / 0.120e3;

  }

}

