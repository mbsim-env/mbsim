/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2022 MBSim-Env

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
   */

#include <config.h>
#include "C3D20.h"

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  C3D20::C3D20() {
    N_[0] = &C3D20::N1;
    N_[1] = &C3D20::N2;
    N_[2] = &C3D20::N3;
    N_[3] = &C3D20::N4;
    N_[4] = &C3D20::N5;
    N_[5] = &C3D20::N6;
    N_[6] = &C3D20::N7;
    N_[7] = &C3D20::N8;
    N_[8] = &C3D20::N9;
    N_[9] = &C3D20::N10;
    N_[10] = &C3D20::N11;
    N_[11] = &C3D20::N12;
    N_[12] = &C3D20::N13;
    N_[13] = &C3D20::N14;
    N_[14] = &C3D20::N15;
    N_[15] = &C3D20::N16;
    N_[16] = &C3D20::N17;
    N_[17] = &C3D20::N18;
    N_[18] = &C3D20::N19;
    N_[19] = &C3D20::N20;

    dNdq_[0][0] = &C3D20::dN1dx;
    dNdq_[0][1] = &C3D20::dN1dy;
    dNdq_[0][2] = &C3D20::dN1dz;
    dNdq_[1][0] = &C3D20::dN2dx;
    dNdq_[1][1] = &C3D20::dN2dy;
    dNdq_[1][2] = &C3D20::dN2dz;
    dNdq_[2][0] = &C3D20::dN3dx;
    dNdq_[2][1] = &C3D20::dN3dy;
    dNdq_[2][2] = &C3D20::dN3dz;
    dNdq_[3][0] = &C3D20::dN4dx;
    dNdq_[3][1] = &C3D20::dN4dy;
    dNdq_[3][2] = &C3D20::dN4dz;
    dNdq_[4][0] = &C3D20::dN5dx;
    dNdq_[4][1] = &C3D20::dN5dy;
    dNdq_[4][2] = &C3D20::dN5dz;
    dNdq_[5][0] = &C3D20::dN6dx;
    dNdq_[5][1] = &C3D20::dN6dy;
    dNdq_[5][2] = &C3D20::dN6dz;
    dNdq_[6][0] = &C3D20::dN7dx;
    dNdq_[6][1] = &C3D20::dN7dy;
    dNdq_[6][2] = &C3D20::dN7dz;
    dNdq_[7][0] = &C3D20::dN8dx;
    dNdq_[7][1] = &C3D20::dN8dy;
    dNdq_[7][2] = &C3D20::dN8dz;
    dNdq_[8][0] = &C3D20::dN9dx;
    dNdq_[8][1] = &C3D20::dN9dy;
    dNdq_[8][2] = &C3D20::dN9dz;
    dNdq_[9][0] = &C3D20::dN10dx;
    dNdq_[9][1] = &C3D20::dN10dy;
    dNdq_[9][2] = &C3D20::dN10dz;
    dNdq_[10][0] = &C3D20::dN11dx;
    dNdq_[10][1] = &C3D20::dN11dy;
    dNdq_[10][2] = &C3D20::dN11dz;
    dNdq_[11][0] = &C3D20::dN12dx;
    dNdq_[11][1] = &C3D20::dN12dy;
    dNdq_[11][2] = &C3D20::dN12dz;
    dNdq_[12][0] = &C3D20::dN13dx;
    dNdq_[12][1] = &C3D20::dN13dy;
    dNdq_[12][2] = &C3D20::dN13dz;
    dNdq_[13][0] = &C3D20::dN14dx;
    dNdq_[13][1] = &C3D20::dN14dy;
    dNdq_[13][2] = &C3D20::dN14dz;
    dNdq_[14][0] = &C3D20::dN15dx;
    dNdq_[14][1] = &C3D20::dN15dy;
    dNdq_[14][2] = &C3D20::dN15dz;
    dNdq_[15][0] = &C3D20::dN16dx;
    dNdq_[15][1] = &C3D20::dN16dy;
    dNdq_[15][2] = &C3D20::dN16dz;
    dNdq_[16][0] = &C3D20::dN17dx;
    dNdq_[16][1] = &C3D20::dN17dy;
    dNdq_[16][2] = &C3D20::dN17dz;
    dNdq_[17][0] = &C3D20::dN18dx;
    dNdq_[17][1] = &C3D20::dN18dy;
    dNdq_[17][2] = &C3D20::dN18dz;
    dNdq_[18][0] = &C3D20::dN19dx;
    dNdq_[18][1] = &C3D20::dN19dy;
    dNdq_[18][2] = &C3D20::dN19dz;
    dNdq_[19][0] = &C3D20::dN20dx;
    dNdq_[19][1] = &C3D20::dN20dy;
    dNdq_[19][2] = &C3D20::dN20dz;

    rN[0](0)  = -1; rN[0](1)  = -1; rN[0](2)  = -1;
    rN[1](0)  =  1; rN[1](1)  = -1; rN[1](2)  = -1;
    rN[2](0)  =  1; rN[2](1)  =  1; rN[2](2)  = -1;
    rN[3](0)  = -1; rN[3](1)  =  1; rN[3](2)  = -1;
    rN[4](0)  = -1; rN[4](1)  = -1; rN[4](2)  =  1;
    rN[5](0)  =  1; rN[5](1)  = -1; rN[5](2)  =  1;
    rN[6](0)  =  1; rN[6](1)  =  1; rN[6](2)  =  1;
    rN[7](0)  = -1; rN[7](1)  =  1; rN[7](2)  =  1;
    rN[8](0)  =  0; rN[8](1)  = -1; rN[8](2)  = -1;
    rN[9](0)  =  1; rN[9](1)  =  0; rN[9](2)  = -1;
    rN[10](0) =  0; rN[10](1) =  1; rN[10](2) = -1;
    rN[11](0) = -1; rN[11](1) =  0; rN[11](2) = -1;
    rN[12](0) =  0; rN[12](1) = -1; rN[12](2) =  1;
    rN[13](0) =  1; rN[13](1) =  0; rN[13](2) =  1;
    rN[14](0) =  0; rN[14](1) =  1; rN[14](2) =  1;
    rN[15](0) = -1; rN[15](1) =  0; rN[15](2) =  1;
    rN[16](0) = -1; rN[16](1) = -1; rN[16](2) =  0;
    rN[17](0) =  1; rN[17](1) = -1; rN[17](2) =  0;
    rN[18](0) =  1; rN[18](1) =  1; rN[18](2) =  0;
    rN[19](0) = -1; rN[19](1) =  1; rN[19](2) =  0;

    double d = sqrt(3./5);
    double e = 5./9;
    double f = 8./9;

    rI[0](0) = -d; rI[0](1) = -d;  rI[0](2) = -d;  wI[0] = e*e*e;
    rI[1](0) = -d; rI[1](1) = -d;  rI[1](2) = 0;   wI[1] = e*e*f;
    rI[2](0) = -d; rI[2](1) = -d;  rI[2](2) = d;   wI[2] = e*e*e;
    rI[3](0) = -d; rI[3](1) = 0;   rI[3](2) = -d;  wI[3] = e*f*e;
    rI[4](0) = -d; rI[4](1) = 0;   rI[4](2) = 0;   wI[4] = e*f*f;
    rI[5](0) = -d; rI[5](1) = 0;   rI[5](2) = d;   wI[5] = e*f*e;
    rI[6](0) = -d; rI[6](1) = d;   rI[6](2) = -d;  wI[6] = e*e*e;
    rI[7](0) = -d; rI[7](1) = d;   rI[7](2) = 0;   wI[7] = e*e*f;
    rI[8](0) = -d; rI[8](1) = d;   rI[8](2) = d;   wI[8] = e*e*e;
    rI[9](0) =  0; rI[9](1) = -d;  rI[9](2) = -d;  wI[9] = f*e*e;
    rI[10](0) = 0; rI[10](1) = -d; rI[10](2) = 0;  wI[10] = f*e*f;
    rI[11](0) = 0; rI[11](1) = -d; rI[11](2) = d;  wI[11] = f*e*e;
    rI[12](0) = 0; rI[12](1) = 0;  rI[12](2) = -d; wI[12] = f*f*e;
    rI[13](0) = 0; rI[13](1) = 0;  rI[13](2) = 0;  wI[13] = f*f*f;
    rI[14](0) = 0; rI[14](1) = 0;  rI[14](2) = d;  wI[14] = f*f*e;
    rI[15](0) = 0; rI[15](1) = d;  rI[15](2) = -d; wI[15] = f*e*e;
    rI[16](0) = 0; rI[16](1) = d;  rI[16](2) = 0;  wI[16] = f*e*f;
    rI[17](0) = 0; rI[17](1) = d;  rI[17](2) = d;  wI[17] = f*e*e;
    rI[18](0) = d; rI[18](1) = -d; rI[18](2) = -d; wI[18] = e*e*e;
    rI[19](0) = d; rI[19](1) = -d; rI[19](2) = 0;  wI[19] = e*e*f;
    rI[20](0) = d; rI[20](1) = -d; rI[20](2) = d;  wI[20] = e*e*e;
    rI[21](0) = d; rI[21](1) = 0;  rI[21](2) = -d; wI[21] = e*f*e;
    rI[22](0) = d; rI[22](1) = 0;  rI[22](2) = 0;  wI[22] = e*f*f;
    rI[23](0) = d; rI[23](1) = 0;  rI[23](2) = d;  wI[23] = e*f*e;
    rI[24](0) = d; rI[24](1) = d;  rI[24](2) = -d; wI[24] = e*e*e;
    rI[25](0) = d; rI[25](1) = d;  rI[25](2) = 0;  wI[25] = e*e*f;
    rI[26](0) = d; rI[26](1) = d;  rI[26](2) = d;  wI[26] = e*e*e;

    indices[0][0] = 3; indices[0][1] = 2; indices[0][2] = 1; indices[0][3] = 0;
    indices[1][0] = 4; indices[1][1] = 5; indices[1][2] = 6; indices[1][3] = 7;
    indices[2][0] = 1; indices[2][1] = 2; indices[2][2] = 6; indices[2][3] = 5;
    indices[3][0] = 2; indices[3][1] = 3; indices[3][2] = 7; indices[3][3] = 6;
    indices[4][0] = 4; indices[4][1] = 7; indices[4][2] = 3; indices[4][3] = 0;
    indices[5][0] = 0; indices[5][1] = 1; indices[5][2] = 5; indices[5][3] = 4;
  }

}
