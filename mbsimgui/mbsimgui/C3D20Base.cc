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
#include "C3D20Base.h"

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  C3D20Base::C3D20Base() {
    N_[0] = &C3D20Base::N1;
    N_[1] = &C3D20Base::N2;
    N_[2] = &C3D20Base::N3;
    N_[3] = &C3D20Base::N4;
    N_[4] = &C3D20Base::N5;
    N_[5] = &C3D20Base::N6;
    N_[6] = &C3D20Base::N7;
    N_[7] = &C3D20Base::N8;
    N_[8] = &C3D20Base::N9;
    N_[9] = &C3D20Base::N10;
    N_[10] = &C3D20Base::N11;
    N_[11] = &C3D20Base::N12;
    N_[12] = &C3D20Base::N13;
    N_[13] = &C3D20Base::N14;
    N_[14] = &C3D20Base::N15;
    N_[15] = &C3D20Base::N16;
    N_[16] = &C3D20Base::N17;
    N_[17] = &C3D20Base::N18;
    N_[18] = &C3D20Base::N19;
    N_[19] = &C3D20Base::N20;

    dNdq_[0][0] = &C3D20Base::dN1dx;
    dNdq_[0][1] = &C3D20Base::dN1dy;
    dNdq_[0][2] = &C3D20Base::dN1dz;
    dNdq_[1][0] = &C3D20Base::dN2dx;
    dNdq_[1][1] = &C3D20Base::dN2dy;
    dNdq_[1][2] = &C3D20Base::dN2dz;
    dNdq_[2][0] = &C3D20Base::dN3dx;
    dNdq_[2][1] = &C3D20Base::dN3dy;
    dNdq_[2][2] = &C3D20Base::dN3dz;
    dNdq_[3][0] = &C3D20Base::dN4dx;
    dNdq_[3][1] = &C3D20Base::dN4dy;
    dNdq_[3][2] = &C3D20Base::dN4dz;
    dNdq_[4][0] = &C3D20Base::dN5dx;
    dNdq_[4][1] = &C3D20Base::dN5dy;
    dNdq_[4][2] = &C3D20Base::dN5dz;
    dNdq_[5][0] = &C3D20Base::dN6dx;
    dNdq_[5][1] = &C3D20Base::dN6dy;
    dNdq_[5][2] = &C3D20Base::dN6dz;
    dNdq_[6][0] = &C3D20Base::dN7dx;
    dNdq_[6][1] = &C3D20Base::dN7dy;
    dNdq_[6][2] = &C3D20Base::dN7dz;
    dNdq_[7][0] = &C3D20Base::dN8dx;
    dNdq_[7][1] = &C3D20Base::dN8dy;
    dNdq_[7][2] = &C3D20Base::dN8dz;
    dNdq_[8][0] = &C3D20Base::dN9dx;
    dNdq_[8][1] = &C3D20Base::dN9dy;
    dNdq_[8][2] = &C3D20Base::dN9dz;
    dNdq_[9][0] = &C3D20Base::dN10dx;
    dNdq_[9][1] = &C3D20Base::dN10dy;
    dNdq_[9][2] = &C3D20Base::dN10dz;
    dNdq_[10][0] = &C3D20Base::dN11dx;
    dNdq_[10][1] = &C3D20Base::dN11dy;
    dNdq_[10][2] = &C3D20Base::dN11dz;
    dNdq_[11][0] = &C3D20Base::dN12dx;
    dNdq_[11][1] = &C3D20Base::dN12dy;
    dNdq_[11][2] = &C3D20Base::dN12dz;
    dNdq_[12][0] = &C3D20Base::dN13dx;
    dNdq_[12][1] = &C3D20Base::dN13dy;
    dNdq_[12][2] = &C3D20Base::dN13dz;
    dNdq_[13][0] = &C3D20Base::dN14dx;
    dNdq_[13][1] = &C3D20Base::dN14dy;
    dNdq_[13][2] = &C3D20Base::dN14dz;
    dNdq_[14][0] = &C3D20Base::dN15dx;
    dNdq_[14][1] = &C3D20Base::dN15dy;
    dNdq_[14][2] = &C3D20Base::dN15dz;
    dNdq_[15][0] = &C3D20Base::dN16dx;
    dNdq_[15][1] = &C3D20Base::dN16dy;
    dNdq_[15][2] = &C3D20Base::dN16dz;
    dNdq_[16][0] = &C3D20Base::dN17dx;
    dNdq_[16][1] = &C3D20Base::dN17dy;
    dNdq_[16][2] = &C3D20Base::dN17dz;
    dNdq_[17][0] = &C3D20Base::dN18dx;
    dNdq_[17][1] = &C3D20Base::dN18dy;
    dNdq_[17][2] = &C3D20Base::dN18dz;
    dNdq_[18][0] = &C3D20Base::dN19dx;
    dNdq_[18][1] = &C3D20Base::dN19dy;
    dNdq_[18][2] = &C3D20Base::dN19dz;
    dNdq_[19][0] = &C3D20Base::dN20dx;
    dNdq_[19][1] = &C3D20Base::dN20dy;
    dNdq_[19][2] = &C3D20Base::dN20dz;

    NA_[0] = &C3D20Base::NA1;
    NA_[1] = &C3D20Base::NA2;
    NA_[2] = &C3D20Base::NA3;
    NA_[3] = &C3D20Base::NA4;
    NA_[4] = &C3D20Base::NA5;
    NA_[5] = &C3D20Base::NA6;
    NA_[6] = &C3D20Base::NA7;
    NA_[7] = &C3D20Base::NA8;

    dNAdq_[0][0] = &C3D20Base::dNA1dx;
    dNAdq_[0][1] = &C3D20Base::dNA1dy;
    dNAdq_[1][0] = &C3D20Base::dNA2dx;
    dNAdq_[1][1] = &C3D20Base::dNA2dy;
    dNAdq_[2][0] = &C3D20Base::dNA3dx;
    dNAdq_[2][1] = &C3D20Base::dNA3dy;
    dNAdq_[3][0] = &C3D20Base::dNA4dx;
    dNAdq_[3][1] = &C3D20Base::dNA4dy;
    dNAdq_[4][0] = &C3D20Base::dNA5dx;
    dNAdq_[4][1] = &C3D20Base::dNA5dy;
    dNAdq_[5][0] = &C3D20Base::dNA6dx;
    dNAdq_[5][1] = &C3D20Base::dNA6dy;
    dNAdq_[6][0] = &C3D20Base::dNA7dx;
    dNAdq_[6][1] = &C3D20Base::dNA7dy;
    dNAdq_[7][0] = &C3D20Base::dNA8dx;
    dNAdq_[7][1] = &C3D20Base::dNA8dy;

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

    indices.resize(6,vector<int>(4));
    indices[0][0] = 3; indices[0][1] = 2; indices[0][2] = 1; indices[0][3] = 0;
    indices[1][0] = 4; indices[1][1] = 5; indices[1][2] = 6; indices[1][3] = 7;
    indices[2][0] = 1; indices[2][1] = 2; indices[2][2] = 6; indices[2][3] = 5;
    indices[3][0] = 2; indices[3][1] = 3; indices[3][2] = 7; indices[3][3] = 6;
    indices[4][0] = 4; indices[4][1] = 7; indices[4][2] = 3; indices[4][3] = 0;
    indices[5][0] = 0; indices[5][1] = 1; indices[5][2] = 5; indices[5][3] = 4;

    fI[0][0] = 4; fI[0][1] = 3; fI[0][2] = 2; fI[0][3] = 1; fI[0][4] = 11; fI[0][5] = 10; fI[0][6] =  9; fI[0][7] = 12;
    fI[1][0] = 5; fI[1][1] = 6; fI[1][2] = 7; fI[1][3] = 8; fI[1][4] = 13; fI[1][5] = 14; fI[1][6] = 15; fI[1][7] = 16;
    fI[2][0] = 1; fI[2][1] = 2; fI[2][2] = 6; fI[2][3] = 5; fI[2][4] =  9; fI[2][5] = 18; fI[2][6] = 13; fI[2][7] = 17;
    fI[3][0] = 2; fI[3][1] = 3; fI[3][2] = 7; fI[3][3] = 6; fI[3][4] = 10; fI[3][5] = 19; fI[3][6] = 14; fI[3][7] = 18;
    fI[4][0] = 3; fI[4][1] = 4; fI[4][2] = 8; fI[4][3] = 7; fI[4][4] = 11; fI[4][5] = 20; fI[4][6] = 15; fI[4][7] = 19;
    fI[5][0] = 4; fI[5][1] = 1; fI[5][2] = 5; fI[5][3] = 8; fI[5][4] = 12; fI[5][5] = 17; fI[5][6] = 16; fI[5][7] = 20;
  }

}
