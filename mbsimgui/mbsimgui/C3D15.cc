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
#include "C3D15.h"

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  C3D15::C3D15() {
    N_[0] = &C3D15::N1;
    N_[1] = &C3D15::N2;
    N_[2] = &C3D15::N3;
    N_[3] = &C3D15::N4;
    N_[4] = &C3D15::N5;
    N_[5] = &C3D15::N6;
    N_[6] = &C3D15::N7;
    N_[7] = &C3D15::N8;
    N_[8] = &C3D15::N9;
    N_[9] = &C3D15::N10;
    N_[10] = &C3D15::N11;
    N_[11] = &C3D15::N12;
    N_[12] = &C3D15::N13;
    N_[13] = &C3D15::N14;
    N_[14] = &C3D15::N15;

    dNdq_[0][0] = &C3D15::dN1dx;
    dNdq_[0][1] = &C3D15::dN1dy;
    dNdq_[0][2] = &C3D15::dN1dz;
    dNdq_[1][0] = &C3D15::dN2dx;
    dNdq_[1][1] = &C3D15::dN2dy;
    dNdq_[1][2] = &C3D15::dN2dz;
    dNdq_[2][0] = &C3D15::dN3dx;
    dNdq_[2][1] = &C3D15::dN3dy;
    dNdq_[2][2] = &C3D15::dN3dz;
    dNdq_[3][0] = &C3D15::dN4dx;
    dNdq_[3][1] = &C3D15::dN4dy;
    dNdq_[3][2] = &C3D15::dN4dz;
    dNdq_[4][0] = &C3D15::dN5dx;
    dNdq_[4][1] = &C3D15::dN5dy;
    dNdq_[4][2] = &C3D15::dN5dz;
    dNdq_[5][0] = &C3D15::dN6dx;
    dNdq_[5][1] = &C3D15::dN6dy;
    dNdq_[5][2] = &C3D15::dN6dz;
    dNdq_[6][0] = &C3D15::dN7dx;
    dNdq_[6][1] = &C3D15::dN7dy;
    dNdq_[6][2] = &C3D15::dN7dz;
    dNdq_[7][0] = &C3D15::dN8dx;
    dNdq_[7][1] = &C3D15::dN8dy;
    dNdq_[7][2] = &C3D15::dN8dz;
    dNdq_[8][0] = &C3D15::dN9dx;
    dNdq_[8][1] = &C3D15::dN9dy;
    dNdq_[8][2] = &C3D15::dN9dz;
    dNdq_[9][0] = &C3D15::dN10dx;
    dNdq_[9][1] = &C3D15::dN10dy;
    dNdq_[9][2] = &C3D15::dN10dz;
    dNdq_[10][0] = &C3D15::dN11dx;
    dNdq_[10][1] = &C3D15::dN11dy;
    dNdq_[10][2] = &C3D15::dN11dz;
    dNdq_[11][0] = &C3D15::dN12dx;
    dNdq_[11][1] = &C3D15::dN12dy;
    dNdq_[11][2] = &C3D15::dN12dz;
    dNdq_[12][0] = &C3D15::dN13dx;
    dNdq_[12][1] = &C3D15::dN13dy;
    dNdq_[12][2] = &C3D15::dN13dz;
    dNdq_[13][0] = &C3D15::dN14dx;
    dNdq_[13][1] = &C3D15::dN14dy;
    dNdq_[13][2] = &C3D15::dN14dz;
    dNdq_[14][0] = &C3D15::dN15dx;
    dNdq_[14][1] = &C3D15::dN15dy;
    dNdq_[14][2] = &C3D15::dN15dz;

    rN[0](0) = 0;   rN[0](1) = 0;   rN[0](2) = 0;
    rN[1](0) = 1;   rN[1](1) = 0;   rN[1](2) = 0;
    rN[2](0) = 0;   rN[2](1) = 1;   rN[2](2) = 0;
    rN[3](0) = 0;   rN[3](1) = 0;   rN[3](2) = 1;
    rN[4](0) = 0.5; rN[4](1) = 0;   rN[4](2) = 0;
    rN[5](0) = 0.5; rN[5](1) = 0.5; rN[5](2) = 0;
    rN[6](0) = 0;   rN[6](1) = 0.5; rN[6](2) = 0;
    rN[7](0) = 0;   rN[7](1) = 0;   rN[7](2) = 0.5;
    rN[8](0) = 0.5; rN[8](1) = 0;   rN[8](2) = 0.5;
    rN[9](0) =  0;  rN[9](1) = 0.5; rN[9](2) = 0.5;

    double d = 1./6;
    double e = sqrt(3./5);
    double f = 2./3;
    double g = 5./54;
    double h = 4./27;

    rI[0](0) = d; rI[0](1) = d; rI[0](2) = -e; wI[0] = g;
    rI[1](0) = f; rI[1](1) = d; rI[1](2) = -e; wI[1] = g;
    rI[2](0) = d; rI[2](1) = f; rI[2](2) = -e; wI[2] = g;
    rI[3](0) = d; rI[3](1) = d; rI[3](2) =  0; wI[3] = h;
    rI[4](0) = f; rI[4](1) = d; rI[4](2) =  0; wI[4] = h;
    rI[5](0) = d; rI[5](1) = f; rI[5](2) =  0; wI[5] = h;
    rI[6](0) = d; rI[6](1) = d; rI[6](2) =  e; wI[6] = g;
    rI[7](0) = f; rI[7](1) = d; rI[7](2) =  e; wI[7] = g;
    rI[8](0) = d; rI[8](1) = f; rI[8](2) =  e; wI[8] = g;

    d = 0.166666666666667;
    e = 0.666666666666667;

    rAI[0][0](0) = d; rAI[0][0](1) = d;  wAI[0][0] = d;
    rAI[0][1](0) = e; rAI[0][1](1) = d;  wAI[0][1] = d;
    rAI[0][2](0) = d; rAI[0][2](1) = e;  wAI[0][2] = d;

    d = sqrt(1./3);

    rAI[1][0](0) = -d; rAI[1][0](1) = -d;  wAI[1][0] = 1;
    rAI[1][1](0) =  d; rAI[1][1](1) = -d;  wAI[1][1] = 1;
    rAI[1][2](0) = -d; rAI[1][2](1) =  d;  wAI[1][2] = 1;
    rAI[1][3](0) =  d; rAI[1][3](1) =  d;  wAI[1][3] = 1;

    A[0][0] =  1.63138; A[0][1] = -0.32628; A[0][2] = -0.32628; A[0][3] = -0.52027; A[0][4] =  0.10405; A[0][5] =  0.10405;
    A[1][0] = -0.32628; A[1][1] =  1.63138; A[1][2] = -0.32628; A[1][3] =  0.10405; A[1][4] = -0.52027; A[1][5] =  0.10405;
    A[2][0] = -0.32628; A[2][1] = -0.32628; A[2][2] =  1.63138; A[2][3] =  0.10405; A[2][4] =  0.10405; A[2][5] = -0.52027;
    A[3][0] =  0.55556; A[3][1] = -0.11111; A[3][2] = -0.11111; A[3][3] =  0.55556; A[3][4] = -0.11111; A[3][5] = -0.11111;
    A[4][0] = -0.11111; A[4][1] =  0.55556; A[4][2] = -0.11111; A[4][3] = -0.11111; A[4][4] =  0.55556; A[4][5] = -0.11111;
    A[5][0] = -0.11111; A[5][1] = -0.11111; A[5][2] =  0.55556; A[5][3] = -0.11111; A[5][4] = -0.11111; A[5][5] =  0.55556;
    A[6][0] = -0.52027; A[6][1] =  0.10405; A[6][2] =  0.10405; A[6][3] =  1.63138; A[6][4] = -0.32628; A[6][5] = -0.32628;
    A[7][0] =  0.10405; A[7][1] = -0.52027; A[7][2] =  0.10405; A[7][3] = -0.32628; A[7][4] =  1.63138; A[7][5] = -0.32628;
    A[8][0] =  0.10405; A[8][1] =  0.10405; A[8][2] = -0.52027; A[8][3] = -0.32628; A[8][4] = -0.32628; A[8][5] =  1.63138;

    B[0][0] = 0; B[0][1] = 1;
    B[1][0] = 1; B[1][1] = 2;
    B[2][0] = 2; B[2][1] = 0;
    B[3][0] = 3; B[3][1] = 4;
    B[4][0] = 4; B[4][1] = 5;
    B[5][0] = 5; B[5][1] = 3;
    B[6][0] = 0; B[6][1] = 3;
    B[7][0] = 1; B[7][1] = 4;
    B[8][0] = 2; B[8][1] = 5;

    indices.resize(5);
    indices[0].resize(4);
    indices[1].resize(4);
    indices[2].resize(4);
    indices[3].resize(3);
    indices[4].resize(3);
    indices[0][0] = 0; indices[0][1] = 1; indices[0][2] = 4; indices[0][3] = 3;
    indices[1][0] = 0; indices[1][1] = 3; indices[1][2] = 5; indices[1][3] = 2;
    indices[2][0] = 2; indices[2][1] = 5; indices[2][2] = 4; indices[2][3] = 1;
    indices[3][0] = 0; indices[3][1] = 2; indices[3][2] = 1;
    indices[4][0] = 3; indices[4][1] = 4; indices[4][2] = 5;

    fI[0][0] = 1; fI[0][1] = 3; fI[0][2] = 2; fI[0][3] =  9; fI[0][4] =  8; fI[0][5] =  7; fI[0][6] =  0; fI[0][7] =  0;
    fI[1][0] = 4; fI[1][1] = 5; fI[1][2] = 6; fI[1][3] = 10; fI[1][4] = 11; fI[1][5] = 12; fI[1][6] =  0; fI[1][7] =  0;
    fI[2][0] = 1; fI[2][1] = 2; fI[2][2] = 5; fI[2][3] =  4; fI[2][4] =  7; fI[2][5] = 14; fI[2][6] = 10; fI[2][7] = 13;
    fI[3][0] = 2; fI[3][1] = 3; fI[3][2] = 6; fI[3][3] =  5; fI[3][4] =  8; fI[3][5] = 15; fI[3][6] = 11; fI[3][7] = 14;
    fI[4][0] = 4; fI[4][1] = 6; fI[4][2] = 3; fI[4][3] =  1; fI[4][4] = 12; fI[4][5] = 15; fI[4][6] =  9; fI[4][7] = 13;
  }

}
