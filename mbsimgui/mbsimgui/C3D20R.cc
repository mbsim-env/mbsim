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
#include "C3D20R.h"

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  C3D20R::C3D20R() {

    double d = sqrt(1./3);

    rI[0](0) = -d; rI[0](1) = -d;  rI[0](2) = -d;  wI[0] = 1;
    rI[1](0) =  d; rI[1](1) = -d;  rI[1](2) = -d;  wI[1] = 1;
    rI[2](0) = -d; rI[2](1) =  d;  rI[2](2) = -d;  wI[2] = 1;
    rI[3](0) =  d; rI[3](1) =  d;  rI[3](2) = -d;  wI[3] = 1;
    rI[4](0) = -d; rI[4](1) = -d;  rI[4](2) =  d;  wI[4] = 1;
    rI[5](0) =  d; rI[5](1) = -d;  rI[5](2) =  d;  wI[5] = 1;
    rI[6](0) = -d; rI[6](1) =  d;  rI[6](2) =  d;  wI[6] = 1;
    rI[7](0) =  d; rI[7](1) =  d;  rI[7](2) =  d;  wI[7] = 1;

    rAI[0](0) = -d; rAI[0](1) = -d;  wAI[0] = 1;
    rAI[1](0) =  d; rAI[1](1) = -d;  wAI[1] = 1;
    rAI[2](0) = -d; rAI[2](1) =  d;  wAI[2] = 1;
    rAI[3](0) =  d; rAI[3](1) =  d;  wAI[3] = 1;

    A[0][0] =  2.549;   A[0][1] = -0.683;   A[0][2] =  0.183;   A[0][3] = -0.683;   A[0][4] = -0.683;   A[0][5] =  0.183;   A[0][6] = -0.04904; A[0][7] =  0.183;
    A[1][0] = -0.683;   A[1][1] =  2.549;   A[1][2] = -0.683;   A[1][3] =  0.183;   A[1][4] =  0.183;   A[1][5] = -0.683;   A[1][6] =  0.183;   A[1][7] = -0.04904;
    A[2][0] = -0.683;   A[2][1] =  0.183;   A[2][2] = -0.683;   A[2][3] =  2.549;   A[2][4] =  0.183;   A[2][5] = -0.04904; A[2][6] =  0.183;   A[2][7] = -0.683;
    A[3][0] =  0.183;   A[3][1] = -0.683;   A[3][2] =  2.549;   A[3][3] = -0.683;   A[3][4] = -0.04904; A[3][5] =  0.183;   A[3][6] = -0.683;   A[3][7] =  0.183;
    A[4][0] = -0.683;   A[4][1] =  0.183;   A[4][2] = -0.04904; A[4][3] =  0.183;   A[4][4] =  2.549;   A[4][5] = -0.683;   A[4][6] =  0.183;   A[4][7] = -0.683;
    A[5][0] =  0.183;   A[5][1] = -0.683;   A[5][2] =  0.183;   A[5][3] = -0.04904; A[5][4] = -0.683;   A[5][5] =  2.549;   A[5][6] = -0.683;   A[5][7] =  0.183;
    A[6][0] =  0.183;   A[6][1] = -0.04904; A[6][2] =  0.183;   A[6][3] = -0.683;   A[6][4] = -0.683;   A[6][5] =  0.183;   A[6][6] = -0.683;   A[6][7] =  2.549;
    A[7][0] = -0.04904; A[7][1] =  0.183;   A[7][2] = -0.683;   A[7][3] =  0.183;   A[7][4] =  0.183;   A[7][5] = -0.683;   A[7][6] =  2.549;   A[7][7] = -0.683;

    B[0][0] = 0; B[0][1] = 1;
    B[1][0] = 1; B[1][1] = 2;
    B[2][0] = 2; B[2][1] = 3;
    B[3][0] = 3; B[3][1] = 0;
    B[4][0] = 4; B[4][1] = 5;
    B[5][0] = 5; B[5][1] = 6;
    B[6][0] = 6; B[6][1] = 7;
    B[7][0] = 7; B[7][1] = 4;
    B[8][0] = 0; B[8][1] = 4;
    B[9][0] = 1; B[9][1] = 5;
    B[10][0] = 2; B[10][1] = 6;
    B[11][0] = 3; B[11][1] = 7;
  }

}
