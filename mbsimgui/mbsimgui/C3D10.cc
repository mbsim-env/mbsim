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
#include "C3D10.h"

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  C3D10::C3D10() {
    N_[0] = &C3D10::N1;
    N_[1] = &C3D10::N2;
    N_[2] = &C3D10::N3;
    N_[3] = &C3D10::N4;
    N_[4] = &C3D10::N5;
    N_[5] = &C3D10::N6;
    N_[6] = &C3D10::N7;
    N_[7] = &C3D10::N8;
    N_[8] = &C3D10::N9;
    N_[9] = &C3D10::N10;

    dNdq_[0][0] = &C3D10::dN1dx;
    dNdq_[0][1] = &C3D10::dN1dy;
    dNdq_[0][2] = &C3D10::dN1dz;
    dNdq_[1][0] = &C3D10::dN2dx;
    dNdq_[1][1] = &C3D10::dN2dy;
    dNdq_[1][2] = &C3D10::dN2dz;
    dNdq_[2][0] = &C3D10::dN3dx;
    dNdq_[2][1] = &C3D10::dN3dy;
    dNdq_[2][2] = &C3D10::dN3dz;
    dNdq_[3][0] = &C3D10::dN4dx;
    dNdq_[3][1] = &C3D10::dN4dy;
    dNdq_[3][2] = &C3D10::dN4dz;
    dNdq_[4][0] = &C3D10::dN5dx;
    dNdq_[4][1] = &C3D10::dN5dy;
    dNdq_[4][2] = &C3D10::dN5dz;
    dNdq_[5][0] = &C3D10::dN6dx;
    dNdq_[5][1] = &C3D10::dN6dy;
    dNdq_[5][2] = &C3D10::dN6dz;
    dNdq_[6][0] = &C3D10::dN7dx;
    dNdq_[6][1] = &C3D10::dN7dy;
    dNdq_[6][2] = &C3D10::dN7dz;
    dNdq_[7][0] = &C3D10::dN8dx;
    dNdq_[7][1] = &C3D10::dN8dy;
    dNdq_[7][2] = &C3D10::dN8dz;
    dNdq_[8][0] = &C3D10::dN9dx;
    dNdq_[8][1] = &C3D10::dN9dy;
    dNdq_[8][2] = &C3D10::dN9dz;
    dNdq_[9][0] = &C3D10::dN10dx;
    dNdq_[9][1] = &C3D10::dN10dy;
    dNdq_[9][2] = &C3D10::dN10dz;

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

    double d = 0.138196601125011;
    double e = 0.585410196624968;
    double f = 0.041666666666667;

    rI[0](0) = d; rI[0](1) = d; rI[0](2) = d; wI[0] = f;
    rI[1](0) = e; rI[1](1) = d; rI[1](2) = d; wI[1] = f;
    rI[2](0) = d; rI[2](1) = e; rI[2](2) = d; wI[2] = f;
    rI[3](0) = d; rI[3](1) = d; rI[3](2) = e; wI[3] = f;

    indices.resize(4,vector<int>(3));
    indices[0][0] = 1; indices[0][1] = 2; indices[0][2] = 3;
    indices[1][0] = 0; indices[1][1] = 1; indices[1][2] = 3;
    indices[2][0] = 2; indices[2][1] = 0; indices[2][2] = 3;
    indices[3][0] = 2; indices[3][1] = 1; indices[3][2] = 0;
  }

}
