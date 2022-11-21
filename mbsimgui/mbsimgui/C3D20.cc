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
  }

}
