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
    rI[1](0) = d;  rI[1](1) = -d;  rI[1](2) = -d;  wI[1] = 1;
    rI[2](0) = -d; rI[2](1) = d;   rI[2](2) = -d;  wI[2] = 1;
    rI[3](0) = d;  rI[3](1) = d;   rI[3](2) = -d;  wI[3] = 1;
    rI[4](0) = -d; rI[4](1) = -d;  rI[4](2) = d;   wI[4] = 1;
    rI[5](0) = d;  rI[5](1) = -d;  rI[5](2) = d;   wI[5] = 1;
    rI[6](0) = -d; rI[6](1) = d;   rI[6](2) = d;   wI[6] = 1;
    rI[7](0) = d;  rI[7](1) = d;   rI[7](2) = d;   wI[7] = 1;
  }

}
