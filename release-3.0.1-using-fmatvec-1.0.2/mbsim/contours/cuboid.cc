/* Copyright (C) 2004-2009 MBSim Development Team
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

#include<config.h>
#include "mbsim/contours/cuboid.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Cuboid::Cuboid(const string &name) : CompoundContour(name) {}

  void Cuboid::init(InitStage stage) {
    if(stage==preInit) {
      Vec Kr[8];
      for(int i=0; i<8; i++) {
        Kr[i] = Vec(3);
      }
      Kr[0](0) = l/2;
      Kr[0](1) = d/2;
      Kr[0](2) = h/2;
  
      Kr[1](0) = -l/2.0;
      Kr[1](1) = d/2.0;
      Kr[1](2) = h/2.0;
  
      Kr[2](0) = -l/2.0;
      Kr[2](1) = -d/2.0;
      Kr[2](2) = h/2.0;
  
      Kr[3](0) = l/2.0;
      Kr[3](1) = -d/2.0;
      Kr[3](2) = h/2.0;
  
      Kr[4](0) = l/2.0;
      Kr[4](1) = d/2.0;
      Kr[4](2) = -h/2.0;
  
      Kr[5](0) = -l/2.0;
      Kr[5](1) = d/2.0;
      Kr[5](2) = -h/2.0;
  
      Kr[6](0) = -l/2.0;
      Kr[6](1) = -d/2.0;
      Kr[6](2) = -h/2.0;
  
      Kr[7](0) = l/2.0;
      Kr[7](1) = -d/2.0;
      Kr[7](2) = -h/2.0;
  
      for(int i=0; i<8; i++) {
        stringstream s;
        s << i+1;
        addContourElement(new Point(s.str()),Kr[i]);
      }
    }
    else
      CompoundContour::init(stage);
  }
}
