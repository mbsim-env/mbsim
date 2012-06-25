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
 * Contact: martin.o.foerg@googlemail.com
 */

#include<config.h>
#include "mbsim/contours/area.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Area::Area(const string &name) : RigidContour(name), lim1(1), lim2(1), Cn(3), Cd1(3), Cd2(3) {}
  void Area::setCd1(const FVec &d) {Cd1 = d/nrm2(d);}
  void Area::setCd2(const FVec &d) {Cd2 = d/nrm2(d);}
  void Area::init(InitStage stage) {
    if(stage==unknownStage) {
      RigidContour::init(stage);
      Cn = crossProduct(Cd1,Cd2);
      Cn = Cn/nrm2(Cn);
    }
    else
      RigidContour::init(stage);
  }
}
