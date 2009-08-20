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
 * Contact: schneidm@users.berlios.de
 */

#include <mbsim/utils/function_library.h>

using namespace fmatvec;

namespace MBSim {
  
  Vec TabularFunction::operator()(const double& xVal) {
    int i=xIndexOld;
    if (xVal<=x(0)) {
      xIndexOld=0;
      return trans(y.row(0));
    }
    else if (xVal>=x(xSize-1)) {
      xIndexOld=xSize-1;
      return trans(y.row(xSize-1));
    }
    else if (xVal<=x(i)) {
      while (xVal<x(i))
        i--;
    }
    else {
      do
        i++;
      while (xVal>x(i));
      i--;
    }
    xIndexOld=i;
    RowVec m=(y.row(i+1)-y.row(i))/(x(i+1)-x(i));
    return trans(y.row(i)+(xVal-x(i))*m);
  }


}
