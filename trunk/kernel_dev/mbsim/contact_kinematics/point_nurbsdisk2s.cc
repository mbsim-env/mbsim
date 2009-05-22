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
 * Contact: mfoerg@users.berlios.de
 *          rzander@users.berlios.de
 */

#include <config.h> 
#include "point_nurbsdisk2s.h"
#include "mbsim/contour.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {
  void ContactKinematicsPointNurbsDisk2s::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      inurbsdisk = 1;
      point = static_cast<Point*>(contour[0]);
      nurbsdisk = static_cast<NurbsDisk2s*>(contour[1]);
    }
    else {
      ipoint = 1;
      inurbsdisk = 0;
      point = static_cast<Point*>(contour[1]);
      nurbsdisk = static_cast<NurbsDisk2s*>(contour[0]);
    }
  }

  void ContactKinematicsPointNurbsDisk2s::updateg(Vec &g, ContourPointData* cpData) {}

}

