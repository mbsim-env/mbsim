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

#include <config.h> 
#include "mbsim/contact_kinematics/point_contourinterpolation.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/contour_interpolation.h"
#include "mbsim/functions/contact/funcpair_point_contourinterpolation.h"
#include "mbsim/utils/nonlinear_algebra.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsPointContourInterpolation::~ContactKinematicsPointContourInterpolation() {
    delete func;
  }

  void ContactKinematicsPointContourInterpolation::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0; idinterpol = 1;
      point = static_cast<Point*>(contour[0]);
      cinterpol = static_cast<ContourInterpolation*>(contour[1]);
    } 
    else {
      ipoint = 1; idinterpol = 0;
      point = static_cast<Point*>(contour[1]);
      cinterpol = static_cast<ContourInterpolation*>(contour[0]);
    }
    func = new FuncPairPointContourInterpolation(point,cinterpol);
  }

  void ContactKinematicsPointContourInterpolation::updateg(double &g, std::vector<ContourFrame*> &cFrame, int index) {
    throw MBSimError("(ContactKinematicsPointContourInterpolation::updateg): Not implemented!");
  }

}

