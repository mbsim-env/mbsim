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
#include "mbsim/contact_kinematics/line_planarcontour.h"
#include "mbsim/contours/line.h"
#include "mbsim/functions/contact/funcpair_planarcontour_line.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsLinePlanarContour::~ContactKinematicsLinePlanarContour() {
    delete func;
  }

  void ContactKinematicsLinePlanarContour::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Line*>(contour[0])) {
      iline = 0;
      icontour = 1;
      line = static_cast<Line*>(contour[0]);
      contour1s = static_cast<Contour*>(contour[1]);
    } 
    else {
      iline = 1;
      icontour = 0;
      line = static_cast<Line*>(contour[1]);
      contour1s = static_cast<Contour*>(contour[0]);
    }
    func= new FuncPairPlanarContourLine(line,contour1s);
  }

  void ContactKinematicsLinePlanarContour::updateg(double t, double &g, std::vector<ContourFrame*> &cFrame, int index) {
    throw MBSimError("(ContactKinematicsLinePlanarContour::updateg): Not implemented!");
  }

}

