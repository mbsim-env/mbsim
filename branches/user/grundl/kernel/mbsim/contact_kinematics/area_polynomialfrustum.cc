/* Copyright (C) 2004-2012 MBSim Development Team
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


#include "area_polynomialfrustum.h"
#include <math.h>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsAreaPolynomialFrustum::ContactKinematicsAreaPolynomialFrustum() :
    iarea(-1), ifrustum(-1), area(0), frustum(0) {
  }

  ContactKinematicsAreaPolynomialFrustum::~ContactKinematicsAreaPolynomialFrustum() {
  }

  void ContactKinematicsAreaPolynomialFrustum::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Area*>(contour[0])) {
      iarea = 0;
      ifrustum = 1;
      area = static_cast<Area*>(contour[0]);
      frustum = static_cast<PolynomialFrustum*>(contour[1]);
    }
    else {
      iarea = 1;
      ifrustum = 0;
      area = static_cast<Area*>(contour[1]);
      frustum = static_cast<PolynomialFrustum*>(contour[0]);
    }

    //TODO: check for convexity of frustum
  }

  void ContactKinematicsAreaPolynomialFrustum::updateg(Vec & g, ContourPointData * cpData, int index) {

    //cout << "Still to come ..." << endl;

  }

} /* namespace MBSim */
