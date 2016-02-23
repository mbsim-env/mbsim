/* Copyright (C) 2004-2015 MBSim Development Team
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
#include "mbsimFlexibleBody/contact_kinematics/point_flexibleband.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/contours/point.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/functions/contact/funcpair_planarcontour_point.h"
#include "mbsim/utils/planar_contact_search.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ContactKinematicsPointFlexibleBand::~ContactKinematicsPointFlexibleBand() {
    delete func;
  }

  void ContactKinematicsPointFlexibleBand::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      icontour = 1;
      point = static_cast<Point*>(contour[0]);
      band = static_cast<FlexibleBand*>(contour[1]);
    }
    else {
      ipoint = 1;
      icontour = 0;
      point = static_cast<Point*>(contour[1]);
      band = static_cast<FlexibleBand*>(contour[0]);
    }
    func = new FuncPairPlanarContourPoint(point, band); // root function for searching contact parameters
  }

  void ContactKinematicsPointFlexibleBand::updateg(double t, double &g, std::vector<ContourFrame*> &cFrame, int index) {

    func->setTime(t);
    PlanarContactSearch search(func);
    search.setNodes(band->getEtaNodes()); // defining search areas for contacts

    if (searchAllCP==false) { // select start value from last search
      search.setInitialValue(cFrame[icontour]->getEta());
    }
    else { // define start search with regula falsi
      search.setSearchAll(true);
      searchAllCP = false;
    }

    cFrame[icontour]->setEta(search.slv());

    cFrame[icontour]->setPosition(band->getPosition(t,cFrame[icontour]->getZeta()));
    cFrame[icontour]->getOrientation(false).set(0, band->getWn(t,cFrame[icontour]->getZeta()));
    cFrame[icontour]->getOrientation(false).set(1, band->getWu(t,cFrame[icontour]->getZeta()));
    cFrame[icontour]->getOrientation(false).set(2, band->getWv(t,cFrame[icontour]->getZeta()));

    cFrame[ipoint]->setPosition(point->getFrame()->getPosition(t)); // position of point
    cFrame[ipoint]->getOrientation(false).set(0, -cFrame[icontour]->getOrientation(false).col(0));
    cFrame[ipoint]->getOrientation(false).set(1, -cFrame[icontour]->getOrientation(false).col(1));
    cFrame[ipoint]->getOrientation(false).set(2, cFrame[icontour]->getOrientation(false).col(2));
    Vec3 Wd = cFrame[ipoint]->getPosition(false) - cFrame[icontour]->getPosition(false);
    cFrame[icontour]->setXi(cFrame[icontour]->getOrientation(false).col(2).T() * Wd); // get contact parameter of second tangential direction
    if (cFrame[icontour]->getEta() < band->getEtaNodes()[0] || cFrame[icontour]->getEta() > band->getEtaNodes()[band->getEtaNodes().size()-1]) {
      g = 1.0;
      return;
    }
    if (cFrame[icontour]->getXi() > 0.5 * band->getWidth() || cFrame[icontour]->getXi() < -0.5 * band->getWidth()) {
      g = 1.0;
      return;
    }
    cFrame[icontour]->getPosition(false) += cFrame[icontour]->getXi() * cFrame[icontour]->getOrientation(false).col(2);
    g = cFrame[icontour]->getOrientation(false).col(0).T() * Wd;
    if(g < -band->getThickness()) g = 1;
  }

  void ContactKinematicsPointFlexibleBand::updatewb(double t, Vec &wb, double g, vector<ContourFrame*> &cFrame) {
    throw MBSimError("(ContactKinematicsPointFlexibleBand::updatewb): Not implemented!");
  }

}

