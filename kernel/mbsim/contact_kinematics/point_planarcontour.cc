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
#include "point_planarcontour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/functions/contact/funcpair_planarcontour_point.h"
#include "mbsim/utils/planar_contact_search.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsPointPlanarContour::~ContactKinematicsPointPlanarContour() {
    delete func;
  }

  void ContactKinematicsPointPlanarContour::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      icontour = 1;
      point = static_cast<Point*>(contour[0]);
      contour1s = static_cast<Contour*>(contour[1]);
    }
    else {
      ipoint = 1;
      icontour = 0;
      point = static_cast<Point*>(contour[1]);
      contour1s = static_cast<Contour*>(contour[0]);
    }
    func = new FuncPairPlanarContourPoint(point, contour1s); // root function for searching contact parameters
  }

  void ContactKinematicsPointPlanarContour::updateg(double t, double &g, std::vector<ContourFrame*> &cFrame, int index) {
    
    func->setTime(t);
    PlanarContactSearch search(func);
    search.setNodes(contour1s->getEtaNodes()); // defining search areas for contacts

    if (searchAllCP==false) { // select start value from last search
      search.setInitialValue(cFrame[icontour]->getEta());
    }
    else { // define start search with regula falsi
      search.setSearchAll(true);
      searchAllCP = false;
    }

    cFrame[icontour]->setEta(search.slv());

    if (cFrame[icontour]->getEta() < contour1s->getEtaNodes()[0] || cFrame[icontour]->getEta() > contour1s->getEtaNodes()[contour1s->getEtaNodes().size()-1]) {
      g = 1.0;
      return;
    }
    cFrame[icontour]->setPosition(contour1s->getPosition(t,cFrame[icontour]->getZeta()));
    cFrame[icontour]->getOrientation(false).set(0, contour1s->getWn(t,cFrame[icontour]->getZeta()));
    cFrame[icontour]->getOrientation(false).set(1, contour1s->getWu(t,cFrame[icontour]->getZeta()));
    cFrame[icontour]->getOrientation(false).set(2, contour1s->getWv(t,cFrame[icontour]->getZeta()));

    cFrame[ipoint]->setPosition(point->getFrame()->getPosition(t)); // position of point
    cFrame[ipoint]->getOrientation(false).set(0, -cFrame[icontour]->getOrientation(false).col(0));
    cFrame[ipoint]->getOrientation(false).set(1, -cFrame[icontour]->getOrientation(false).col(1));
    cFrame[ipoint]->getOrientation(false).set(2, cFrame[icontour]->getOrientation(false).col(2));
    g = cFrame[icontour]->getOrientation(false).col(0).T() * (cFrame[ipoint]->getPosition(false) - cFrame[icontour]->getPosition(false));
    if(g < -contour1s->getThickness()) g = 1;
  }

  void ContactKinematicsPointPlanarContour::updatewb(double t, Vec &wb, double g, vector<ContourFrame*> &cFrame) {
    throw MBSimError("(ContactKinematicsPointPlanarContour::updatewb): Not implemented!");
  }

}
