/* Copyright (C) 2004-2016 MBSim Development Team
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
      iplanarcontour = 1;
      point = static_cast<Point*>(contour[0]);
      planarcontour = static_cast<Contour*>(contour[1]);
    }
    else {
      ipoint = 1;
      iplanarcontour = 0;
      point = static_cast<Point*>(contour[1]);
      planarcontour = static_cast<Contour*>(contour[0]);
    }
    func = new FuncPairPlanarContourPoint(point, planarcontour); // root function for searching contact parameters
  }

  void ContactKinematicsPointPlanarContour::updateg(double t, double &g, std::vector<ContourFrame*> &cFrame, int index) {
    
    func->setTime(t);
    PlanarContactSearch search(func);
    search.setNodes(planarcontour->getEtaNodes()); // defining search areas for contacts

    if (searchAllCP==false) { // select start value from last search
      search.setInitialValue(cFrame[iplanarcontour]->getEta());
    }
    else { // define start search with regula falsi
      search.setSearchAll(true);
      searchAllCP = false;
    }

    cFrame[iplanarcontour]->setEta(search.slv());

    cFrame[iplanarcontour]->setPosition(planarcontour->getPosition(t,cFrame[iplanarcontour]->getZeta()));
    cFrame[iplanarcontour]->getOrientation(false).set(0, planarcontour->getWn(t,cFrame[iplanarcontour]->getZeta()));
    cFrame[iplanarcontour]->getOrientation(false).set(1, planarcontour->getWu(t,cFrame[iplanarcontour]->getZeta()));
    cFrame[iplanarcontour]->getOrientation(false).set(2, planarcontour->getWv(t,cFrame[iplanarcontour]->getZeta()));

    cFrame[ipoint]->setPosition(point->getFrame()->evalPosition());
    cFrame[ipoint]->getOrientation(false).set(0, -cFrame[iplanarcontour]->getOrientation(false).col(0));
    cFrame[ipoint]->getOrientation(false).set(1, -cFrame[iplanarcontour]->getOrientation(false).col(1));
    cFrame[ipoint]->getOrientation(false).set(2, cFrame[iplanarcontour]->getOrientation(false).col(2));

    if(planarcontour->isZetaOutside(cFrame[iplanarcontour]->getZeta()))
      g = 1;
    else
      g = cFrame[iplanarcontour]->getOrientation(false).col(0).T() * (cFrame[ipoint]->getPosition(false) - cFrame[iplanarcontour]->getPosition(false));
    if(g < -planarcontour->getThickness()) g = 1;
  }

  void ContactKinematicsPointPlanarContour::updatewb(double t, Vec &wb, double g, vector<ContourFrame*> &cFrame) {
    throw MBSimError("(ContactKinematicsPointPlanarContour::updatewb): Not implemented!");
  }

}
