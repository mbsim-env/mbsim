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
#include "mbsim/contact_kinematics/point_extrusion.h"
#include "mbsim/contours/point.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/functions/contact/funcpair_planarcontour_point.h"
#include "mbsim/utils/planar_contact_search.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSim {

  ContactKinematicsPointExtrusion::~ContactKinematicsPointExtrusion() {
    delete func;
  }

  void ContactKinematicsPointExtrusion::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      iextrusion = 1;
      point = static_cast<Point*>(contour[0]);
      extrusion = static_cast<Contour*>(contour[1]);
    }
    else {
      ipoint = 1;
      iextrusion = 0;
      point = static_cast<Point*>(contour[1]);
      extrusion = static_cast<Contour*>(contour[0]);
    }
    func = new FuncPairPlanarContourPoint(point, extrusion); // root function for searching contact parameters
  }

  void ContactKinematicsPointExtrusion::updateg(double t, double &g, std::vector<ContourFrame*> &cFrame, int index) {

    func->setTime(t);
    PlanarContactSearch search(func);
    search.setNodes(extrusion->getEtaNodes()); // defining search areas for contacts

    if (searchAllCP==false) { // select start value from last search
      search.setInitialValue(cFrame[iextrusion]->getEta());
    }
    else { // define start search with regula falsi
      search.setSearchAll(true);
      searchAllCP = false;
    }

    cFrame[iextrusion]->setEta(search.slv());

    cFrame[iextrusion]->setPosition(extrusion->getPosition(t,cFrame[iextrusion]->getZeta()));
    cFrame[iextrusion]->getOrientation(false).set(0, extrusion->getWn(t,cFrame[iextrusion]->getZeta()));
    cFrame[iextrusion]->getOrientation(false).set(1, extrusion->getWu(t,cFrame[iextrusion]->getZeta()));
    cFrame[iextrusion]->getOrientation(false).set(2, extrusion->getWv(t,cFrame[iextrusion]->getZeta()));

    cFrame[ipoint]->setPosition(point->getFrame()->getPosition(t)); // position of point
    cFrame[ipoint]->getOrientation(false).set(0, -cFrame[iextrusion]->getOrientation(false).col(0));
    cFrame[ipoint]->getOrientation(false).set(1, -cFrame[iextrusion]->getOrientation(false).col(1));
    cFrame[ipoint]->getOrientation(false).set(2, cFrame[iextrusion]->getOrientation(false).col(2));
    Vec3 Wd = cFrame[ipoint]->getPosition(false) - cFrame[iextrusion]->getPosition(false);
    cFrame[iextrusion]->setXi(cFrame[iextrusion]->getOrientation(false).col(2).T() * Wd); // get contact parameter of second tangential direction
    cFrame[iextrusion]->getPosition(false) += cFrame[iextrusion]->getXi() * cFrame[iextrusion]->getOrientation(false).col(2);
    if(extrusion->isZetaOutside(cFrame[iextrusion]->getZeta()))
      g = 1;
    else
      g = cFrame[iextrusion]->getOrientation(false).col(0).T() * Wd;
    if(g < -extrusion->getThickness()) g = 1;
  }

  void ContactKinematicsPointExtrusion::updatewb(double t, Vec &wb, double g, vector<ContourFrame*> &cFrame) {
    throw MBSimError("(ContactKinematicsPointExtrusion::updatewb): Not implemented!");
  }

}

