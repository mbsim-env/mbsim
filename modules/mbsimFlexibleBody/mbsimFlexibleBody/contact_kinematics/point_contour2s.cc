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
 * Contact: thorsten.schindler@mytum.de
 *          rzander@users.berlios.de
 */

#include <config.h> 
#include "mbsimFlexibleBody/contact_kinematics/point_contour2s.h"
#include "mbsimFlexibleBody/contours/contour_2s_neutral_factory.h"
#include "mbsim/functions/contact/funcpair_spatialcontour_point.h"
#include "mbsim/contours/point.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/utils/spatial_contact_search.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ContactKinematicsPointContour2s::~ContactKinematicsPointContour2s() {
    delete func;
  }

  void ContactKinematicsPointContour2s::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0; icontour2s = 1;
      point = static_cast<Point*>(contour[0]);
      contour2s = static_cast<Contour2s*>(contour[1]);
    }
    else {
      ipoint = 1; icontour2s = 0;
      point = static_cast<Point*>(contour[1]);
      contour2s = static_cast<Contour2s*>(contour[0]);
    }
    func= new FuncPairSpatialContourPoint(point,contour2s);
  }

  void ContactKinematicsPointContour2s::updateg(double t, double &g, vector<ContourFrame*> &cFrame, int index) {
    // contact search on cylinder flexible
    SpatialContactSearch search(func);
    // if the search grid is given by user, use that; otherwise call setEqualspacing to create a 11 * 11 grid to for initial searching.
    if ((contour2s->getNodesU().size() != 0) && (contour2s->getNodesV().size() != 0))
      search.setNodes(contour2s->getNodesU(), contour2s->getNodesV());
    else
      search.setEqualSpacing(10, 10, 0, 0, 0.1, 0.1);

    if (searchAllCP==false) { // select start value from last search
      search.setInitialValue(cFrame[icontour2s]->getZeta());
    }
    else { // define start search with regula falsi
      search.setSearchAll(true);
      searchAllCP = false;
    }

    cFrame[icontour2s]->setZeta(search.slv());
    cFrame[icontour2s]->setPosition(contour2s->getPosition(t,cFrame[icontour2s]->getZeta()));
    cFrame[icontour2s]->getOrientation(false).set(0, contour2s->getWn(t,cFrame[icontour2s]->getZeta()));
    cFrame[icontour2s]->getOrientation(false).set(1, contour2s->getWu(t,cFrame[icontour2s]->getZeta()));
    cFrame[icontour2s]->getOrientation(false).set(2, contour2s->getWv(t,cFrame[icontour2s]->getZeta()));

    cFrame[ipoint]->setPosition(point->getFrame()->getPosition(t)); // position of point
    cFrame[ipoint]->getOrientation(false).set(0, -cFrame[icontour2s]->getOrientation(false).col(0));
    cFrame[ipoint]->getOrientation(false).set(1, -cFrame[icontour2s]->getOrientation(false).col(1));
    cFrame[ipoint]->getOrientation(false).set(2, cFrame[icontour2s]->getOrientation(false).col(2));

    if(contour2s->isZetaOutside(cFrame[icontour2s]->getZeta()))
      g = 1;
    else
      g = cFrame[icontour2s]->getOrientation(false).col(0).T() * (cFrame[ipoint]->getPosition(false) - cFrame[icontour2s]->getPosition(false));
    if(g < -contour2s->getThickness()) g = 1;

    //    Vec3 WrD = cFrame[ipoint]->getPosition(false) - cFrame[icontour2s]->getPosition(t,cFrame[icontour2s]->getZeta());

    // contact in estimated contact area?
    //    if(cpData[icontour2s].getLagrangeParameterPosition()(0) < contour2s->getAlphaStart()(0) || cpData[icontour2s].getLagrangeParameterPosition()(0) > contour2s->getAlphaEnd()(0) ||
    //       cpData[icontour2s].getLagrangeParameterPosition()(1) < contour2s->getAlphaStart()(1) || cpData[icontour2s].getLagrangeParameterPosition()(1) > contour2s->getAlphaEnd()(1))
    //      g = cpData[icontour2s].getFrameOfReference().getOrientation().col(0).T() * (cpData[ipoint].getFrameOfReference().getPosition() - cpData[icontour2s].getFrameOfReference().getPosition());

  }

  void ContactKinematicsPointContour2s::updatewb(double t, Vec &wb, double g, vector<ContourFrame*> &cFrame) {
    throw MBSim::MBSimError("(ContactKinematicsPointContour2s:updatewb): Not implemented!");
  }

}

