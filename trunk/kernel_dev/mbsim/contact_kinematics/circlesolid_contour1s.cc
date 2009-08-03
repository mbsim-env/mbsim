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
#include "mbsim/contact_kinematics/circlesolid_contour1s.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/contour1s.h"
#include "mbsim/contours/contour1s_analytical.h"
#include "mbsim/functions_contact.h"
#include "mbsim/utils/eps.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsCircleSolidContour1s::~ContactKinematicsCircleSolidContour1s() { 
    delete func; 
  }

  void ContactKinematicsCircleSolidContour1s::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0;
      icontour1s = 1;
      circle = static_cast<CircleSolid*>(contour[0]);
      contour1s = static_cast<Contour1s*>(contour[1]);
    } 
    else {
      icircle = 1; 
      icontour1s = 0;
      circle = static_cast<CircleSolid*>(contour[1]);
      contour1s = static_cast<Contour1s*>(contour[0]);
    }

    func = new FuncPairContour1sCircleSolid(circle,contour1s);

    if (dynamic_cast<Contour1sAnalytical*>(contour1s)) {
      double minRadius=1./epsroot();
      for (double alpha=contour1s->getAlphaStart(); alpha<=contour1s->getAlphaEnd(); alpha+=(contour1s->getAlphaEnd()-contour1s->getAlphaStart())*1e-4) {
        double radius=static_cast<Contour1sAnalytical*>(contour1s)->getUserFunction()->computeR(alpha);
        minRadius=(radius<minRadius)?radius:minRadius;
      }
      if (circle->getRadius()>minRadius) {
        cout << "Error! Just one contact point is allowed in Contactpairing Contour1s-CircleSolid, but either the circle radius is to big or the minimal Radius of Contour1s is to small. Continuing anyway..." << endl;
        cout << "minimal Radius of Contour1sAnalytical=" << minRadius << endl;
        cout << "Radius of CircleSolid=" << circle->getRadius() << endl;
      }
    }

  }

  void ContactKinematicsCircleSolidContour1s::updateg(fmatvec::Vec &g, ContourPointData *cpData) {
    Contact1sSearch search(func);
    search.setNodes(contour1s->getNodes());

    if(cpData[icontour1s].getLagrangeParameterPosition().size() == 1)
      search.setInitialValue(cpData[icontour1s].getLagrangeParameterPosition()(0));
    else { 
      search.setSearchAll(true);
      cpData[icontour1s].getLagrangeParameterPosition() = Vec(1);
    }
    cpData[icontour1s].getLagrangeParameterPosition()(0) = search.slv();

    cpData[icontour1s].getFrameOfReference().getPosition() = 
      contour1s->computePosition(cpData[icontour1s]);
    cpData[icontour1s].getFrameOfReference().getOrientation().col(0) = 
      contour1s->computeNormal(cpData[icontour1s]);
    cpData[icontour1s].getFrameOfReference().getOrientation().col(1) = 
      contour1s->computeTangent(cpData[icontour1s]);
    cpData[icontour1s].getFrameOfReference().getOrientation().col(2) = 
      -contour1s->computeBinormal(cpData[icontour1s]);

    cpData[icircle].getFrameOfReference().getPosition() =
      circle->getFrame()->getPosition()-
      circle->getRadius()*cpData[icontour1s].getFrameOfReference().getOrientation().col(0);
    cpData[icircle].getFrameOfReference().getOrientation().col(0) = 
      -cpData[icontour1s].getFrameOfReference().getOrientation().col(0);
    cpData[icircle].getFrameOfReference().getOrientation().col(1) =
      -cpData[icontour1s].getFrameOfReference().getOrientation().col(1);
    cpData[icircle].getFrameOfReference().getOrientation().col(2) =
      cpData[icontour1s].getFrameOfReference().getOrientation().col(2);

    Vec WrD = func->computeWrD(cpData[icontour1s].getLagrangeParameterPosition()(0));
    g(0) = -trans(cpData[icontour1s].getFrameOfReference().getOrientation().col(0))*WrD;
  }

}

