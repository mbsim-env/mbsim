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
 */

#include <config.h> 
#include "mbsimFlexibleBody/contact_kinematics/circle_nurbsdisk2s.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsimFlexibleBody/functions_contact.h"
#include "mbsim/utils/planar_contact_search.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ContactKinematicsCircleNurbsDisk2s::ContactKinematicsCircleNurbsDisk2s()  {
  }

  ContactKinematicsCircleNurbsDisk2s::~ContactKinematicsCircleNurbsDisk2s() = default;

  void ContactKinematicsCircleNurbsDisk2s::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      inurbsdisk = 1;
      circle = static_cast<Circle*>(contour[0]);
      nurbsdisk = static_cast<NurbsDisk2s*>(contour[1]);
    }
    else {
      icircle = 1;
      inurbsdisk = 0;
      circle = static_cast<Circle*>(contour[1]);
      nurbsdisk = static_cast<NurbsDisk2s*>(contour[0]);
    }
  }

  void ContactKinematicsCircleNurbsDisk2s::updateg(double &g, vector<ContourFrame*> &cFrame, int index) {
    auto *func= new FuncPairCircleNurbsDisk2s(circle, nurbsdisk); // root function for searching contact parameters
    PlanarContactSearch search(func);

    if(LOCALSEARCH) { // select start value from last search (local search)
      search.setInitialValue(cFrame[icircle]->getEta());
    }
    else { // define start search with regula falsi (global search)
      search.setSearchAll(true);
    }

    int SEC = 16; // partition for regula falsi
    double drho = 2.*M_PI/SEC * 1.01; // 10% intersection for improved convergence of solver
    double rhoStartSpacing = -2.*M_PI*0.01*0.5;
    search.setEqualSpacing(SEC,rhoStartSpacing,drho); //set the nodes for regula falsi
    cFrame[icircle]->setEta(search.slv()); // get contact parameter

    // point on the circle
    fmatvec::Vec P_circle(3,fmatvec::INIT,0);
    P_circle(0) = cos(cFrame[icircle]->getEta());
    P_circle(1) = sin(cFrame[icircle]->getEta());
    P_circle = circle->getFrame()->evalPosition() + circle->getRadius() * circle->getFrame()->evalOrientation() * P_circle;
    cFrame[icircle]->setPosition(P_circle); // position of the point in world coordinates

    cFrame[inurbsdisk]->setZeta(nurbsdisk->transformCW(nurbsdisk->evalOrientation().T()*(cFrame[icircle]->getPosition(false) - nurbsdisk->evalPosition()))(0,1));

    if(nurbsdisk->isZetaOutside(cFrame[inurbsdisk]->getZeta()))
      g = 1.;
    else {

      cFrame[inurbsdisk]->setPosition(nurbsdisk->evalPosition(cFrame[inurbsdisk]->getZeta()));
      cFrame[inurbsdisk]->getOrientation(false).set(0, nurbsdisk->evalWn(cFrame[inurbsdisk]->getZeta()));
      cFrame[inurbsdisk]->getOrientation(false).set(1, nurbsdisk->evalWu(cFrame[inurbsdisk]->getZeta()));
      cFrame[inurbsdisk]->getOrientation(false).set(2, nurbsdisk->evalWv(cFrame[inurbsdisk]->getZeta()));

      cFrame[icircle]->getOrientation(false).set(0, -cFrame[inurbsdisk]->getOrientation(false).col(0));
      cFrame[icircle]->getOrientation(false).set(1, -cFrame[inurbsdisk]->getOrientation(false).col(1));
      cFrame[icircle]->getOrientation(false).set(2,  cFrame[inurbsdisk]->getOrientation(false).col(2));   // to have a legal framework the second tangent is not the negative of the tanget of the disk

      g = cFrame[inurbsdisk]->getOrientation(false).col(0).T() * (cFrame[icircle]->getPosition(false) - cFrame[inurbsdisk]->getPosition(false));
    }

    delete func;
  }

  void ContactKinematicsCircleNurbsDisk2s::updatewb(Vec &wb, double g, vector<ContourFrame*> &cFrame) {
    throw runtime_error("(ContactKinematicsCircleNurbsDisk2s:updatewb): Not implemented!");
  }

}
