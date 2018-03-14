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
#include "mbsim/contact_kinematics/edge_edge.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/edge.h"
#include "mbsim/links/single_contact.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsEdgeEdge::assignContours(const vector<Contour*> &contour) {
    iedge0 = 0; iedge1 = 1;
    edge0 = static_cast<Edge*>(contour[0]);
    edge1 = static_cast<Edge*>(contour[1]);
  }

  bool ContactKinematicsEdgeEdge::updateg(SingleContact &contact) {
    double g;
    Vec Wd = edge1->getFrame()->evalPosition() - edge0->getFrame()->evalPosition();
    Vec Wd0 = edge0->getFrame()->evalOrientation().col(1);
    Vec Wd1 = edge1->getFrame()->evalOrientation().col(1);
    Vec Wn = crossProduct(Wd0,Wd1);
    Wn = Wn/nrm2(Wn);
    double d = Wn.T()*Wd;
    if(d<0) {
      Wn *= -1.;
      d *= -1.;
    }
    Vec We0 = -edge0->getFrame()->getOrientation().col(0);
    Vec We1 = -edge1->getFrame()->getOrientation().col(0);
    if(Wn.T()*We0 >= 0 && Wn.T()*We1 <= 0) {
      if(d > max(edge0->getThickness(),edge1->getThickness())) {
        g = 1;
      } else {
        double t0 = trans(Wd0)*(Wd - Wd1*trans(Wd1)*Wd)/(1.0-trans(Wd0)*Wd1*trans(Wd1)*Wd0);
        double t1 = t0*trans(Wd1)*Wd0 - trans(Wd1)*Wd;

        if(fabs(t1) <= edge1->getLength()/2 and fabs(t0) <= edge0->getLength()/2) {
          contact.getContourFrame(iedge0)->setPosition(edge0->getFrame()->getPosition() + t0*Wd0);
          contact.getContourFrame(iedge1)->setPosition(edge1->getFrame()->getPosition() + t1*Wd1);
          contact.getContourFrame(iedge0)->getOrientation(false).set(0, -Wn);
          contact.getContourFrame(iedge1)->getOrientation(false).set(0, Wn);
          contact.getContourFrame(iedge0)->getOrientation(false).set(1, Wd0);
          contact.getContourFrame(iedge1)->getOrientation(false).set(1, -Wd0);
          contact.getContourFrame(iedge0)->getOrientation(false).set(2, crossProduct(Wn,Wd0));
          contact.getContourFrame(iedge1)->getOrientation(false).set(2, contact.getContourFrame(iedge0)->getOrientation(false).col(2));

          g = -d;
        }
        else
          g = 1;
      }
    } else if(Wn.T()*We0 < 0 && Wn.T()*We1 > 0)
      g = d;
    else
      g = 1;
    contact.getGeneralizedRelativePosition(false)(0) = g;
    return g <= 0;
  }

  void ContactKinematicsEdgeEdge::updateg(double &g, std::vector<ContourFrame*> &cFrame, int index) {
    Vec Wd = edge1->getFrame()->evalPosition() - edge0->getFrame()->evalPosition();
    Vec Wd0 = edge0->getFrame()->evalOrientation().col(1);
    Vec Wd1 = edge1->getFrame()->evalOrientation().col(1);
    Vec Wn = crossProduct(Wd0,Wd1);
    Wn = Wn/nrm2(Wn);
    double d = Wn.T()*Wd;
    if(d<0) {
      Wn *= -1.;
      d *= -1.;
    }
    Vec We0 = -edge0->getFrame()->getOrientation().col(0);
    Vec We1 = -edge1->getFrame()->getOrientation().col(0);
    if(Wn.T()*We0 >= 0 && Wn.T()*We1 <= 0) {
      if(d > max(edge0->getThickness(),edge1->getThickness())) {
        g = 1;
      } else {
        double t0 = trans(Wd0)*(Wd - Wd1*trans(Wd1)*Wd)/(1.0-trans(Wd0)*Wd1*trans(Wd1)*Wd0);
        double t1 = t0*trans(Wd1)*Wd0 - trans(Wd1)*Wd;

        if(fabs(t1) <= edge1->getLength()/2 and fabs(t0) <= edge0->getLength()/2) {
          cFrame[iedge0]->setPosition(edge0->getFrame()->getPosition() + t0*Wd0);
          cFrame[iedge1]->setPosition(edge1->getFrame()->getPosition() + t1*Wd1);
          cFrame[iedge0]->getOrientation(false).set(0, -Wn);
          cFrame[iedge1]->getOrientation(false).set(0, -cFrame[iedge0]->getOrientation(false).col(0));
          cFrame[iedge0]->getOrientation(false).set(1, Wd0);
          cFrame[iedge1]->getOrientation(false).set(1, -cFrame[iedge0]->getOrientation(false).col(1));
          cFrame[iedge0]->getOrientation(false).set(2, crossProduct(Wn,Wd0));
          cFrame[iedge1]->getOrientation(false).set(2, cFrame[iedge0]->getOrientation(false).col(2));

          g = -d;
        }
        else
          g = 1;
      }
    } else if(Wn.T()*We0 < 0 && Wn.T()*We1 > 0)
      g = d;
    else
      g = 1;
  }

}
