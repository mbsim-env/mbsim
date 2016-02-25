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

 * Contact: thorsten.schindler@mytum.de
 
 */

#include<config.h>
#include "mbsimFlexibleBody/contact_kinematics/circle_flexibleband.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/contours/circle.h"
#include "mbsim/functions/contact/funcpair_planarcontour_circle.h"
#include "mbsim/utils/planar_contact_search.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  class ContactKinematicsCircleNode : public MBSim::ContactKinematics {
    public:
      ContactKinematicsCircleNode(double rCircle_, double wBand_, double node_) : rCircle(rCircle_), wBand(wBand_), node(node_), circle(0), band(0) { }
      virtual void assignContours(const vector<Contour*> &contour);
      virtual void updateg(double t, double &g, vector<ContourFrame*> &cFrame, int index = 0);
      virtual void updatewb(double t, Vec &wb, double g, vector<ContourFrame*> &cFrame) { throw MBSim::MBSimError("ContactKinematicsCircleNode::updatewb not implemented!"); }
    private:
      double rCircle, wBand, node;
      int icircle, inode;
      MBSim::Circle *circle;
      FlexibleBand *band;
  };

  void ContactKinematicsCircleNode::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      inode = 1;
      circle = static_cast<Circle*>(contour[0]);
      band = static_cast<FlexibleBand*>(contour[1]);
    }
    else {
      icircle = 1;
      inode = 0;
      circle = static_cast<Circle*>(contour[1]);
      band = static_cast<FlexibleBand*>(contour[0]);
    }
  }

  void ContactKinematicsCircleNode::updateg(double t, double &g, vector<ContourFrame*> &cFrame, int index) {

    cFrame[inode]->setEta(node);

    cFrame[inode]->setPosition(band->getPosition(t,cFrame[inode]->getZeta()));

    const Vec3 WrD = cFrame[inode]->getPosition(false) - circle->getFrame()->getPosition(t);
    
    cFrame[icircle]->getOrientation(false).set(0, WrD/nrm2(WrD));
    cFrame[icircle]->getOrientation(false).set(2, circle->getFrame()->getOrientation(false).col(2));
    cFrame[icircle]->getOrientation(false).set(1, crossProduct(cFrame[icircle]->getOrientation(false).col(2), cFrame[icircle]->getOrientation(false).col(0)));

    cFrame[inode]->getOrientation(false).set(0, -cFrame[icircle]->getOrientation(false).col(0));
    cFrame[inode]->getOrientation(false).set(1, -cFrame[icircle]->getOrientation(false).col(1));
    cFrame[inode]->getOrientation(false).set(2, cFrame[icircle]->getOrientation(false).col(2));

    cFrame[icircle]->setPosition(circle->getFrame()->getPosition() + cFrame[icircle]->getOrientation(false).col(0)*circle->getRadius());

    cFrame[inode]->setXi(cFrame[inode]->getOrientation(false).col(2).T() * WrD); // get contact parameter of second tangential direction
    cFrame[inode]->getPosition(false) += cFrame[inode]->getXi() * cFrame[inode]->getOrientation(false).col(2);

    if(band->isZetaOutside(cFrame[inode]->getZeta()))
      g = 1;
    else
      g = cFrame[inode]->getOrientation(false).col(0).T() * (cFrame[icircle]->getPosition(false) - cFrame[inode]->getPosition(false));
    if(g < -band->getThickness()) g = 1;
  }

  class ContactKinematicsCircleNodeInterpolation : public MBSim::ContactKinematics {
    public:
      ContactKinematicsCircleNodeInterpolation(double rCircle_, double wBand_, Vec nodes_) : rCircle(rCircle_), wBand(wBand_), nodes(nodes_), circle(0), band(0) { }
      virtual void assignContours(const vector<Contour*> &contour);
      virtual void updateg(double t, double &g, vector<ContourFrame*> &cFrame, int index = 0);
      virtual void updatewb(double t, Vec &wb, double g, vector<ContourFrame*> &cFrame) { throw MBSim::MBSimError("ContactKinematicsCircleNodeInterpolation::updatewb not implemented!"); }
    private:
      double rCircle, wBand;
      Vec nodes;
      int icircle, inode;
      MBSim::Circle *circle;
      FlexibleBand *band;
  };

  void ContactKinematicsCircleNodeInterpolation::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      inode = 1;
      circle = static_cast<Circle*>(contour[0]);
      band = static_cast<FlexibleBand*>(contour[1]);
    }
    else {
      icircle = 1;
      inode = 0;
      circle = static_cast<Circle*>(contour[1]);
      band = static_cast<FlexibleBand*>(contour[0]);
    }
  }

  void ContactKinematicsCircleNodeInterpolation::updateg(double t, double &g, vector<ContourFrame*> &cFrame, int index) {
    FuncPairPlanarContourCircle *func = new FuncPairPlanarContourCircle(circle, band); // root function for searching contact parameters
    PlanarContactSearch search(func);

    search.setNodes(nodes); // defining search areas for contacts
    Mat result = search.slvAll();
    delete func;

    if (result.rows()) {

      cFrame[inode]->setEta(result(0,0));

      cFrame[inode]->getOrientation(false).set(0, band->getWn(t,cFrame[inode]->getZeta()));
      cFrame[inode]->getOrientation(false).set(1, band->getWu(t,cFrame[inode]->getZeta()));
      cFrame[inode]->getOrientation(false).set(2, band->getWv(t,cFrame[inode]->getZeta()));
      cFrame[icircle]->getOrientation(false).set(0, -cFrame[inode]->getOrientation(false).col(0));
      cFrame[icircle]->getOrientation(false).set(2, circle->getFrame()->getOrientation(t).col(2));
      cFrame[icircle]->getOrientation(false).set(1, crossProduct(cFrame[icircle]->getOrientation(false).col(2),cFrame[icircle]->getOrientation(false).col(0)));

      cFrame[inode]->setPosition(band->getPosition(t,cFrame[inode]->getZeta()));
      cFrame[icircle]->setPosition(circle->getFrame()->getPosition(t)+circle->getRadius()*cFrame[icircle]->getOrientation(false).col(0));

      Vec Wd = circle->getFrame()->getPosition(t) - cFrame[inode]->getPosition(false);
      cFrame[inode]->setXi(cFrame[inode]->getOrientation(false).col(2).T() * Wd); // get contact parameter of second tangential direction
      cFrame[inode]->getPosition(false) += cFrame[inode]->getXi() * cFrame[inode]->getOrientation(false).col(2);

      if(band->isZetaOutside(cFrame[inode]->getZeta()))
        g = 1;
      else
        g = cFrame[inode]->getOrientation(false).col(0).T() * (cFrame[icircle]->getPosition(false) - cFrame[inode]->getPosition(false));
      if(g < -band->getThickness()) g = 1;
    }
  }

  ContactKinematicsCircleFlexibleBand::ContactKinematicsCircleFlexibleBand() : ContactKinematics(), icircle(0), icontour(0), possibleContactsPerNode(1), circle(0), band(0) { }

  void ContactKinematicsCircleFlexibleBand::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      icontour = 1;
      circle = static_cast<Circle*>(contour[0]);
      band = static_cast<FlexibleBand*>(contour[1]);
    }
    else {
      icircle = 1;
      icontour = 0;
      circle = static_cast<Circle*>(contour[1]);
      band = static_cast<FlexibleBand*>(contour[0]);
    }
    wBand = band->getWidth();
    hBand = 0;
    rCircle = circle->getRadius();

    staticNodes = band->getEtaNodes();
    numberOfPotentialContactPoints = 2 * possibleContactsPerNode * band->getEtaNodes().size() - 1;  // dies braeuchte einen eigenen init-Call
//    l0 = 1.0 * fabs(band->getAlphaEnd() - band->getAlphaStart()) / staticNodes.size(); /* bandwidth of mesh deformer: higher values leads to stronger attraction of last contact points */
//    epsTol = 5.e-2 * l0; /* distance, when two contact points should be treated as one */

    cout << numberOfPotentialContactPoints << endl;
    cout << possibleContactsPerNode << endl;
    cout << staticNodes.size() << endl;
    for (int i = 0; i < staticNodes.size(); i++) {
      ContactKinematicsCircleNode *ck = new ContactKinematicsCircleNode(rCircle, wBand, staticNodes(i));
      ck->assignContours(contour);
      contactKinematics.push_back(ck);
    }
    for (int i = 0; i < numberOfPotentialContactPoints - staticNodes.size(); i++) {
      ContactKinematicsCircleNodeInterpolation *ck = new ContactKinematicsCircleNodeInterpolation(rCircle, wBand, staticNodes(i, i + 1));
      ck->assignContours(contour);
      contactKinematics.push_back(ck);
    }
  }

  void ContactKinematicsCircleFlexibleBand::updateg(double t, double &g, vector<ContourFrame*> &cFrame, int index) {
    throw MBSim::MBSimError("ContactKinematicsCircleFlexibleBand::updateg not implemented!");
  }

  void ContactKinematicsCircleFlexibleBand::updatewb(double t, Vec &wb, double g, vector<ContourFrame*> &cFrame) {
    throw MBSim::MBSimError("ContactKinematicsCircleFlexibleBand::updatewb not implemented!");
  }
}
