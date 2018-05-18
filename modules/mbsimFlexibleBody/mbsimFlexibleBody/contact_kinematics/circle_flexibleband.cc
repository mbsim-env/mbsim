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
#include "mbsim/contours/circle.h"
#include "mbsim/functions/contact/funcpair_planarcontour_circle.h"
#include "mbsim/utils/planar_contact_search.h"
#include "mbsim/links/single_contact.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  class ContactKinematicsCircleNode : public ContactKinematics {
    public:
      ContactKinematicsCircleNode(double node_) : node(node_), circle(nullptr), extrusion(nullptr) { }
      void assignContours(const vector<Contour*> &contour) override;
      void updateg(SingleContact &contact, int i=0) override;
    private:
      double node;
      int icircle, inode;
      Circle *circle;
      Contour *extrusion;
  };

  void ContactKinematicsCircleNode::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      inode = 1;
      circle = static_cast<Circle*>(contour[0]);
      extrusion = static_cast<Contour*>(contour[1]);
    }
    else {
      icircle = 1;
      inode = 0;
      circle = static_cast<Circle*>(contour[1]);
      extrusion = static_cast<Contour*>(contour[0]);
    }
  }

  void ContactKinematicsCircleNode::updateg(SingleContact &contact, int i) {

    contact.getContourFrame(inode)->setEta(node);

    contact.getContourFrame(inode)->setPosition(extrusion->evalPosition(contact.getContourFrame(inode)->getZeta(false)));

    const Vec3 WrD = contact.getContourFrame(inode)->getPosition(false) - circle->getFrame()->evalPosition();
    
    contact.getContourFrame(icircle)->getOrientation(false).set(0, WrD/nrm2(WrD));
    contact.getContourFrame(icircle)->getOrientation(false).set(2, circle->getFrame()->getOrientation(false).col(2));
    contact.getContourFrame(icircle)->getOrientation(false).set(1, crossProduct(contact.getContourFrame(icircle)->getOrientation(false).col(2), contact.getContourFrame(icircle)->getOrientation(false).col(0)));

    contact.getContourFrame(inode)->getOrientation(false).set(0, -contact.getContourFrame(icircle)->getOrientation(false).col(0));
    contact.getContourFrame(inode)->getOrientation(false).set(1, -contact.getContourFrame(icircle)->getOrientation(false).col(1));
    contact.getContourFrame(inode)->getOrientation(false).set(2, contact.getContourFrame(icircle)->getOrientation(false).col(2));

    contact.getContourFrame(icircle)->setPosition(circle->getFrame()->getPosition() + contact.getContourFrame(icircle)->getOrientation(false).col(0)*circle->getRadius());

    contact.getContourFrame(inode)->setXi(contact.getContourFrame(inode)->getOrientation(false).col(2).T() * WrD); // get contact parameter of second tangential direction
    contact.getContourFrame(inode)->getPosition(false) += contact.getContourFrame(inode)->getXi(false) * contact.getContourFrame(inode)->getOrientation(false).col(2);

    double g;
    if(extrusion->isZetaOutside(contact.getContourFrame(inode)->getZeta(false)))
      g = 1;
    else
      g = contact.getContourFrame(inode)->getOrientation(false).col(0).T() * (contact.getContourFrame(icircle)->getPosition(false) - contact.getContourFrame(inode)->getPosition(false));
    if(g < -extrusion->getThickness()) g = 1;
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

  class ContactKinematicsCircleNodeInterpolation : public ContactKinematics {
    public:
      ContactKinematicsCircleNodeInterpolation(const Vec &nodes_) : nodes(nodes_), circle(nullptr), extrusion(nullptr) { }
      void assignContours(const vector<Contour*> &contour) override;
      void updateg(SingleContact &contact, int i=0) override;
    private:
      Vec nodes;
      int icircle, inode;
      Circle *circle;
      Contour *extrusion;
  };

  void ContactKinematicsCircleNodeInterpolation::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      inode = 1;
      circle = static_cast<Circle*>(contour[0]);
      extrusion = static_cast<Contour*>(contour[1]);
    }
    else {
      icircle = 1;
      inode = 0;
      circle = static_cast<Circle*>(contour[1]);
      extrusion = static_cast<Contour*>(contour[0]);
    }
  }

  void ContactKinematicsCircleNodeInterpolation::updateg(SingleContact &contact, int i) {
    auto *func = new FuncPairPlanarContourCircle(circle, extrusion); // root function for searching contact parameters
    PlanarContactSearch search(func);

    search.setNodes(nodes); // defining search areas for contacts
    Mat result = search.slvAll();
    delete func;

    if (result.rows()) {

      contact.getContourFrame(inode)->setEta(result(0,0));

      contact.getContourFrame(inode)->getOrientation(false).set(0, extrusion->evalWn(contact.getContourFrame(inode)->getZeta(false)));
      contact.getContourFrame(inode)->getOrientation(false).set(1, extrusion->evalWu(contact.getContourFrame(inode)->getZeta(false)));
      contact.getContourFrame(inode)->getOrientation(false).set(2, extrusion->evalWv(contact.getContourFrame(inode)->getZeta(false)));
      contact.getContourFrame(icircle)->getOrientation(false).set(0, -contact.getContourFrame(inode)->getOrientation(false).col(0));
      contact.getContourFrame(icircle)->getOrientation(false).set(2, circle->getFrame()->evalOrientation().col(2));
      contact.getContourFrame(icircle)->getOrientation(false).set(1, crossProduct(contact.getContourFrame(icircle)->getOrientation(false).col(2),contact.getContourFrame(icircle)->getOrientation(false).col(0)));

      contact.getContourFrame(inode)->setPosition(extrusion->evalPosition(contact.getContourFrame(inode)->getZeta(false)));
      contact.getContourFrame(icircle)->setPosition(circle->getFrame()->evalPosition()+circle->getRadius()*contact.getContourFrame(icircle)->getOrientation(false).col(0));

      Vec Wd = circle->getFrame()->evalPosition() - contact.getContourFrame(inode)->getPosition(false);
      contact.getContourFrame(inode)->setXi(contact.getContourFrame(inode)->getOrientation(false).col(2).T() * Wd); // get contact parameter of second tangential direction
      contact.getContourFrame(inode)->getPosition(false) += contact.getContourFrame(inode)->getXi(false) * contact.getContourFrame(inode)->getOrientation(false).col(2);

      double g;
      if(extrusion->isZetaOutside(contact.getContourFrame(inode)->getZeta(false)))
        g = 1;
      else
        g = contact.getContourFrame(inode)->getOrientation(false).col(0).T() * (contact.getContourFrame(icircle)->getPosition(false) - contact.getContourFrame(inode)->getPosition(false));
      if(g < -extrusion->getThickness()) g = 1;
      contact.getGeneralizedRelativePosition(false)(0) = g;
    }
  }

  ContactKinematicsCircleFlexibleBand::~ContactKinematicsCircleFlexibleBand() {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++)
      delete contactKinematics[cK];
  }

  void ContactKinematicsCircleFlexibleBand::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      icontour = 1;
      circle = static_cast<Circle*>(contour[0]);
      extrusion = static_cast<Contour*>(contour[1]);
    }
    else {
      icircle = 1;
      icontour = 0;
      circle = static_cast<Circle*>(contour[1]);
      extrusion = static_cast<Contour*>(contour[0]);
    }

    staticNodes = extrusion->getEtaNodes();
    maxNumContacts = 2 * possibleContactsPerNode * extrusion->getEtaNodes().size() - 1;  // dies braeuchte einen eigenen init-Call
//    l0 = 1.0 * fabs(extrusion->getAlphaEnd() - extrusion->getAlphaStart()) / staticNodes.size(); /* bandwidth of mesh deformer: higher values leads to stronger attraction of last contact points */
//    epsTol = 5.e-2 * l0; /* distance, when two contact points should be treated as one */

    msg(Info) << maxNumContacts << endl;
    msg(Info) << possibleContactsPerNode << endl;
    msg(Info) << staticNodes.size() << endl;
    for (int i = 0; i < staticNodes.size(); i++) {
      auto *ck = new ContactKinematicsCircleNode(staticNodes(i));
      ck->assignContours(contour);
      contactKinematics.push_back(ck);
    }
    for (int i = 0; i < maxNumContacts - staticNodes.size(); i++) {
      ContactKinematicsCircleNodeInterpolation *ck = new ContactKinematicsCircleNodeInterpolation(staticNodes(i, i + 1));
      ck->assignContours(contour);
      contactKinematics.push_back(ck);
    }
  }

  void ContactKinematicsCircleFlexibleBand::updateg(vector<SingleContact> &contact) {
    for(size_t i=0; i<contactKinematics.size(); i++)
      contactKinematics[i]->updateg(contact[i]);
  }

}
