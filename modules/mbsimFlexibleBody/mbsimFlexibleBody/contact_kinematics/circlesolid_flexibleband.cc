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
 *          rzander@users.berlios.de
 
 */

#include<config.h>
#include "mbsimFlexibleBody/contact_kinematics/circlesolid_flexibleband.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/contours/contour.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/point.h"
#include "mbsim/functions_contact.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  class ContactKinematicsCircleSolidNode : public MBSim::ContactKinematics {
    public:
      ContactKinematicsCircleSolidNode(double rCircle_, double wBand_, double node_) :
          rCircle(rCircle_), wBand(wBand_), node(node_), circle(0), band(0) {
      }
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(double t, double &g, MBSim::ContourPointData *cpData, int index = 0);
      virtual void updatewb(double t, fmatvec::Vec &wb, double g, ContourPointData* cpData) {
        throw MBSimError("not implemented");
      }
    private:
      double rCircle, wBand, node;
      int icircle, inode;
      MBSim::CircleSolid *circle;
      FlexibleBand *band;
  };

  void ContactKinematicsCircleSolidNode::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0;
      inode = 1;
      circle = static_cast<CircleSolid*>(contour[0]);
      band = static_cast<FlexibleBand*>(contour[1]);
    }
    else {
      icircle = 1;
      inode = 0;
      circle = static_cast<CircleSolid*>(contour[1]);
      band = static_cast<FlexibleBand*>(contour[0]);
    }
  }

  void ContactKinematicsCircleSolidNode::updateg(double t, double &g, MBSim::ContourPointData *cpData, int index) {
    throw;
//    // set lagrange parateter position of current node
//
//    cpData[inode].getLagrangeParameterPosition()(0) = node;
//    cpData[inode].getLagrangeParameterPosition()(1) = 0;
//
//    if (cpData[inode].getLagrangeParameterPosition()(0) < band->getAlphaStart() || cpData[inode].getLagrangeParameterPosition()(0) > band->getAlphaEnd())
//      g = 1.0;
//    else {
//        band->updateKinematicsForFrame(cpData[inode],Frame::position_cosy);
//      Vec Wd = circle->getFrame()->getPosition() - cpData[inode].getFrameOfReference().getPosition();
//      g = nrm2(Wd) - rCircle;
//
//      Vec Wb = cpData[inode].getFrameOfReference().getOrientation().col(2);
//      cpData[inode].getLagrangeParameterPosition()(1) = trans(Wb) * Wd; // get contact parameter of second tangential direction
//
//      if (fabs(cpData[inode].getLagrangeParameterPosition()(1)) > 0.5 * wBand)
//        g = 1.0;
//      else { // calculate the normal distance
//        cpData[inode].getFrameOfReference().getPosition() += cpData[inode].getLagrangeParameterPosition()(1) * Wb;
//        cpData[icircle].getFrameOfReference().getOrientation().set(0, -cpData[inode].getFrameOfReference().getOrientation().col(0));
//        cpData[icircle].getFrameOfReference().getOrientation().set(1, -cpData[inode].getFrameOfReference().getOrientation().col(1));
//        cpData[icircle].getFrameOfReference().getOrientation().set(2, cpData[inode].getFrameOfReference().getOrientation().col(2));
//        //cpData[icircle] .getFrameOfReference().getPosition()            =  circle->getFrame()->getPosition() + cpData[icircle].getFrameOfReference().getOrientation().col(0)*rCircle;
//        cpData[icircle].getFrameOfReference().getPosition() = circle->getFrame()->getPosition() - rCircle * Wd / nrm2(Wd);
//
//      }
//    }
  }

  class ContactKinematicsCircleSolidNodeInterpolation : public MBSim::ContactKinematics {
    public:
      ContactKinematicsCircleSolidNodeInterpolation(double rCircle_, double wBand_, Vec nodes_) :
          rCircle(rCircle_), wBand(wBand_), nodes(nodes_), circle(0), band(0) {
      }
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(double t, double &g, MBSim::ContourPointData *cpData, int index = 0);
      virtual void updatewb(double t, fmatvec::Vec &wb, double g, ContourPointData* cpData) {
        throw MBSimError("not implemented");
      }
    private:
      double rCircle, wBand;
      Vec nodes;
      int icircle, inode;
      MBSim::CircleSolid *circle;
      FlexibleBand *band;
  };

  void ContactKinematicsCircleSolidNodeInterpolation::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0;
      inode = 1;
      circle = static_cast<CircleSolid*>(contour[0]);
      band = static_cast<FlexibleBand*>(contour[1]);
    }
    else {
      icircle = 1;
      inode = 0;
      circle = static_cast<CircleSolid*>(contour[1]);
      band = static_cast<FlexibleBand*>(contour[0]);
    }
  }

  void ContactKinematicsCircleSolidNodeInterpolation::updateg(double t, double &gNri, MBSim::ContourPointData *cpData, int index) {
    throw;
//    FuncPairContour1sCircleSolid *func = new FuncPairContour1sCircleSolid(circle, band); // root function for searching contact parameters
//    Contact1sSearch search(func);
//
//    search.setNodes(nodes); // defining search areas for contacts
//    Mat result = search.slvAll();
//    delete func;
//
//    if (result.rows()) {
//
//      cpData[inode].getLagrangeParameterPosition()(0) = result(0, 0);
//      cpData[inode].getLagrangeParameterPosition()(1) = 0.0;
//
//      if (cpData[inode].getLagrangeParameterPosition()(0) < band->getAlphaStart() || cpData[inode].getLagrangeParameterPosition()(0) > band->getAlphaEnd())
//        gNri = 1.0;
//      else {
//          band->updateKinematicsForFrame(cpData[inode],Frame::position_cosy);
//        Vec Wd = circle->getFrame()->getPosition() - cpData[inode].getFrameOfReference().getPosition();
//        gNri = nrm2(Wd) - rCircle;
//
//        {
//          Vec Wb = cpData[inode].getFrameOfReference().getOrientation().col(2);
//          cpData[inode].getLagrangeParameterPosition()(1) = trans(Wb) * Wd; // get contact parameter of second tangential direction
//
//          if (fabs(cpData[inode].getLagrangeParameterPosition()(1)) > 0.5 * wBand)
//            ;
//          else { // calculate the normal distance
//            cpData[inode].getFrameOfReference().getPosition() += cpData[inode].getLagrangeParameterPosition()(1) * Wb;
//            cpData[icircle].getFrameOfReference().getOrientation().set(0, -cpData[inode].getFrameOfReference().getOrientation().col(0));
//            cpData[icircle].getFrameOfReference().getOrientation().set(1, -cpData[inode].getFrameOfReference().getOrientation().col(1));
//            cpData[icircle].getFrameOfReference().getOrientation().set(2, cpData[inode].getFrameOfReference().getOrientation().col(2));
//            cpData[icircle].getFrameOfReference().getPosition() = circle->getFrame()->getPosition() + cpData[icircle].getFrameOfReference().getOrientation().col(0) * rCircle;
//          }
//        }
//      }
//    }
  }

  ContactKinematicsCircleSolidFlexibleBand::ContactKinematicsCircleSolidFlexibleBand() :
      ContactKinematics(), icircle(0), icontour(0), possibleContactsPerNode(1), circle(0), band(0) {
  }
  ContactKinematicsCircleSolidFlexibleBand::~ContactKinematicsCircleSolidFlexibleBand() {
  }

  void ContactKinematicsCircleSolidFlexibleBand::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0;
      icontour = 1;
      circle = static_cast<CircleSolid*>(contour[0]);
      band = static_cast<FlexibleBand*>(contour[1]);
    }
    else {
      icircle = 1;
      icontour = 0;
      circle = static_cast<CircleSolid*>(contour[1]);
      band = static_cast<FlexibleBand*>(contour[0]);
    }
    wBand = band->getWidth();
    hBand = band->getNormalDistance();
    rCircle = circle->getRadius();

    staticNodes = band->getNodes();
    numberOfPotentialContactPoints = 2 * possibleContactsPerNode * band->getNodes().size() - 1;  // dies braeuchte einen eigenen init-Call
    l0 = 1.0 * fabs(band->getAlphaEnd() - band->getAlphaStart()) / staticNodes.size(); /* bandwidth of mesh deformer: higher values leads to stronger attraction of last contact points */
    epsTol = 5.e-2 * l0; /* distance, when two contact points should be treated as one */

    cout << numberOfPotentialContactPoints << endl;
    cout << possibleContactsPerNode << endl;
    cout << staticNodes.size() << endl;
    for (int i = 0; i < staticNodes.size(); i++) {
      ContactKinematicsCircleSolidNode *ck = new ContactKinematicsCircleSolidNode(rCircle, wBand, staticNodes(i));
      ck->assignContours(contour);
      contactKinematics.push_back(ck);
    }
    for (int i = 0; i < numberOfPotentialContactPoints - staticNodes.size(); i++) {
      ContactKinematicsCircleSolidNodeInterpolation *ck = new ContactKinematicsCircleSolidNodeInterpolation(rCircle, wBand, staticNodes(i, i + 1));
      ck->assignContours(contour);
      contactKinematics.push_back(ck);
    }
  }

}

