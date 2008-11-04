/* Copyright (C) 2004-2006  Martin Förg, Roland Zander
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *   rzander@users.berlios.de
 *
 */
#include <config.h> 
#include "contact.h"
#include "object.h"
#include "contour.h"
#include "functions_contact.h"
#include "multi_body_system.h"
#include "nonlinear_algebra.h"

// --- List of contact kinematic implementations - BEGIN ---
#include "point_line.h"
#include "circlesolid_contour1s.h"
#include "circlesolid_line.h"
#include "circlesolid_plane.h"
#include "point_plane.h"
#include "point_area.h"
#include "edge_edge.h"
#include "circlehollow_cylinderflexible.h"
#include "point_cylinderflexible.h"
#include "sphere_plane.h"
#include "point_contourinterpolation.h"
#include "point_contour1s.h"
#include "line_contour1s.h"
#include "circlesolid_circlehollow.h"
#include "circlesolid_circlesolid.h"
#include "sphere_sphere.h"
#include "sphere_frustum.h"
#include "point_frustum.h"
#include "circlesolid_frustum2d.h"
// --- List of contact kinematic implementations -  END  ---

namespace MBSim {

  int min(int i, int j) {
    return i<j?i:j;
  }

  Contact::Contact(const string &name, bool setValued) : Link(name,setValued), contactKinematics(0) {

    gActive = 1;
    gdActive[0] = 1;
    gdActive[1] = 1;

    gActive0 = 1;
    gdActive0[0] = 1;
    gdActive0[1] = 1;
  }

  Contact::~Contact() {
    if (contactKinematics) delete contactKinematics;
  }
  
  void Contact::calcxSize() {
    Link::calcxSize();
    xSize = 0;
  }

  void Contact::calclaSize() {
    Link::calclaSize();
    laSize = gdActive[0]+gdActive[1]*getFrictionDirections();
  }

  void Contact::calcgSize() {
    Link::calcgSize();
    gSize = gActive;
  }

  void Contact::calcgdSize() {
    Link::calcgdSize();
    gdSize = gdActive[0]+gdActive[1]*getFrictionDirections();
  }

  void Contact::calcrFactorSize() {
    Link::calcrFactorSize();
    rFactorSize = gdActive[0]+min(getFrictionDirections(),1)*gdActive[1];
  }

  void Contact::updateWRef(const Mat& WParent) {
   for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->gethInd(),contour[i]->gethInd()+contour[i]->gethSize()-1);
      W[i].resize()>>WParent(I,J);
    }
  } 

  void Contact::updateVRef(const Mat& VParent) {
    for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->gethInd(),contour[i]->gethInd()+contour[i]->getWJP().cols()-1);
      V[i].resize()>>VParent(I,J);
    }
  } 

  void Contact::updater(double t) {

    for(unsigned i=0; i<contour.size(); i++) 
      r[i] += V[i]*la;
  }

  void Contact::init() {
    Link::init();

    g.resize(1);
    gd.resize(1+getFrictionDirections());
    la.resize(1+getFrictionDirections());

    if(contactKinematics);

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<Line*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<Line*>(contour[0]))) 
      contactKinematics = new ContactKinematicsPointLine; 

    else if((dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<Line*>(contour[1])) || (dynamic_cast<CircleSolid*>(contour[1]) && dynamic_cast<Line*>(contour[0]))) 
      contactKinematics = new ContactKinematicsCircleSolidLine;

    else if((dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<Plane*>(contour[1])) || (dynamic_cast<CircleSolid*>(contour[1]) && dynamic_cast<Plane*>(contour[0]))) 
      contactKinematics = new ContactKinematicsCircleSolidPlane;

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<Plane*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<Plane*>(contour[0]))) 
      contactKinematics = new ContactKinematicsPointPlane;

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<Area*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<Area*>(contour[0]))) 
      contactKinematics = new ContactKinematicsPointArea;

    else if(dynamic_cast<Edge*>(contour[0]) && dynamic_cast<Edge*>(contour[1])) 
      contactKinematics = new ContactKinematicsEdgeEdge;

    else if((dynamic_cast<Sphere*>(contour[0]) && dynamic_cast<Plane*>(contour[1])) || (dynamic_cast<Sphere*>(contour[1]) && dynamic_cast<Plane*>(contour[0]))) 
      contactKinematics = new ContactKinematicsSpherePlane;

    // INTERPOLATIONSGESCHICHTEN - Interpol-Point
    else if((dynamic_cast<Point*>(contour[0]) &&  dynamic_cast<ContourInterpolation*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) &&  dynamic_cast<ContourInterpolation*>(contour[0]))) 
      contactKinematics = new ContactKinematicsPointContourInterpolation;
    // INTERPOLATIONSGESCHICHTEN

    else if((dynamic_cast<CircleHollow*>(contour[0]) && dynamic_cast<CylinderFlexible*>(contour[1])) || (dynamic_cast<CircleHollow*>(contour[1]) && dynamic_cast<CylinderFlexible*>(contour[0]))) 
      contactKinematics = new ContactKinematicsCircleHollowCylinderFlexible;

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<CylinderFlexible*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<CylinderFlexible*>(contour[0])))
      contactKinematics = new ContactKinematicsPointCylinderFlexible;

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<Contour1s*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<Contour1s*>(contour[0]))) 
      contactKinematics = new ContactKinematicsPointContour1s;

    else if((dynamic_cast<Line*>(contour[0]) && dynamic_cast<Contour1s*>(contour[1])) || (dynamic_cast<Line*>(contour[1]) && dynamic_cast<Contour1s*>(contour[0]))) 
      contactKinematics = new ContactKinematicsLineContour1s;

    else if((dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<Contour1s*>(contour[1])) || (dynamic_cast<CircleSolid*>(contour[1]) && dynamic_cast<Contour1s*>(contour[0])))
      contactKinematics = new ContactKinematicsCircleSolidContour1s;


    else if((dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<CircleHollow*>(contour[1])) || (dynamic_cast<CircleSolid*>(contour[1]) && dynamic_cast<CircleHollow*>(contour[0])))
      contactKinematics = new ContactKinematicsCircleSolidCircleHollow;

    else if(dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<CircleSolid*>(contour[1]))
      contactKinematics = new ContactKinematicsCircleSolidCircleSolid;

    else if(dynamic_cast<Sphere*>(contour[0]) && dynamic_cast<Sphere*>(contour[1]))
      contactKinematics = new ContactKinematicsSphereSphere;

    else if((dynamic_cast<Sphere*>(contour[0]) && dynamic_cast<Frustum*>(contour[1])) || (dynamic_cast<Sphere*>(contour[1]) && dynamic_cast<Frustum*>(contour[0])))
      contactKinematics = new ContactKinematicsSphereFrustum;

    else if((dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<Frustum2D*>(contour[1])) || (dynamic_cast<CircleSolid*>(contour[1]) && dynamic_cast<Frustum2D*>(contour[0]))) 
      contactKinematics = new ContactKinematicsCircleSolidFrustum2D;

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<Frustum*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<Frustum*>(contour[0])))
      contactKinematics = new ContactKinematicsPointFrustum;  

    else {
      cout << "Unkown contact pairing" <<endl;
      throw 5;
    }

    //cpData.push_back(new ContourPointData);
    //cpData.push_back(new ContourPointData);
    cpData[0].type = CONTINUUM; // default-Wert
    cpData[0].Wn.resize(3,1);
    cpData[0].Wt.resize(3,getFrictionDirections());
    cpData[1].type = CONTINUUM; // default-Wert
    cpData[1].Wn.resize(3,1);
    cpData[1].Wt.resize(3,getFrictionDirections());
    
    cpData[0].cosy.setName("0");
    cpData[1].cosy.setName("1");
    cpData[0].cosy.getJacobianOfTranslation().resize(3,contour[0]->getWJP().cols());
    cpData[0].cosy.getJacobianOfRotation().resize(3,contour[0]->getWJR().cols());
    cpData[1].cosy.getJacobianOfTranslation().resize(3,contour[1]->getWJP().cols());
    cpData[1].cosy.getJacobianOfRotation().resize(3,contour[1]->getWJR().cols());

    //connectHitSpheres(contour[0],contour[1]);

    iT = Index(1,getFrictionDirections());
    contactKinematics->assignContours(contour);
  }

  void Contact::connect(Contour *contour0, Contour* contour1) {
    Link::connect(contour0,0);
    Link::connect(contour1,1);
  }

  void Contact::connectHitSpheres(Contour *contour0, Contour* contour1) {
    // wird eh erst im init() ausgefuehrt
   // if(checkHSLink) {
   //   Object* obj0 = contour0->getObject();
   //   Object* obj1 = contour1->getObject();

   //   HSLink = parent->getHitSphereLink(obj0,obj1);
   //   HSLink->setParents(obj0,obj1,this);
   // }
  }

  void Contact::updateg(double t) {
    contactKinematics->updateg(g,cpData);
      for(unsigned int i=0; i<2; i++) {
      Vec WrPC = cpData[i].cosy.getPosition() - contour[i]->getCoordinateSystem()->getPosition();

      cpData[i].cosy.setAngularVelocity(contour[i]->getCoordinateSystem()->getAngularVelocity());
      cpData[i].cosy.setVelocity(contour[i]->getCoordinateSystem()->getVelocity() + crossProduct(contour[i]->getCoordinateSystem()->getAngularVelocity(),WrPC));

      Mat tWrPC = tilde(WrPC);
      cpData[i].cosy.setJacobianOfTranslation(contour[i]->getCoordinateSystem()->getJacobianOfTranslation() - tWrPC*contour[i]->getCoordinateSystem()->getJacobianOfRotation());
      cpData[i].cosy.setJacobianOfRotation(contour[i]->getCoordinateSystem()->getJacobianOfRotation());
      cpData[i].cosy.setGyroscopicAccelerationOfTranslation(contour[i]->getCoordinateSystem()->getGyroscopicAccelerationOfTranslation() - tWrPC*contour[i]->getCoordinateSystem()->getGyroscopicAccelerationOfRotation() + crossProduct(contour[i]->getCoordinateSystem()->getAngularVelocity(),crossProduct(contour[i]->getCoordinateSystem()->getAngularVelocity(),WrPC)));
      cpData[i].cosy.setGyroscopicAccelerationOfRotation(contour[i]->getCoordinateSystem()->getGyroscopicAccelerationOfRotation());
    }

  }

  void Contact::updategd(double t) {
  
    Vec Wn = cpData[1].cosy.getOrientation().col(1);

    Vec WvD = cpData[1].cosy.getVelocity() - cpData[0].cosy.getVelocity();

    gd(0) = trans(Wn)*WvD;

    if(gd.size()>1) {
      Mat Wt(3,gd.size()-1);
      Wt.col(0) = cpData[1].cosy.getOrientation().col(0);
      if(gd.size() > 2)
	Wt.col(1) = cpData[1].cosy.getOrientation().col(2);

      gd(1,gd.size()-1) = trans(Wt)*WvD;
    }

   }

//  bool Contact::activeConstraintsChanged() {
//    return activeHolonomicConstraintsChanged() || activeNonHolonomicConstraintsChanged();
//  }

//  bool Contact::activeConstraintsChanged() {
//    bool changed = gActive0 != gActive;
//    gActive0 = gActive;
//    bool changed2 = false;
//    bool changed1 = gdActive0[0] != gdActive[0];
//    gdActive0[0] = gdActive[0];
//    if(getFrictionDirections()) {
//      changed2= gdActive0[1] != gdActive[1];
//      gdActive0[1] = gdActive[1];
//    }
//    return changed || changed1 || changed2;
//  }

  bool Contact::gActiveChanged() {
    bool changed = gActive0 != gActive;
    gActive0 = gActive;
    return changed;
  }

  void Contact::save(const string& path, ofstream &outputfile) {
    Link::save(path, outputfile);

   // outputfile << "# Number of friction directions:" << endl;
   // outputfile << nFric << endl;
   // outputfile << endl;

   // outputfile << "# Friction coefficient:" << endl;
   // //outputfile << mu << endl;
   // outputfile << endl;
  }

  void Contact::load(const string& path, ifstream &inputfile) {
    Link::load(path,inputfile);
   // string dummy;

   // getline(inputfile,dummy); // # Number of friction directions
   // inputfile >> nFric;
   // getline(inputfile,dummy); // Rest of line
   // getline(inputfile,dummy); // Newline

   // getline(inputfile,dummy); // # Friction coefficient
   // //inputfile >> mu;
   // getline(inputfile,dummy); // Rest of line
   // getline(inputfile,dummy); // Newline
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// ZUR NUMERISCHEN BESTIMMUNG DER GENERALISIERTEN KRAFTRICHTUNGEN WN
/////////////////////////////////////////////////////////////////////////////////////////////////////////

//       if (false) {
// // Numerische Ableitung gp(0) -> WN    
//     static bool rootCall = true;
//     static double eps = 5.e-9;

//     if(rootCall) {
// 	rootCall = false;

// 	int WNSize = cylinder->getObject()->getqSize();
// 	Mat WN(WNSize,1);
// 	const double gAlt = g(0);

// 	for(int i=0;i<WNSize;i++) {
// 	    Vec qTemp = cylinder->getObject()->getq();
// 	    qTemp(i) += eps;
// 	    cylinder->getObject()->setq(qTemp);
// 	    pairingCircleHollowCylinderFlexibleStage1(icircle,icylinder);
// 	    WN(i,0) = (g(0) - gAlt) / eps ;
// 	    qTemp(i) -= eps;
// 	    cylinder->getObject()->setq(qTemp);
// 	}

// 	// reset
// 	pairingCircleHollowCylinderFlexibleStage2(icircle,icylinder);

// // 	cout << "numerisches WN = " << trans(WN) << endl;

// 	rootCall = true;
//     }

//       }
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

