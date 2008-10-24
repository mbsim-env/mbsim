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

  Contact::Contact(const string &name, bool setValued) : Link(name,setValued), hca(true), nhca(true), closed(true), sticking(true), contactKinematics(0) {

    //active = false;
  }

  Contact::Contact(const Contact *master, const string &name) : Link(name,master->setValued), iT(1,master->iT.end()), contactKinematics(0) {

   // mu = master->mu; TODO

    //active = false;
  }

  Contact::~Contact() {
    if (contactKinematics) delete contactKinematics;
  }
  
  void Contact::calcxSize() {
    Link::calcxSize();
    xSize = 0;
    //gSize = 1;
    //gdSize = 1+getFrictionDirections();
    //laSize = 1+getFrictionDirections();
    //rFactorSize = setValued?1+min(getFrictionDirections(),1):0;
  }

  void Contact::calclaSize() {
    Link::calclaSize();
    laSize = hca ? (nhca ? 1+getFrictionDirections() : 1) : 0;
  }

  void Contact::calcgSize() {
    Link::calcgSize();
    gSize = hca ? 1 : 0;
  }

  void Contact::calcgdSize() {
    Link::calcgdSize();
    gdSize = hca ? (nhca ? 1+getFrictionDirections() : 1) : 0;
  }

  void Contact::calcrFactorSize() {
    Link::calcrFactorSize();
    rFactorSize = setValued ? (hca ? (nhca ? 1+min(getFrictionDirections(),1) : 1) : 0) : 0;
  }

  void Contact::updateWRef(const Mat& WParent) {
   for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->gethInd(),contour[i]->gethInd()+contour[i]->gethSize()-1);
      W[i]>>WParent(I,J);
    }
  } 

  void Contact::updateVRef(const Mat& VParent) {
    for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->gethInd(),contour[i]->gethInd()+contour[i]->getWJP().cols()-1);
      V[i]>>VParent(I,J);
    }
  } 

  void Contact::updater(double t) {

    for(unsigned i=0; i<contour.size(); i++) 
      r[i] += V[i]*la;
      //r[i] += (W[i]+V[i])*la;
  }

  void Contact::updateb(double t) {
    for(unsigned i=0; i<contour.size(); i++) 
      b += trans(fF[i](Index(0,2),Index(0,laSize-1)))*contour[i]->getMovingFrame()->getGyroscopicAccelerationOfTranslation();
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

    ContourPointData cpd[2];
    cpData.push_back(cpd[0]);
    cpData.push_back(cpd[1]);
    cpData[0].type = CONTINUUM; // default-Wert
    cpData[0].Wn.resize(3,1);
    cpData[0].Wt.resize(3,getFrictionDirections());
    cpData[1].type = CONTINUUM; // default-Wert
    cpData[1].Wn.resize(3,1);
    cpData[1].Wt.resize(3,getFrictionDirections());
    iT = Index(1,getFrictionDirections());
    connectHitSpheres(contour[0],contour[1]);
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
    contactKinematics->stage1(g,cpData);
  }

  void Contact::updategd(double t) {
    contactKinematics->stage2(g,gd,cpData);
  }

  bool Contact::activeConstraintsChanged() {
    return activeHolonomicConstraintsChanged() || activeNonHolonomicConstraintsChanged();
  }

  bool Contact::activeNonHolonomicConstraintsChanged() {
    bool changed = false;
    if(getFrictionDirections()) {
      changed = sticking != isSticking();
      sticking = isSticking();
    }
    return changed;
  }
  bool Contact::activeHolonomicConstraintsChanged() {
    bool changed = closed != isClosed();
    closed = isClosed();
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

