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
 * Contact: rzander@users.berlios.de
 */

#include <config.h>
#include "mbsim/utils/contact_utils.h"
#include "stdio.h"

// --- List of contact kinematic implementations - BEGIN ---
//#include <mbsim/contact_kinematics/circle_frustum.h>
#include <mbsim/contact_kinematics/circlesolid_circlehollow.h>
#include <mbsim/contact_kinematics/circlesolid_circlesolid.h>
//#include <mbsim/contact_kinematics/circlesolid_contour1s.h>
//#include <mbsim/contact_kinematics/circlesolid_frustum2d.h>
#include <mbsim/contact_kinematics/circlesolid_line.h>
//#include <mbsim/contact_kinematics/circlesolid_linesegment.h>
//#include <mbsim/contact_kinematics/circlesolid_plane.h>
//#include <mbsim/contact_kinematics/compoundcontour_compoundcontour.h>
#include <mbsim/contact_kinematics/compoundcontour_contour.h>
//#include <mbsim/contact_kinematics/edge_edge.h>
//#include <mbsim/contact_kinematics/line_contour1s.h>
//#include <mbsim/contact_kinematics/point_area.h>
//#include <mbsim/contact_kinematics/point_contour1s.h>
//#include <mbsim/contact_kinematics/point_contourinterpolation.h>
//#include <mbsim/contact_kinematics/point_frustum.h>
#include <mbsim/contact_kinematics/point_line.h>
//#include <mbsim/contact_kinematics/point_circlesolid.h>
#include <mbsim/contact_kinematics/point_plane.h>
//#include <mbsim/contact_kinematics/point_planewithfrustum.h>
//#include <mbsim/contact_kinematics/sphere_frustum.h>
//#include <mbsim/contact_kinematics/sphere_plane.h>
//#include <mbsim/contact_kinematics/sphere_sphere.h>
//#include <mbsim/contact_kinematics/point_line_segment.h>
// --- List of contact kinematic implementations - END ---

namespace MBSim {

  double computeAngleOnUnitCircle(const fmatvec::Vec& r) {
    return r(1)>=0 ? acos(r(0)) : 2*M_PI-acos(r(0));
  }

  fmatvec::Vec computeAnglesOnUnitSphere(const fmatvec::Vec& r) {
    fmatvec::Vec zeta(2,fmatvec::NONINIT);
    double l = sqrt(r(0)*r(0) + r(1)*r(1));
    zeta(0)= r(1)>=0 ? acos(r(0)/l) : 2*M_PI-acos(r(0)/l);
    zeta(1)= asin(r(2));
    return zeta;
  }

  ContactKinematics* findContactPairingRigidRigid(const char* contour0, const char* contour1) {

////      if(( strcmp(contour0, "Circle")==0 && strcmp(contour1, "Frustum")==0 ) ||
////         ( strcmp(contour0, "CircleSolid")==0 && strcmp(contour1, "Frustum")==0 ) ||
////         ( strcmp(contour0, "CircleHollow")==0 && strcmp(contour1, "Frustum")==0 ) )
////        return new ContactKinematicsCircleFrustum;
////      
      if ( strcmp(contour0, "CircleSolid")==0 && strcmp(contour1, "CircleHollow")==0 )
        return new ContactKinematicsCircleSolidCircleHollow;
  
      else if ( strcmp(contour0, "CircleSolid")==0 && strcmp(contour1, "CircleSolid")==0 )
        return new ContactKinematicsCircleSolidCircleSolid;
////  
////      else if ( strcmp(contour0, "CircleSolid")==0 && strcmp(contour1, "Contour1sAnalytical")==0 )
////        return new ContactKinematicsCircleSolidContour1s;
////  
////      else if ( strcmp(contour0, "Line")==0 && strcmp(contour1, "Contour1sAnalytical")==0 )
////        return new ContactKinematicsLineContour1s;
////  
////      else if ( strcmp(contour0, "CircleSolid")==0 && strcmp(contour1, "Frustum2D")==0 )
////        return new ContactKinematicsCircleSolidFrustum2D;
////  
      else if ( strcmp(contour0, "CircleSolid")==0 && strcmp(contour1, "Line")==0 )
        return new ContactKinematicsCircleSolidLine;
  
////      else if ( strcmp(contour0, "CircleSolid")==0 && strcmp(contour1, "LineSegment")==0 )
////        return new ContactKinematicsCircleSolidLineSegment;
////  
////      else if ( strcmp(contour0, "CircleSolid")==0 && strcmp(contour1, "Plane")==0 )
////        return new ContactKinematicsCircleSolidPlane;
////      
      else if ( strcmp(contour0, "Cuboid")==0 && strcmp(contour1, "Plane")==0 )
        return new ContactKinematicsCompoundContourContour;  
////  
////      /*
////       *else if ( strcmp(contour0, "CompoundContour")==0 )
////       *  if ( strcmp(contour1, "CompoundContour")==0 )
////       *    return new ContactKinematicsCompoundContourCompoundContour;  
////       *  else 
////       *    return new ContactKinematicsCompoundContourContour;  
////       */
////  
////      else if ( strcmp(contour0, "Edge")==0 && strcmp(contour1, "Edge")==0 )
////        return new ContactKinematicsEdgeEdge;
////  
////      else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "Area")==0 )
////        return new ContactKinematicsPointArea;
////  
////      else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "Contour1s")==0 )
////        return new ContactKinematicsPointContour1s;
////  
////      else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "ContourInterpolation")==0 )
////        return new ContactKinematicsPointContourInterpolation;
////  
////      else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "Frustum")==0 )
////        return new ContactKinematicsPointFrustum;  
////  
      else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "Line")==0 )
       return new ContactKinematicsPointLine; 
////  
////      else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "CircleSolid")==0 )
////        return new ContactKinematicsPointCircleSolid; 
////  
      else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "Plane")==0 )
        return new ContactKinematicsPointPlane;
////  
////      else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "PlaneWithFrustum")==0 )
////        return new ContactKinematicsPointPlaneWithFrustum;
////  
////      else if ( strcmp(contour0, "Sphere")==0 && strcmp(contour1, "Frustum")==0 )
////        return new ContactKinematicsSphereFrustum;
////  
////      else if ( strcmp(contour0, "Sphere")==0 && strcmp(contour1, "Plane")==0 )
////        return new ContactKinematicsSpherePlane;
////  
////      else if ( strcmp(contour0, "Sphere")==0 && strcmp(contour1, "Sphere")==0 )
////        return new ContactKinematicsSphereSphere;
////  
////      else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "LineSegment")==0 )
////        return new ContactKinematicsPointLineSegment; 
////  
    else
      return 0;
  }
}

