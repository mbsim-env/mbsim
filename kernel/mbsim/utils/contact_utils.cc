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

// --- List of contour implementations - BEGIN ---
#include "mbsim/contours/area.h"
#include "mbsim/contours/circle.h"
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/compound_contour.h"
#include "mbsim/contours/contour1s.h"
#include "mbsim/contours/contour_interpolation.h"
#include "mbsim/contours/contour_quad.h"
#include "mbsim/contours/cuboid.h"
#include "mbsim/contours/cylinder_flexible.h"
#include "mbsim/contours/edge.h"
#include "mbsim/contours/flexible_band.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/frustum2d.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/nurbs_disk_2s.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/planewithfrustum.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/sphere.h"
// --- List of contour implementations - END ---

// --- List of contact kinematic implementations - BEGIN ---
#include <mbsim/contact_kinematics/circle_frustum.h>
#include <mbsim/contact_kinematics/circle_nurbsdisk2s.h>
#include <mbsim/contact_kinematics/circlehollow_cylinderflexible.h>
#include <mbsim/contact_kinematics/circlesolid_circlehollow.h>
#include <mbsim/contact_kinematics/circlesolid_circlesolid.h>
#include <mbsim/contact_kinematics/circlesolid_contour1s.h>
#include <mbsim/contact_kinematics/circlesolid_frustum2d.h>
#include <mbsim/contact_kinematics/circlesolid_line.h>
#include <mbsim/contact_kinematics/circlesolid_plane.h>
#include <mbsim/contact_kinematics/compoundcontour_compoundcontour.h>
#include <mbsim/contact_kinematics/compoundcontour_contour.h>
#include <mbsim/contact_kinematics/edge_edge.h>
#include <mbsim/contact_kinematics/line_contour1s.h>
#include <mbsim/contact_kinematics/point_area.h>
#include <mbsim/contact_kinematics/point_contour1s.h>
#include <mbsim/contact_kinematics/point_contourinterpolation.h>
#include <mbsim/contact_kinematics/point_cylinderflexible.h>
#include <mbsim/contact_kinematics/point_flexibleband.h>
#include <mbsim/contact_kinematics/circlesolid_flexibleband.h>
#include <mbsim/contact_kinematics/point_frustum.h>
#include <mbsim/contact_kinematics/point_line.h>
#include <mbsim/contact_kinematics/point_nurbsdisk2s.h>
#include <mbsim/contact_kinematics/point_plane.h>
#include <mbsim/contact_kinematics/point_planewithfrustum.h>
#include <mbsim/contact_kinematics/sphere_frustum.h>
#include <mbsim/contact_kinematics/sphere_plane.h>
#include <mbsim/contact_kinematics/sphere_sphere.h>
// --- List of contact kinematic implementations - END ---

using namespace fmatvec;

namespace MBSim {

  double computeAngleOnUnitCircle(const Vec& r) {
    return r(1)>=0 ? acos(r(0)) : 2*M_PI-acos(r(0));
  }

  Vec computeAnglesOnUnitSphere(const Vec& r) {
    Vec zeta(2,NONINIT);
    double l = sqrt(r(0)*r(0) + r(1)*r(1));
    zeta(0)= r(1)>=0 ? acos(r(0)/l) : 2*M_PI-acos(r(0)/l);
    zeta(1)= asin(r(2));
    return zeta;
  }

  ContactKinematics* findContactPairing(Contour *contour0, Contour *contour1) {

	// evtl. besser, alle Contour-Paarungen zu testen, dann wird immer die hoechste Spezialisierung verwendet

    if((dynamic_cast<Circle*>(contour0) && dynamic_cast<Frustum*>(contour1)) || (dynamic_cast<Circle*>(contour1) && dynamic_cast<Frustum*>(contour0)))
      return new ContactKinematicsCircleFrustum;

    else if((dynamic_cast<Circle*>(contour0) && dynamic_cast<NurbsDisk2s*>(contour1)) || (dynamic_cast<Circle*>(contour1) && dynamic_cast<NurbsDisk2s*>(contour0)))
      return new ContactKinematicsCircleNurbsDisk2s;

     else if((dynamic_cast<CircleHollow*>(contour0) && dynamic_cast<CylinderFlexible*>(contour1)) || (dynamic_cast<CircleHollow*>(contour1) && dynamic_cast<CylinderFlexible*>(contour0))) 
       return new ContactKinematicsCircleHollowCylinderFlexible;

    else if((dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<CircleHollow*>(contour1)) || (dynamic_cast<CircleSolid*>(contour1) && dynamic_cast<CircleHollow*>(contour0)))
      return new ContactKinematicsCircleSolidCircleHollow;

    else if(dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<CircleSolid*>(contour1))
      return new ContactKinematicsCircleSolidCircleSolid;
//
    else if((dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<FlexibleBand*>(contour1)) || (dynamic_cast<CircleSolid*>(contour1) && dynamic_cast<FlexibleBand*>(contour0)))
      return new ContactKinematicsCircleSolidFlexibleBand;  
//
    else if((dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<Contour1s*>(contour1)) || (dynamic_cast<CircleSolid*>(contour1) && dynamic_cast<Contour1s*>(contour0)))
      return new ContactKinematicsCircleSolidContour1s;

    else if((dynamic_cast<Line*>(contour0) && dynamic_cast<Contour1s*>(contour1)) || (dynamic_cast<Line*>(contour1) && dynamic_cast<Contour1s*>(contour0))) 
      return new ContactKinematicsLineContour1s;

    else if((dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<Frustum2D*>(contour1)) || (dynamic_cast<CircleSolid*>(contour1) && dynamic_cast<Frustum2D*>(contour0))) 
      return new ContactKinematicsCircleSolidFrustum2D;

    else if((dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<Line*>(contour1)) || (dynamic_cast<CircleSolid*>(contour1) && dynamic_cast<Line*>(contour0))) 
      return new ContactKinematicsCircleSolidLine;

    else if((dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<Plane*>(contour1)) || (dynamic_cast<CircleSolid*>(contour1) && dynamic_cast<Plane*>(contour0))) 
      return new ContactKinematicsCircleSolidPlane;

    else if((dynamic_cast<CompoundContour*>(contour0) && dynamic_cast<Contour*>(contour1)) || (dynamic_cast<CompoundContour*>(contour1) && dynamic_cast<Contour*>(contour0))) 
      return new ContactKinematicsCompoundContourContour;  

    else if((dynamic_cast<CompoundContour*>(contour0) && dynamic_cast<CompoundContour*>(contour1))) 
      return new ContactKinematicsCompoundContourCompoundContour;  

    else if(dynamic_cast<Edge*>(contour0) && dynamic_cast<Edge*>(contour1)) 
      return new ContactKinematicsEdgeEdge;

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<Area*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<Area*>(contour0))) 
      return new ContactKinematicsPointArea;

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<FlexibleBand*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<FlexibleBand*>(contour0)))
      return new ContactKinematicsPointFlexibleBand;  

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<Contour1s*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<Contour1s*>(contour0))) 
      return new ContactKinematicsPointContour1s;

    else if((dynamic_cast<Point*>(contour0) &&  dynamic_cast<ContourInterpolation*>(contour1)) || (dynamic_cast<Point*>(contour1) &&  dynamic_cast<ContourInterpolation*>(contour0))) 
      return new ContactKinematicsPointContourInterpolation;

    // else if((dynamic_cast<Point*>(contour0) && dynamic_cast<CylinderFlexible*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<CylinderFlexible*>(contour0)))
    //   return new ContactKinematicsPointCylinderFlexible;

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<Frustum*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<Frustum*>(contour0)))
      return new ContactKinematicsPointFrustum;  

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<Line*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<Line*>(contour0))) 
      return new ContactKinematicsPointLine; 

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<NurbsDisk2s*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<NurbsDisk2s*>(contour0))) 
      return new ContactKinematicsPointNurbsDisk2s; 

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<Plane*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<Plane*>(contour0))) 
      return new ContactKinematicsPointPlane;

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<PlaneWithFrustum*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<PlaneWithFrustum*>(contour0))) 
      return new ContactKinematicsPointPlaneWithFrustum;

    else if((dynamic_cast<Sphere*>(contour0) && dynamic_cast<Frustum*>(contour1)) || (dynamic_cast<Sphere*>(contour1) && dynamic_cast<Frustum*>(contour0)))
      return new ContactKinematicsSphereFrustum;

    else if((dynamic_cast<Sphere*>(contour0) && dynamic_cast<Plane*>(contour1)) || (dynamic_cast<Sphere*>(contour1) && dynamic_cast<Plane*>(contour0))) 
      return new ContactKinematicsSpherePlane;

    else if(dynamic_cast<Sphere*>(contour0) && dynamic_cast<Sphere*>(contour1))
      return new ContactKinematicsSphereSphere;

    else 
      return 0;
  }
}

