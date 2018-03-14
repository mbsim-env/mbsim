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
#include "mbsim/utils/contact_utils.h"
#include <cstdio>

// --- List of contact implementations - BEGIN ---
#include <mbsim/contours/circle.h>
#include <mbsim/contours/compound_contour.h>
#include <mbsim/contours/contour_interpolation.h>
#include <mbsim/contours/cuboid.h>
#include <mbsim/contours/edge.h>
#include <mbsim/contours/frustum.h>
#include <mbsim/contours/line.h>
#include <mbsim/contours/line_segment.h>
#include <mbsim/contours/planar_contour.h>
#include <mbsim/contours/planar_frustum.h>
#include <mbsim/contours/plane.h>
#include <mbsim/contours/planewithfrustum.h>
#include <mbsim/contours/plate.h>
#include <mbsim/contours/point.h>
#include <mbsim/contours/room.h>
#include <mbsim/contours/spatial_contour.h>
#include <mbsim/contours/sphere.h>
#include <mbsim/contours/fcl_box.h>
// --- List of contact implementations - END ---

// --- List of contact kinematic implementations - BEGIN ---
#include <mbsim/contact_kinematics/circle_frustum.h>
#include <mbsim/contact_kinematics/circle_circle.h>
#include <mbsim/contact_kinematics/circle_planarcontour.h>
#include <mbsim/contact_kinematics/circle_planarfrustum.h>
#include <mbsim/contact_kinematics/circle_line.h>
#include <mbsim/contact_kinematics/circle_linesegment.h>
#include <mbsim/contact_kinematics/circle_plane.h>
#include <mbsim/contact_kinematics/compoundcontour_compoundcontour.h>
#include <mbsim/contact_kinematics/compoundcontour_contour.h>
#include <mbsim/contact_kinematics/edge_edge.h>
#include <mbsim/contact_kinematics/line_planarcontour.h>
#include <mbsim/contact_kinematics/point_plate.h>
#include <mbsim/contact_kinematics/point_planarcontour.h>
#include <mbsim/contact_kinematics/point_contourinterpolation.h>
#include <mbsim/contact_kinematics/point_frustum.h>
#include <mbsim/contact_kinematics/point_line.h>
#include <mbsim/contact_kinematics/point_circle.h>
#include <mbsim/contact_kinematics/point_plane.h>
#include <mbsim/contact_kinematics/point_sphere.h>
#include <mbsim/contact_kinematics/point_planewithfrustum.h>
#include <mbsim/contact_kinematics/point_linesegment.h>
#include <mbsim/contact_kinematics/sphere_frustum.h>
#include <mbsim/contact_kinematics/sphere_plane.h>
#include <mbsim/contact_kinematics/sphere_plate.h>
#include <mbsim/contact_kinematics/sphere_polynomialfrustum.h>
#include <mbsim/contact_kinematics/sphere_sphere.h>
#include <mbsim/contact_kinematics/plate_polynomialfrustum.h>
#include <mbsim/contact_kinematics/point_polynomialfrustum.h>
#include <mbsim/contact_kinematics/point_spatialcontour.h>
#include <mbsim/contact_kinematics/plane_spatialcontour.h>
#include <mbsim/contact_kinematics/fclcontour_fclcontour.h>
// --- List of contact kinematic implementations - END ---

using namespace std;

namespace MBSim {

  double computeAngleOnUnitCircle(const fmatvec::Vec3& r) {
    return r(1)>=0 ? acos(r(0)) : 2*M_PI-acos(r(0));
  }

  fmatvec::Vec2 computeAnglesOnUnitSphere(const fmatvec::Vec3& r) {
    fmatvec::Vec2 zeta(fmatvec::NONINIT);
    double l = sqrt(r(0)*r(0) + r(1)*r(1));
    zeta(0) = r(1)>=0 ? acos(r(0)/l) : 2*M_PI-acos(r(0)/l);
    zeta(1) = asin(r(2));
    return zeta;
  }

  ContactKinematics* findContactPairingRigidRigid(const type_info &contour0, const type_info &contour1) {

    if ( contour0==typeid(Circle) && contour1==typeid(Frustum) )
      return new ContactKinematicsCircleFrustum;
    
    else if ( contour0==typeid(Circle) && contour1==typeid(Circle) )
      return new ContactKinematicsCircleCircle;

    else if ( contour0==typeid(Circle) && contour1==typeid(PlanarContour) )
      return new ContactKinematicsCirclePlanarContour;

    else if ( contour0==typeid(Line) && contour1==typeid(PlanarContour) )
      return new ContactKinematicsLinePlanarContour;

    else if ( contour0==typeid(Circle) && contour1==typeid(PlanarFrustum) )
      return new ContactKinematicsCirclePlanarFrustum;

    else if ( contour0==typeid(Circle) && contour1==typeid(Line) )
      return new ContactKinematicsCircleLine;

    else if ( contour0==typeid(Circle) && contour1==typeid(LineSegment) )
      return new ContactKinematicsCircleLineSegment;

    else if ( contour0==typeid(Circle) && contour1==typeid(Plane) )
      return new ContactKinematicsCirclePlane;
    
    else if (( contour0==typeid(Cuboid) && contour1==typeid(Plane) ) or
        ( contour0==typeid(Room) && contour1==typeid(Point) ) or
        ( contour0==typeid(Cuboid) && contour1==typeid(Frustum) ))
      return new ContactKinematicsCompoundContourContour;

    else if (( contour0==typeid(Cuboid) && contour1==typeid(Room) ) or
        ( contour0==typeid(Cuboid) && contour1==typeid(Cuboid) ))
      return new ContactKinematicsCompoundContourCompoundContour(4);

    else if ( contour0==typeid(Edge) && contour1==typeid(Edge) )
      return new ContactKinematicsEdgeEdge;

    else if ( contour0==typeid(Point) && contour1==typeid(Plate) )
      return new ContactKinematicsPointPlate;

    else if ( contour0==typeid(Point) && contour1==typeid(ContourInterpolation) )
      return new ContactKinematicsPointContourInterpolation;

    else if ( contour0==typeid(Point) && contour1==typeid(Frustum) )
      return new ContactKinematicsPointFrustum;

    else if ( contour0==typeid(Point) && contour1==typeid(PolynomialFrustum) )
      return new ContactKinematicsPointPolynomialFrustum;

    else if ( contour0==typeid(Point) && contour1==typeid(Line) )
      return new ContactKinematicsPointLine; 

    else if ( contour0==typeid(Point) && contour1==typeid(Circle) )
      return new ContactKinematicsPointCircle;

    else if ( contour0==typeid(Point) && contour1==typeid(Plane) )
      return new ContactKinematicsPointPlane;

    else if ( contour0==typeid(Point) && contour1==typeid(Sphere) )
      return new ContactKinematicsPointSphere;

    else if ( contour0==typeid(Point) && contour1==typeid(PlaneWithFrustum) )
      return new ContactKinematicsPointPlaneWithFrustum;

    else if ( contour0==typeid(Point) && contour1==typeid(LineSegment) )
      return new ContactKinematicsPointLineSegment;

    else if ( contour0==typeid(Point) && contour1==typeid(PlanarContour) )
      return new ContactKinematicsPointPlanarContour;

    else if ( contour0==typeid(Point) && contour1==typeid(SpatialContour) )
      return new ContactKinematicsPointSpatialContour;

    else if ( contour0==typeid(Sphere) && contour1==typeid(Frustum) )
      return new ContactKinematicsSphereFrustum;

    else if ( contour0==typeid(Sphere) && contour1==typeid(Plane) )
      return new ContactKinematicsSpherePlane;

    else if ( contour0==typeid(Sphere) && contour1==typeid(Plate) )
      return new ContactKinematicsSpherePlate;

    else if ( contour0==typeid(Sphere) && contour1==typeid(PolynomialFrustum) )
      return new ContactKinematicsSpherePolynomialFrustum;

    else if ( contour0==typeid(Sphere) && contour1==typeid(Sphere) )
      return new ContactKinematicsSphereSphere;

    else if ( contour0==typeid(Plate) && contour1==typeid(PolynomialFrustum) )
      return new ContactKinematicsPlatePolynomialFrustum;

    else if ( contour0==typeid(Plane) && contour1==typeid(SpatialContour) )
      return new ContactKinematicsPlaneSpatialContour;

    else if ( contour0==typeid(FCLBox) && contour1==typeid(FCLBox) )
      return new ContactKinematicsFCLContourFCLContour(4);

    else
      return nullptr;
  }
}
