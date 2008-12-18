/* Copyright (C) 2006  Roland Zander
 
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
 *   rzander@users.berlios.de
 *
 */

#include <config.h>
#include "contact_utils.h"
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
#include "compoundcontour_contour.h"
#include "compoundcontour_compoundcontour.h"

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

    if((dynamic_cast<Point*>(contour0) && dynamic_cast<Line*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<Line*>(contour0))) 
      return new ContactKinematicsPointLine; 

    else if((dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<Line*>(contour1)) || (dynamic_cast<CircleSolid*>(contour1) && dynamic_cast<Line*>(contour0))) 
      return new ContactKinematicsCircleSolidLine;

    else if((dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<Plane*>(contour1)) || (dynamic_cast<CircleSolid*>(contour1) && dynamic_cast<Plane*>(contour0))) 
      return new ContactKinematicsCircleSolidPlane;

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<Plane*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<Plane*>(contour0))) 
      return new ContactKinematicsPointPlane;

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<Area*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<Area*>(contour0))) 
      return new ContactKinematicsPointArea;

    else if(dynamic_cast<Edge*>(contour0) && dynamic_cast<Edge*>(contour1)) 
      return new ContactKinematicsEdgeEdge;

    else if((dynamic_cast<Sphere*>(contour0) && dynamic_cast<Plane*>(contour1)) || (dynamic_cast<Sphere*>(contour1) && dynamic_cast<Plane*>(contour0))) 
      return new ContactKinematicsSpherePlane;

    // INTERPOLATIONSGESCHICHTEN - Interpol-Point
    else if((dynamic_cast<Point*>(contour0) &&  dynamic_cast<ContourInterpolation*>(contour1)) || (dynamic_cast<Point*>(contour1) &&  dynamic_cast<ContourInterpolation*>(contour0))) 
      return new ContactKinematicsPointContourInterpolation;
    // INTERPOLATIONSGESCHICHTEN

    else if((dynamic_cast<CircleHollow*>(contour0) && dynamic_cast<CylinderFlexible*>(contour1)) || (dynamic_cast<CircleHollow*>(contour1) && dynamic_cast<CylinderFlexible*>(contour0))) 
      return new ContactKinematicsCircleHollowCylinderFlexible;

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<CylinderFlexible*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<CylinderFlexible*>(contour0)))
      return new ContactKinematicsPointCylinderFlexible;

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<Contour1s*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<Contour1s*>(contour0))) 
      return new ContactKinematicsPointContour1s;

    else if((dynamic_cast<Line*>(contour0) && dynamic_cast<Contour1s*>(contour1)) || (dynamic_cast<Line*>(contour1) && dynamic_cast<Contour1s*>(contour0))) 
      return new ContactKinematicsLineContour1s;

    else if((dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<Contour1s*>(contour1)) || (dynamic_cast<CircleSolid*>(contour1) && dynamic_cast<Contour1s*>(contour0)))
      return new ContactKinematicsCircleSolidContour1s;


    else if((dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<CircleHollow*>(contour1)) || (dynamic_cast<CircleSolid*>(contour1) && dynamic_cast<CircleHollow*>(contour0)))
      return new ContactKinematicsCircleSolidCircleHollow;

    else if(dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<CircleSolid*>(contour1))
      return new ContactKinematicsCircleSolidCircleSolid;

    else if(dynamic_cast<Sphere*>(contour0) && dynamic_cast<Sphere*>(contour1))
      return new ContactKinematicsSphereSphere;

    else if((dynamic_cast<Sphere*>(contour0) && dynamic_cast<Frustum*>(contour1)) || (dynamic_cast<Sphere*>(contour1) && dynamic_cast<Frustum*>(contour0)))
      return new ContactKinematicsSphereFrustum;

    else if((dynamic_cast<CircleSolid*>(contour0) && dynamic_cast<Frustum2D*>(contour1)) || (dynamic_cast<CircleSolid*>(contour1) && dynamic_cast<Frustum2D*>(contour0))) 
      return new ContactKinematicsCircleSolidFrustum2D;

    else if((dynamic_cast<Point*>(contour0) && dynamic_cast<Frustum*>(contour1)) || (dynamic_cast<Point*>(contour1) && dynamic_cast<Frustum*>(contour0)))
      return new ContactKinematicsPointFrustum;  

    else if((dynamic_cast<CompoundContour*>(contour0) && dynamic_cast<Contour*>(contour1)) || (dynamic_cast<CompoundContour*>(contour1) && dynamic_cast<Contour*>(contour0))) 
      return new ContactKinematicsCompoundContourContour;  

    else if((dynamic_cast<CompoundContour*>(contour0) && dynamic_cast<CompoundContour*>(contour1))) 
      return new ContactKinematicsCompoundContourCompoundContour;  


    else 
      return 0;
  }
}
