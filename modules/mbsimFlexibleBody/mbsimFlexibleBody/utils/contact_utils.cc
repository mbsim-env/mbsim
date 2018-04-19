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
 *
 * Contact: markus.ms.schneider@googlemail.com
 */

#include <config.h>
#include "mbsimFlexibleBody/utils/contact_utils.h"

// --- List of contact kinematic implementations - BEGIN ---
#include "mbsim/contours/circle.h"
#include "mbsim/contours/point.h"
#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include "mbsimFlexibleBody/contours/contour_1s_neutral_factory.h"
#include "mbsimFlexibleBody/contours/cylinder_flexible.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"
#include "mbsimFlexibleBody/contours/flexible_planar_nurbs_contour.h"
#include "mbsimFlexibleBody/contours/flexible_planar_nurbs_contour_ffr.h"
#include "mbsimFlexibleBody/contours/flexible_spatial_nurbs_contour.h"
#include "mbsimFlexibleBody/contours/flexible_spatial_nurbs_contour_ffr.h"
// --- List of contact kinematic implementations - END ---

// --- List of contact kinematic implementations - BEGIN ---
#include <mbsim/contact_kinematics/point_extrusion.h>
#include <mbsim/contact_kinematics/circle_extrusion.h>

//#include <mbsimFlexibleBody/contact_kinematics/circlehollow_cylinderflexible.h>
//#include <mbsimFlexibleBody/contact_kinematics/point_cylinderflexible.h>
//#include <mbsimFlexibleBody/contact_kinematics/point_flexibleband.h>
#include <mbsimFlexibleBody/contact_kinematics/circle_flexibleband.h>
#include <mbsimFlexibleBody/contact_kinematics/point_nurbsdisk2s.h>
//#include <mbsimFlexibleBody/contact_kinematics/circle_nurbsdisk2s.h>
//#include <mbsimFlexibleBody/contact_kinematics/point_contour2s.h>
#include <mbsim/contact_kinematics/point_planarcontour.h>
#include <mbsimFlexibleBody/contact_kinematics/point_flexiblespatialcontour.h>
// --- List of contact kinematic implementations - END ---

using namespace MBSim;

namespace MBSimFlexibleBody {

  MBSim::ContactKinematics* findContactPairingFlexible(const std::type_info &contour0, const std::type_info &contour1) {

//    if(contour0==typeid(CircleHollow) && contour1==typeid(CylinderFlexible))
//      return new ContactKinematicsCircleHollowCylinderFlexible;
//
//    //else if ( contour0==typeid(Point) && contour1==typeid(CylinderFlexible) )
//      //return new ContactKinematicsPointCylinderFlexible;
//

    if(contour0==typeid(Point) && contour1==typeid(FlexibleBand))
      return new MBSim::ContactKinematicsPointExtrusion;

    else if(contour0==typeid(Circle) && contour1==typeid(FlexibleBand))
      return new ContactKinematicsCircleFlexibleBand;
      //return new MBSim::ContactKinematicsCircleExtrusion;

//    else if(contour0==typeid(Point) && contour1==typeid(Contour1sFlexible))
//      return new MBSim::ContactKinematicsPointContour1s;
//
//    else if(contour0==typeid(Point) && contour1==typeid(Contour1sNeutralFactory))
//          return new MBSim::ContactKinematicsPointContour1s;

//    else if(contour0==typeid(Point) && contour1==typeid(Contour2sNeutralFactory))
//          return new MBSimFlexibleBody::ContactKinematicsPointContour2s;
//
    else if(contour0==typeid(Point) && contour1==typeid(NurbsDisk2s))
      return new ContactKinematicsPointNurbsDisk2s;
//
//    else if(contour0==typeid(Circle) && contour1==typeid(NurbsDisk2s))
//      return new ContactKinematicsCircleNurbsDisk2s;  
//
    else if(contour0==typeid(Point) && contour1==typeid(FlexiblePlanarNurbsContour))
      return new ContactKinematicsPointPlanarContour;
    else if(contour0==typeid(Point) && contour1==typeid(FlexiblePlanarNurbsContourFFR))
      return new ContactKinematicsPointPlanarContour;
    else if(contour0==typeid(Point) && contour1==typeid(FlexibleSpatialNurbsContour))
      return new ContactKinematicsPointFlexibleSpatialContour;
    else if(contour0==typeid(Point) && contour1==typeid(FlexibleSpatialNurbsContourFFR))
      return new ContactKinematicsPointFlexibleSpatialContour;
    else
      return 0;
  }
}

