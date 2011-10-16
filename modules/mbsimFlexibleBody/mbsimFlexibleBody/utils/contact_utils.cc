/* Copyright (C) 2004-2011 MBSim Development Team
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
#include <mbsim/contact_kinematics/point_contour1s.h>

#include <mbsimFlexibleBody/contact_kinematics/circlehollow_cylinderflexible.h>
#include <mbsimFlexibleBody/contact_kinematics/point_cylinderflexible.h>
#include <mbsimFlexibleBody/contact_kinematics/point_flexibleband.h>
#include <mbsimFlexibleBody/contact_kinematics/circlesolid_flexibleband.h>
#include <mbsimFlexibleBody/contact_kinematics/point_nurbsdisk2s.h>
#include <mbsimFlexibleBody/contact_kinematics/circle_nurbsdisk2s.h>
// --- List of contact kinematic implementations - END ---

namespace MBSimFlexibleBody {

  MBSim::ContactKinematics* findContactPairingFlexible(const char *contour0, const char *contour1) {

    if(strcmp(contour0, "CircleHollow")==0 && strcmp(contour1, "CylinderFlexible")==0)
      return new ContactKinematicsCircleHollowCylinderFlexible;

    //else if ( strcmp(contour0, "Point")==0 && strcmp(contour1, "CylinderFlexible")==0 )
      //return new ContactKinematicsPointCylinderFlexible;

    else if(strcmp(contour0, "CircleSolid")==0 && strcmp(contour1, "FlexibleBand")==0)
      return new ContactKinematicsCircleSolidFlexibleBand;
    
    else if(strcmp(contour0, "Point")==0 && strcmp(contour1, "FlexibleBand")==0)
      return new ContactKinematicsPointFlexibleBand;  
    
    else if(strcmp(contour0, "Point")==0 && strcmp(contour1, "Contour1sFlexible")==0) 
      return new MBSim::ContactKinematicsPointContour1s;

    else if(strcmp(contour0, "Point")==0 && strcmp(contour1, "NurbsDisk2s")==0)
      return new ContactKinematicsPointNurbsDisk2s;  

    else if(strcmp(contour0, "Circle")==0 && strcmp(contour1, "NurbsDisk2s")==0)
      return new ContactKinematicsCircleNurbsDisk2s;  

    else 
      return 0;
  }
}

