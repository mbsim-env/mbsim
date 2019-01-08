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
#include "contact_utils.h"

#include <mbsimFcl/fcl_box.h>
#include <mbsimFcl/fcl_sphere.h>
#include <mbsimFcl/fcl_plane.h>
#include <mbsimFcl/fcl_mesh.h>

#include <mbsimFcl/fclcontour_fclcontour.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFcl {

  ContactKinematics* findContactPairingFcl(const type_info &contour0, const type_info &contour1) {

    if ( contour0==typeid(FclBox) && contour1==typeid(FclBox) )
      return new ContactKinematicsContourContour(4);

    else if ( contour0==typeid(FclBox) && contour1==typeid(FclSphere) )
      return new ContactKinematicsContourContour(1);

//    else if ( contour0==typeid(FclBox) && contour1==typeid(FclPlane) )
//      return new ContactKinematicsContourContour(4);

    else if ( contour0==typeid(FclSphere) && contour1==typeid(FclPlane) )
      return new ContactKinematicsContourContour(1);

//    else if ( contour0==typeid(FclBox) && contour1==typeid(FclMesh) )
//      return new ContactKinematicsContourContour(1);

    else if ( contour0==typeid(FclMesh) && contour1==typeid(FclMesh) )
      return new ContactKinematicsContourContour(1);

//    else if ( dynamic_cast<Contour*>(c0) && dynamic_cast<Contour*>(c1) )
//      return new ContactKinematicsContourContour(1);

    else
      return nullptr;
  }
}
