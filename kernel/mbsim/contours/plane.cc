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
#include "mbsim/contours/plane.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/utils.h"

#include <openmbvcppinterface/grid.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Plane)

  Vec3 Plane::evalWu(const Vec2 &zeta) {
    return evalWs(zeta);
  }

  Vec3 Plane::evalWv(const Vec2 &zeta) {
    return evalWt(zeta);
  }

  Vec3 Plane::evalWn(const Vec2 &zeta) {
    return R->evalOrientation().col(0);
  }

  Vec3 Plane::evalWs(const Vec2 &zeta) {
    return R->evalOrientation().col(1);
  }

  Vec3 Plane::evalWt(const Vec2 &zeta) {
    return R->evalOrientation().col(2);
  }

  Vec2 Plane::evalZeta(const Vec3 &WrPoint) {
    return (R->evalOrientation().T() *(WrPoint - R->evalPosition()) )(Range<Fixed<1>,Fixed<2> >());
  }

 void Plane::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVPlane ombv;
      openMBVRigidBody=ombv.createOpenMBV(e); 
    }
  }

}
