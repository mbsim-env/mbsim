/* Copyright (C) 2004-2018 MBSim Development Team
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
#ifdef HAVE_FCL
#include "mbsim/contours/fcl_plane.h"
#include "mbsim/utils/fcl_utils.h"
#include "fcl/geometry/shape/plane.h"
#include <openmbvcppinterface/grid.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace fcl;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, FCLPlane)

  void FCLPlane::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit)
      cg = shared_ptr<CollisionGeometry<double> >(new Plane<double>(Vec3ToVector3d(normal),offset));
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
        double al=0, be=0, ga=0;
        if(normal(2)>1e-10) {
          al = atan(normal(1)/normal(2));
          be = acos(normal(0));
        }
        else if(normal(1)>1e-10) {
          be = acos(normal(0));
          al = asin(normal(1)/sin(be));
        }
        Vec3 tr = normal*offset;
        openMBVRigidBody->setInitialTranslation(tr(0),tr(1),tr(2));
        openMBVRigidBody->setInitialRotation(al,be,ga);
      }
    }
    FCLContour::init(stage, config);
  }

  void FCLPlane::initializeUsingXML(DOMElement *element) {
    FCLContour::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"normal");
    setNormal(E(e)->getText<Vec3>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"offset");
    setOffset(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVPlane ombv;
      ombv.initializeUsingXML(e);
      openMBVRigidBody=ombv.createOpenMBV();
    }
  }

}
#endif
