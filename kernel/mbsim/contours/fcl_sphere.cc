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
#include "mbsim/contours/fcl_sphere.h"
#include "fcl/geometry/shape/sphere.h"
#include <openmbvcppinterface/sphere.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace fcl;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, FCLSphere)

  void FCLSphere::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit)
      cg = shared_ptr<CollisionGeometry<double> >(new Sphere<double>(r));
    else if (stage == plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody)
        static_pointer_cast<OpenMBV::Sphere>(openMBVRigidBody)->setRadius(r);
    }
    FCLContour::init(stage, config);
  }

  void FCLSphere::initializeUsingXML(DOMElement *element) {
    FCLContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"radius");
    setRadius(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVSphere ombv;
      ombv.initializeUsingXML(e);
      openMBVRigidBody=ombv.createOpenMBV(); 
    }
  }

}
#endif
