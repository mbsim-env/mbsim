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
#include "mbsim/contours/fcl_box.h"

#include <openmbvcppinterface/cuboid.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace fcl;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, FCLBox)

  void FCLBox::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit)
      cg = shared_ptr<CollisionGeometry<double> >(new Box<double>(lx,ly,lz));
    else if (stage == plotting) {
      if(openMBVRigidBody)
        static_pointer_cast<OpenMBV::Cuboid>(openMBVRigidBody)->setLength(lx,ly,lz);
    }
    RigidContour::init(stage, config);
  }

  void FCLBox::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"length");
    setLength(E(e)->getText<Vec3>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      DOMElement *d, *t;
      d=E(e)->getFirstElementChildNamed(MBSIM%"diffuseColor");
      t=E(e)->getFirstElementChildNamed(MBSIM%"transparency");
      if( d &&  t) enableOpenMBV(_diffuseColor=E(d)->getText<Vec3>(), _transparency=E(e)->getText<double>());
      if(!d &&  t) enableOpenMBV(                          _transparency=E(e)->getText<double>());
      if( d && !t) enableOpenMBV(_diffuseColor=E(d)->getText<Vec3>()                                       );
      if(!d && !t) enableOpenMBV(                                                                          );
    }
  }

}
