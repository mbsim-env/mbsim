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
 * Contact: thschindler@users.berlios.de
 */

#include<config.h>
#include "mbsim/contours/circle.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frustum.h>
#endif

using namespace std;

namespace MBSim {
  Circle::Circle(const string& name) : RigidContour(name),r(0.),outCont(false) {}

  Circle::Circle(const string& name, bool outCont_) : RigidContour(name),r(0.),outCont(outCont_) {}

  Circle::~Circle() {}

#ifdef HAVE_OPENMBVCPPINTERFACE
  void Circle::enableOpenMBV(bool enable) {
    if(enable) {
      openMBVRigidBody=new OpenMBV::Frustum;
      ((OpenMBV::Frustum*)openMBVRigidBody)->setBaseRadius(r);
      ((OpenMBV::Frustum*)openMBVRigidBody)->setTopRadius(r);
      ((OpenMBV::Frustum*)openMBVRigidBody)->setHeight(0);
    }
    else openMBVRigidBody=0;
  }
#endif

}

