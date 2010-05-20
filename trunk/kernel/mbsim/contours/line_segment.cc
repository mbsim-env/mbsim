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
 * Contact: foerg@users.berlios.de
 */

#include<config.h>
#include "mbsim/contours/line_segment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/grid.h>
#endif

using namespace std;

namespace MBSim {
  LineSegment::LineSegment(const std::string& name) : RigidContour(name), bound(2) {
    bound(0) = 0;
    bound(1) = 1;
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void LineSegment::enableOpenMBV(bool enable, double size, int number) {
    if(enable) {
      openMBVRigidBody=new OpenMBV::Grid;
      ((OpenMBV::Grid*)openMBVRigidBody)->setXSize(size);
      ((OpenMBV::Grid*)openMBVRigidBody)->setYSize(0.01);
      ((OpenMBV::Grid*)openMBVRigidBody)->setXNumber(number);
      ((OpenMBV::Grid*)openMBVRigidBody)->setYNumber(1);
      ((OpenMBV::Grid*)openMBVRigidBody)->setInitialRotation(0.,M_PI/2.,0.);
    }
    else openMBVRigidBody=0;
  }
#endif
}

