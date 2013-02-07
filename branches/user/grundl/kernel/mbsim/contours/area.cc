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

#include<config.h>
#include "mbsim/contours/area.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/grid.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Area::Area(const string &name) : Plane(name), limy(1), limz(1) {}

  double Area::prj_Area_Point(fmatvec::Vec3& Point){
	  Vec3 C_A=this->getFrame()->getPosition();//center of area
	  Vec3 Dis=Point-C_A;// distance vector from center of area to point
	  Vec3 Norm=this->getFrame()->getOrientation().col(0);//norm of the area
	  Vec3 Proj_n= (Dis.T()*Norm)*Norm;//projection on normal
	  Vec3 Proj_a= Dis - Proj_n;//projection on the area
	  double prj = nrm2(Proj_a);//length of the projection
	  return prj;

  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void Area::enableOpenMBV(bool enable, int number) {
    if(enable) {
      openMBVRigidBody=new OpenMBV::Grid;
      ((OpenMBV::Grid*)openMBVRigidBody)->setYSize(limy);
      ((OpenMBV::Grid*)openMBVRigidBody)->setXSize(limz);
      ((OpenMBV::Grid*)openMBVRigidBody)->setXNumber(number);
      ((OpenMBV::Grid*)openMBVRigidBody)->setYNumber(number);
      ((OpenMBV::Grid*)openMBVRigidBody)->setInitialRotation(0.,M_PI/2.,0.);
    }
    else openMBVRigidBody=0;
  }
#endif

}
