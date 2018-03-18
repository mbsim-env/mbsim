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

#ifndef _FCL_UTILS_H_
#define _FCL_UTILS_H_

#include "fmatvec/fmatvec.h"
#include "fcl/common/types.h"

namespace MBSim {

  fcl::Vector3d Vec3ToVector3d(const fmatvec::Vec3 &x);

  fcl::Matrix3d SqrMat3ToMatrix3d(const fmatvec::SqrMat3 &A);

  fmatvec::Vec3 Vector3dToVec3(const fcl::Vector3d &x);

  fmatvec::SqrMat3 Matrix3dToSqrMat3(const fcl::Matrix3d &A);

}

#endif
