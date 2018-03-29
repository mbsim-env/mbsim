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
#include "fcl_utils.h" 

using namespace fmatvec;
using namespace std;
using namespace fcl;

namespace MBSim {

  Vector3d Vec3ToVector3d(const Vec3 &x) {
    Vector3d y;
    for(int i=0; i<3; i++)
      y(i) = x(i);
    return y;
  }

  Matrix3d SqrMat3ToMatrix3d(const SqrMat3 &A) {
    Matrix3d B;
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        B(i,j) = A(i,j);
    return B;
  }

  Vec3 Vector3dToVec3(const Vector3d &x) {
    Vec3 y;
    for(int i=0; i<3; i++)
      y(i) = x(i);
    return y;
  }

  SqrMat3 Matrix3dToSqrMat3(const Matrix3d &A) {
    SqrMat3 B;
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        B(i,j) = A(i,j);
    return B;
  }

}

#endif
