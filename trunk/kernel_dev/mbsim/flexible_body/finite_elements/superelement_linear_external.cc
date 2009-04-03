/* Copyright (C) 2005-2006  Roland Zander

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
 * Contact:
 *   rzander@users.berlios.de
 *
 */
#include <config.h>
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK

#include "mbsim/flexible_body/finite_elements/superelement_linear_external.h"
#include <fstream>
using namespace std;

namespace MBSim {
  //-------------------------------------------------------------------------
  SuperElementLinearExternal::SuperElementLinearExternal(int warnLevel_)
    :DiscretizationInterface(),warnLevel(warnLevel_),M(0),K(0) {}

  void SuperElementLinearExternal::init() {
    D = static_cast<SqrMat>( alpha*M + beta * K );
  }

  void SuperElementLinearExternal::setM(const SymMat &M_) {
    if(M_.size()==getSizeOfPositions() || getSizeOfPositions()==0)
      M = M_;
    else
      cout << "Massmatrix and stiffnessmatrix have unequal sizes!!!" << endl;
  }
  void SuperElementLinearExternal::setK(const SqrMat &K_) {
    if(K_.size()==getSizeOfVelocities() || getSizeOfVelocities()==0)
      K = K_;
    else
      cout << "Massmatrix and stiffnessmatrix have unequal sizes!!!" << endl;
  }

  Vec SuperElementLinearExternal::computeTranslation      (const Vec&q,const ContourPointData& cp) {
    return trans(computeJacobianOfMinimalRepresentationRegardingPhysics(q,cp))*q + KrP[cp.getNodeNumber()];
  }
  Mat SuperElementLinearExternal::computeJacobianOfMinimalRepresentationRegardingPhysics(const Vec&q,const ContourPointData& cp) {
    return J[cp.getNodeNumber()];
  }

  ContourPointData SuperElementLinearExternal::addInterface(Mat J_, Vec KrP_) {
    if( (J_.rows()!= M.size() && M.size()!=0) || (J_.rows()!= K.size() && K.size()!=0) ) {
      cout << "Jacobimatrix of interface with requested position " << KrP_ ;
      cout << "  does not fit in size to massmatrix and stiffnessmatrix!!!" << endl;
      throw 1;
    }
    J.push_back(J_);
    KrP.push_back(KrP_);
    if(J.size()!=KrP.size()) {
      cout << "ERROR in memory management for SuperElementLinearExternal interfaces" << endl;
      throw 1;
    }
    ContourPointData CP;
    CP.getContourParameterType() = NODE;
    CP.getNodeNumber()   = J.size() - 1; // position of current Jacobian in array
    CP.getFrameOfReference().getPosition() = KrP[CP.getNodeNumber()];

    return CP;
  }
}

