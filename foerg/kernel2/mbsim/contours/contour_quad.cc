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
#include "mbsim/contours/contour_quad.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  ContourQuad::ContourQuad(const string &name) : ContourInterpolation(name,2,4) {
    iPoints.resize(numberOfPoints);
  }

  void ContourQuad::init(InitStage stage) {
    Contour::init(stage);
  }

  bool ContourQuad::testInsideBounds(const ContourPointData &cp) {
    if( cp.getLagrangeParameterPosition().size()!=2 ) {
      return false;
    }
    if( 0 <= cp.getLagrangeParameterPosition()(0) && cp.getLagrangeParameterPosition()(0) <= 1 && 0 <= cp.getLagrangeParameterPosition()(1) && cp.getLagrangeParameterPosition()(1) <= 1) return true;
    else return false;
  }

  double ContourQuad::computePointWeight(const Vec &s, int i) {
    double xi  = s(0);
    double eta = s(1);

    switch(i) {
      case 0: return (1-xi)*(1-eta);
      case 1: return (  xi)*(1-eta);
      case 2: return (  xi)*(  eta);
      case 3: return (1-xi)*(  eta);
      default: return 0.0;  // new, probably not OK
    }
  }

  double ContourQuad::computePointWeight(const Vec &s, int i, int diff) {
    double xi  = s(0);
    double eta = s(1);

    if (diff == 0) // Ableitungen nach xi
      switch(i) {
        case 0: return -(1-eta);
        case 1: return  (1-eta);
        case 2: return  (  eta);
        case 3: return -(  eta);
        default: return 0.0;  // new, probably not OK
      }
    else // Ableitungen nach eta
      switch(i) {
        case 0: return - (1-xi);
        case 1: return - (  xi);
        case 2: return   (  xi);
        case 3: return   (1-xi);
        default: return 0.0;  // new, probably not OK
      }
  }

  FVec ContourQuad::computeWn(const ContourPointData &cp) {
    const VVec &s = cp.getLagrangeParameterPosition();
    FVMat tTemp = computeWt(s);
    return crossProduct(tTemp.col(1),tTemp.col(0)); // Achtung: Interpoation mit einem Konturparameter-> t.col(1) = Cb;
  }
}
