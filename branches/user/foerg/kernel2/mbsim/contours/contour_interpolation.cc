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
#include "mbsim/contours/contour_interpolation.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  ContourInterpolation::ContourInterpolation(const string &name,int parameters_,int nPoints_) : Contour(name),contourParameters(parameters_),numberOfPoints(nPoints_) {}
  void ContourInterpolation::setPoint(Point *point_, int n) {
    assert(0 <= n);
    assert(n <  numberOfPoints);
    iPoints[n] = point_;
  }

  VVec ContourInterpolation::computePointWeights(const VVec &s) {
    VVec weights(numberOfPoints);
    for(int i=0;i<numberOfPoints; i++) weights(i) = computePointWeight(s,i);
    return weights;
  }

  FVec ContourInterpolation::computeWrOC(const ContourPointData &cp) {
    const VVec &s = cp.getLagrangeParameterPosition();
    FVec r(INIT,0.0);
    for(int i=0; i<numberOfPoints;i++) r += computePointWeight(s,i) * iPoints[i]->getReferencePosition();
    return r;
  }

  FVec ContourInterpolation::computeWvC(const ContourPointData &cp) {
    const VVec &s = cp.getLagrangeParameterPosition();
    FVec v(INIT,0.0);
    for(int i=0; i<numberOfPoints;i++) v += computePointWeight(s,i) * iPoints[i]->getReferenceVelocity();
    return v;
  }

  FVMat ContourInterpolation::computeWt(const ContourPointData &cp) {
    const VVec &s = cp.getLagrangeParameterPosition();
    FVMat t(contourParameters,INIT,0.0);

    for(int i=0; i<contourParameters; i++) {
      FVec tTemp = t.col(i);
      for(int j=0; j<numberOfPoints;j++) tTemp += computePointWeight(s,j,i) * iPoints[j]->getReferencePosition();
      tTemp /= nrm2(tTemp);
    }   
    return t;
  }

  FVec ContourInterpolation::computeWrOC(const VVec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWrOC(cp);}
  FVec ContourInterpolation::computeWvC (const VVec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWvC (cp);}
  FVMat ContourInterpolation::computeWt  (const VVec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWt  (cp);}
  FVec ContourInterpolation::computeWn  (const VVec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWn  (cp);}
}
