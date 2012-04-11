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

  Vec ContourInterpolation::computePointWeights(const Vec &s) {
    Vec weights(numberOfPoints);
    for(int i=0;i<numberOfPoints; i++) weights(i) = computePointWeight(s,i);
    return weights;
  }

  Vec ContourInterpolation::computeWrOC(const ContourPointData &cp) {
    const Vec &s = cp.getLagrangeParameterPosition();
    Vec r(3,INIT,0.0);
    for(int i=0; i<numberOfPoints;i++) r += computePointWeight(s,i) * iPoints[i]->getReferencePosition();
    return r;
  }

  Vec ContourInterpolation::computeWvC(const ContourPointData &cp) {
    const Vec &s = cp.getLagrangeParameterPosition();
    Vec v(3,INIT,0.0);
    for(int i=0; i<numberOfPoints;i++) v += computePointWeight(s,i) * iPoints[i]->getReferenceVelocity();
    return v;
  }

  Mat ContourInterpolation::computeWt(const ContourPointData &cp) {
    const Vec &s = cp.getLagrangeParameterPosition();
    Mat t(3,contourParameters,INIT,0.0);

    for(int i=0; i<contourParameters; i++) {
      Vec tTemp = t.col(i);
      for(int j=0; j<numberOfPoints;j++) tTemp += computePointWeight(s,j,i) * iPoints[j]->getReferencePosition();
      tTemp /= nrm2(tTemp);
    }   
    return t;
  }

  Vec ContourInterpolation::computeWrOC(const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWrOC(cp);}
  Vec ContourInterpolation::computeWvC (const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWvC (cp);}
  Mat ContourInterpolation::computeWt  (const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWt  (cp);}
  Vec ContourInterpolation::computeWn  (const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWn  (cp);}
}
