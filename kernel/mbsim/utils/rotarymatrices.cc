/* Copyright (C) 2004-2012  Robert Huber

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
#include "rotarymatrices.h"
#include "utils.h"
#include <cmath>

using namespace fmatvec;

namespace MBSim {

  // Wie im TMKernfach Skript

  // Koordinatensystem K gegenueber dem KOSY I im mathematisch positiven Sinn um den Winkel phi verdreht
  /*  I_Y|
      |   /K_X 
      |  /
      | /  phi
      |/_________
      I_X        */


  SqrMat3 BasicRotAKIx(double phi) {
    SqrMat3 AKI(NONINIT);
    AKI(0,0)= 1.0;
    AKI(1,1)= cos(phi);
    AKI(2,2)= AKI(1,1);
    AKI(1,2)= sin(phi);
    AKI(2,1)=-AKI(1,2); 
    AKI(0,1)= 0.0;
    AKI(1,0)= 0.0;
    AKI(0,2)= 0.0;
    AKI(2,0)= 0.0;
    return AKI;
  }

  SqrMat3 BasicRotAKIy(double phi) {
    SqrMat3 AKI(NONINIT);
    AKI(1,1)= 1.0;
    AKI(0,0)= cos(phi);
    AKI(2,2)= AKI(0,0);
    AKI(0,2)=-sin(phi);
    AKI(2,0)=-AKI(0,2);
    AKI(0,1)= 0.0;
    AKI(1,0)= 0.0;
    AKI(1,2)= 0.0;
    AKI(2,1)= 0.0;
    return AKI; 
  }

  SqrMat3 BasicRotAKIz(double phi) {
    SqrMat3 AKI(NONINIT);
    AKI(2,2)= 1.0;
    AKI(0,0)= cos(phi);
    AKI(1,1)= AKI(0,0);
    AKI(0,1)= sin(phi);
    AKI(1,0)= -AKI(0,1);
    AKI(0,2)= 0.0;
    AKI(2,0)= 0.0;
    AKI(1,2)= 0.0;
    AKI(2,1)= 0.0;
    return AKI; 
  }

  Vec3 AIK2Cardan(const SqrMat3 &AIK) {
    Vec3 AlphaBetaGamma(NONINIT);
    assert(AIK(0,2)>=-1-1e-12 && AIK(0,2)<1+1e-12);
    AlphaBetaGamma(1)= asin(std::max(std::min(AIK(0,2), 1.0), -1.0));
    double nenner = cos(AlphaBetaGamma(1));
    if (fabs(nenner)>1e-10) {
      AlphaBetaGamma(0) = atan2(-AIK(1,2),AIK(2,2));
      AlphaBetaGamma(2) = atan2(-AIK(0,1),AIK(0,0));
    } else {
      AlphaBetaGamma(0)=0;
      AlphaBetaGamma(2)=atan2(AIK(1,0),AIK(1,1));
    }
    return AlphaBetaGamma;
  }

  Vec3 AIK2RevCardan(const SqrMat3 &AIK) {
    Vec3 AlphaBetaGamma(NONINIT);
    assert(-AIK(2,0)>=-1-1e-12 && -AIK(2,0)<1+1e-12);
    AlphaBetaGamma(1)= asin(std::max(std::min(-AIK(2,0), 1.0), -1.0));
    double nenner = cos(AlphaBetaGamma(1));
    if (fabs(nenner)>1e-10) {
      AlphaBetaGamma(0) = atan2(AIK(2,1),AIK(2,2));
      AlphaBetaGamma(2) = ArcTan(AIK(0,0),AIK(1,0));
    } else {
      AlphaBetaGamma(0)=0;
      AlphaBetaGamma(2)=atan2(-AIK(0,1),AIK(1,1));
    }
    return AlphaBetaGamma;
  }

  Vec3 AIK2ParametersZXY(const SqrMat3 &AIK) {
    Vec3 AlphaBetaGamma(NONINIT);
    assert(AIK(2,1)>=-1-1e-12 && AIK(2,1)<1+1e-12);
    AlphaBetaGamma(0) = asin(std::max(std::min(AIK(2,1), 1.0), -1.0));
    double nenner = cos(AlphaBetaGamma(0));
    if (fabs(nenner)>1e10) {
      AlphaBetaGamma(2) = atan2(-AIK(0,1), AIK(1,1));
      AlphaBetaGamma(1) = atan2(-AIK(2,0),AIK(2,2));
    }
    else {
      AlphaBetaGamma(1)=0;
      AlphaBetaGamma(2)=atan2(AIK(0,2),AIK(0,0));
    }
    return AlphaBetaGamma;
  }

  Vec3 calcParametersDotZXY(const SqrMat3 &AIK, const Vec3 &KomegaK) {
    Vec3 AlBeGa = AIK2ParametersZXY(AIK);
    SqrMat3 B(NONINIT);
    double cosAl = cos(AlBeGa(0));
    double sinAl = sin(AlBeGa(0));
    double cosBe = cos(AlBeGa(1));
    double sinBe = sin(AlBeGa(1));
    B(0,0) = cosAl*cosBe;
    B(1,0) = sinAl*sinBe;
    B(2,0) = -sinBe;
    B(0,2) = cosAl*sinBe;
    B(1,2) = -sinAl*cosBe;
    B(2,2) = cosBe;
    B /= cosAl;
    B(0,1) = 0.0;
    B(1,1) = 1.0;
    B(2,1) = 0.0;
    return B*KomegaK;
  }

  SqrMat3 RotationAboutAxis(const Vec3 &a, double phi) {
    const double cosq=cos(phi);
    const double sinq=sin(phi);
    const double onemcosq=1-cosq;
    const double a0a1=a.e(0)*a.e(1);
    const double a0a2=a.e(0)*a.e(2);
    const double a1a2=a.e(1)*a.e(2);
    SqrMat3 A(NONINIT);
    A.e(0,0) = cosq+onemcosq*a.e(0)*a.e(0);
    A.e(1,0) = onemcosq*a0a1+a.e(2)*sinq;
    A.e(2,0) = onemcosq*a0a2-a.e(1)*sinq;
    A.e(0,1) = onemcosq*a0a1-a.e(2)*sinq;
    A.e(1,1) = cosq+onemcosq*a.e(1)*a.e(1);
    A.e(2,1) = onemcosq*a1a2+a.e(0)*sinq;
    A.e(0,2) = onemcosq*a0a2+a.e(1)*sinq;
    A.e(1,2) = onemcosq*a1a2-a.e(0)*sinq;
    A.e(2,2) = cosq+onemcosq*a.e(2)*a.e(2);
    return A;
  }

  fmatvec::Vec3 AIK2Phi(const fmatvec::SqrMat3 &AIK) {
    Vec3 AlphaBetaGamma(NONINIT);
    AlphaBetaGamma.e(0) = AIK.e(2,1);
    AlphaBetaGamma.e(1) = AIK.e(0,2);
    AlphaBetaGamma.e(2) = AIK.e(1,0);
    return AlphaBetaGamma;
  }

}
