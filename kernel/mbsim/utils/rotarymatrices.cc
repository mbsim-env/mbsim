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
 * Contact:
 *   rzander@users.berlios.de
 *
 */
#include <config.h>
#include "rotarymatrices.h"
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


  SqrMat BasicRotAKIx(double phi) {
    SqrMat AKI(3,3,INIT,0.0);
    AKI(0,0)= 1.0;
    AKI(1,1)= cos(phi);
    AKI(2,2)= AKI(1,1);
    AKI(1,2)= sin(phi);
    AKI(2,1)=-AKI(1,2); 
    return AKI;
  }

  SqrMat BasicRotAKIy(double phi) {
    SqrMat AKI(3,3,INIT,0.0);
    AKI(1,1)= 1.0;
    AKI(0,0)= cos(phi);
    AKI(2,2)= AKI(0,0);
    AKI(0,2)=-sin(phi);
    AKI(2,0)=-AKI(0,2);
    return AKI; 
  }

  SqrMat BasicRotAKIz(double phi) {
    SqrMat AKI(3,3,INIT,0.0);
    AKI(2,2)= 1.0;
    AKI(0,0)= cos(phi);
    AKI(1,1)= AKI(0,0);
    AKI(0,1)= sin(phi);
    AKI(1,0)= -AKI(0,1);
    return AKI; 
  }

  SqrMat BasicRotAIKx(double phi) {
    SqrMat AIK(3,3,INIT,0.0);
    AIK = BasicRotAKIx(-phi);
    return AIK;
  }

  SqrMat BasicRotAIKy(double phi) {
    SqrMat AIK(3,3,INIT,0.0);
    AIK = BasicRotAKIy(-phi);
    return AIK; 
  }

  SqrMat BasicRotAIKz(double phi) {
    SqrMat AIK(3,3,INIT,0.0);
    AIK = BasicRotAKIz(-phi);
    return AIK; 
  }

  SqrMat Cardan2AIK(double alpha,double beta,double gamma) {
    //x
    SqrMat AIKx(3,3,INIT,0);
    AIKx =  BasicRotAIKx(alpha);
    //y
    SqrMat AIKy(3,3,INIT,0); 
    AIKy =  BasicRotAIKy(beta);

    //z
    SqrMat AIKz(3,3,INIT,0);  
    AIKz = BasicRotAIKz(gamma);

    return AIKx*AIKy*AIKz;          //Wie im TM VI Skript
  }

  Vec AIK2Cardan(const SqrMat &AIK) { 
    Vec AlphaBetaGamma(3,INIT,0.0);    
    AlphaBetaGamma(1)= asin(AIK(0,2));
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

  Vec AKI2Cardan(const SqrMat &AKI) {
    return AIK2Cardan(trans(AKI));
  }

  Vec AIK2RevCardan(const SqrMat &AIK) {
    Vec AlphaBetaGamma(3,INIT,0.0);
    AlphaBetaGamma(1)= asin(-AIK(2,0));
    double nenner = cos(AlphaBetaGamma(1));
    if (fabs(nenner)>1e-10) {
      AlphaBetaGamma(0) = atan2(AIK(2,1),AIK(2,2));
      AlphaBetaGamma(2) = atan2(AIK(1,0),AIK(0,0));
    } else {
      AlphaBetaGamma(0)=0;
      AlphaBetaGamma(2)=atan2(-AIK(0,1),AIK(1,1));
    }
    return AlphaBetaGamma;
  }

  Vec AKI2RevCardan(const SqrMat &AKI) {
    return AIK2RevCardan(trans(AKI));
  }

  SqrMat Euler2AIK(double psi,double theta,double phi) {
    //z Preazession
    SqrMat AIKz_psi(3,3,INIT,0);
    AIKz_psi =  BasicRotAIKz(psi);
    //x Nutation (Kippen)
    SqrMat AIKx(3,3,INIT,0); 
    AIKx =  BasicRotAIKx(theta);

    //z Rotation (um eigene Achse)
    SqrMat AIKz_phi(3,3,INIT,0);  
    AIKz_phi = BasicRotAIKz(phi);

    return AIKz_psi*AIKx*AIKz_phi; 
  }

  Vec AIK2ParametersZXY(const SqrMat &AIK) {
    Vec AlphaBetaGamma(3,INIT,0.0);
    AlphaBetaGamma(0) = asin(AIK(2,1));
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

  Vec calcParametersDotZXY(const SqrMat &AIK, const Vec &KomegaK) {
    Vec AlBeGa = AIK2ParametersZXY(AIK);
    SqrMat B(3,INIT,0.0);
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
    B = B/cosAl;
    B(1,1)=1;
    return B*KomegaK;
  }
}
