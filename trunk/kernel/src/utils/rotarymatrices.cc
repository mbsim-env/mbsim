/* Copyright (C) 2004-2006  Robert Huber
 
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

}
