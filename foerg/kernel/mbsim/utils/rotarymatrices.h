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
#ifndef ROTARYMATRICES_H
#define ROTARYMATRICES_H
#include "fmatvec.h"

namespace MBSim {

  // Basic Rotations (see Script TM Grundlagenfach)
  // Rotate CoSy I (angle phi) to obtain CoSy K

  fmatvec::SqrMat BasicRotAKIx(double phi);
  fmatvec::SqrMat BasicRotAKIy(double phi);
  fmatvec::SqrMat BasicRotAKIz(double phi);

  fmatvec::SqrMat BasicRotAIKx(double phi);
  fmatvec::SqrMat BasicRotAIKy(double phi);
  fmatvec::SqrMat BasicRotAIKz(double phi);
  
  fmatvec::Vec AKI2Cardan(const fmatvec::SqrMat &AKI);
  fmatvec::Vec AIK2Cardan(const fmatvec::SqrMat &AIK);
  // ZXY Parametrisation (z -x -y)  with Parameters (Angles) = [al; be; ga]
  // first rotation: z-Axis (ga)
  // 2nd   rotation: x-Axis (al)
  // 3rd   rotation: y-Axis (be)
  fmatvec::Vec AIK2ParametersZXY(const fmatvec::SqrMat &AIK);				//  returns [al; be; ga]
  fmatvec::Vec calcParametersDotZXY(const fmatvec::SqrMat &AIK, const fmatvec::Vec &KomegaK);	// kinematic equations to calculate [al_dot; be_dot; ga_dot] from AIK and KomegaK

  fmatvec::SqrMat Cardan2AIK(double alpha,double beta,double gamma);
  fmatvec::SqrMat Euler2AIK(double psi, double theta, double phi);	//psi: Preazession;  theta: Nutation;  phi: Rotation
}

#endif
