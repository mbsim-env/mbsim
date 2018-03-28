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

#ifndef ROTARYMATRICES_H
#define ROTARYMATRICES_H

#include "fmatvec/fmatvec.h"

namespace MBSim {

  /*!
   * \brief Basic Rotations (see Script TM Grundlagenfach)
   */

  /**
   *  \brief Rotate CoSy I (angle phi) to obtain CoSy K
   */
  fmatvec::SqrMat3 BasicRotAKIx(double phi);
  fmatvec::SqrMat3 BasicRotAKIy(double phi);
  fmatvec::SqrMat3 BasicRotAKIz(double phi);

  /*
   *  \brief Rotate CoSy K (angle phi) to obtain CoSy I
   */
  inline fmatvec::SqrMat3 BasicRotAIKx(double phi) { return BasicRotAKIx(-phi); }
  inline fmatvec::SqrMat3 BasicRotAIKy(double phi) { return BasicRotAKIy(-phi); }
  inline fmatvec::SqrMat3 BasicRotAIKz(double phi) { return BasicRotAKIz(-phi); }

  /**
   * \brief Cardan parametrisation (x y z): calculate angles (alpha, beta, gamma) from rotation matrix AKI or AIK
   */
  fmatvec::Vec3 AIK2Cardan(const fmatvec::SqrMat3 &AIK);

  inline fmatvec::Vec3 AKI2Cardan(const fmatvec::SqrMat3 &AKI) { return AIK2Cardan(AKI.T()); }

  /**
   * \brief reversed Cardan parametrisation (z y x): calculate angles (alpha, beta, gamma) from rotation matrix AKI or AIK
   */
  fmatvec::Vec3 AIK2RevCardan(const fmatvec::SqrMat3 &AIK);

  inline fmatvec::Vec3 AKI2RevCardan(const fmatvec::SqrMat3 &AKI) { return AIK2RevCardan(AKI.T()); }

  /**
   * \brief ZXY parametrisation (z -x -y)  with parameters (angles) = [al; be; ga]
   * first rotation: z-Axis (ga), 2nd rotation: x-Axis (al), 3rd rotation: y-Axis (be)
   * calculates angles (alpha, beta, gamma) from rotation matrix AIK
   */
  fmatvec::Vec3 AIK2ParametersZXY(const fmatvec::SqrMat3 &AIK);

  /**
   * \brief kinematic equations to calculate [al_dot; be_dot; ga_dot] from AIK and KomegaK
   */
  fmatvec::Vec3 calcParametersDotZXY(const fmatvec::SqrMat3 &AIK, const fmatvec::Vec3 &KomegaK);

  /**
   * \brief Cardan parametrisation (x y z): calculate AIK matrix from angles (alpha, beta, gamma)
   */
  inline fmatvec::SqrMat3 Cardan2AIK(double alpha,double beta,double gamma) { return BasicRotAIKx(alpha)*BasicRotAIKy(beta)*BasicRotAIKz(gamma); }

  /**
   * \brief Euler parametrisation: calculate AIK matrix from angles (psi, theta, phi); psi: Preazession;  theta: Nutation;  phi: Rotation
   */
  inline fmatvec::SqrMat3 Euler2AIK(double psi, double theta, double phi) { return BasicRotAIKz(psi)*BasicRotAIKx(theta)*BasicRotAIKz(phi); }

}

#endif
