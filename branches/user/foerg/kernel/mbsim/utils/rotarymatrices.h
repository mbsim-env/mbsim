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
#ifndef ROTARYMATRICES_H
#define ROTARYMATRICES_H
#include "fmatvec.h"

namespace MBSim {

  /*!
   * \brief Basic Rotations (see Script TM Grundlagenfach)
   */

  /**
   *  \brief Rotate CoSy I (angle phi) to obtain CoSy K
   */
  fmatvec::SqrMat BasicRotAKIx(double phi);
  fmatvec::SqrMat BasicRotAKIy(double phi);
  fmatvec::SqrMat BasicRotAKIz(double phi);

  /*
   *  \brief Rotate CoSy K (angle phi) to obtain CoSy I
   */
  fmatvec::SqrMat BasicRotAIKx(double phi);
  fmatvec::SqrMat BasicRotAIKy(double phi);
  fmatvec::SqrMat BasicRotAIKz(double phi);

  /**
   * \brief Cardan parametrisation (x y z): calculate angles (alpha, beta, gamma) from rotation matrix AKI or AIK
   */
  fmatvec::FVec AKI2Cardan(const fmatvec::FSqrMat &AKI);
  fmatvec::FVec AIK2Cardan(const fmatvec::FSqrMat &AIK);

  /**
   * \brief reversed Cardan parametrisation (z y x): calculate angles (alpha, beta, gamma) from rotation matrix AKI or AIK
   */
  fmatvec::FVec AKI2RevCardan(const fmatvec::FSqrMat &AKI);
  fmatvec::FVec AIK2RevCardan(const fmatvec::FSqrMat &AIK);

  /**
   * \brief ZXY parametrisation (z -x -y)  with parameters (angles) = [al; be; ga]
   * first rotation: z-Axis (ga), 2nd rotation: x-Axis (al), 3rd rotation: y-Axis (be)
   * calculates angles (alpha, beta, gamma) from rotation matrix AIK
   */
  fmatvec::Vec AIK2ParametersZXY(const fmatvec::SqrMat &AIK);

  /**
   * \brief kinematic equations to calculate [al_dot; be_dot; ga_dot] from AIK and KomegaK
   */
  fmatvec::Vec calcParametersDotZXY(const fmatvec::SqrMat &AIK, const fmatvec::Vec &KomegaK);

  /**
   * \brief Cardan parametrisation (x y z): calculate AIK matrix from angles (alpha, beta, gamma)
   */
  fmatvec::SqrMat Cardan2AIK(double alpha,double beta,double gamma);

  /**
   * \brief Euler parametrisation: calculate AIK matrix from angles (psi, theta, phi); psi: Preazession;  theta: Nutation;  phi: Rotation
   */
  fmatvec::SqrMat Euler2AIK(double psi, double theta, double phi);
}

#endif
