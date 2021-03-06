/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef ANGLES_H_
#define ANGLES_H_

#include <mbsim/functions/function.h>

namespace MBSimFlexibleBody {

  /**
   * \brief node class for angle parametrisation
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   * \date 2012-03-20 added T matrix (Schindler / Cebulla)
   * \todo unify with Rotation TODO
   */
  class Angles : public MBSim::Function<fmatvec::RotMat3(fmatvec::VecV, double)> {
    public:
      /**
       * \brief constructor
       */
      Angles();
      /**
       * \brief destructor
       */
      ~Angles() override;

      /* INTERFACE OF ROTATION */
      fmatvec::RotMat3 operator()(const fmatvec::VecV &q, const double &t) override;
      /***************************************************/

      /* INTERFACE */
      /**
       * \return tangent 
       */
      virtual fmatvec::Vec3 computet(const fmatvec::Vec& q) const = 0;

      /**
       * \param angles
       * \return normal
       */
      virtual fmatvec::Vec3 computen(const fmatvec::Vec& q) const = 0;

      /**
       * \param angles
       * \return binormal 
       */
      virtual fmatvec::Vec3 computeb(const fmatvec::Vec& q) const = 0;

      /**
       * \param angles
       * \return tilde normal 
       */
      virtual fmatvec::Vec computentil(const fmatvec::Vec& q) const = 0;		

      /**
       * \param angles
       * \return tilde binormal
       */
      virtual fmatvec::Vec computebtil(const fmatvec::Vec& q) const = 0;	

      /**
       * \param angles
       * \return derivative of tangent with respect to angles
       */
      virtual fmatvec::SqrMat computetq(const fmatvec::Vec& q) const = 0;		

      /**
       * \param angles
       * \return derivative of normal with respect to angles 
       */
      virtual fmatvec::SqrMat computenq(const fmatvec::Vec& q) const = 0;		

      /**
       * \param angles
       * \return derivative of binormal with respect to angles 
       */
      virtual fmatvec::SqrMat computebq(const fmatvec::Vec& q) const = 0;

      /**
       * \param angles
       * \return derivative of tilde normal with respect to angles
       */
      virtual fmatvec::SqrMat computentilq(const fmatvec::Vec& q) const = 0;		

      /**
       * \param angles
       * \return derivative of tilde binormal with respect to angles 
       */
      virtual fmatvec::SqrMat computebtilq(const fmatvec::Vec& q) const = 0;

      /**
       * \param angles
       * \return 2nd derivative of tangent with respect to angles
       */
      virtual fmatvec::Mat computetq2(const fmatvec::Vec& q) const = 0;		

      /**
       * \param angles
       * \return 2nd derivative of normal with respect to angles
       */
      virtual fmatvec::Mat computenq2(const fmatvec::Vec& q) const = 0;		

      /**
       * \param angles
       * \return 2nd derivative of binormal with respect to angles
       */
      virtual fmatvec::Mat computebq2(const fmatvec::Vec& q) const = 0;

      /**
       * \param angles
       * \return 2nd derivative of tilde normal with respect to angles
       */
      virtual fmatvec::Mat computentilq2(const fmatvec::Vec& q) const = 0;		

      /**
       * \param angles
       * \return 2nd derivative of tilde binormal with respect to angles 
       */
      virtual fmatvec::Mat computebtilq2(const fmatvec::Vec& q) const = 0;
      /********************************************************/

      /* NODAL FUNCTIONS */
      /**
       * \param angles
       * \param derivative of angles
       * \return time derivative of tangent 
       */
      fmatvec::Vec computett(const fmatvec::Vec& q,const fmatvec::Vec& qt) const;		

      /**
       * \param angles
       * \param derivative of angles
       * \return time derivative of normal 
       */
      fmatvec::Vec computent(const fmatvec::Vec& q,const fmatvec::Vec& qt) const;		

      /**
       * \param angles
       * \param derivative of angles
       * \return time derivative of binormal 
       */
      fmatvec::Vec computebt(const fmatvec::Vec& q,const fmatvec::Vec& qt) const;

      /**
       * \param angles
       * \param derivative of angles
       * \return time derivative of tilde normal
       */
      fmatvec::Vec computentilt(const fmatvec::Vec& q,const fmatvec::Vec& qt) const;		

      /**
       * \param angles
       * \param derivative of angles
       * \return time derivative of tilde binormal 
       */
      fmatvec::Vec computebtilt(const fmatvec::Vec& q,const fmatvec::Vec& qt) const;

      /**
       * \param angles
       * \param derivative of angles
       * \return time derivative of tq 
       */
      fmatvec::SqrMat computetqt(const fmatvec::Vec& q,const fmatvec::Vec& qt) const;

      /**
       * \param angles
       * \param derivative of angles
       * \return time derivative of nq
       */
      fmatvec::SqrMat computenqt(const fmatvec::Vec& q,const fmatvec::Vec& qt) const;

      /**
       * \param angles
       * \param derivative of angles
       * \return time derivative of bq 
       */
      fmatvec::SqrMat computebqt(const fmatvec::Vec& q,const fmatvec::Vec& qt) const;

      /**
       * \param angles
       * \param derivative of angles
       * \return time derivative of ntilq 
       */
      fmatvec::SqrMat computentilqt(const fmatvec::Vec& q,const fmatvec::Vec& qt) const;

      /**
       * \param angles
       * \param derivative of angles
       * \return time derivative of btilq
       */
      fmatvec::SqrMat computebtilqt(const fmatvec::Vec& q,const fmatvec::Vec& qt) const;

      /**
       * \param angles
       * \param derivative of angles
       * \return angular velocity
       */
      virtual fmatvec::Vec computeOmega(const fmatvec::Vec& q,const fmatvec::Vec& qt) const;

      /**
       * \param angles
       * \return T-matrix (transformation matrix from differentiated angles to angular velocity omega)
       */
      virtual fmatvec::SqrMat computeT(const fmatvec::Vec& q) const; // JacobianOfRotation = G
      /********************************************************/
  };

}

#endif /*ANGLES_H_*/

