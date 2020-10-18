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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _CONTACT_KINEMATICS_CIRCLE_SPATIALCONTOUR_H_
#define _CONTACT_KINIMATICS_CIRCLE_SPATIALCONTOUR_H_

#include "contact_kinematics.h"
#include "mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h"
#include "mbsim/functions/symbolic_function.h"

namespace MBSim {

  class Circle;
  class SpatialContour;

  /*! \brief pairing circle outer side to spatial contour
   * \author Markus Friedrich
   */
  class ContactKinematicsCircleSpatialContour : public ContactKinematics {
    public:
      ContactKinematicsCircleSpatialContour();
      /* INHERITED INTERFACE */
      void assignContours(const std::vector<Contour*> &contour) override;
      void updateg(SingleContact &contact, int i=0) override;
      void setInitialGuess(const fmatvec::MatV &zeta0_) override;
      /***************************************************/

    private:
      /**
       * \brief contour index
       */
      int icircle, ispatialContour;

      /**
       * \brief contour classes
       */
      Circle *circle;
      SpatialContour *spatialContour;

      fmatvec::Vector<fmatvec::Fixed<3>, fmatvec::IndependentVariable> circlePosition;
      fmatvec::SquareMatrix<fmatvec::Fixed<3>, fmatvec::IndependentVariable> circleOrientation;
      fmatvec::Vector<fmatvec::Fixed<3>, fmatvec::IndependentVariable> spatialContourPosition;
      fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<3>, fmatvec::Fixed<3>, fmatvec::IndependentVariable> spatialContourOrientation;
      fmatvec::Vector<fmatvec::Fixed<3>, fmatvec::IndependentVariable> parS;
      fmatvec::Vec3 par;
      fmatvec::Vector<fmatvec::Fixed<3>, fmatvec::SymbolicExpression> W_r_WC;
      fmatvec::Vector<fmatvec::Fixed<3>, fmatvec::SymbolicExpression> W_r_WS;
      fmatvec::Vector<fmatvec::Fixed<3>, fmatvec::SymbolicExpression> t0S;
      fmatvec::Vector<fmatvec::Fixed<3>, fmatvec::SymbolicExpression> t1S;
      fmatvec::Vector<fmatvec::Fixed<3>, fmatvec::SymbolicExpression> tC;
      SymbolicFunction<fmatvec::Vec(fmatvec::Vec)> rootFunction;
      class MyJac : public NewtonJacobianFunction {
        public:
          MyJac() : indep(fmatvec::NONINIT) {}
          void setIndep(const fmatvec::Vector<fmatvec::Fixed<3>, fmatvec::IndependentVariable>& indep_) { indep = indep_; };
          void setDep(const fmatvec::SquareMatrix<fmatvec::Fixed<3>, fmatvec::SymbolicExpression>& dep_) { dep = dep_; };
          fmatvec::SqrMat operator()(const fmatvec::Vec &x) override {
            indep ^= x;
            return fmatvec::eval(dep);
          }
        private:
          fmatvec::Vector<fmatvec::Fixed<3>, fmatvec::IndependentVariable> indep;
          fmatvec::SquareMatrix<fmatvec::Fixed<3>, fmatvec::SymbolicExpression> dep;
      };
      MyJac jacobianFunction;
      GlobalResidualCriteriaFunction criteria;
      StandardDampingFunction damping;
      MultiDimensionalNewtonMethod rootFinding;
  };

}

#endif /* _CONTACT_KINEMATICS_CIRCLESOLID_SPATIALCONTOUR_H_ */
