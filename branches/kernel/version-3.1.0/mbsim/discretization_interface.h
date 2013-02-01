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
 *          rzander@users.berlios.de
 */

#ifndef _DISCRETIZATION_INTERFACE_H_
#define _DISCRETIZATION_INTERFACE_H_

#include <fmatvec.h>

namespace MBSim {

  class ContourPointData;

/*!
   * \brief discretization interface for flexible systems
   * \author Thorsten Schindler
   * \author Roland Zander
   * \date 2009-03-09 initial commit in kernel_dev (Thorsten Schindler)
   * \date 2009-07-23 own file / split updateM and updateh (Thorsten Schindler)
   * 
   * interface for the desription of flexible systems using global and FE ansatz functions
   */
  template<class Col>
  class DiscretizationInterface {
    public:
      /*!
       * \brief constructor
       */
      DiscretizationInterface() {}    

      /*! 
       * \brief destructor
       */
      virtual ~DiscretizationInterface() {}    

      /*!
       * \return mass matrix of discretization
       */
      virtual const fmatvec::Matrix<fmatvec::Symmetric, Col, Col, double> & getM() const = 0;

      /*!
       * \return smooth right hand side of discretization
       */
      virtual const fmatvec::Vector<Col,double >& geth() const = 0;

      /*!
       * \return Jacobian of implicit integration regarding position
       */
      virtual const fmatvec::SquareMatrix<Col, double >& getdhdq() const = 0;

      /*!
       * \return Jacobian of implicit integration regarding velocity
       */
      virtual const fmatvec::SquareMatrix<Col, double >& getdhdu() const = 0;

      /*!
       * \return dimension of positions
       */
      virtual int getqSize() const = 0;

      /*!
       * \return dimension of velocities
       */
      virtual int getuSize() const = 0;

      /*!
       * \brief compute mass matrix
       * \param generalised positions
       */
      virtual void computeM(const fmatvec::Vector<Col, double >& q) = 0;

      /*!
       * \brief compute smooth right hand side
       * \param generalised positions
       * \param generalised velocities
       */
      virtual void computeh(const fmatvec::Vector<Col, double >& q,const fmatvec::Vector<Col, double >& u) = 0;

      /*!
       * \brief compute Jacobian for implicit integration
       * \param generalised positions
       * \param generalised velocities
       */
      virtual void computedhdz(const fmatvec::Vector<Col, double >& q,const fmatvec::Vector<Col, double >& u) = 0;

      /*!
       * \brief compute kinetic energy
       * \param generalised positions
       * \param generalised velocities
       */
      virtual double computeKineticEnergy(const fmatvec::Vector<Col, double >& q,const fmatvec::Vector<Col, double >& u) = 0;

      /*! 
       * \brief compute gravitational energy
       * \param generalised positions
       */
      virtual double computeGravitationalEnergy(const fmatvec::Vector<Col, double >& q) = 0;

      /*!
       * \brief compute elastic energy
       * \param generalised positions
       */
      virtual double computeElasticEnergy(const fmatvec::Vector<Col, double >& q) = 0;

      /*! 
       * \brief compute position of contour in physical representation
       * \param generalised positions
       * \param contour location
       */
      virtual fmatvec::Vec3 computePosition(const fmatvec::Vector<Col, double >& q, const ContourPointData &data) = 0;

      /*!
       * \brief compute orientation of contour in physical representation
       * \param generalised coordiantes
       * \param contour location
       */
      virtual fmatvec::SqrMat3 computeOrientation(const fmatvec::Vector<Col, double >& q, const ContourPointData &data) = 0;

      /*!
       * \brief compute translational velocity of contour in physical representation
       * \param generalised positions
       * \param generalised velocities
       * \param contour location
       */
      virtual fmatvec::Vec3 computeVelocity(const fmatvec::Vector<Col, double >& q, const fmatvec::Vector<Col, double >& u, const ContourPointData &data) = 0;

      /*!
       * \brief compute angular velocity of contour in physical representation
       * \param generalised positions
       * \param generalised velocities
       * \param contour location
       */
      virtual fmatvec::Vec3 computeAngularVelocity(const fmatvec::Vector<Col, double >& q, const fmatvec::Vector<Col, double >& u, const ContourPointData &data) = 0;

      /*!
       * \brief compute Jacobian of minimal representation regarding physical representation
       * \param generalised positions
       * \param contour location
       */
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vector<Col, double >& q,const ContourPointData &data) = 0;
  };

}

#endif /* _DISCRETIZATION_INTERFACE_H_ */

