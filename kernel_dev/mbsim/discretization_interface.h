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
      virtual const fmatvec::SymMat& getM() const = 0;    

      /*!
       * \return smooth right hand side of discretization
       */
      virtual const fmatvec::Vec& geth() const = 0;

      /*!
       * \return Jacobian of implicit integration regarding position
       */
      virtual const fmatvec::SqrMat& getdhdq() const = 0;    

      /*!
       * \return Jacobian of implicit integration regarding velocity
       */
      virtual const fmatvec::SqrMat& getdhdu() const = 0;

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
      virtual void computeM(const fmatvec::Vec& q) = 0;

      /*!
       * \brief compute smooth right hand side
       * \param generalised positions
       * \param generalised velocities
       */
      virtual void computeh(const fmatvec::Vec& q,const fmatvec::Vec& u) = 0;

      /*!
       * \brief compute Jacobian for implicit integration
       * \param generalised positions
       * \param generalised velocities
       */
      virtual void computedhdz(const fmatvec::Vec& q,const fmatvec::Vec& u) = 0;

      /*!
       * \brief compute kinetic energy
       * \param generalised positions
       * \param generalised velocities
       */
      virtual double computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u) = 0;

      /*! 
       * \brief compute gravitational energy
       * \param generalised positions
       */
      virtual double computeGravitationalEnergy(const fmatvec::Vec& q) = 0;    

      /*!
       * \brief compute elastic energy
       * \param generalised positions
       */
      virtual double computeElasticEnergy(const fmatvec::Vec& q) = 0;

      /*! 
       * \brief compute position of contour in physical representation
       * \param generalised positions
       * \param contour location
       */
      virtual fmatvec::Vec computePosition(const fmatvec::Vec& q, const ContourPointData &data) = 0;

      /*!
       * \brief compute orientation of contour in physical representation
       * \param generalised coordiantes
       * \param contour location
       */
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const ContourPointData &data) = 0;

      /*!
       * \brief compute translational velocity of contour in physical representation
       * \param generalised positions
       * \param generalised velocities
       * \param contour location
       */
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data) = 0;

      /*!
       * \brief compute angular velocity of contour in physical representation
       * \param generalised positions
       * \param generalised velocities
       * \param contour location
       */
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data) = 0;

      /*!
       * \brief compute Jacobian of minimal representation regarding physical representation
       * \param generalised positions
       * \param contour location
       */
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& q,const ContourPointData &data) = 0;
  };

}

#endif /* _DISCRETIZATION_INTERFACE_H_ */

