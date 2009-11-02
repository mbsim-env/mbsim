/* Copyright (c) 2005-2008 Thorsten Schindler, Roland Zander
 
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
 * Version: 18.10.2008
 */

#ifndef DISCRETIZATION_INTERFACE_H_
#define DISCRETIZATION_INTERFACE_H_

#include "fmatvec.h"
#include "contour_pdata.h"

namespace MBSim {
  /*! \brief discretization interface for flexible systems
   * 
   *  Interface for the desription of flexible systems using global and FE ansatz functions;
   *  this does not provide User-Interfaces but strictly unifies internal access on
   *  interfaces of structural data exchange. User interfaces must be implemented in
   *  BodyFlexible... whereas parameters are passed and stored here, e.g. in derived
   *  classes
   */
  class DiscretizationInterface {
    public:
      /*! constructor */
      DiscretizationInterface() {}    
      /*! destructor */
      virtual ~DiscretizationInterface() {}    
      
      /*! get mass matrix of discretization */
      virtual fmatvec::SymMat getMassMatrix() const = 0;    
      /*! get smooth right hand side of discretization */
      virtual fmatvec::Vec getGeneralizedForceVector() const = 0;
      /*! get Jacobian of implicit integration regarding position */
      virtual fmatvec::SqrMat getJacobianForImplicitIntegrationRegardingPosition() const = 0;    
      /*! get Jacobian of implicit integration regarding velocity */
      virtual fmatvec::SqrMat getJacobianForImplicitIntegrationRegardingVelocity() const = 0;
      /*! get dimension of positions */
      virtual int getSizeOfPositions() const = 0;
      /*! get dimension of velocities */
      virtual int getSizeOfVelocities() const = 0;
       
      /*! compute equations of motion */
      virtual void computeEquationsOfMotion(const fmatvec::Vec& q,const fmatvec::Vec& u) = 0;
      /*! compute kinetic energy */
      virtual double computeKineticEnergy(const fmatvec::Vec& q,const fmatvec::Vec& u) = 0;
      /*! compute gravitational energy */
      virtual double computeGravitationalEnergy(const fmatvec::Vec& q) = 0;    
      /*! compute elastic energy */
      virtual double computeElasticEnergy(const fmatvec::Vec& q) = 0;

      /*! compute position of contour in physical representation */
      virtual fmatvec::Vec computeTranslation(const fmatvec::Vec& q, const ContourPointData &data) = 0;
      /*! compute orientation of contour in physical representation */
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const ContourPointData &data) = 0;
      /*! compute translational velocity of contour in physical representation */
      virtual fmatvec::Vec computeTranslationalVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data) = 0;
      /*! compute angular velocity of contour in physical representation */
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const ContourPointData &data) = 0;
      /*! compute Jacobian of minimal representation regarding physical representation */
      virtual fmatvec::Mat computeJacobianOfMinimalRepresentationRegardingPhysics(const fmatvec::Vec& q,const ContourPointData &data) = 0;
  };
}

#endif /*DISCRETIZATION_INTERFACE_H_*/
