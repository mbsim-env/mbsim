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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _FLEXIBLE_FFR_BODY_H_
#define _FLEXIBLE_FFR_BODY_H_

#include "mbsimFlexibleBody/flexible_body/generic_flexible_ffr_body.h"
#include <openmbvcppinterface/flexiblebody.h>
#include <openmbvcppinterface/objectfactory.h>
#include <openmbvcppinterface/group.h>

namespace OpenMBV {
  class FlexibleBody;
}

namespace MBSimFlexibleBody {

  /*!
   *  \brief Flexible body using a floating frame of reference formulation
   *
   * */
  class FlexibleFfrBody : public GenericFlexibleFfrBody {

    public:
      FlexibleFfrBody(const std::string &name="") : GenericFlexibleFfrBody(name) { }

      // Interface for basic data
      /*! \brief Set mass
       *
       * Set the mass of the flexible body.
       * \param m The mass of the body
       * */
      void setMass(double m_) { m = m_; }
      void setPositionIntegral(const fmatvec::Vec3 &rdm_) { rdm = rdm_; }
      void setPositionPositionIntegral(const fmatvec::SymMat3& rrdm_) { rrdm = rrdm_; }
      void setShapeFunctionIntegral(const fmatvec::Mat3xV &Pdm_) { Pdm <<= Pdm_; }

      void setPositionShapeFunctionIntegral(const std::vector<fmatvec::Mat3xV> &rPdm) { setPositionShapeFunctionIntegralArray(rPdm); }
      void setPositionShapeFunctionIntegralArray(const std::vector<fmatvec::Mat3xV> &rPdm_) { rPdm = rPdm_; }
      void setPositionShapeFunctionIntegral(const fmatvec::MatV &rPdm_) { rPdm = getCellArray1D<fmatvec::Mat3xV>(3,rPdm_); }

      void setShapeFunctionShapeFunctionIntegral(const std::vector<std::vector<fmatvec::SqrMatV>> &PPdm) { setShapeFunctionShapeFunctionIntegralArray(PPdm); }
      void setShapeFunctionShapeFunctionIntegralArray(const std::vector<std::vector<fmatvec::SqrMatV>> &PPdm_) { PPdm = PPdm_; }
      void setShapeFunctionShapeFunctionIntegral(const fmatvec::MatV &PPdm_) { PPdm = getCellArray2D<fmatvec::SqrMatV>(PPdm_.cols(),3,PPdm_); }

      void setStiffnessMatrix(const fmatvec::SymMatV &Ke0_) { Ke0 <<= Ke0_; }
      void setDampingMatrix(const fmatvec::SymMatV &De0_) { De0 <<= De0_; }
      // End of interface

      // Interface for nonlinear stiffness matrices
      void setNonlinearStiffnessMatrixOfFirstOrder(const std::vector<fmatvec::SqrMatV> &Knl1) { setNonlinearStiffnessMatrixOfFirstOrderArray(Knl1); }
      void setNonlinearStiffnessMatrixOfFirstOrderArray(const std::vector<fmatvec::SqrMatV> &Knl1_) { Knl1 = Knl1_; }
      void setNonlinearStiffnessMatrixOfFirstOrder(const fmatvec::MatV &Knl1_) { Knl1 = getCellArray1D<fmatvec::SqrMatV>(Knl1_.cols(),Knl1_); }

      void setNonlinearStiffnessMatrixOfSecondOrder(const std::vector<std::vector<fmatvec::SqrMatV>> &Knl2) { setNonlinearStiffnessMatrixOfSecondOrderArray(Knl2); }
      void setNonlinearStiffnessMatrixOfSecondOrderArray(const std::vector<std::vector<fmatvec::SqrMatV>> &Knl2_) { Knl2 = Knl2_; }
      void setNonlinearStiffnessMatrixOfSecondOrder(const fmatvec::MatV &Knl2_) { Knl2 = getCellArray2D<fmatvec::SqrMatV>(Knl2_.cols(),Knl2_.cols(),Knl2_); }
      // End of interface

      // Interface for reference stresses 
      void setInitialStressIntegral(const fmatvec::VecV &ksigma0_) { ksigma0 <<= ksigma0_; }
      void setNonlinearInitialStressIntegral(const fmatvec::SqrMatV &ksigma1_) { ksigma1 <<= ksigma1_; }
      // End of interface

      // Interface for geometric stiffness matrices
      void setGeometricStiffnessMatrixDueToAcceleration(const std::vector<fmatvec::SqrMatV> &K0t) { setGeometricStiffnessMatrixDueToAccelerationArray(K0t); }
      void setGeometricStiffnessMatrixDueToAccelerationArray(const std::vector<fmatvec::SqrMatV> &K0t_) { K0t = K0t_; }
      void setGeometricStiffnessMatrixDueToAcceleration(const fmatvec::MatV &K0t_) { K0t = getCellArray1D<fmatvec::SqrMatV>(K0t_.cols(),K0t_); }

      void setGeometricStiffnessMatrixDueToAngularAcceleration(const std::vector<fmatvec::SqrMatV> &K0r) { setGeometricStiffnessMatrixDueToAngularAccelerationArray(K0r); }
      void setGeometricStiffnessMatrixDueToAngularAccelerationArray(const std::vector<fmatvec::SqrMatV> &K0r_) { K0r = K0r_; }
      void setGeometricStiffnessMatrixDueToAngularAcceleration(const fmatvec::MatV &K0r_) { K0r = getCellArray1D<fmatvec::SqrMatV>(K0r_.cols(),K0r_); }

      void setGeometricStiffnessMatrixDueToAngularVelocity(const std::vector<fmatvec::SqrMatV> &K0om) { setGeometricStiffnessMatrixDueToAngularVelocityArray(K0om); }
      void setGeometricStiffnessMatrixDueToAngularVelocityArray(const std::vector<fmatvec::SqrMatV> &K0om_) { K0om = K0om_; }
      void setGeometricStiffnessMatrixDueToAngularVelocity(const fmatvec::MatV &K0om_) { K0om = getCellArray1D<fmatvec::SqrMatV>(K0om_.cols(),K0om_); }
      // End of interface

      void setNodeNumbers(const std::vector<int> &n);

      void setNodalRelativePosition(const std::vector<fmatvec::Vec3> &r) { setNodalRelativePositionArray(r); }
      void setNodalRelativePositionArray(const std::vector<fmatvec::Vec3> &r) { KrKP = r; }
      void setNodalRelativePosition(const fmatvec::VecV &r) { KrKP = getCellArray1D<fmatvec::Vec3>(3,r); }

      void setNodalRelativeOrientation(const std::vector<fmatvec::SqrMat3> &A) { setNodalRelativeOrientationArray(A); }
      void setNodalRelativeOrientationArray(const std::vector<fmatvec::SqrMat3> &A) { ARP = A; }
      void setNodalRelativeOrientation(const fmatvec::MatVx3 &A) { ARP = getCellArray1D<fmatvec::SqrMat3>(3,A); }

      void setNodalShapeMatrixOfTranslation(const std::vector<fmatvec::Mat3xV> &Phi) { setNodalShapeMatrixOfTranslationArray(Phi); }
      void setNodalShapeMatrixOfTranslationArray(const std::vector<fmatvec::Mat3xV> &Phi_) { Phi = Phi_; }
      void setNodalShapeMatrixOfTranslation(const fmatvec::MatV &Phi_) { Phi = getCellArray1D<fmatvec::Mat3xV>(3,Phi_); }

      void setNodalShapeMatrixOfRotation(const std::vector<fmatvec::Mat3xV> &Psi) { setNodalShapeMatrixOfRotationArray(Psi); }
      void setNodalShapeMatrixOfRotationArray(const std::vector<fmatvec::Mat3xV> &Psi_) { Psi = Psi_; }
      void setNodalShapeMatrixOfRotation(const fmatvec::MatV &Psi_) { Psi = getCellArray1D<fmatvec::Mat3xV>(3,Psi_); }

      void setNodalStressMatrix(const std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>> &sigmahel) { setNodalStressMatrixArray(sigmahel); }
      void setNodalStressMatrixArray(const std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>> &sigmahel_) { sigmahel = sigmahel_; }
      void setNodalStressMatrix(const fmatvec::MatV &sigmahel_) { sigmahel = getCellArray1D<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>>(6,sigmahel_); }

      void setNodalNonlinearStressMatrix(const std::vector<std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>> > &sigmahen) { setNodalNonlinearStressMatrixArray(sigmahen); }
      void setNodalNonlinearStressMatrixArray(const std::vector<std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>> > &sigmahen_) { sigmahen = sigmahen_; }
      void setNodalNonlinearStressMatrix(const fmatvec::MatV &sigmahen_) { sigmahen = getCellArray2D<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>>(6,sigmahen_.cols(),sigmahen_); }

      void setNodalInitialStress(const std::vector<fmatvec::Vector<fmatvec::Fixed<6>, double>> &sigma0) { setNodalInitialStressArray(sigma0); }
      void setNodalInitialStressArray(const std::vector<fmatvec::Vector<fmatvec::Fixed<6>, double>> &sigma0_) { sigma0 = sigma0_; }
      void setNodalInitialStress(const fmatvec::VecV &sigma0_) { sigma0 = getCellArray1D<fmatvec::Vector<fmatvec::Fixed<6>, double>>(6,sigma0_); }

      void setNodalGeometricStiffnessMatrixDueToForce(const std::vector<std::vector<fmatvec::SqrMatV>> &K0F) { setNodalGeometricStiffnessMatrixDueToForceArray(K0F); }
      void setNodalGeometricStiffnessMatrixDueToForceArray(const std::vector<std::vector<fmatvec::SqrMatV>> &K0F_) { K0F = K0F_; }
      void setNodalGeometricStiffnessMatrixDueToForce(const fmatvec::MatV &K0F_) { K0F = getCellArray2D<fmatvec::SqrMatV>(K0F_.cols(),K0F_.cols(),K0F_); }

      void setNodalGeometricStiffnessMatrixDueToMoment(const std::vector<std::vector<fmatvec::SqrMatV>> &K0M) { setNodalGeometricStiffnessMatrixDueToMomentArray(K0M); }
      void setNodalGeometricStiffnessMatrixDueToMomentArray(const std::vector<std::vector<fmatvec::SqrMatV>> &K0M_) { K0M = K0M_; }
      void setNodalGeometricStiffnessMatrixDueToMoment(const fmatvec::MatV &K0M_) { K0M = getCellArray2D<fmatvec::SqrMatV>(K0M_.cols(),K0M_.cols(),K0M_); }

      void initializeUsingXML(xercesc::DOMElement *element) override;

      void setOpenMBVFlexibleBody(const std::shared_ptr<OpenMBV::FlexibleBody> &body);
      void setOpenMBVColorRepresentation(OpenMBVFlexibleBody::ColorRepresentation ombvColorRepresentation_) { ombvColorRepresentation = ombvColorRepresentation_; }
      void setPlotNodeNumbers(const fmatvec::VecVI &plotNodes_) { plotNodes = plotNodes_; }
  };

}

#endif
