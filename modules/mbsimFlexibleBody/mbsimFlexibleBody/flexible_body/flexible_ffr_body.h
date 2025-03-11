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

      void setPositionShapeFunctionIntegralArray(const std::vector<fmatvec::Mat3xV> &rPdm_) { rPdm = rPdm_; }

      void setShapeFunctionShapeFunctionIntegralArray(const std::vector<std::vector<fmatvec::SqrMatV>> &PPdm_) { PPdm = PPdm_; }

      void setStiffnessMatrix(const fmatvec::SymMatV &Ke0_) { Ke0 <<= Ke0_; }
      void setDampingMatrix(const fmatvec::SymMatV &De0_) { De0 <<= De0_; }
      void setModalDamping(const fmatvec::VecV &mDamping_) { mDamping <<= mDamping_; }
      void setProportionalDamping(const fmatvec::Vec2 &beta_) { beta = beta_; }
      // End of interface

      // Interface for nonlinear stiffness matrices
      void setNonlinearStiffnessMatrixOfFirstOrderArray(const std::vector<fmatvec::SqrMatV> &Knl1_) { Knl1 = Knl1_; }

      void setNonlinearStiffnessMatrixOfSecondOrderArray(const std::vector<std::vector<fmatvec::SqrMatV>> &Knl2_) { Knl2 = Knl2_; }
      // End of interface

      // Interface for reference stresses 
      void setInitialStressIntegral(const fmatvec::VecV &ksigma0_) { ksigma0 <<= ksigma0_; }
      void setNonlinearInitialStressIntegral(const fmatvec::SqrMatV &ksigma1_) { ksigma1 <<= ksigma1_; }
      // End of interface

      // Interface for geometric stiffness matrices
      void setGeometricStiffnessMatrixDueToAccelerationArray(const std::vector<fmatvec::SqrMatV> &K0t_) { K0t = K0t_; }

      void setGeometricStiffnessMatrixDueToAngularAccelerationArray(const std::vector<fmatvec::SqrMatV> &K0r_) { K0r = K0r_; }

      void setGeometricStiffnessMatrixDueToAngularVelocityArray(const std::vector<fmatvec::SqrMatV> &K0om_) { K0om = K0om_; }
      // End of interface

      void setNodeNumbers(const std::vector<int> &nodeNumbers_) { nodeNumbers = nodeNumbers_; }

      void setNodalRelativePositionArray(const std::vector<fmatvec::Vec3> &r) { KrKP = r; }

      void setNodalRelativeOrientationArray(const std::vector<fmatvec::SqrMat3> &A) { ARP = A; }

      void setNodalShapeMatrixOfTranslationArray(const std::vector<fmatvec::Mat3xV> &Phi_) { Phi = Phi_; }

      void setNodalShapeMatrixOfRotationArray(const std::vector<fmatvec::Mat3xV> &Psi_) { Psi = Psi_; }

      void setNodalStressMatrixArray(const std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>> &sigmahel_) { sigmahel = sigmahel_; }

      void setNodalNonlinearStressMatrixArray(const std::vector<std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>> > &sigmahen_) { sigmahen = sigmahen_; }

      void setNodalInitialStressArray(const std::vector<fmatvec::Vector<fmatvec::Fixed<6>, double>> &sigma0_) { sigma0 = sigma0_; }

      void setNodalGeometricStiffnessMatrixDueToForceArray(const std::vector<std::vector<fmatvec::SqrMatV>> &K0F_) { K0F = K0F_; }

      void setNodalGeometricStiffnessMatrixDueToMomentArray(const std::vector<std::vector<fmatvec::SqrMatV>> &K0M_) { K0M = K0M_; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void setOpenMBVFlexibleBody(const std::shared_ptr<OpenMBV::FlexibleBody> &body);
      void setOpenMBVNodeNumbers(const std::vector<int> &visuNodes_) { visuNodes = visuNodes_; }
      void setOpenMBVColorRepresentation(OpenMBVFlexibleBody::ColorRepresentation ombvColorRepresentation_) { ombvColorRepresentation = ombvColorRepresentation_; }
      void setPlotNodeNumbers(const std::vector<int> &plotNodes_) { plotNodes = plotNodes_; }

    private:
      fmatvec::Vec2 beta;
      fmatvec::VecV mDamping;
  };

}

#endif
