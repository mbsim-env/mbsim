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

#ifndef _FLEXIBLE_BODY_1S_23_BTA_H_
#define _FLEXIBLE_BODY_1S_23_BTA_H_

#include "mbsimFlexibleBody/flexible_body/flexible_body_1s.h"

namespace MBSimFlexibleBody {

  /*! 
   * \brief bending torsional axis
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-11-22 initial commit kernel_dev
   * \todo gravity, handling for contour-node information, tangents and AWK TODO
   */
  class FlexibleBody1s23BTA : public FlexibleBody1s {
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      FlexibleBody1s23BTA(const std::string &name="");

      /**
       * \brief destructor
       */
      ~FlexibleBody1s23BTA() override = default;

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      void BuildElements() override;
      void GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) override;
      void GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) override;
      void GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat) override;

      void updatePositions(Frame1s* frame) override;
      void updateVelocities(Frame1s* frame) override;
      void updateAccelerations(Frame1s* frame) override;
      void updateJacobians(Frame1s* frame, int j=0) override;
      void updateGyroscopicAccelerations(Frame1s* frame) override;

      void updatePositions(NodeFrame* frame) override;
      void updateVelocities(NodeFrame* frame) override;
      void updateAccelerations(NodeFrame* frame) override;
      void updateJacobians(NodeFrame* frame, int j=0) override;
      void updateGyroscopicAccelerations(NodeFrame* frame) override;

      fmatvec::Vec3 getAngles(double s) override;
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      /* GETTER / SETTER */
      /**
       * \brief sets size of positions and velocities
       */
      void setNumberElements(int n); 
      void setElastModuls(double E_, double G_) { E = E_;G = G_; }
      void setDensity(double rho_) { rho = rho_; }
      void setCrossSectionalArea(double A_) { A = A_; }
      void setMomentsInertia(double Iyy_,double Izz_,double It_) { Iyy = Iyy_; Izz = Izz_; It = It_; }
//      void setContourRadius(double r) { cylinderFlexible->setRadius(r); }
      void setTorsionalDamping(double d) { dTorsional = d; }
      /***************************************************/

      /**
       * \brief compute positions and angle at Lagrangian coordinate in local FE coordinates
       * \param Lagrangian coordinate
       */
      fmatvec::Vector<fmatvec::Fixed<6>, double> getPositions(double x);

      /**
       * \brief compute velocities and differentiated angles at Lagrangian coordinate in local FE coordinates
       * \param Lagrangian coordinate
       */
      fmatvec::Vector<fmatvec::Fixed<6>, double> getVelocities(double x);

      fmatvec::SqrMat3 getOrientation(double x) override;

    protected:
      /**
       * \brief detect current finite element
       * \param global parametrisation
       * \param local parametrisation
       * \param finite element number
       */
      void BuildElement(const double& sGlobal, double& sLocal, int& currentElement);

      /**
       * \brief number of elements 
       */
      int Elements;

      /**
       * \brief length of entire beam and finite elements
       */
      double l0{0.};

      /**
       * \brief elastic modules 
       */
      double E{0}, G;

      /**
       * \brief area of cross-section
       */
      double A{0};

      /**
       * \brief area moment of inertia 
       */
      double Iyy{0}, Izz{0}, It;

      /**
       * \brief density 
       */
      double rho{0};

      /**
       * \brief contour radius
       */
      double rc{0};

      /**
       * \brief damping 
       */
      double dTorsional;

//      /** 
//       * \brief contour of body
//       */
//      CylinderFlexible *cylinderFlexible;
  };

}

#endif /* _FLEXIBLE_BODY_1S_23_BTA_H_ */
