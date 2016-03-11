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

#ifndef _FLEXIBLE_BODY_1S_33_RCM_H_
#define _FLEXIBLE_BODY_1S_33_RCM_H_

#include "mbsimFlexibleBody/flexible_body/flexible_body_1s.h"
#include "mbsimFlexibleBody/pointer.h"

namespace MBSimFlexibleBody {

  /**
   * \brief spatial beam using Redundant Coordinate Method (RCM)
   * \author Thorsten Schindler
   * \date 2009-04-17 initial commit kernel_dev (Thorsten Schindler)
   * \date 2009-05-08 visualisation (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-23 implicit integration (Thorsten Schindler)
   * \date 2012-01-10 import / export position and velocity (Cebulla / Grundl)
   * \date 2012-05-14 added Contour1sFlexible for perlchain example (Thomas Cebulla)
   * \todo gyroscopic accelerations TODO
   * \todo inverse kinetics TODO
   */
  class FlexibleBody1s33RCM : public FlexibleBody1s {
    public:
      /**
       * \brief constructor
       * \param name of body
       * \param bool to specify open (cantilever) or closed (ring) structure
       */
      FlexibleBody1s33RCM(const std::string &name="",bool openStructure=false);

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void BuildElements(double t);

      virtual void GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat);

      virtual void exportPositionVelocity(const std::string & filenamePos, const std::string & filenameVel = std::string( ), const int & deg = 3, const bool & writePsFile = false);
      virtual void importPositionVelocity(const std::string & filenamePos, const std::string & filenameVel = std::string( ));

      virtual void updatePositions(double t, Frame1s* frame);
      virtual void updateVelocities(double t, Frame1s* frame);
      virtual void updateAccelerations(double t, Frame1s* frame);
      virtual void updateJacobians(double t, Frame1s* frame, int j=0);
      virtual void updateGyroscopicAccelerations(double t, Frame1s* frame);

      virtual void updatePositions(double t, NodeFrame* frame);
      virtual void updateVelocities(double t, NodeFrame* frame);
      virtual void updateAccelerations(double t, NodeFrame* frame);
      virtual void updateJacobians(double t, NodeFrame* frame, int j=0);
      virtual void updateGyroscopicAccelerations(double t, NodeFrame* frame);

      virtual double getLocalTwist(double t, double s);
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "FlexibleBody1s33RCM"; }
      void initializeUsingXML(xercesc::DOMElement * element);
      /***************************************************/

      /* GETTER / SETTER */
      void setNumberElements(int n);
      int getNumberElements(){ return Elements; }
      void setEGModuls(double E_, double G_) { E = E_; G = G_; }
      void setDensity(double rho_) { rho = rho_; }
      void setCrossSectionalArea(double A_) { A = A_; }
      void setMomentsInertia(double I1_, double I2_, double I0_) { I1 = I1_; I2 = I2_; I0 = I0_; }
      void setMaterialDamping(double epstD_,double k0D_);

      void setGauss(int nGauss_) { nGauss = nGauss_; }
      void setCurlRadius(double R1_, double R2_);
      void setLehrDamping(double epstL_, double k0L_);
      /***************************************************/

      /**
       * \brief compute positions and angle at Lagrangian coordinate in local FE coordinates
       * \param Lagrangian coordinate
       */
      fmatvec::Vector<fmatvec::Fixed<6>, double> getPositions(double t, double x);

      /**
       * \brief compute velocities and differentiated angles at Lagrangian coordinate in local FE coordinates
       * \param Lagrangian coordinate
       */
      fmatvec::Vector<fmatvec::Fixed<6>, double> getVelocities(double t, double x);

      /*!
       * \brief compute the physical strain of the element
       * \param Lagrangian coordinate
       */
      double computePhysicalStrain(double t, const double sGlobal);

      /**
       * initialise beam only for giving information with respect to state, number elements, length, (not for simulation)
       */
      void initInfo();

    private:
      /**
       * \brief angle parametrisation
       */
      RevCardanPtr angle;

      /**
       * \brief number of elements
       */
      int Elements;

      /**
       * \brief length of entire beam and finite elements
       */
      double l0;

      /**
       * \brief elastic modules
       */
      double E, G;

      /**
       * \brief area of cross-section
       */
      double A;

      /**
       * \brief area moment of inertia
       */
      double I1, I2, I0;

      /**
       * \brief density
       */
      double rho;

      /**
       * \brief radius of undeformed shape
       */
      double R1, R2;

      /**
       * \brief damping
       */
      double epstD, k0D, epstL, k0L;

      /**
       * \brief initialised FLAG
       */
      bool initialised;

      /**
       * \brief number of Gauss points for rotational kinetic energy
       */
      int nGauss;

      /**
       * \brief detect current finite element
       * \param global parametrisation
       * \param local parametrisation
       * \param finite element number
       */
      void BuildElement(const double& sGlobal, double& sLocal, int& currentElement);
  };

}

#endif /* _FLEXIBLE_BODY_1S_33_RCM_H_ */
