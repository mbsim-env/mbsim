/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _JOINT_H_
#define _JOINT_H_

#include "mbsim/links/floating_frame_link.h"

namespace MBSim {

  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;
  class FrictionForceLaw;
  class FrictionImpactLaw;
  class Body;

  /** 
   * \brief class for connections: constraints on frames
   * \author Martin Foerg
   * \date 2009-04-06 MechanicalLink added (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-08-21 one force direction (Thorsten Schindler)
   * \date 2014-09-16 contact forces are calculated on acceleration level (Thorsten Schindler)
   * \todo hSize Frame C
   */
  class Joint : public FloatingFrameLink {
    public:
      /**
       * \brief constructor
       * \param name
       */
      Joint(const std::string &name = "");

      /**
       * \brief destructor
       */
      ~Joint() override;

      void updatelaF() override { (this->*updatelaF_)(); updlaF = false; }
      void updatelaM() override { (this->*updatelaM_)(); updlaM = false; }
      void updatelaF0() { }
      void updatelaFS();
      void updatelaFM();
      void updatelaM0() { }
      void updatelaMS();
      void updatelaMM();
      void (Joint::*updatelaF_)();
      void (Joint::*updatelaM_)();
      void updateh(int i=0) override;
      void updateW(int i=0) override;
      void updatewb() override;
      void updatexd() override;

      void calcxSize() override { if(integrateGeneralizedRelativeVelocityOfRotation) xSize = momentDir.cols(); }
      void init(InitStage stage, const InitConfigSet &config) override;

      bool isSetValued() const override;
      bool isSingleValued() const override;
      bool isActive() const override { return true; }
      bool gActiveChanged() override { return false; }
      void solveImpactsFixpointSingle() override;
      void solveConstraintsFixpointSingle() override;
      void solveImpactsGaussSeidel() override;
      void solveConstraintsGaussSeidel() override;
      void solveImpactsRootFinding() override;
      void solveConstraintsRootFinding() override;
      void jacobianConstraints() override;
      void jacobianImpacts() override;
      void updaterFactors() override;
      void checkImpactsForTermination() override;
      void checkConstraintsForTermination() override;

      void setForceLaw(GeneralizedForceLaw * rc);
      void setMomentLaw(GeneralizedForceLaw * rc);

      /**
       * \param local force direction represented in first frame
       */
      void setForceDirection(const fmatvec::Mat3xV& fd);

      /**
       * \param local moment direction represented in first frame
       */
      void setMomentDirection(const fmatvec::Mat3xV& md);

      void setIntegrateGeneralizedRelativeVelocityOfRotation(bool integrateGeneralizedRelativeVelocityOfRotation_) { integrateGeneralizedRelativeVelocityOfRotation = integrateGeneralizedRelativeVelocityOfRotation_; }

      fmatvec::VecV evalGeneralizedRelativePositionOfRotation() override;
      fmatvec::Vec3 evalGlobalRelativeAngle();

      void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      /**
       * \brief translational JACOBIAN (not empty for e.g. prismatic joints)
       */
      fmatvec::Mat3xV JT;

      /**
       * constitutive law on acceleration level for forces and torques (f?-force-law)
       */
      GeneralizedForceLaw *ffl{nullptr};

      /**
       * constitutive law on acceleration level for moments/torques (f?-moment-law)
       */
      GeneralizedForceLaw * fml{nullptr};

      /**
       * constitutive law on velocity level for forces and torques (f?-impact-force-law)
       */
      GeneralizedImpactLaw *fifl{nullptr};

      /**
       * constitutive law on velocity level for moments/torques (f?-impact-moment-law)
       */
      GeneralizedImpactLaw * fiml{nullptr};

      /**
       * \brief relative velocity and acceleration after an impact for event driven scheme summarizing all possible contacts
       */
      fmatvec::Vec gdn, gdd;

      fmatvec::Vec3 WphiK0K1, eR;

      size_t iR{2};

      bool integrateGeneralizedRelativeVelocityOfRotation{false};
  };

  class InverseKineticsJoint : public Joint {
    friend class RigidBody;
    friend class JointConstraint;
    public:
      InverseKineticsJoint(const std::string &name);
      void updateb() override;
      void calcbSize() override;
      void setBody(Body* body_) {
        body = body_;
      }
      void init(InitStage stage, const InitConfigSet &config) override;
      bool isSetValued() const override { return true; }

    protected:
      Body* body;
  };

}

#endif
