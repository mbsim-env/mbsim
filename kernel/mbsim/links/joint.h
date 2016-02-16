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
      virtual ~Joint();

      /* INHERITED INTERFACE OF LINKINTERFACE */
      void updatelaF(double t) { (this->*updatelaF_)(t); updlaF = false; }
      void updatelaM(double t) { (this->*updatelaM_)(t); updlaM = false; }
      void updatelaF0(double t) { }
      void updatelaFS(double t);
      void updatelaFM(double t);
      void updatelaM0(double t) { }
      void updatelaMS(double t);
      void updatelaMM(double t);
      void (Joint::*updatelaF_)(double t);
      void (Joint::*updatelaM_)(double t);
      void updateh(double t, int i=0);
      void updateW(double t, int i=0);
      void updatewb(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void updatexd(double t);
      virtual void updatedx(double t, double dt);
      virtual void calcxSize();
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual bool isSetValued() const;
      virtual bool isSingleValued() const;
      virtual bool isActive() const { return true; }
      virtual bool gActiveChanged() { return false; }
      virtual void solveImpactsFixpointSingle(double t, double dt);
      virtual void solveConstraintsFixpointSingle(double t);
      virtual void solveImpactsGaussSeidel(double t, double dt);
      virtual void solveConstraintsGaussSeidel(double t);
      virtual void solveImpactsRootFinding(double t, double dt);
      virtual void solveConstraintsRootFinding(double t);
      virtual void jacobianConstraints(double t);
      virtual void jacobianImpacts(double t);
      virtual void updaterFactors(double t);
      virtual void checkImpactsForTermination(double t, double dt);
      virtual void checkConstraintsForTermination(double t);
      /***************************************************/

      /* GETTER / SETTER */
      void setForceLaw(GeneralizedForceLaw * rc);
      void setMomentLaw(GeneralizedForceLaw * rc);
      /***************************************************/

      /**
       * \param local force direction represented in first frame
       */
      void setForceDirection(const fmatvec::Mat3xV& fd);

      /**
       * \param local moment direction represented in first frame
       */
      void setMomentDirection(const fmatvec::Mat3xV& md);

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

      virtual std::string getType() const { return "Joint"; }

    protected:
      /**
       * \brief translational JACOBIAN (not empty for e.g. prismatic joints)
       */
      fmatvec::Mat3xV JT;

      /**
       * constitutive law on acceleration level for forces and torques (f?-force-law)
       */
      GeneralizedForceLaw *ffl;

      /**
       * constitutive law on acceleration level for moments/torques (f?-moment-law)
       */
      GeneralizedForceLaw * fml;

      /**
       * constitutive law on velocity level for forces and torques (f?-impact-force-law)
       */
      GeneralizedImpactLaw *fifl;

      /**
       * constitutive law on velocity level for moments/torques (f?-impact-moment-law)
       */
      GeneralizedImpactLaw * fiml;

      /**
       * \brief relative velocity and acceleration after an impact for event driven scheme summarizing all possible contacts
       */
      fmatvec::Vec gdn, gdd;
  };

  class InverseKineticsJoint : public Joint {
    public:
      InverseKineticsJoint(const std::string &name);
      virtual void updateb(double t);
      void calcbSize();
      void setBody(Body* body_) {
        body = body_;
      }
      virtual void init(InitStage stage);
      virtual bool isSetValued() const { return true; }

    protected:
      Body* body;
  };

}

#endif
