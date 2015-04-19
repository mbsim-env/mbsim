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

#include "mbsim/mechanical_link.h"
#include "mbsim/fixed_relative_frame.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

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
  class Joint : public MechanicalLink {
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
      virtual void updatewb(double t, int i = 0);
      virtual void updateW(double t, int i = 0);
      virtual void updateh(double t, int i = 0);
      virtual void updateg(double t);
      virtual void updategd(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void updatexd(double t);
      virtual void updatedx(double t, double dt);
      virtual void calcxSize();
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual void calclaSize(int j);
      virtual void calcgSize(int j);
      virtual void calcgdSize(int j);
      virtual void calcrFactorSize(int j);
      virtual void calccorrSize(int j);
      virtual bool isSetValued() const;
      virtual bool isSingleValued() const;
      virtual bool isActive() const {
        return true;
      }
      virtual bool gActiveChanged() {
        return false;
      }
      virtual void solveImpactsFixpointSingle(double dt);
      virtual void solveConstraintsFixpointSingle();
      virtual void solveImpactsGaussSeidel(double dt);
      virtual void solveConstraintsGaussSeidel();
      virtual void solveImpactsRootFinding(double dt);
      virtual void solveConstraintsRootFinding();
      virtual void jacobianConstraints();
      virtual void jacobianImpacts();
      virtual void updaterFactors();
      virtual void checkImpactsForTermination(double dt);
      virtual void checkConstraintsForTermination();
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt = 1);
      /***************************************************/

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \brief \param first frame to connect
       * \brief second frame to connect
       */
      virtual void connect(Frame *frame1, Frame* frame2);
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

      /** \brief The frame of reference ID for the force/moment direction vectors.
       * If ID=0 (default) the first frame, if ID=1 the second frame is used.
       */
      void setFrameOfReferenceID(int ID) {
        refFrameID = ID;
      }

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

      virtual std::string getType() const {
        return "Joint";
      }

#ifdef HAVE_OPENMBVCPPINTERFACE
      /** \brief Visualize a force arrow acting on frame2 */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        setOpenMBVForce(ombv.createOpenMBV());
      }
      void setOpenMBVForce(const boost::shared_ptr<OpenMBV::Arrow> &arrow) {
        std::vector<bool> which; which.resize(2, false);
        which[1]=true;
        MechanicalLink::setOpenMBVForceArrow(arrow, which);
      }

      /** \brief Visualize a moment arrow */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toDoubleHead,referencePoint,scaleLength,scaleSize);
        setOpenMBVMoment(ombv.createOpenMBV());
      }
      void setOpenMBVMoment(const boost::shared_ptr<OpenMBV::Arrow> &arrow) {
        std::vector<bool> which; which.resize(2, false);
        which[1]=true;
        MechanicalLink::setOpenMBVMomentArrow(arrow, which);
      }
#endif

    void resetUpToDate() { C.resetUpToDate(); }

    protected:
      /**
       * \brief frame of reference the force is defined in
       */
      Frame *refFrame;
      int refFrameID;

      /**
       * \brief indices of forces and torques
       */
      fmatvec::Index IT, IR;

      /**
       * \brief local force and moment direction
       */
      fmatvec::Mat3xV forceDir, momentDir;

      /**
       * \brief global force and moment direction
       */
      fmatvec::Mat3xV Wf, Wm;

      /**
       * \brief translational JACOBIAN (not empty for e.g. prismatic joints)
       */
      fmatvec::Mat3xV JT;

      /**
       * \brief difference vector of position, velocity and angular velocity
       */
      fmatvec::Vec3 WrP0P1, WvP0P1, WomP0P1;

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

      /**
       * \brief own frame located in second partner with same orientation as first partner 
       */
      FixedRelativeFrame C;

    private:
      std::string saved_ref1, saved_ref2;
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
      virtual bool isSetValued() const {
        return true;
      }

    protected:
      Body* body;
  };

}

#endif

