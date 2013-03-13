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
 * Contact: martin.o.foerg@googlemail.com
 *          rzander@users.berlios.de
 */

#ifndef _CONTOUR_H_
#define _CONTOUR_H_

#include "mbsim/element.h"
#include "mbsim/contour_pdata.h"
#include "mbsim/frame.h"
#include "mbsim/mbsim_event.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class RigidBody;
}
#endif

namespace MBSim {

  class Object;
  class ContourPointData;
  class ContactKinematics;

  /**
   * \brief basic class for contour definition for rigid (which do not know about their shape) and flexible (they know how they look like) bodies
   * \author Martin Foerg
   * \date 2009-03-23 some comments (Thorsten Schindler)
   * \date 2009-04-20 RigidContour added (Thorsten Schindler)
   * \date 2009-06-04 not rigid things are in separate files
   * \date 2009-07-16 split from concret contours into new folder contours
   *
   * kinematics is stored in coordinate system class and is individually evaluated in specific contact kinematics
   */
  class Contour : public Element {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Contour();

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "Contour"; }
      virtual void plot(double t, double dt = 1);
      /***************************************************/

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \brief cartesian kinematic for contour (normal, tangent, binormal) is set by implementation class
       * \param contour parameter
       * \param selection of specific calculations for frames
       */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) = 0;

      /**
       * \brief JACOBIAN for contour (normal, tangent, binormal) is set by implementation class
       * \param contour parameter
       * \param selection of specific calculations for frames
       */
      virtual void updateJacobiansForFrame(ContourPointData &cp, int j=0) = 0;

      /**
       * \return normal in world frame
       * \param contour position
       */
      virtual fmatvec::Vec3 computeNormal(ContourPointData &cp) { updateKinematicsForFrame(cp,normal); return cp.getFrameOfReference().getOrientation().col(0); }

      /**
       * \return position in world frame
       * \param contour position
       */
      virtual fmatvec::Vec3 computePosition(ContourPointData &cp) { updateKinematicsForFrame(cp,position); return cp.getFrameOfReference().getPosition(); }

      /**
       * \return velocity in world frame
       * \param contour position
       */
      virtual fmatvec::Vec3 computeVelocity(ContourPointData &cp) { updateKinematicsForFrame(cp,velocity); return cp.getFrameOfReference().getVelocity(); }

      /**
       * \return angular in world frame
       * \param contour position
       */
      virtual fmatvec::Vec3 computeAngularVelocity(ContourPointData &cp) { updateKinematicsForFrame(cp,angularVelocity); return cp.getFrameOfReference().getAngularVelocity(); } 

      /**
       * \return curvature of contour in contour point
       * \param contour position
       */
      virtual double computeCurvature(ContourPointData &cp) { throw MBSimError("ERROR in "+getName()+"(Contour::computeCurvature): Not implemented."); return 0; }

      /**
       * \brief projects the point on the contour to compute the Lagrange parameter
       * \param WrPoint point in world coordinates
       * \return Vec of lagrange parameters
       *
       * \remark: a vector length of 2 is necessary (Vec(2,INIT,0.))
       */
      virtual fmatvec::Vec2 computeLagrangeParameter(const fmatvec::Vec3 &WrPoint) {throw MBSimError("ERROR computeLagrangeParameter(const fmatvec::Vec3 &WrPoint): Not implemented for this contour: \"" + this->getType() + "\"."); return 0; }

      /**
       * \param position of contour in inertial frame
       */
      virtual void setReferencePosition(const fmatvec::Vec3 &WrOP) { R->setPosition(WrOP); }

      /**
       * \param orientation of contour to inertial frame
       */
      virtual void setReferenceOrientation(const fmatvec::SqrMat3 &AWC) { R->setOrientation(AWC); }

      /**
       * \param velocity of contour in inertial frame
       */
      virtual void setReferenceVelocity(const fmatvec::Vec3 &WvP) { R->setVelocity(WvP); }

      /**
       * \param angular velocity of contour in inertial frame
       */
      virtual void setReferenceAngularVelocity(const fmatvec::Vec3 &WomegaC) { R->setAngularVelocity(WomegaC); }

      /**
       * \param JACOBIAN of translation of contour in inertial frame
       */
      virtual void setReferenceJacobianOfTranslation(const fmatvec::Mat3V &WJP, int j=0) { R->setJacobianOfTranslation(WJP,j); }

      /**
       * \param gyroscopic acceleration of translation of contour in inertial frame
       */
      virtual void setReferenceGyroscopicAccelerationOfTranslation(const fmatvec::Vec3 &WjP, int j=0) { R->setGyroscopicAccelerationOfTranslation(WjP,j); }

      /**
       * \param JACOBIAN of rotation of contour in inertial frame
       */
      virtual void setReferenceJacobianOfRotation(const fmatvec::Mat3V &WJR, int j=0) { R->setJacobianOfRotation(WJR,j); }

      /**
       * \param gyroscopic acceleration of rotation of contour in inertial frame
       */
      virtual void setReferenceGyroscopicAccelerationOfRotation(const fmatvec::Vec3 &WjR, int j=0) { R->setGyroscopicAccelerationOfRotation(WjR,j); }

      /**
       * \param acceleration of contour in inertial frame
       */
      virtual void setReferenceAcceleration(const fmatvec::Vec3 &WaP) { R->setAcceleration(WaP); }

      /**
       * \param angular acceleration of contour in inertial frame
       */
      virtual void setReferenceAngularAcceleration(const fmatvec::Vec3 &WpsiC) { R->setAngularAcceleration(WpsiC); }

      /**
       * \brief TODO
       */
      virtual void init(InitStage stage);
      /***************************************************/

      /**
       * \brief find contact kinematics
       * \author Markus Schneider
       * \date 2010-11-05 initial commit (Markus Schneider)
       */
      virtual ContactKinematics * findContactPairingWith(std::string type0, std::string type1) = 0;

      /* GETTER / SETTER */
      Frame* getFrame() { return R; }
      Frame* getFrameOfReference() { return R; }
      const fmatvec::Vec3& getReferencePosition() const { return R->getPosition(); }
      const fmatvec::SqrMat3& getReferenceOrientation() const { return R->getOrientation(); }
      const fmatvec::Vec3& getReferenceVelocity() const { return R->getVelocity(); }
      const fmatvec::Vec3& getReferenceAngularVelocity() const { return R->getAngularVelocity(); }
      const fmatvec::Mat3V& getReferenceJacobianOfTranslation(int j=0) const { return R->getJacobianOfTranslation(j); }
      const fmatvec::Mat3V& getReferenceJacobianOfRotation(int j=0) const { return R->getJacobianOfRotation(j); }
      fmatvec::Mat3V& getReferenceJacobianOfTranslation(int j=0) { return R->getJacobianOfTranslation(j); }
      fmatvec::Mat3V& getReferenceJacobianOfRotation(int j=0) { return R->getJacobianOfRotation(j); }
      const fmatvec::Vec3& getReferenceGyroscopicAccelerationOfTranslation() const { return R->getGyroscopicAccelerationOfTranslation(); }
      const fmatvec::Vec3& getReferenceGyroscopicAccelerationOfRotation() const { return R->getGyroscopicAccelerationOfRotation(); }
      fmatvec::Vec3& getReferenceGyroscopicAccelerationOfTranslation() { return R->getGyroscopicAccelerationOfTranslation(); }
      fmatvec::Vec3& getReferenceGyroscopicAccelerationOfRotation() { return R->getGyroscopicAccelerationOfRotation(); }

      int gethSize(int i=0) const { return hSize[i]; }
      int gethInd(int i=0) const { return hInd[i]; }
      void sethSize(int size, int i=0) { hSize[i] = size; }
      void sethInd(int ind, int i=0) { hInd[i] = ind; }

      //virtual void setPlotFeature(PlotFeature pf, PlotFeatureStatus value) { Element::setPlotFeature(pf,value); R->setPlotFeature(pf,value);}
//      ObjectInterface* getParent() { return parent; }
//      void setParent(ObjectInterface* parent_) { parent = parent_; }
      /***************************************************/

      virtual Element * getByPathSearch(std::string path);

      void setFrameOfReference(Frame *frame) { R = frame; }
      void setFrameOfReference(const std::string &frame) { saved_frameOfReference = frame; }

      virtual void updateStateDependentVariables(double t) {}
      virtual void updateJacobians(double t, int j=0) {}
      virtual void updateStateDerivativeDependentVariables(const fmatvec::Vec &ud, double t) {}

      virtual void initializeUsingXML(TiXmlElement *element);

    protected:
//      /**
//       * \brief object the contour belongs to
//       */
//      ObjectInterface* parent;

      /**
       * \brief size and index of right hand side for frame JACOBIAN settings
       */
      int hSize[2], hInd[2];

      /**
       * \brief coordinate system of contour
       */
      Frame *R;

      std::string saved_frameOfReference;
  };

  /**
   * \brief basic class for rigid contours
   * \author Thorsten Schindler
   * \date 2009-04-20 initial commit (Thorsten Schindler)
   * \date 2009-07-15 initPlot (Thorsten Schindler)
   */
  class RigidContour : public Contour {
    public:
      /**
       * \brief constructor
       * \param name of point
       */
      RigidContour(const std::string &name) : Contour(name)
# ifdef HAVE_OPENMBVCPPINTERFACE
                                              , openMBVRigidBody(0)
# endif
                                              {}

      virtual ~RigidContour();

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "RigidContour"; }
      virtual void plot(double t, double dt = 1);
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff);
      virtual void updateJacobiansForFrame(ContourPointData &cp, int j=0);
      /***************************************************/

      /**
       * \brief contact search for RigidContours
       * \author Markus Schneider
       * \date 2010-11-05 initial commit (Markus Schneider)
       */
      ContactKinematics * findContactPairingWith(std::string type0, std::string type1);

    protected:
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::RigidBody *openMBVRigidBody;
#endif

  };
}

#endif /* _CONTOUR_H_ */

