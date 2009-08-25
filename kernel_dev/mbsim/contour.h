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
 * Contact: mfoerg@users.berlios.de
 *          rzander@users.berlios.de
 */

#ifndef _CONTOUR_H_
#define _CONTOUR_H_

#include "mbsim/element.h"
#include "mbsim/contour_pdata.h"
#include "mbsim/frame.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/rigidbody.h>
#endif

namespace MBSim {

  class Object;

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
      virtual void updateJacobiansForFrame(ContourPointData &cp) = 0;

      /**
       * \return normal in world frame
       * \param contour position
       */
      virtual fmatvec::Vec computeNormal(ContourPointData &cp) { updateKinematicsForFrame(cp,normal); return cp.getFrameOfReference().getOrientation().col(0); }

      /**
       * \return position in world frame
       * \param contour position
       */
      virtual fmatvec::Vec computePosition(ContourPointData &cp) { updateKinematicsForFrame(cp,position); return cp.getFrameOfReference().getPosition(); }

      /**
       * \return velocity in world frame
       * \param contour position
       */
      virtual fmatvec::Vec computeVelocity(ContourPointData &cp) { updateKinematicsForFrame(cp,velocity); return cp.getFrameOfReference().getVelocity(); }

      /**
       * \return angular in world frame
       * \param contour position
       */
      virtual fmatvec::Vec computeAngularVelocity(ContourPointData &cp) { updateKinematicsForFrame(cp,angularVelocity); return cp.getFrameOfReference().getAngularVelocity(); } 

      /**
       * \param position of contour in inertial frame
       */
      virtual void setReferencePosition(const fmatvec::Vec &WrOP) { R.setPosition(WrOP); }

      /**
       * \param orientation of contour to inertial frame
       */
      virtual void setReferenceOrientation(const fmatvec::SqrMat &AWC) { R.setOrientation(AWC); }

      /**
       * \param velocity of contour in inertial frame
       */
      virtual void setReferenceVelocity(const fmatvec::Vec &WvP) { R.setVelocity(WvP); }

      /**
       * \param angular velocity of contour in inertial frame
       */
      virtual void setReferenceAngularVelocity(const fmatvec::Vec &WomegaC) { R.setAngularVelocity(WomegaC); }

      /**
       * \param JACOBIAN of translation of contour in inertial frame
       */
      virtual void setReferenceJacobianOfTranslation(const fmatvec::Mat &WJP) { R.setJacobianOfTranslation(WJP); }

      /**
       * \param gyroscopic acceleration of translation of contour in inertial frame
       */
      virtual void setReferenceGyroscopicAccelerationOfTranslation(const fmatvec::Vec &WjP) { R.setGyroscopicAccelerationOfTranslation(WjP); }

      /**
       * \param JACOBIAN of rotation of contour in inertial frame
       */
      virtual void setReferenceJacobianOfRotation(const fmatvec::Mat &WJR) { R.setJacobianOfRotation(WJR); }

      /**
       * \param gyroscopic acceleration of rotation of contour in inertial frame
       */
      virtual void setReferenceGyroscopicAccelerationOfRotation(const fmatvec::Vec &WjR) { R.setGyroscopicAccelerationOfRotation(WjR); }

      /**
       * \brief TODO
       */
      virtual void init(InitStage stage);
      /***************************************************/

      /* GETTER / SETTER */
      Frame* getFrame() { return &R; }
      const fmatvec::Vec& getReferencePosition() const { return R.getPosition(); }
      const fmatvec::SqrMat& getReferenceOrientation() const { return R.getOrientation(); }
      const fmatvec::Vec& getReferenceVelocity() const { return R.getVelocity(); }
      const fmatvec::Vec& getReferenceAngularVelocity() const { return R.getAngularVelocity(); }
      const fmatvec::Mat& getReferenceJacobianOfTranslation() const { return R.getJacobianOfTranslation(); }
      const fmatvec::Mat& getReferenceJacobianOfRotation() const { return R.getJacobianOfRotation(); }
      fmatvec::Mat& getReferenceJacobianOfTranslation() { return R.getJacobianOfTranslation(); }
      fmatvec::Mat& getReferenceJacobianOfRotation() { return R.getJacobianOfRotation(); }
      const fmatvec::Vec& getReferenceGyroscopicAccelerationOfTranslation() const { return R.getGyroscopicAccelerationOfTranslation(); }
      const fmatvec::Vec& getReferenceGyroscopicAccelerationOfRotation() const { return R.getGyroscopicAccelerationOfRotation(); }
      fmatvec::Vec& getReferenceGyroscopicAccelerationOfTranslation() { return R.getGyroscopicAccelerationOfTranslation(); }
      fmatvec::Vec& getReferenceGyroscopicAccelerationOfRotation() { return R.getGyroscopicAccelerationOfRotation(); }

      int gethSize(int i=0) const { return hSize[i]; }
      int gethInd(int i=0) const { return hInd[i]; }
      void sethSize(int size, int i=0) { hSize[i] = size; }
      void sethInd(int ind, int i=0) { hInd[i] = ind; }

      ObjectInterface* getParent() { return parent; }
      void setParent(ObjectInterface* parent_) { parent = parent_; }
      /***************************************************/

      /**
       * \brief TODO
       */
      void resizeJacobians(int j);

    protected:
      /**
       * \brief object the contour belongs to
       */
      ObjectInterface* parent;

      /**
       * \brief size and index of right hand side for frame JACOBIAN settings
       */
      int hSize[2], hInd[2];

      /**
       * coordinate system of contour
       */
      Frame R;
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

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "RigidContour"; }
      virtual void plot(double t, double dt = 1);
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff);
      virtual void updateJacobiansForFrame(ContourPointData &cp);
      /***************************************************/

    protected:
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::RigidBody *openMBVRigidBody;
#endif

  };
}

#endif /* _CONTOUR_H_ */

