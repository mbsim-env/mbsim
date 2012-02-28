/*
 * ContourPairing.h
 *
 *  Created on: 25.09.2011
 *      Author: grundl
 */

#ifndef CONTOURPAIRING_H_
#define CONTOURPAIRING_H_

//#include <mbsim/link_mechanics.h>

#include <mbsim/object.h>
#include <mbsim/link.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#endif

namespace MBSim {
  class ContourPairing : public Object {
    public:
      ContourPairing(const std::string & name_, Contour* contour1, Contour* contour2, bool plotContact = true, ContactKinematics * contactKinematics_ = 0);

      virtual ~ContourPairing();

      /*INHERITED INTERFACE OF Object*/
      void updateStateDependentVariables(double t) {
        throw MBSimError("ContourPairing::updateStateDependentVariables(double t): Not implemented!");
      }
      void updateJacobians(double t, int j = 0);
      void updateInverseKineticsJacobians(double t) {
        throw MBSimError("ContourPairing::updateInverseKineticsJacobians(double t): Not implemented!");
      }
      void closePlot();
      /*******************************/

      void setContactKinematics(ContactKinematics* ck) {
        contactKinematics = ck;
      }
      ContactKinematics* getContactKinematics() const {
        return contactKinematics;
      }
      ContourPointData* getContourPointData() {
        return cpData[0];
      }
      Contour* getContour(const int & index) {
        if (index == 0)
          return contour1;
        return contour2;
      }
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group* getOpenMBVGrp() {return openMBVGroup;}
#endif

      bool isActive() {
        return gActive;
      }
      bool wasActive() {
        return gActive0;
      }
      fmatvec::Vec& getRelativeDistance() {
        return gk;
      }

      fmatvec::Vec& getRelativeVelocity() {
        return gdk;
      }

      fmatvec::Vec& getContactForce() {
        return lak;
      }

      fmatvec::Vec& getForce(int index) {
        return *(WF[index]);
      }

      void setFrictionForceLaw(FrictionForceLaw* frictionForceLaw_) {
        fdf = frictionForceLaw_;
      }

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBVContactPoints(double size = 1., bool enable = true) {
        openMBVContactFrameSize = size;
        openMBVContactFrameEnabled = enable;
      }
      std::vector<OpenMBV::Frame*> getOpenMBVContactFrame() const {
        return openMBVContactFrames;
      }
      OpenMBV::Arrow* getOpenMBVNormalForceArrow() const {
        return openMBVNormalForceArrow;
      }
      void enableOpenMBVNormalForceArrow(OpenMBV::Arrow* NormalForceArrow_) {
        openMBVNormalForceArrow = NormalForceArrow_;
      }
      OpenMBV::Arrow* getOpenMBVFrictionForceArrow() const {
        return openMBVFrictionForceArrow;
      }
      void enableOpenMBVFrictionForceArrow(OpenMBV::Arrow* FrictionForceArrow_) {
        openMBVFrictionForceArrow = FrictionForceArrow_;
      }
#endif
      /******************/

      void init(InitStage stage);

      void plot(double t, double dt);

      void updateg(double t) {
        contactKinematics->updateg(gk, cpData[0]);
      }

      void updategd(double t);

      void checkActiveg();

      int getFrictionDirections();

      FrictionForceLaw* getFrictionForceLaw() {
        return fdf;
      }
    protected:

      /**
       * \brief first of the two contours, that build the contour pairing together
       *
       * \todo: maybe use array or vector of contours --> usable for loops with contours and cpDatas
       */
      Contour* contour1;

      /**
       * \brief second of the two contours, that build the contour pairing together
       */
      Contour* contour2;

      /**
       * \brief saves contact kinematics  for each contour-pairing
       */
      ContactKinematics* contactKinematics;

      /**
       * \brief friction force law between contours
       */
      FrictionForceLaw* fdf;

      /**
       * \brief an array of contourPointData (length of 2) for definition of relative contact situation
       *
       * Every contour-pairing has its own cpData. For the both contours there is a array of the length of 2 that saves the kinematic data of the both contours.
       * So cpData saves ContourPointData for both contours
       */
      std::vector<ContourPointData*> cpData;

      /**
       * \brief tolerance value for active contact or not
       */
      double gTol;

      /**
       * \brief is contour pairing active in current time step
       */
      bool gActive;

      /**
       * \brief was contour pairing active in last time step
       */
      bool gActive0;

      /**
       * \brief relative distance of the contours
       */
      fmatvec::Vec gk;

      /**
       * \brief relative velocities of the contours
       */
      fmatvec::Vec gdk;

      /**
       * \brief vector of the contact force
       */
      fmatvec::Vec lak;

      /**
       * \brief force vectors for both contours, that are transformed into the DOF space with the Jacobian matrix (updateh())
       */
      std::vector<fmatvec::Vec*> WF;

#ifdef HAVE_OPENMBVCPPINTERFACE

      /**
       * \brief contact Group for OpenMBV-Plotting (inherits contact frames and arrows)
       */
      OpenMBV::Group* openMBVGroup;

      /**
       * \brief enable flag of ContactFrames to draw
       */
      bool openMBVContactFrameEnabled;

      /**
       * \brief vector of contact frames
       */
      std::vector<OpenMBV::Frame*> openMBVContactFrames;

      /**
       * \brief size of ContactFrames to draw
       */
      double openMBVContactFrameSize;

      /**
       * \brief style (size, color etc.) of contact frame in normal direction
       */
      OpenMBV::Arrow* openMBVNormalForceArrow;

      /**
       * \brief style (size, color etc.) of contact frame in frictional direction
       */
      OpenMBV::Arrow* openMBVFrictionForceArrow;
#endif
  };
}

#endif /* CONTOURPAIRING_H_ */
