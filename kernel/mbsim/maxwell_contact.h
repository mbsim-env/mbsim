/* Copyright (C) 2004-2012  MBSim Development Team

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

#ifndef MAXWELL_CONTACT_H_
#define MAXWELL_CONTACT_H_

#include <mbsim/object.h>
#include <mbsim/contour.h>
#include <mbsim/contour_pairing.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <mbsim/link_mechanics.h>
#include <mbsim/utils/function.h>
#include <mbsim/utils/nonlinear_algebra.h>
#ifdef HAVE_MBSIMNUMERICS
#include <numerics/linear_complementarity_problem/linear_complementarity_problem.h>
#else
#include <mbsim/utils/linear_complementarity_problem.h>
#endif

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/frame.h>
#endif

#include <fmatvec.h>
#include <map>

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class Frame;
  class Arrow;
}
#endif

namespace MBSim {

  class GeneralizedForceLaw;
  class FrictionForceLaw;
  class GeneralizedImpactLaw;
  class FrictionImpactLaw;
  class ContourPointData;

  /*! \brief class for Maxwell contact force law
   * \author Kilian Grundl
   * \date 2011-09-02 first check in
   *
   * Remarks:
   * - constitutive laws on acceleration and velocity level have to be set pairwise
   */

  class MaxwellContact : public MBSim::LinkMechanics {
    public:

      /**
       * \brief constructor
       * \param name of contact
       */
      MaxwellContact(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~MaxwellContact() {
      }

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updateh(double t, int k=0);
      virtual void updateg(double t);
      virtual void updategd(double t);
      virtual void updateJacobians(double t, int j=0);
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual void updatehRef(const fmatvec::Vec &hRef, int k=0);
      virtual void init(InitStage stage);
      virtual bool isSetValued() const {
        return false; //The MaxwellContact isn't a set valued contact, but also not regularized (single valued) but its kind of multi-valued
      }
      virtual bool isActive() const {
        return true; //the Maxwell contact is always active as the the distances are computed dependent on the forces
      }
      virtual bool gActiveChanged() {
        return false; //gActive (probably) changes every timestep //TODO what does this function?
      }
      virtual void checkActiveg();
      /***************************************************/

      /*INHERITED INTERFACE OF OBJECT*/
      void setParent(DynamicSystem *sys);
      /*******************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const {
        return "MaxwellContact";
      }
      virtual void plot(double t, double dt = 1);
      virtual void closePlot();
      /***************************************************/

      /* GETTER / SETTER */
      ContourPairing* getContourPariing(const int & contactNumber) {
        return contourPairing[contactNumber];
      }
#ifdef HAVE_MBSIMNUMERICS
      void setLCPSolvingStrategy(MBSimNumerics::LCPSolvingStrategy strategy) {
        lcpSolvingStrategy = strategy;
      }

      MBSimNumerics::LCPSolvingStrategy getLCPSolvingStrategy(void) {
        return lcpSolvingStrategy;
      }
#endif

      /**
       * \brief output information to console?
       */
      void setDebuglevel(const int & Debuglevel_) {
        DEBUGLEVEL = Debuglevel_;
      }


      /*!
       * \brief add a contour-pairing to force law
       * \param first contour
       * \param second contour
       * \param contactKinematics for both contours
       *
       * \General Remark: The parameters (LagrangeParameterPositions) of the function (type: Function2) for the influence numbers have to be in the same order like the contours here.
       *                  So the first input parameter for the function will be the LagrangeParameterPosition of contour1 and the second input parameter for the function will be the LagrangeParameterPosition for contour2
       */
      void addContourPairing(ContourPairing* contourPairing_);// (Contour *contour1, Contour *contour2, std::string name, bool plotContact = true, ContactKinematics* contactKinematics_ = 0);

      /**
       * \brief add a function that represents the coupling between two contours
       * \param name of first contour
       * \param name of second contour
       * \param Function to describe coupling between contours
       *
       * \General Remark: The parameters (LagrangeParameterPositions) of the function have to be in the same order as it was given the add(...)-method
       */
      void addContourCoupling(Contour *contour1, Contour *contour2, InfluenceFunction *fct);

    protected:
      /**
       * \brief saves all possible contacts in a vector
       */
      virtual void updatePossibleContactPoints();

      /**
       * \brief updates the influence matrix C
       */
      virtual void updateInfluenceMatrix(const double t);

      /**
       * \brief update the rigid body distances (gaps) for the single contacts
       */
      void updateRigidBodyGap(const double & t);

      /**
       * \brief computes the coupling factor for the influence matrix on one contact point (two contours)
       * \param number of contact point
       */
      virtual double computeInfluenceCoefficient(const int &currentContactNumber);

      /**
       * \brief computes the coupling factor for the influence matrix between two contact points (four contours)
       * \param number of contact point
       * \param number of coupling contact point
       */
      virtual double computeInfluenceCoefficient(const int &currentContactNumber, const int &couplingContactNumber);

      /*
       * \brief computes the "material constant" to have a good guess for the lambda-vector
       */
      virtual void computeMaterialConstant(const double & t);

      /**
       * \brief fit the solution vector for the solveLCP-routine if the size of the LCP has changed
       */
      virtual void fitSolution0(const double & t);

      /**
       * \brief vector of ContourPairing (frames and Arrows)
       */
      std::vector<ContourPairing*> contourPairing;

      /**
       * \brief saves the indices of all active contacts
       */
      std::vector<int> possibleContactPoints;

      /**
       * \brief Influence matrix between contact points
       */
      fmatvec::SymMat C;

      /*
       * \brief vector of rigid body distances(gaps) for the active contacts
       */
      fmatvec::Vec rigidBodyGap;

      /**
       * \brief saves the influence functions for a pair of contours. The key is the pair of contour names
       */
      std::map<std::pair<Contour*, Contour*>, InfluenceFunction*> influenceFunctions;

#ifdef HAVE_MBSIMNUMERICS
      /**
       * \brief strategy for solving the LCP
       */
      MBSimNumerics::LCPSolvingStrategy lcpSolvingStrategy;
#endif

      /**
       * \brief Solution of the last time, where contact has to be solved (can be used as starting guess for the next algorithm)
       */
      fmatvec::Vec solution0;

      /**
       * \brief parameter for guessing starting values of contact force (average eigenvalue of influence-matrix)
       */
      double matConst;

      /**
       * \brief parameter to save if matConst has been computed already
       */
      bool matConstSetted;

      /**
       * \brief print INFO output?
       *
       * 0 = no DEBUGOutput
       * 1 = most important information
       * 2 = ...
       * 5 = Matrices and Vectors
       * \todo wouldn't a logger for MBSim be nice
       */
      int DEBUGLEVEL;
  };

}
#endif /* MAXWELL_CONTACT_H_ */
