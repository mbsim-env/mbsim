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

#ifndef _MAXWELL_CONTACT_H_
#define _MAXWELL_CONTACT_H_

#include <mbsim/links/contact.h>
#include <mbsim/numerics/linear_complementarity_problem/linear_complementarity_problem.h>
#include <map>

namespace MBSim {

  class ContactKinematics;
  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;
  class FrictionForceLaw;
  class FrictionImpactLaw;
  class InfluenceFunction;

  /*! \brief class for contacts
   * \author Martin Foerg
   * \date 2009-04-02 some comments (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-08-03 contacts can now visualize their ContactPointFrames (Markus Schneider)
   * \date 2010-07-06 added LinkStatus and LinearImpactEstimation for timestepper ssc (Robert Huber)
   * \date 2012-05-08 added LinkStatusReg for AutoTimeSteppingSSCIntegrator (Jan Clauberg)
   * \date 2014-09-16 contact forces are calculated on acceleration level (Thorsten Schindler)
   *
   * basic class for contacts between contours, mainly implementing geometrical informations of contact-pairings
   * 
   * Remarks:
   * - constitutive laws on acceleration and velocity level have to be set pairwise
   */
  class MaxwellContact : public Contact {
    public:
      /*!
       * \brief constructor
       * \param name of contact
       */
      MaxwellContact(const std::string &name = "");

      /**
       * \brief destructor
       */
      virtual ~MaxwellContact();

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      virtual void init(InitStage stage, const InitConfigSet &config);
      virtual void initializeContourCouplings();
      virtual void initializeUsingXML(xercesc::DOMElement *element);

      /**
       * \brief add a function that represents the coupling between two contours
       * \param name of first contour
       * \param name of second contour
       * \param Function to describe coupling between contours
       *
       * \General Remark: The parameters (LagrangeParameterPositions) of the function have to be in the same order as it was given the add(...)-method
       */
      void addContourCoupling(Contour *contour1, Contour *contour2, InfluenceFunction *fct);

      void setDampingCoefficient(double d) { dampingCoefficient = d; }

      void setGapLimit(double gLim_) { gLim = gLim_; }

     /**
       * \brief saves all possible contacts in a vector
       */
      virtual void updatePossibleContactPoints();

      /**
       * \brief updates the influence matrix C
       * \param contours vector of contours that are part of the contact
       * \param cpData   vector of ContourPointDatas
       */
      virtual void updateInfluenceMatrix();

      /**
       * \brief update the rigid body distances (gaps) for the single contacts
       */
      void updateRigidBodyGap();

      /**
       * \brief computes the coupling factor for the influence matrix on one contact point (two contours)
       * \param contours     vector of contours that are part of the contact
       * \param cpData       vector of ContourPointDatas
       * \param contactIndex index pair of contact point
       */
      virtual double computeInfluenceCoefficient(const std::pair<int, int> & contactIndex);

      /**
       * \brief computes the coupling factor for the influence matrix between two contact points (four contours)
       * \param contours            vector of contours that are part of the contact
       * \param cpData              vector of ContourPointDatas
       * \param contactIndex        index pair of contact point
       * \param coupledContactIndex index pair of coupling contact point
       */
      virtual double computeInfluenceCoefficient(const std::pair<int, int> & contactIndex, const std::pair<int, int> & couplingContactIndex);

      /*
       * \brief computes the "material constant" to have a good guess for the lambda-vector
       */
      virtual void computeMaterialConstant();

      virtual void updateGeneralizedNormalForce();

    protected:
      /**
       * \brief saves the indices of all active contacts in pairs
       *
       * pair.first: number of contact kinematics
       * pair.second: number of subcontact point of contact kinematics
       */
      std::vector<std::pair<int, int> > possibleContactPoints;

      /*!
       * \brief variable for the LCP
       */
      LinearComplementarityProblem LCP;

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

      /**
       * \brief Solution of the last time, where contact has to be solved (can be used as starting guess for the next algorithm)
       */
      fmatvec::Vec solution0;

      /*!
       * \brief coefficient for possible contact damping
       */
      double dampingCoefficient;

      /*!
       * \brief relative contact point distance limit under which damping is active
       */
      double gLim;

      /**
       * \brief parameter for guessing starting values of contact force (average eigenvalue of influence-matrix)
       */
      double matConst;

      /**
       * \brief parameter to save if matConst has been computed already
       */
      bool matConstSetted;

    private:
      struct xmlInfo {
          InfluenceFunction * function;
          std::string name1;
          std::string name2;
      };
      std::vector<xmlInfo> referenceXML;
 };

}

#endif /* _CONTACT_H_ */

