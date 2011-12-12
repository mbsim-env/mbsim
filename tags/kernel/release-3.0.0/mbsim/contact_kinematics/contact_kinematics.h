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
 */

#ifndef _CONTACT_KINEMATICS_H_
#define _CONTACT_KINEMATICS_H_

#include "fmatvec.h"
#include "mbsim/mbsim_event.h"
#include <vector>

namespace MBSim {

  class Contour;
  class ContourPointData;

  /**
   * \return perpendicular vector
   * \param input vector 
   */	
  fmatvec::Vec computeTangential(const fmatvec::Vec &n);

  /** 
   * \brief basic class for contact kinematical calculations
   * \author Martin Foerg
   * \date 2009-03-18 some comments (Thorsten Schindler)
   * \date 2009-04-02 velocity part deleted (Thorsten Schindler)
   * \date 2009-07-28 updates are pure virtual (Thorsten Schindler)
   */
  class ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematics() : numberOfPotentialContactPoints(1) {}

      /**
       * \brief destructor
       */
      virtual ~ContactKinematics() {}

      /* INTERFACE FOR DERIVED CLASSES */
      /** 
       * \brief treats ordering of contours 
       * \param contour vector
       */
      virtual void assignContours(const std::vector<Contour*> &contour) = 0;

      /**
       * \brief compute normal distance, possible contact point positions and orientation (cf. contact.cc)
       * \param normal distance
       * \param contact point parametrisation
       */
      virtual void updateg(fmatvec::Vec &g, ContourPointData *cpData) = 0;

      /**
       * \brief compute acceleration in terms of contour parameters for event driven integration
       * \param acceleration in terms of contour parameters
       * \param normal distance
       * \param contact point parametrisation
       */
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, ContourPointData* cpData) = 0;
      
      /**
       * \brief compute normal distance, possible contact point positions and orientation for several possible contact points
       * \param normal distance
       * \param contact point parametrisation
       */
      virtual void updateg(std::vector<fmatvec::Vec>::iterator ig, std::vector<ContourPointData*>::iterator icpData) { if((*ig).size()) updateg(*ig,*icpData); }
      
      /**
       * \brief compute acceleration in terms of contour parameters for event driven integration and several contact points
       * \param acceleration in terms of contour parameters
       * \param normal distance
       * \param contact point parametrisation
       */
      virtual void updatewb(std::vector<fmatvec::Vec>::iterator iwb, std::vector<fmatvec::Vec>::iterator ig, std::vector<ContourPointData*>::iterator icpData) { if((*iwb).size()) updatewb(*iwb,*ig,*icpData); }
      /*******************************************************/ 

      /** 
       * \brief treats ordering of contours
       * \param first contour
       * \param second contour
       */
      void assignContours(Contour *contour1, Contour *contour2) { std::vector<Contour*> c; c.push_back(contour1);c.push_back(contour2); assignContours(c); }

      /**
       * \return number of potential contact points
       */
      int getNumberOfPotentialContactPoints() const { return numberOfPotentialContactPoints; }

      /**
       * \return compute the radii in contact points
       */
//      virtual void computeCurvatures(std::vector<fmatvec::Vec>::iterator ir, std::vector<ContourPointData*>::iterator icpData) { if((*ir).size()) computeRadii(*ir, *icpData); }
      virtual void computeCurvatures(fmatvec::Vec &r, ContourPointData *cpData) {throw MBSimError("ERROR (ContactKinematics::computeCurvatures): Not implemented.");}

    protected:
      /**
       * \brief number of potential contact points
       */
      int numberOfPotentialContactPoints;
  };

}

#endif /* _CONTACT_KINEMATICS_H_ */

