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

#include "fmatvec/fmatvec.h"
#include "fmatvec/atom.h"
#include <vector>

namespace MBSim {

  class ContourFrame;
  class Contour;

  /**
   * \return perpendicular vector
   * \param input vector 
   */	
  fmatvec::Vec3 computeTangential(const fmatvec::Vec3 &n);

  /** 
   * \brief basic class for contact kinematical calculations
   * \author Martin Foerg
   * \date 2009-03-18 some comments (Thorsten Schindler)
   * \date 2009-04-02 velocity part deleted (Thorsten Schindler)
   * \date 2009-07-28 updates are pure virtual (Thorsten Schindler)
   */
  class ContactKinematics : virtual public fmatvec::Atom {
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
       * \param t      time
       * \param g      normal distance
       * \param cFrame contour point Frame
       * \param index  index of the contact point that should be updated
       */
      virtual void updateg(double t, double &g, std::vector<ContourFrame*> &cFrame, int index = 0) = 0;

      /**
       * \brief compute acceleration in terms of contour parameters for event driven integration
       * \param t      time
       * \param wb acceleration in terms of contour parameters
       * \param g normal distance
       * \param cFrame contact point parametrisation
       */
      virtual void updatewb(double t, fmatvec::Vec &wb, double g, std::vector<ContourFrame*> &cFrame) = 0;
      
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

      virtual ContactKinematics* getContactKinematics(int i=0) const { return 0; }

      virtual void setSearchAllContactPoints(bool searchAllCP_=true) { }

    protected:
      /**
       * \brief number of potential contact points
       */
      int numberOfPotentialContactPoints;
  };

}

#endif /* _CONTACT_KINEMATICS_H_ */

