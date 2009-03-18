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
 */

#ifndef _CONTACT_KINEMATICS_H_
#define _CONTACT_KINEMATICS_H_

#include "fmatvec.h"
#include <vector>

using namespace fmatvec;

namespace MBSim {

  class CoordinateSystem;
  class Contour;
  class ContourPointData;

  /**
   * \return perpendicular vector
   * \param input vector 
   */	
  Vec computeTangential(const Vec &n);

  /** 
   * \brief basic class for contact kinematical calculations
   * \author Martin Foerg
   * \date 18.03.09
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

      /** 
       * \brief treats ordering of contours 
       * \param contour vector
       */
      virtual void assignContours(const vector<Contour*> &contour) = 0;

      /** 
       * \brief treats ordering of contours
       * \param first contour
       * \param second contour
       */
      void assignContours(Contour *contour1, Contour *contour2) { vector<Contour*> c; c.push_back(contour1);c.push_back(contour2); assignContours(c); }

      /**
       * \brief compute normal distance, possible contact point positions and orientation
       * \param normal distance
       * \param contact point parametrisation
       */
      virtual void updateg(Vec &g, ContourPointData *cpData) { cout << "ERROR (ContactKinematics::updateg): not implemented" << endl; throw 5; }

      /** 
       * \brief compute normal and tangential relative velocities, velocity and angular velocity of possible contact point if necessary (cf. contact.cc)
       * \param normal distance
       * \param relative velocity vector (normal and tangential)
       * \param contact point parametrisation
       */
      virtual void updategd(const Vec& g, Vec &gd, ContourPointData* cpData) { cout << "ERROR (ContactKinematics::updategd): not implemented" << endl; throw 5; }

      /**
       * \brief compute acceleration in terms of contour parameters for event driven integration
       * \param acceleration in terms of contour parameters
       * \param normal distance
       * \param contact point parametrisation
       */
      virtual void updatewb(Vec &wb, const Vec &g, ContourPointData* cpData) { cout << "ERROR (ContactKinematics::updatewb): not implemented" << endl; throw 5;};

      /**
       * \brief compute normal distance, possible contact point positions and orientation for several possible contact points
       * \param normal distance
       * \param contact point parametrisation
       */
      virtual void updateg(vector<Vec> &g, vector<ContourPointData*> &cpData) { updateg(g[0],cpData[0]); }

      /** 
       * \brief compute normal and tangential relative velocities, velocity and angular velocity of possible contact point if necessary for several possible contact points (cf. contact.cc)
       * \param normal distance
       * \param relative velocity vector (normal and tangential)
       * \param contact point parametrisation
       */
      virtual void updategd(vector<Vec> &g, vector<Vec> &gd, vector<ContourPointData*> &cpData) { updategd(g[0],gd[0],cpData[0]); }

      /**
       * \brief compute acceleration in terms of contour parameters for event driven integration and several contact points
       * \param acceleration in terms of contour parameters
       * \param normal distance
       * \param contact point parametrisation
       */
      virtual void updatewb(vector<Vec> &wb, vector<Vec> &g, vector<ContourPointData*> &cpData) { updatewb(wb[0],g[0],cpData[0]); }

      /**
       * \return number of potential contact points
       */
      int getNumberOfPotentialContactPoints() const { return numberOfPotentialContactPoints; }

    protected:
      /**
       * \brief number of potential contact points
       */
      int numberOfPotentialContactPoints;
  };

}

#endif

