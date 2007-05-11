/* Copyright (C) 2007  Martin Förg, Roland Zander
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *   rzander@users.berlios.de
 *
 */

#ifndef _CONTACT_KINEMATICS_H_
#define _CONTACT_KINEMATICS_H_

#include "fmatvec.h"
#include <vector>


using namespace fmatvec;

namespace MBSim {

  class ContourPointData;
  class Contour;

  Vec computeTangential(const Vec &n);

  /*! \brief Basis class for contact kinematical calculations
   *
   *  the generalised position to the contact point including normal and tangential directions need to be up-to-date latest at the end of stage2();
   *
   *  per default non data is stored or managed within this class
   */
  class ContactKinematics {
    public:
      virtual ~ContactKinematics() {}

      /*! compute \f$\vg_N\f$
       * \param contour  vector of Contour holding both contours
       * \param i1       index of first contour within all data-vectors
       * \param i2       index of second contour within all data-vectors
       * \param g        normal distance
       * \param cpData   vector of generalised position vectors(ContourPointData) for both contours
       */
      virtual void stage1(Vec &g, vector<ContourPointData> &cpData) = 0;
      /*! compute \f$\dot{\vg}\f$, generalised position vectors must be up-to-date at end of method
       * \param contour  vector of Contour holding both contours
       * \param i1       index of first contour within all data-vectors
       * \param i2       index of second contour within all data-vectors
       * \param g        normal distance
       * \param gd       contact velocities
       * \param cpData   vector of generalised position vectors(ContourPointData) for both contours
       */
      virtual void stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) = 0;


      virtual void assignContours(const vector<Contour*> &contour) = 0;
      void assignContours(Contour *contour1, Contour *contour2) { vector<Contour*> c; c.push_back(contour1);c.push_back(contour2); assignContours(c);}
    protected:

  };

}

#endif
