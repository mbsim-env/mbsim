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

#ifndef _CONTACT_KINEMATICS_COMPOUND_CONTOUR_H_
#define _CONTACT_KINEMATICS_COMPOUND_CONTOUR_H_

#include "contact_kinematics.h"

namespace MBSim {

  class CompoundContour;
  class Contour;

  /**
   * \brief pairing of arbitrary contour and set of contours
   * \author Martin Foerg
   * \date 2009-07-28 pure virtual updates (Thorsten Schindler)
   */
  class ContactKinematicsCompoundContourContour : public ContactKinematics {
    public:
      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec &g, ContourPointData *cpData) { throw new MBSimError("ERROR (ContactKinematicsCompoundContourContour::updateg): Not implemented!"); }
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, ContourPointData* cpData) { throw new MBSimError("ERROR (ContactKinematicsCompoundContourContour::updatewb): Not implemented!"); };
      virtual void updateg(std::vector<fmatvec::Vec> &g, std::vector<ContourPointData*> &cpData);
      virtual void updatewb(std::vector<fmatvec::Vec> &wb, std::vector<fmatvec::Vec> &g, std::vector<ContourPointData*> &cpData);
      /***************************************************/

    private:
      /**
       * \brief contour index
       */
      int icompound, icontour;
      
      /**
       * \brief contour classes
       */
      CompoundContour *compound;
      Contour *contour;
      
      /**
       * \brief vector of involved contact kinematics
       */
      std::vector<ContactKinematics*> contactKinematics;
  };

}

#endif /* _CONTACT_KINEMATICS_COMPOUND_CONTOUR_H_ */

