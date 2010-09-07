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

#ifndef _CONTACT_KINEMATICS_LINE_CONTOUR1S_H_
#define _CONTACT_KINEMATICS_LINE_CONTOUR1S_H_

#include "contact_kinematics.h"

namespace MBSim {

  class Line;
  class Contour1s;
  class FuncPairContour1sLine;

  /**
   * \brief pairing Line to Contour1s
   * \author Felix Kahr
   * \date 2009-07-28 pure virtual updates (Thorsten Schindler)
   * \todo change stage to new interface TODO
   */
  class ContactKinematicsLineContour1s : public ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsLineContour1s() {}

      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsLineContour1s();
      
      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec &g, ContourPointData *cpData);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, ContourPointData* cpData) { throw MBSimError("ERROR (ContactKinematicsLineContour1s::updatewb): Not implemented!"); };
      /***************************************************/

    private:
      /**
       * \brief contour index
       */
      int iline, icontour;

      /**
       * \brief contour classes
       */
      Line *line;
      Contour1s *contour1s;

      /**
       * \brief root function
       */
      FuncPairContour1sLine *func;
  };

}

#endif /* _CONTACT_KINEMATICS_LINE_CONTOUR1S_H_ */

