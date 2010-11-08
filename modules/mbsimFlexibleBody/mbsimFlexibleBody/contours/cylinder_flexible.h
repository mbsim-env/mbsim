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
 * Contact: thschindler@users.berlios.de
 */

#ifndef _CYLINDER_FLEXIBLE_H_
#define _CYLINDER_FLEXIBLE_H_

#include "mbsimFlexibleBody/contours/contour1s_flexible.h"

namespace MBSimFlexibleBody {

  /** 
   * \brief flexible cylinder for one dimensional flexible bodies
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-04-20 frame concept (Thorsten Schindler)
   * \date 2009-06-04 new file (Thorsten Schindler)
   */
  class CylinderFlexible : public Contour1sFlexible {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      CylinderFlexible(const std::string &name) : Contour1sFlexible(name) {}

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "CylinderFlexible"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setRadius(double r_) { r = r_; }
      double getRadius() const  { return r; }
      /***************************************************/

    protected:
      /**
       * \brief radius
       */
      double r;
  };

}

#endif /* _CYLINDER_FLEXIBLE_H_ */

