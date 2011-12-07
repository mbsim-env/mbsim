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

#ifndef _CUBOID_H_
#define _CUBOID_H_

#include "mbsim/contour.h"
#include "mbsim/contours/compound_contour.h"

namespace MBSim {

  /**
   * \brief Cuboid with 8 vertices, 12 edges and 6 faces
   */
  class Cuboid : public CompoundContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Cuboid(const std::string &name);

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Cuboid"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setLength(double l_) { l = l_; }
      void setHeight(double h_) { h = h_; }
      void setDepth(double d_) { d = d_; }
      /***************************************************/

    private:
      /**
       * \brief length, height and depth of cuboid
       */
      double l,h,d;

      void init(InitStage stage);
  };
}

#endif /* _CUBOID_H_ */

