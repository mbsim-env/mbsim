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

#ifndef _CONTOUR_QUAD_H_
#define _CONTOUR_QUAD_H_

#include "mbsim/contour.h"
#include "mbsim/contours/contour_interpolation.h"

namespace MBSim {

  /**
   * \brief Quad for 3D interpolation 
   * \see{OpenGL-documentation}
   */ 
  class ContourQuad : public MBSim::ContourInterpolation {	
    public:
      /**
       * \brief constructor
       * \param name of point
       */
      ContourQuad(const std::string & name);

      virtual void init();

      bool testInsideBounds(const ContourPointData &cp);
      double computePointWeight(const fmatvec::Vec &s, int i);
      double computePointWeight(const fmatvec::Vec &s, int i, int diff);

      fmatvec::Vec computeWn(const ContourPointData &cp);
  };
}

#endif /* _CONTOUR_QUAD_H_ */

