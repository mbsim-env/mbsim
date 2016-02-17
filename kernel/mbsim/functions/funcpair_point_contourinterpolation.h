/* Copyright (C) 2004-2010 MBSim Development Team
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

#ifndef _FUNCPAIR_POINT_CONTOURINTERPOLATION_H_
#define _FUNCPAIR_POINT_CONTOURINTERPOLATION_H_

#include <mbsim/functions/distance_function.h>

namespace MBSim {

  class Point;
  class ContourInterpolation;

  /*!
   * \brief root function for pairing ContourInterpolation and Point
   * \author Roland Zander
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class FuncPairPointContourInterpolation : public DistanceFunction<fmatvec::Vec2(fmatvec::Vec2)> {
    public:
      /**
       * \brief constructor
       * \param point contour
       * \param contour based on interpolation
       */
      FuncPairPointContourInterpolation(Point* point_, ContourInterpolation *contour_) : contour(contour_), point(point_) { }

      fmatvec::Vec2 operator()(const fmatvec::Vec2 &alpha);

      fmatvec::Vec3 getWrD(const fmatvec::Vec2 &alpha);

    private:
      /**
       * \brief contours
       */
      ContourInterpolation *contour;
      Point *point;
  };

}

#endif
