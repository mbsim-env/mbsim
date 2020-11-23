/* Copyright (C) 2004-2020 MBSim Development Team
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

#ifndef _FUNCPAIR_SPATIALCONTOUR_CIRCLE_H_
#define _FUNCPAIR_SPATIALCONTOUR_CIRCLE_H_

#include <mbsim/functions/function.h>

namespace MBSim {

  class Contour;
  class Circle;

  /*!
   * \brief root function for pairing SpatialContour and Circle
   */
  class FuncPairSpatialContourCircle : public Function<fmatvec::Vec(fmatvec::Vec)> {
    public:
      /**
       * \brief constructor
       * \param circle contour
       * \param contour contour2s surface
       */
      FuncPairSpatialContourCircle(Circle* circle_, Contour *contour_) : contour(contour_), circle(circle_) { }

      fmatvec::Vec operator()(const fmatvec::Vec &alpha) override;

    private:
      /**
       * \brief contours
       */
      Contour *contour;
      Circle *circle;
  };

}

#endif
