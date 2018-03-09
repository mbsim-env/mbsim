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

#ifndef _FUNCPAIR_SPATIALCONTOUR_PLANE_H_
#define _FUNCPAIR_SPATIALCONTOUR_PLANE_H_

#include <mbsim/functions/contact/distance_function.h>

namespace MBSim {

  class Contour;
  class Plane;

  /*!
   * \brief root function for pairing SpatialContour and Plane
   * \author Zhan Wang
   * \date 2013-12-05
   */
  class FuncPairSpatialContourPlane : public DistanceFunction<fmatvec::Vec(fmatvec::Vec)> {
    public:
      /**
       * \brief constructor
       * \param plane contour
       * \param contour contour2s surface
       */
      FuncPairSpatialContourPlane(Plane* plane_, Contour *contour_) : contour(contour_), plane(plane_) { }

      fmatvec::Vec operator()(const fmatvec::Vec &alpha) override;

      fmatvec::Vec3 evalWrD(const fmatvec::Vec &alpha) override;

    private:
      /**
       * \brief contours
       */
      Contour *contour;
      Plane *plane;
  };

}

#endif
