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

#ifndef _FUNCPAIR_SPATIALCONTOUR_SPATIALCONTOUR_H_
#define _FUNCPAIR_SPATIALCONTOUR_SPATIALCONTOUR_H_

#include <mbsim/functions/function.h>

namespace MBSim {

  class Contour;

  /*!
   * \brief root function for pairing spatial contour and spatial contour
   */
  class FuncPairSpatialContourSpatialContour : public Function<fmatvec::Vec(fmatvec::Vec)> {
    public:
      FuncPairSpatialContourSpatialContour(Contour *contour1_, Contour *contour2_) : contour1(contour1_), contour2(contour2_), I1(0,1), I2(2,3) { }

      fmatvec::Vec operator()(const fmatvec::Vec &zeta) override;

    private:
      Contour *contour1;
      Contour *contour2;
      fmatvec::RangeV I1;
      fmatvec::RangeV I2;
  };

}

#endif
