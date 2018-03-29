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

#ifndef _FUNCPAIR_PLANARCONTOUR_LINE_H_
#define _FUNCPAIR_PLANARCONTOUR_LINE_H_

#include <mbsim/functions/contact/distance_function.h>
#include "mbsim/frames/frame.h"
#include "mbsim/contours/contour.h"
#include "mbsim/contours/line.h"

namespace MBSim {

  class Contour;
  class Line;

  /*!
   * \brief root function for pairing PlanarContour and Line
   * \author Martin Foerg
   */
  class FuncPairPlanarContourLine : public DistanceFunction<double(double)> {
    public:
      /**
       * \brief constructor
       */
      FuncPairPlanarContourLine(Line* line_, Contour *contour_) : contour(contour_), line(line_) { }

      double operator()(const double &alpha) override;

      fmatvec::Vec3 evalWrD(const double &alpha) override;

    private:
      Contour *contour;
      Line *line;

      fmatvec::Vec2 zeta;
  };

}

#endif
