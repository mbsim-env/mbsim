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

#include <config.h>
#include "mbsim/functions/contact/funcpair_planarcontour_line.h"

using namespace fmatvec;

namespace MBSim {

  double FuncPairPlanarContourLine::operator()(const double &alpha) {
    zeta(0) = alpha;
    Vec3 Wu = contour->evalWu(zeta);
    return Wu.T() * line->getFrame()->evalOrientation().col(0); // projection of the lines normal vector into the first tangent direction: scalar value
  }

  Vec3 FuncPairPlanarContourLine::evalWrD(const double &alpha) {
    zeta(0) = alpha;
    Vec3 Wn = line->getFrame()->evalOrientation().col(0);
    double g = Wn.T()*(contour->evalPosition(zeta) - line->getFrame()->getPosition());
    return Wn*g;
  }

}
