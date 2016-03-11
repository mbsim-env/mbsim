/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef MBSIMFLEXIBLEBODY_FUNCTIONS_CONTACT_H_
#define MBSIMFLEXIBLEBODY_FUNCTIONS_CONTACT_H_

#include <mbsim/functions/contact/distance_function.h>

namespace MBSim {
  class Circle;
}

namespace MBSimFlexibleBody {

  class NurbsDisk2s;

  /*!
   * \brief root function for pairing Circle and NurbsDisk2s
   * \author Kilian Grundl
   * \date 2009-10-06 initial commit (Thorsten Schindler)
   */
  class FuncPairCircleNurbsDisk2s : public MBSim::DistanceFunction<double(double)> {
    public:
      /**
       * \brief constructor
       * \param circle
       * \param nurbsdisk
       */
      FuncPairCircleNurbsDisk2s(MBSim::Circle* circle_, NurbsDisk2s* nurbsdisk_) : nurbsdisk(nurbsdisk_), circle(circle_) { }

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &alpha);

      fmatvec::Vec3 getWrD(const double &alpha);
      /***************************************************/

    private:
      /**
       * \brief contours
       */
      NurbsDisk2s *nurbsdisk;
      MBSim::Circle *circle;
  };

}

#endif /* MBSIMFLEXIBLEBODY_FUNCTIONS_CONTACT_H_ */
