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

#ifndef _CONTOUR2SFLEXIBLE_H_
#define _CONTOUR2SFLEXIBLE_H_

#include "mbsim/contours/contour_continuum.h"

namespace MBSim {

  /** 
   * \brief numerical description of contours with two contour parameter
   * \author Thorsten Schindler
   * \date 2009-04-21 initial comment (Thorsten Schindler)
   */
  class Contour2sFlexible : public Contour2s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour2sFlexible(const std::string &name) : Contour2s(name) {}

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) { static_cast<FlexibleBody*>(parent)->updateKinematicsForFrame(cp,ff); }
      virtual void updateJacobiansForFrame(ContourPointData &cp) { static_cast<FlexibleBody*>(parent)->updateJacobiansForFrame(cp); }
      /***************************************************/
  };

}

#endif /* _CONTOUR2SFLEXIBLE_H_ */

