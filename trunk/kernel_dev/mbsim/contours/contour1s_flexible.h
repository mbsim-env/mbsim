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

#ifndef _CONTOUR1S_FLEXIBLE_H_
#define _CONTOUR1S_FLEXIBLE_H_

#include "mbsim/contours/contour1s.h"
#include "mbsim/flexible_body.h"

namespace MBSim {

  /** 
   * \brief numerical description of contours with one contour parameter
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-03-18 initial comment (Thorsten Schindler)
   * \date 2009-04-05 adapted to non-template FlexibleBody (Schindler / Zander)
   * \date 2009-06-04 new file (Thorsten Schindler)
   */
  class Contour1sFlexible : public Contour1s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour1sFlexible(const std::string &name) : Contour1s(name) {}

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) { static_cast<FlexibleBody*>(parent)->updateKinematicsForFrame(cp,ff); }
      virtual void updateJacobiansForFrame(ContourPointData &cp) { static_cast<FlexibleBody*>(parent)->updateJacobiansForFrame(cp); }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOURCONTINUUM */
      virtual void computeRootFunctionPosition(ContourPointData &cp) { updateKinematicsForFrame(cp,position); }
      virtual void computeRootFunctionFirstTangent(ContourPointData &cp) { updateKinematicsForFrame(cp,firstTangent); }
      virtual void computeRootFunctionNormal(ContourPointData &cp) { updateKinematicsForFrame(cp,normal); }
      virtual void computeRootFunctionSecondTangent(ContourPointData &cp) { updateKinematicsForFrame(cp,secondTangent); }
      /***************************************************/
  };

}

#endif /* _CONTOUR1S_FLEXIBLE_H_ */

