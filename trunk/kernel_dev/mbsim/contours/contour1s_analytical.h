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

#ifndef _CONTOUR1S_ANALYTICAL_H_
#define _CONTOUR1S_ANALYTICAL_H_

#include "mbsim/contours/contour1s.h"
#include "mbsim/userfunction_contour.h"

namespace MBSim {

  /** 
   * \brief analytical description of contours with one contour parameter
   * \author Robert Huber
   * \date 2009-04-20 some comments (Thorsten Schindler)
   * \date 2009-06-04 new file (Thorsten Schindler)
   */
  class Contour1sAnalytical : public Contour1s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour1sAnalytical(const std::string &name) : Contour1s(name) {}

      /**
       * \brief destructor
       */
      virtual ~Contour1sAnalytical() { if (funcCrPC) delete funcCrPC; }

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Contour1sAnalytical"; }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff);
      /***************************************************/

      /* GETTER / SETTER */
      void setUserFunction(UserFunctionContour1s* f) { funcCrPC = f; }
      UserFunctionContour1s* getUserFunction() { return funcCrPC; }
      /***************************************************/

    protected:
      UserFunctionContour1s  *funcCrPC;
  };

}

#endif /* _CONTOUR1S_ANALYTICAL_H_ */

