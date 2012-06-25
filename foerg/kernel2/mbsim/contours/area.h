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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _AREA_H_
#define _AREA_H_

#include "mbsim/contour.h"

namespace MBSim {

  /**
   *  \brief RigidContour Area
   *  \date 2009-07-14 some comments (Bastian Esefeld)
   *  \date 2009-07-16 new file (Bastian Esefeld)
   *  \todo adapt to new interface TODO
   */ 
  class Area : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Area(const std::string &name);

      /* GETTER / SETTER */
      void setLimit1(double l) {lim1 = l;}
      void setLimit2(double l) {lim2 = l;}
      void setCd1(const fmatvec::FVec& Cd);
      void setCd2(const fmatvec::FVec& Cd);
      virtual void init(InitStage stage);
      double getLimit1() const { return lim1; }
      double getLimit2() const { return lim2; }
      /***************************************************/

      fmatvec::FVec computeWn() { return R.getOrientation()*Cn; }
      fmatvec::FVec computeWd1() { return R.getOrientation()*Cd1; }
      fmatvec::FVec computeWd2() { return R.getOrientation()*Cd2; }

    private:
      double lim1, lim2;
      fmatvec::FVec Cn, Cd1, Cd2;
  };      
}

#endif /* _AREA_H_ */
