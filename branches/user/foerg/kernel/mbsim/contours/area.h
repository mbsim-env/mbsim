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

#include "mbsim/contours/plane.h"
#include "fmatvec.h"



namespace MBSim {

  /**
   *  \brief RigidContour Area
   *  \date 2009-07-14 some comments (Bastian Esefeld)
   *  \date 2009-07-16 new file (Bastian Esefeld)
   *  \todo adapt to new interface TODO
   */ 
  class Area : public Plane {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Area(const std::string &name);

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Area"; }
      /**********************************/

      /* GETTER / SETTER */
      void setLimitY(double l) {limy = l;}
      void setLimitZ(double l) {limz = l;}
      double getLimitY() const { return limy; }
      double getLimitZ() const { return limz; }

      /*!
       * \brief projection from a point to center of the area
       */
      double prj_Area_Point(fmatvec::Vec3& Point);

      /*!
       * \brief check if the area has contact with the sphere
       *
       */
      /***************************************************/

#ifdef HAVE_OPENMBVCPPINTERFACE
  virtual void enableOpenMBV(bool enable = true, int number = 10);
#endif

    protected:
      double limy, limz;
  };      
}

#endif /* _AREA_H_ */
