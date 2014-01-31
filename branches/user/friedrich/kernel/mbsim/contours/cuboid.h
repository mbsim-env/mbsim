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

#ifndef _CUBOID_H_
#define _CUBOID_H_

#include "mbsim/contour.h"
#include "mbsim/contours/compound_contour.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <mbsim/utils/openmbv_utils.h>
#endif

namespace MBSim {

  /**
   * \brief Cuboid with 8 vertices, 12 edges and 6 faces
   */
  class Cuboid : public CompoundContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Cuboid(const std::string &name="", Frame *R=0);

      Cuboid(const std::string &name, double lx, double ly, double lz, Frame *R=0);

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Cuboid"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setXLength(double lx_) { lx = lx_; }
      void setYLength(double ly_) { ly = ly_; }
      void setZLength(double lz_) { lz = lz_; }
      /***************************************************/

      virtual void plot(double t, double dt = 1);

#ifdef HAVE_OPENMBVCPPINTERFACE
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVCuboid ombv(fmatvec::Vec3(),diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }
#endif

    private:
      /**
       * \brief x-, y- and z-length of cuboid
       */
      double lx,ly,lz;

      void init(InitStage stage);
  };
}

#endif /* _CUBOID_H_ */

