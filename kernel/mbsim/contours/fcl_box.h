/* Copyright (C) 2004-2018 MBSim Development Team
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

#ifndef _FCL_BOX_H_
#define _FCL_BOX_H_

#include "mbsim/contours/rigid_contour.h"
#include "fcl/geometry/shape/box.h"

#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  /**
   * \brief FCLBox
   */
  class FCLBox : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      FCLBox(const std::string &name="", Frame *R=nullptr) : RigidContour(name,R) { }

      FCLBox(const std::string &name, double lx_, double ly_, double lz_, Frame *R=nullptr);

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      void initializeUsingXML(xercesc::DOMElement *element) override;

      /* GETTER / SETTER */
      void setXLength(double lx_) { lx = lx_; }
      void setYLength(double ly_) { ly = ly_; }
      void setZLength(double lz_) { lz = lz_; }
      void setLength(const fmatvec::Vec3 &length) { lx = length(0); ly = length(1); lz = length(2); }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVCuboid ombv(fmatvec::Vec3(),diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }

      std::shared_ptr<fcl::CollisionGeometry<double> > getCollisionGeometry() const { return cg; }

    private:
      /**
       * \brief x-, y- and z-length of cuboid
       */
      double lx{1.0};
      double ly{1.0};
      double lz{1.0};

      std::shared_ptr<fcl::CollisionGeometry<double> > cg;

      void init(InitStage stage, const InitConfigSet &config) override;
  };
}

#endif
