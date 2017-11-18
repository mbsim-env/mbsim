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

#ifndef _POINT_H_
#define _POINT_H_

#include "mbsim/contours/rigid_contour.h"

#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  /**
   * \brief most primitive contour: the point (no extention)
   * \author Martin Foerg
   * \date 2009-03-19 comments (Thorsten Schindler)
   */
  class Point : public MBSim::RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of point
       */
      Point(const std::string& name="", Frame *R=nullptr) : RigidContour(name,R) {}

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      fmatvec::Vec3 evalParDer1Ku(const fmatvec::Vec2 &zeta) override;

      fmatvec::Vec3 evalParDer2Ku(const fmatvec::Vec2 &zeta) override;

      fmatvec::Vec3 evalParDer1Kv(const fmatvec::Vec2 &zeta) override;

      fmatvec::Vec3 evalParDer2Kv(const fmatvec::Vec2 &zeta) override;

      fmatvec::Vec3 evalParDer1Wn(const fmatvec::Vec2 &zeta) override;

      fmatvec::Vec3 evalParDer2Wn(const fmatvec::Vec2 &zeta) override;

      /* INHERITED INTERFACE OF CONTOUR */
      fmatvec::Vec2 evalZeta(const fmatvec::Vec3 &WrPS) override { return fmatvec::Vec2(fmatvec::INIT,0.); }
      /**********************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (size,(double),0.001)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVSphere ombv(size,diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }
      
      void initializeUsingXML(xercesc::DOMElement *element) override;
  };

}

#endif /* _POINT_H_ */
