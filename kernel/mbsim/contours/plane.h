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

#ifndef _PLANE_H_
#define _PLANE_H_

#include "mbsim/contours/rigid_contour.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#include <openmbvcppinterface/cuboid.h>

namespace MBSim {

  /** 
   * \brief plane without borders
   * \author Martin Foerg
   * \date 2009-03-23 some comments (Thorsten Schindler)
   * \date 2009-10-30 visualization added (Markus Schneider)
   *
   * normal equals first column in orientation matrix (x-axis)
   */
  class Plane : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Plane(const std::string &name="", Frame *R=nullptr) : RigidContour(name,R) { }
      
      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      /* INTERFACE OF CONTOUR */
      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta) override { return ey; }
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta) override { return ez; }
      fmatvec::Vec3 evalKu(const fmatvec::Vec2 &zeta) override { return evalKs(zeta); }
      fmatvec::Vec3 evalKv(const fmatvec::Vec2 &zeta) override { return evalKt(zeta); }
      fmatvec::Vec3 evalKn(const fmatvec::Vec2 &zeta) override { return ex; }
      fmatvec::Vec3 evalParDer1Ku(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer2Ku(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer1Kv(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer2Kv(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer1Kn(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer2Kn(const fmatvec::Vec2 &zeta) override { return zero3; }

      fmatvec::Vec3 evalWs(const fmatvec::Vec2 &zeta) override { return R->evalOrientation().col(1); }
      fmatvec::Vec3 evalWt(const fmatvec::Vec2 &zeta) override { return R->evalOrientation().col(2); }
      fmatvec::Vec3 evalWu(const fmatvec::Vec2 &zeta) override { return evalWs(zeta); }
      fmatvec::Vec3 evalWv(const fmatvec::Vec2 &zeta) override { return evalWt(zeta); }
      fmatvec::Vec3 evalWn(const fmatvec::Vec2 &zeta) override { return R->evalOrientation().col(0); }
      fmatvec::Vec3 evalParDer1Wu(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer2Wu(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer1Wv(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer2Wv(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer1Wn(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer2Wn(const fmatvec::Vec2 &zeta) override { return zero3; }

      fmatvec::Vec2 evalZeta(const fmatvec::Vec3 &WrPoint) override;
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (length,(fmatvec::Vec2),fmatvec::Vec2(fmatvec::INIT,1))(diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        OpenMBVPlane ombv(length,diffuseColor,transparency,pointSize,lineWidth);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }

      void initializeUsingXML(xercesc::DOMElement *element) override;
  };
}

#endif /* _PLANE_H_ */
