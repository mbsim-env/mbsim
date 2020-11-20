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

#ifndef SPHERE_H_
#define SPHERE_H_

#include "mbsim/contours/rigid_contour.h"
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#include <openmbvcppinterface/sphere.h>

namespace MBSim {

  /**
   * \brief sphere 
   * \author Martin Foerg
   */
  class Sphere : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of sphere
       * \param radius of sphere
       * \param R frame of reference
       */
      Sphere(const std::string &name="", double r_=1, Frame *R=nullptr) : RigidContour(name,R), r(r_) { }
      
      /* INHERITED INTERFACE OF ELEMENT */
      void init(InitStage stage, const InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKu(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKv(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKn(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Ku(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Ku(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer1Kv(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Kv(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Kn(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Kn(const fmatvec::Vec2 &zeta) override;

      fmatvec::Vec3 evalParDer2Wu(const fmatvec::Vec2 &zeta) override { return zero3; }

      fmatvec::Vec2 evalZeta(const fmatvec::Vec3 &WrPoint) override;
      /**********************************/

      /* GETTER / SETTER */
      void setRadius(double r_) { r = r_; }
      double getRadius() const { return r; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        OpenMBVColoredBody ombv(diffuseColor,transparency,pointSize,lineWidth);
        openMBVRigidBody=ombv.createOpenMBV<OpenMBV::Sphere>();
      }

    protected:
      /** 
       * \brief radius
       */
      double r{1};
  };

}

#endif /* SPHERE_H_ */
