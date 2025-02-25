/* Copyright (C) 2004-2025 MBSim Development Team
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

#ifndef _ROTATIONALLY_SYMMETRIC_H
#define _ROTATIONALLY_SYMMETRIC_H

#include "mbsim/contours/rigid_contour.h"
#include "mbsim/functions/function.h"
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {

  class Revolution : public RigidContour {
    public:
      Revolution(const std::string& name="", Frame *R=nullptr) : RigidContour(name,R) { }
      ~Revolution() override = default;

      void init(InitStage stage, const InitConfigSet &config) override;

      void setPositionOfReferencePoint(const fmatvec::Vec2 &r0_) { r0 = r0_; }
      void setWidth(double w_) { w = w_; }
      void setProfileFunction(Function<double(double)> *fz_); 

      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Ks(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Ks(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Kt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Kt(const fmatvec::Vec2 &zeta) override;

      bool isZetaOutside(const fmatvec::Vec2 &zeta) override;

//      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
//        OpenMBVColoredBody ombv(diffuseColor,transparency,pointSize,lineWidth);
//        openMBVRigidBody=ombv.createOpenMBV<OpenMBV::Frustum>();
//      }
      
    void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
    fmatvec::Vec2 r0;
      double w{0.5};
      Function<double(double)> *fz{nullptr};
  };

}

#endif
