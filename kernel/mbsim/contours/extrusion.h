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

#ifndef _EXTRUSION_H
#define _EXTRUSION_H

#include "mbsim/contours/rigid_contour.h"
#include "mbsim/functions/function.h"
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {

  class Extrusion : public RigidContour {
    public:
      Extrusion(const std::string& name="", Frame *R=nullptr);
      ~Extrusion() override = default;

      void init(InitStage stage, const InitConfigSet &config) override;

      void setPositionFunction(Function<fmatvec::Vec3(double)> *fr_); 
      void setOrientationFunction(Function<fmatvec::RotMat3(double)> *fA_); 
      void setProfileFunction(Function<fmatvec::Vec2(double)> *fyz_); 

      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Ks(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Ks(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Kt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Kt(const fmatvec::Vec2 &zeta) override;

      bool isZetaOutside(const fmatvec::Vec2 &zeta) override;

      void setOpenEta(bool openEta_) { openEta = openEta_; }
      void setOpenXi(bool openXi_) { openXi = openXi_; }

//      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
//        OpenMBVColoredBody ombv(diffuseColor,transparency,pointSize,lineWidth);
//        openMBVRigidBody=ombv.createOpenMBV<OpenMBV::Frustum>();
//      }
      
      void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      fmatvec::Vec3 rOQ, rQP, rOQp, rQPp, rOQpp, rQPpp;
      fmatvec::RotMat3 A;
      fmatvec::Mat3x3 Ap, App;
      Function<fmatvec::Vec3(double)> *fr{nullptr};
      Function<fmatvec::RotMat3(double)> *fA{nullptr};
      Function<fmatvec::Vec2(double)> *fyz{nullptr};
      bool openEta{true};
      bool openXi{false};
      std::shared_ptr<OpenMBVSpatialContour> ombv;
  };

}

#endif
