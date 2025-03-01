/* Copyright (C) 2004-2022 MBSim Development Team
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

#ifndef _TYRE_H
#define _TYRE_H

#include "mbsim/contours/rigid_contour.h"
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {

  class Tyre : public RigidContour {
    public:
      enum ShapeOfCrossSectionContour {
        flat=0,
        circular,
	elliptical,
        unknown
      };

      Tyre(const std::string& name="", Frame *R=nullptr) : RigidContour(name,R) { }
      ~Tyre() override = default;

      void init(InitStage stage, const InitConfigSet &config) override;

      void setRadius(double r_) { r = r_; }
      void setWidth(double w_) { w = w_; }
      void setContourParameters(const fmatvec::VecV &cp_) { cp <<= cp_; }
      void setShapeOfCrossSectionContour(ShapeOfCrossSectionContour shape_) { shape = shape_; }
      double getRadius() const { return r; }
      double getWidth() const { return w; }
      const fmatvec::VecV& getContourParameters() const { return cp; }
      ShapeOfCrossSectionContour getShapeOfCrossSectionContour() const { return shape; }

//      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
//        OpenMBVColoredBody ombv(diffuseColor,transparency,pointSize,lineWidth);
//        openMBVRigidBody=ombv.createOpenMBV<OpenMBV::Frustum>();
//      }
      
      void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      double r{0.3};
      double w{-1};
      ShapeOfCrossSectionContour shape{flat};
      fmatvec::VecV cp;
  };

}

#endif
