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

#ifndef CIRCLE_H_
#define CIRCLE_H_

#include "mbsim/contours/rigid_contour.h"

#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  /**
   * \brief circular contour with contact possibility from outside and inside and binormal in direction of the third column of the contour reference frame
   * \author Thorsten Schindler
   * \date 2009-07-13 initial commit (Thorsten Schindler)
   * \date 2009-12-21 adaptations concerning HollowCircle and SolidCircle
   */
  class Circle : public RigidContour {
    public:

      Circle(const std::string& name="", double r_=1., bool solid_=true, Frame *R=0) : RigidContour(name,R), r(r_), solid(solid_) { }

      /*!
       * \brief destructor
       */
      virtual ~Circle() { }

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void init(InitStage stage, const InitConfigSet &config);
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta) { return Kt; }
      virtual fmatvec::Vec3 evalParDer1Kn(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalParDer1Ku(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec2 evalZeta(const fmatvec::Vec3& WrPoint);
      /***************************************************/

      /* GETTER / SETTER */
      void setRadius(double r_) { r = r_; }
      double getRadius() const { return r; }
      double getSign() const { return sign; }
      double getCurvature(const fmatvec::Vec2 &zeta) { return sign/r; }

      void setSolid(bool solid_=true) { solid = solid_; }
      bool getSolid() const { return solid; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVCircle ombv(1,diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }
      
    virtual void initializeUsingXML(xercesc::DOMElement *element);

    protected:
      /** 
       * \brief radius
       */
      double r;

      double sign;

    private:
      /** 
       * \brief contact on outer surface?
       */
      bool solid;

      fmatvec::Vec3 Kt;
  };

}

#endif /* CIRCLE_H_ */

