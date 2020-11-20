/* Copyright (C) 2004-2015 MBSim Development Team
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

#ifndef _LINE_SEGMENT_H_
#define _LINE_SEGMENT_H_

#include "mbsim/contours/rigid_contour.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#include <openmbvcppinterface/indexedlineset.h>

namespace MBSim {

  /**
   * \brief line segment with two bounds
   * \author Martin Foerg
   * \date 2010-05-20 initial commit (Martin Foerg)
   * \date 2011-01-27 some comments (Thomas Cebulla)
   */ 
  class LineSegment : public MBSim::RigidContour {	
    public:
      /**
       * \brief constructor
       * \param name of line segment
       */
      LineSegment(const std::string& name, double l, double t, Frame *R=nullptr);

      LineSegment(const std::string& name, double l, Frame *R=nullptr);

      LineSegment(const std::string& name="", Frame *R=nullptr);

      /* INHERITED INTERFACE OF ELEMENT */
      void init(InitStage stage, const InitConfigSet &config) override;
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta) override { return ey; }
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalKu(const fmatvec::Vec2 &zeta) override { return evalKs(zeta); }
      fmatvec::Vec3 evalKv(const fmatvec::Vec2 &zeta) override { return evalKt(zeta); }
      fmatvec::Vec3 evalKn(const fmatvec::Vec2 &zeta) override { return ex; }
      fmatvec::Vec3 evalParDer1Ku(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer1Kv(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer1Kn(const fmatvec::Vec2 &zeta) override { return zero3; }

      fmatvec::Vec3 evalWs(const fmatvec::Vec2 &zeta) override { return R->evalOrientation().col(1); }
      fmatvec::Vec3 evalWt(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalWu(const fmatvec::Vec2 &zeta) override { return evalWs(zeta); }
      fmatvec::Vec3 evalWv(const fmatvec::Vec2 &zeta) override { return evalWt(zeta); }
      fmatvec::Vec3 evalWn(const fmatvec::Vec2 &zeta) override { return R->evalOrientation().col(0); }
      fmatvec::Vec3 evalParDer1Wu(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer1Wv(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer1Wn(const fmatvec::Vec2 &zeta) override { return zero3; }
      /***************************************************/

      virtual double getCurvature(const fmatvec::Vec2 &zeta) { return 0; }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        OpenMBVLine ombv(1,diffuseColor,transparency,pointSize,lineWidth);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }

      void initializeUsingXML(xercesc::DOMElement *element) override;

      /* GETTER / SETTER */
      void setLength(double length_) {length = length_; }
      double getLength() const {return length; }
      /***************************************************/

    private:
      /**
       * \brief length of line segment
       */
      double length{1};

  };      
}

#endif /* _LINE_SEGMENT_H_ */
