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

#ifndef _FCL_PLANE_H_
#define _FCL_PLANE_H_

#include "mbsim/contours/fcl_contour.h"
#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  /**
   * \brief FCLPlane
   */
  class FCLPlane : public FCLContour {
    public:
      /**
       * \brief constructor
       * \param name of plane
       * \param R frame of reference
       */
      FCLPlane(const std::string &name="", Frame *R=nullptr) : FCLContour(name,R) { normal(0) = 1; }

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/
      void init(InitStage stage, const InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setNormal(const fmatvec::Vec3 &normal_) { normal = normal_; }
      void setOffset(double offset_) { offset = offset_; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (length,(fmatvec::Vec2),fmatvec::Vec2(fmatvec::INIT,1))(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {       
        OpenMBVPlane ombv(length,diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }

    private:
      /**
       * \brief normal
       */
      fmatvec::Vec3 normal;

      /**
       * \brief offset
       */
      double offset;
  };
}

#endif
