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

#ifndef _FCL_SPHERE_H_
#define _FCL_SPHERE_H_

#include "mbsim/contours/fcl_contour.h"
#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  /**
   * \brief FCLSphere
   */
  class FCLSphere : public FCLContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      FCLSphere(const std::string &name="", double r_=1, Frame *R=nullptr) : FCLContour(name,R), r(r_) { }

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      void init(InitStage stage, const InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setRadius(double r_) { r = r_; }
      double getRadius() const { return r; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVSphere ombv(1,diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }

    private:
      /** 
       * \brief radius
       */
      double r{1};
  };
}

#endif
