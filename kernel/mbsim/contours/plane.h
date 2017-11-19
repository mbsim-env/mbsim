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

#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>

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
     /**
       * \return first tangent in world frame
       * \param t time
       * \param zeta contour position
       */
      fmatvec::Vec3 evalWu(const fmatvec::Vec2 &zeta) override;

      /**
       * \return second tangent in world frame
       * \param t time
       * \param zeta contour position
       */
      fmatvec::Vec3 evalWv(const fmatvec::Vec2 &zeta) override;

      /**
       * \return second tangent in world frame
       * \param t time
       * \param zeta contour position
       */
      fmatvec::Vec3 evalWn(const fmatvec::Vec2 &zeta) override;

     /**
       * \return first tangent in world frame
       * \param t time
       * \param zeta contour position
       */
      fmatvec::Vec3 evalWs(const fmatvec::Vec2 &zeta) override;

      /**
       * \return second tangent in world frame
       * \param t time
       * \param zeta contour position
       */
      fmatvec::Vec3 evalWt(const fmatvec::Vec2 &zeta) override;

      fmatvec::Vec2 evalZeta(const fmatvec::Vec3 &WrPoint) override;
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (length,(fmatvec::Vec2),fmatvec::Vec2(fmatvec::INIT,1))(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {       
        OpenMBVPlane ombv(length,diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }

      void initializeUsingXML(xercesc::DOMElement *element) override;

  };
}

#endif /* _PLANE_H_ */
