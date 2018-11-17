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

#ifndef DISK_H_
#define DISK_H_

#include "mbsim/contours/rigid_contour.h"

#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  /**
   * \brief disk contour
   * \author Martin Förg
   */
  class Disk : public RigidContour {
    public:

      Disk(const std::string& name="", Frame *R=nullptr) : RigidContour(name,R) { }

      /*!
       * \brief destructor
       */
      ~Disk() override = default;

      /* INHERITED INTERFACE OF ELEMENT */
      void init(InitStage stage, const InitConfigSet &config) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setOuterRadius(double rO_) { rO = rO_; }
      double getOuterRadius() const { return rO; }
      void setInnerRadius(double rI_) { rI = rI_; }
      double getInnerRadius() const { return rI; }
      void setWidth(double w_) { w = w_; }
      double getWidth() const { return w; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        OpenMBVCircle ombv(1,diffuseColor,transparency,pointSize,lineWidth);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }
      
    void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      double rO{1};
      double rI{0};
      double w{0.1};
  };

}

#endif
