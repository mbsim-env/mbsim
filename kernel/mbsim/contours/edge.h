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

#ifndef _EDGE_H_
#define _EDGE_H_

#include "mbsim/contours/rigid_contour.h"

#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  /**
   * \brief RigidContour Edge
   * \author Martin Foerg
   * \date 2009-07-14 some comments (Bastian Esefeld)
   * \todo adapt to new interface TODO
   */
  class Edge : public MBSim::RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Edge(const std::string &name="", Frame *R=nullptr) : RigidContour(name,R) { }

      void init(InitStage stage, const InitConfigSet &config) override;

      /* GETTER / SETTER */
      void setLength(double length_) {length = length_;}
      double getLength() const { return length; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVLine ombv(1,diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }

    private:
      double length{1};
  };
}

#endif /* _EDGE_H_ */
