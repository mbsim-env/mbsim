/* Copyright (C) 2004-2016 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _DUAL_RIGID_BODY_LINK_H_
#define _DUAL_RIGID_BODY_LINK_H_

#include "mbsim/links/rigid_body_link.h"

#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {

  class DualRigidBodyLink : public RigidBodyLink {
    public:
      DualRigidBodyLink(const std::string &name="") : RigidBodyLink(name) { }

      void connect(RigidBody *body);
      void connect(RigidBody *body1, RigidBody *body2);

      std::string getType() const { return "DualRigidBodyLink"; }
      void init(InitStage stage);

      void initializeUsingXML(xercesc::DOMElement * element);

    private:
      std::string saved_ref, saved_ref1, saved_ref2;
  };

}

#endif
