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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _NODE_BASED_BODY_H_
#define _NODE_BASED_BODY_H_

#include "mbsim/objects/body.h"
#include "mbsimFlexibleBody/namespace.h"

namespace MBSimFlexibleBody {

  class NodeFrame;

  class NodeBasedBody : public MBSim::Body {
    public:
      NodeBasedBody(const std::string &name);
      virtual void updatePositions(NodeFrame* frame);
      virtual void updateVelocities(NodeFrame* frame);
      virtual void updateAccelerations(NodeFrame* frame);
      virtual void updateJacobians(NodeFrame* frame, int j=0);
      virtual void updateGyroscopicAccelerations(NodeFrame* frame);
      using MBSim::Body::addFrame;
      /**
       * \param node frame
       */
      void addFrame(NodeFrame *frame);
  };

}

#endif
