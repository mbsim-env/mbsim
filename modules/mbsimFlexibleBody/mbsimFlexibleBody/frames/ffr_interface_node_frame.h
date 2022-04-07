/* Copyright (C) 2004-2019 MBSim Development Team
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

#ifndef _FFR_INTERFACE_NODE_FRAME_H__
#define _FFR_INTERFACE_NODE_FRAME_H__

#include "mbsimFlexibleBody/frames/generic_ffr_interface_node_frame.h"

namespace MBSimFlexibleBody {

  /**
   * \brief interface node frame for flexible ffr bodies
   * \author Martin Förg
   */
  class FfrInterfaceNodeFrame : public GenericFfrInterfaceNodeFrame {

    public:
      FfrInterfaceNodeFrame(const std::string &name = "dummy") : GenericFfrInterfaceNodeFrame(name) { }

      void setNodeNumbers(const fmatvec::VecVI &nodes_) { nodes <<= nodes_; }
      const fmatvec::VecVI& getNodeNumbers() const { return nodes; }

      void setWeightingFactors(const fmatvec::VecV &weights_) { weights <<= weights_; }
      const fmatvec::VecV& getWeightingFactors() const { return weights; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
  };

}

#endif
