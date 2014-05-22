/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _NODE_FRAME_H__
#define _NODE_FRAME_H__

#include "mbsim/frame.h"

namespace MBSimFlexibleBody {

  /**
   * \brief cartesian frame on nodes of flexible bodies
   * \author Kilian Grundl
   */
  class NodeFrame : public MBSim::Frame {

    public:
      NodeFrame(const std::string &name = "dummy", const size_t nodenumber = 0) :
          Frame(name), nodeNumber(nodenumber) {
      }

      std::string getType() const {
        return "NodeFrame";
      }

      virtual void init(MBSim::InitStage stage);

      void setNodeNumber(const size_t nodeNumber_) {
        nodeNumber = nodeNumber_;
      }

      const size_t & getNodeNumber() const {
        return nodeNumber;
      }

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

    protected:
      /*!
       * \brief node number of the frame
       */
      size_t nodeNumber;
  };

}

#endif /* _FRAME_H_ */

