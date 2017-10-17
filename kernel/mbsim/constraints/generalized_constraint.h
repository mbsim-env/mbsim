/* Copyright (C) 2004-2016 MBSim Development Team
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

#ifndef _GENERALIZED_CONSTRAINT_H
#define _GENERALIZED_CONSTRAINT_H

#include "mechanical_constraint.h"

namespace MBSim {

  /** 
   * \brief Class for generalized constraints
   * \author Martin Foerg
   */
  class GeneralizedConstraint : public MechanicalConstraint {
    protected:
      Frame *support;

    public:
      GeneralizedConstraint(const std::string &name) : MechanicalConstraint(name), support(NULL) { }

      void init(InitStage stage, const InitConfigSet &config);

      virtual void setSupportFrame(Frame *frame) { support = frame; }

      void initializeUsingXML(xercesc::DOMElement * element);

    private:
      std::string saved_DependentBody, saved_supportFrame;
  };

}

#endif
