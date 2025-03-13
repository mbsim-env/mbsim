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

#ifndef _MBSIM_TESTELEMENT_H_
#define _MBSIM_TESTELEMENT_H_

#include <mbsim/links/link.h>

namespace MBSim {

  class TestElement : public Link {
    public:
      TestElement(const std::string &name = "");
      void updateWRef(fmatvec::Mat&, int) override {}
      void updateVRef(fmatvec::Mat&, int) override {}
      void updatehRef(fmatvec::Vec&, int) override {}
      void updaterRef(fmatvec::Vec&, int) override {}
      bool isActive() const override { return false; }
      bool gActiveChanged() override { return false; }
  };

}

#endif
