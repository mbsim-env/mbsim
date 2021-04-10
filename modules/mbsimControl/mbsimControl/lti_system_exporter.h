/* Copyright (C) 2004-2021 MBSim Development Team
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

#ifndef _LTI_SYSTEM_EXPORTER_H_
#define _LTI_SYSTEM_EXPORTER_H_

#include "mbsim/solver.h"

namespace MBSimControl {

  /*!
   * \brief LTI system exporter
   * \author Martin Foerg
   */
  class LTISystemExporter : public MBSim::Solver {
    public:
      LTISystemExporter() { }
      void execute() override;
      void setInitialState(const fmatvec::Vec &z0_) { z0 <<= z0_; }
      void setInitialInput(const fmatvec::Vec &u0_) { u0 <<= u0_; }
      const fmatvec::Vec& getInitialState() const override { return z0; }
      void setInitialTime(double t_) { t=t_; }
      void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      fmatvec::Vec z0, zEq, u0;
      double t{0};
  };

}

#endif
