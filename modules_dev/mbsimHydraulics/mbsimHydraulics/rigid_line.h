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
 * Contact: schneidm@users.berlios.de
 */

#ifndef  _RIGID_LINE_H_
#define  _RIGID_LINE_H_

#include "mbsimHydraulics/hline.h"

namespace MBSimControl {
  class Signal;
}

namespace MBSimHydraulics {

  class LinePressureLoss;

  /*! RigidLine */
  class RigidLine : public RigidHLine {
    public:
      RigidLine(const std::string &name) : RigidHLine(name), diameter(0), pL(NULL) {}
      virtual std::string getType() const { return "RigidLine"; }

      void setDiameter(double diameter_) {diameter=diameter_; }
      double getDiameter() const {return diameter; }
      void setLinePressureLoss(LinePressureLoss * pL_) {pL=pL_; }

      void init(MBSim::InitStage stage);

      void initializeUsingXML(TiXmlElement *element);
    private:
      double diameter;
      LinePressureLoss * pL;
  };


  class ClosablePressureLoss;

  /*! ClosableRigidLine */
  class ClosableRigidLine : public RigidLine {
    public:
      ClosableRigidLine(const std::string &name) : RigidLine(name), cpL(NULL), cpLSignal(NULL), cpLMinValue(0), cpLUnilateral(false), cpLBilateral(false) {}
      virtual std::string getType() const { return "ClosableRigidLine"; }

      void setClosablePressureLoss(ClosablePressureLoss * cpL_) {cpL=cpL_; }
      ClosablePressureLoss * getClosablePressureLoss() const {return cpL; }
      void setSignal(MBSimControl::Signal * s) {cpLSignal = s; }
      MBSimControl::Signal * getSignal() const {return cpLSignal; }
      void setMinimalValue(double v) {cpLMinValue=v; }
      double getMinimalValue() const {return cpLMinValue; }
      void setUnilateral(bool u=true) {cpLUnilateral=u; }
      void setBilateral(bool b=true) {cpLBilateral=b; }
      bool isClosed() const;
      double getRegularizedValue() const;

      void init(MBSim::InitStage stage);

      void initializeUsingXML(TiXmlElement *element);
    private:
      ClosablePressureLoss * cpL;
      MBSimControl::Signal * cpLSignal;
      double cpLMinValue;
      bool cpLUnilateral, cpLBilateral;
  };
}

#endif   /* ----- #ifndef _RIGID_LINE_H_  ----- */

