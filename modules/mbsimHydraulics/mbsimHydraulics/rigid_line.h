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
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef  _RIGID_LINE_H_
#define  _RIGID_LINE_H_

#include "mbsimHydraulics/hline.h"

namespace MBSimHydraulics {

  class LinePressureLoss;

  /*! RigidLine */
  class RigidLine : public RigidHLine {
    public:
      RigidLine(const std::string &name="") : RigidHLine(name)  {}
      ~RigidLine() override;

      void setDiameter(double diameter_) {diameter=diameter_; }
      double getDiameter() const {return diameter; }
      void setLinePressureLoss(LinePressureLoss * pL_);

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      void plot() override;

      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      double diameter{0};
      LinePressureLoss * pL{nullptr};
      double ReynoldsFactor{0};
  };


  class ClosablePressureLoss;

  /*! ClosableRigidLine */
  class ClosableRigidLine : public RigidLine {
    public:
      ClosableRigidLine(const std::string &name="") : RigidLine(name) {}
      ~ClosableRigidLine() override;

      void setClosablePressureLoss(ClosablePressureLoss * cpL_);
      ClosablePressureLoss * getClosablePressureLoss() const {return cpL; }
      void setFunction(MBSim::Function<double(double)> * s) {
        cpLFunction = s;
        cpLFunction->setParent(this);
        cpLFunction->setName("cpLFunction");
      }
      MBSim::Function<double(double)> * getFunction() const {return cpLFunction; }
      void setMinimalValue(double v) {cpLMinValue=v; }
      double getMinimalValue() const {return cpLMinValue; }
      void setBilateral(bool b=true) {cpLBilateral=b; }
      bool isClosed() const;
      double evalRegularizedValue() const;

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      ClosablePressureLoss * cpL{nullptr};
      MBSim::Function<double(double)> * cpLFunction{nullptr};
      double cpLMinValue{0};
      bool cpLBilateral{false};
  };


  class UnidirectionalPressureLoss;

  /*! UnidirectionalRigidLine */
  class UnidirectionalRigidLine : public RigidLine {
    public:
      UnidirectionalRigidLine(const std::string &name="") : RigidLine(name) {}

      void setUnidirectionalPressureLoss(UnidirectionalPressureLoss * upL_) {upL=upL_; }
      UnidirectionalPressureLoss * getUnidirectionalPressureLoss() const {return upL; }
      void setMinimalPressureDrop(double v) {dpMin=v; }
      double getMinimalPressureDrop() const {return dpMin; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      void initializeUsingXML(xercesc::DOMElement *element) override;
    private:
      UnidirectionalPressureLoss * upL{nullptr};
      double dpMin{0};
  };

}

#endif   /* ----- #ifndef _RIGID_LINE_H_  ----- */

