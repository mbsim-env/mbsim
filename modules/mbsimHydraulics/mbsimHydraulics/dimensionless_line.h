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

#ifndef  _DIMENSIONLESS_LINE_H_
#define  _DIMENSIONLESS_LINE_H_

#include "mbsimHydraulics/hline.h"

namespace MBSimHydraulics {

  /*! DimensionlessLine */
  class DimensionlessLine : public HLine {
    public:
      DimensionlessLine(const std::string &name) : HLine(name), length(0) {}
      
      void calcSize() override { nu = 0; updSize = false; }
      void setLength(double length_) {length=length_; }
      double getLength() const {return length; }

      fmatvec::VecV getInflowFactor() override {return fmatvec::VecV(1, fmatvec::INIT, -1.); }
      fmatvec::VecV getOutflowFactor() override {return fmatvec::VecV(1, fmatvec::INIT, 1.); }
      void calcqSize() override {qSize=0; }
      void calcuSize(int j) override {uSize[j]=0; }
      
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void plot() override;
    protected:
      double length;
  };
  
  class LeakagePressureLoss;
  class PlaneLeakagePressureLoss;
  class CircularLeakagePressureLoss;

  class Leakage0DOF : public DimensionlessLine {
    public:
      Leakage0DOF(const std::string &name) : DimensionlessLine(name), lpl(nullptr), s1vFunction(nullptr), s2vFunction(nullptr), glFunction(nullptr) {}
      ~Leakage0DOF() override;

      void setGapLengthFunction(MBSim::Function<double(double)> * s) {
        glFunction=s;
        glFunction->setParent(this);
      }
      double evalGapLength() const;
      void setSurface1VelocityFunction(MBSim::Function<double(double)> * s) {
        s1vFunction=s;
        s1vFunction->setParent(this);
      }
      double evalSurface1Velocity() const;
      void setSurface2VelocityFunction(MBSim::Function<double(double)> * s) {
        s2vFunction=s;
        s2vFunction->setParent(this);
      }
      double evalSurface2Velocity() const;

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      void updateQ() override;

      void initializeUsingXML(xercesc::DOMElement * element) override;
    protected:
      LeakagePressureLoss * lpl;
    private:
      MBSim::Function<double(double)> *s1vFunction, *s2vFunction, *glFunction;
  };

  class PlaneLeakage0DOF : public Leakage0DOF {
    public:
      PlaneLeakage0DOF(const std::string &name="") : Leakage0DOF(name) {}

      void setGapWidth(double wGap_) {wGap=wGap_; }
      double getGapWidth() const {return wGap; }
      void setGapHeight(double hGap_) {hGap=hGap_; }
      double getGapHeight() const {return hGap; }
      void setPlaneLeakagePressureLoss(PlaneLeakagePressureLoss * plpl);

      void initializeUsingXML(xercesc::DOMElement * element) override;
    private:
      double hGap{0}, wGap{0};
  };

  class CircularLeakage0DOF : public Leakage0DOF {
    public:
      CircularLeakage0DOF(const std::string &name="") : Leakage0DOF(name) {}

      void setInnerRadius(double rI_) {rI=rI_; }
      double getInnerRadius() const {return rI; }
      void setGapHeight(double hGap_) {hGap=hGap_; }
      double getGapHeight() const {return hGap; }
      double getOuterRadius() const {return rO; }
      void setCircularLeakagePressureLoss(CircularLeakagePressureLoss * clpl);

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      void initializeUsingXML(xercesc::DOMElement * element) override;
    private:
      double rI{0}, rO{0}, hGap{0};
  };

}

#endif   /* ----- #ifndef _DIMENSIONLESS_LINE_H_  ----- */

