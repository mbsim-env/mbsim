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

#ifndef  _LEAKAGE_LINE_H_
#define  _LEAKAGE_LINE_H_

#include "mbsimHydraulics/hline.h"
#include <mbsim/functions/function.h>

namespace MBSimHydraulics {

  class LeakagePressureLoss;
  class PlaneLeakagePressureLoss;
  class CircularLeakagePressureLoss;

  /*! LeakageLine */
  class LeakageLine : public RigidHLine {
    public:
      LeakageLine(const std::string &name) : RigidHLine(name), lpl(NULL), s1vFunction(NULL), s2vFunction(NULL), glFunction(NULL) {}
      ~LeakageLine();
      virtual std::string getType() const { return "LeakageLine"; }

      void setGapLengthFunction(MBSim::Function<double(double)> * s) {glFunction=s; }
      double getGapLength(double t) const;
      void setSurface1VelocityFunction(MBSim::Function<double(double)> * s) {s1vFunction=s; }
      double getSurface1Velocity(double t) const;
      void setSurface2VelocityFunction(MBSim::Function<double(double)> * s) {s2vFunction=s; }
      double getSurface2Velocity(double t) const;

      void init(InitStage stage);

      void initializeUsingXML(xercesc::DOMElement * element);
    protected:
      LeakagePressureLoss * lpl;
    private:
      MBSim::Function<double(double)> *s1vFunction, *s2vFunction, *glFunction;
  };

  /*! PlaneLeakageLine */
  class PlaneLeakageLine : public LeakageLine {
    public:
      PlaneLeakageLine(const std::string &name="") : LeakageLine(name), hGap(0), wGap(0) {};
      virtual std::string getType() const { return "PlaneLeakageLine"; }

      void setGapWidth(double wGap_) {wGap=wGap_; }
      double getGapWidth() const {return wGap; }
      void setGapHeight(double hGap_) {hGap=hGap_; }
      double getGapHeight() const {return hGap; }
      void setPlaneLeakagePressureLoss(PlaneLeakagePressureLoss * plpl);

      void init(InitStage stage);

      void initializeUsingXML(xercesc::DOMElement * element);
    private:
      double hGap, wGap;
  };

  /*! CircularLeakageLine */
  class CircularLeakageLine : public LeakageLine {
    public:
      CircularLeakageLine(const std::string &name="") : LeakageLine(name), rI(0), rO(0), hGap(0) {}
      virtual std::string getType() const { return "CircularLeakageLine"; }

      void setInnerRadius(double rI_) {rI=rI_; }
      double getInnerRadius() const {return rI; }
      void setGapHeight(double hGap_) {hGap=hGap_; }
      double getGapHeight() const {return hGap; }
      double getOuterRadius() const {return rO; }
      void setCircularLeakagePressureLoss(CircularLeakagePressureLoss * clpl);

      void init(InitStage stage);

      void initializeUsingXML(xercesc::DOMElement * element);
    private:
      double rI, rO, hGap;
  };

}

#endif   /* ----- #ifndef _LEAKAGE_H_  ----- */

