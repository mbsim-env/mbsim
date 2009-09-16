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

#ifndef  _DIMENSIONLESS_LINE_H_
#define  _DIMENSIONLESS_LINE_H_

#include "mbsimHydraulics/hline.h"

namespace MBSim {

  /*! DimensionlessLine */
  class DimensionlessLine : public HLine {
    public:
      DimensionlessLine(const std::string &name) : HLine(name), Q(1), length(0) {}
      virtual std::string getType() const { return "DimensionlessLine"; }
      
      void setLength(double length_) {length=length_; }
      double getLength() const {return length; }

      virtual fmatvec::Vec getQIn(double t) {return Q; }
      virtual fmatvec::Vec getQOut(double t) {return -Q; }
      virtual fmatvec::Vec getInflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, -1.); }
      virtual fmatvec::Vec getOutflowFactor() {return fmatvec::Vec(1, fmatvec::INIT, 1.); }
      void calcqSize() {qSize=0; }
      void calcuSize(int j) {uSize[j]=0; }
      
      void initializeUsingXML(TiXmlElement *element);
      void init(InitStage stage);
      void plot(double t, double dt);
    protected:
      fmatvec::Vec Q;
      double length;
  };
  
  class LeakagePressureLoss;
  class PlaneLeakagePressureLoss;
  class CircularLeakagePressureLoss;

  class Leakage0DOF : public DimensionlessLine {
    public:
      Leakage0DOF(const std::string &name) : DimensionlessLine(name), lpl(NULL), s1vSignal(NULL), s2vSignal(NULL), glSignal(NULL), s1vPath(""), s2vPath(""), glPath("") {}
      virtual std::string getType() const { return "Leakage0DOF"; }

      void setGapLengthSignal(Signal * s) {glSignal=s; }
      double getGapLength() const;
      void setSurface1VelocitySignal(Signal * s) {s1vSignal=s; }
      double getSurface1Velocity() const;
      void setSurface2VelocitySignal(Signal * s) {s2vSignal=s; }
      double getSurface2Velocity() const;

      void init(InitStage stage);

      void updateStateDependentVariables(double t);

      void initializeUsingXML(TiXmlElement * element);
    protected:
      LeakagePressureLoss * lpl;
    private:
      Signal *s1vSignal, *s2vSignal, *glSignal;
      std::string s1vPath, s2vPath, glPath;
  };

  class PlaneLeakage0DOF : public Leakage0DOF {
    public:
      PlaneLeakage0DOF(const std::string &name) : Leakage0DOF(name), hGap(0), wGap(0) {}
      virtual std::string getType() const { return "PlaneLeakage0DOF"; }

      void setGapWidth(double wGap_) {wGap=wGap_; }
      double getGapWidth() const {return wGap; }
      void setGapHeight(double hGap_) {hGap=hGap_; }
      double getGapHeight() const {return hGap; }
      void setPlaneLeakagePressureLoss(PlaneLeakagePressureLoss * plpl);

      void initializeUsingXML(TiXmlElement * element);
    private:
      double hGap, wGap;
  };

  class CircularLeakage0DOF : public Leakage0DOF {
    public:
      CircularLeakage0DOF(const std::string &name) : Leakage0DOF(name), rI(0), rO(0), hGap(0) {}
      virtual std::string getType() const { return "CircularLeakage0DOF"; }

      void setInnerRadius(double rI_) {rI=rI_; }
      double getInnerRadius() const {return rI; }
      void setGapHeight(double hGap_) {hGap=hGap_; }
      double getGapHeight() const {return hGap; }
      double getOuterRadius() const {return rO; }
      void setCircularLeakagePressureLoss(CircularLeakagePressureLoss * clpl);

      void init(InitStage stage);

      void initializeUsingXML(TiXmlElement * element);
    private:
      double rI, rO, hGap;
  };




}

#endif   /* ----- #ifndef _DIMENSIONLESS_LINE_H_  ----- */

