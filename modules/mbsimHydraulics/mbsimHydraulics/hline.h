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

#ifndef  _HLINE_H_
#define  _HLINE_H_

#include "mbsim/objects/object.h"
#include <mbsim/functions/function.h>

namespace MBSim {
  class Frame;
}

namespace MBSimHydraulics {

  extern const MBSim::PlotFeatureEnum volumeFlow, pressure;

  class HNode;
  class HydlinePressureloss;
  class PressureLoss;

  /*! HLine */
  class HLine : public MBSim::Object {
    public:
      HLine(const std::string &name) : MBSim::Object(name), nFrom(nullptr), nTo(nullptr), nFromRelative(false), nToRelative(false), direction(fmatvec::Vec(3, fmatvec::INIT, 0)), Mlocal(), QIn(1), QOut(1), Jacobian(), frameOfReference(nullptr), updQ(true), saved_frameOfReference("") { }
      void calcSize() override;

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      void updateh(int j=0) override { }
      void updateJacobians(int j=0) { }
      void updateInverseKineticsJacobians() { }
      std::shared_ptr<OpenMBV::Group> getOpenMBVGrp() override { return std::shared_ptr<OpenMBV::Group>(); }
      /***************************************************/

      virtual void setFrameOfReference(MBSim::Frame *frame);
      void setFromNode(HNode * nFrom_) {nFrom=nFrom_; }
      void setToNode(HNode * nTo_) {nTo=nTo_; }
      void setDirection(fmatvec::Vec dir) {direction=((nrm2(dir)>0)?dir/nrm2(dir):fmatvec::Vec(3, fmatvec::INIT, 0)); }
      HNode * getFromNode() { return nFrom; }
      HNode * getToNode() {return nTo; }
      void setOutflowRelative(bool rel=true) { nToRelative=rel; }
      void setInflowRelative(bool rel=true) { nFromRelative=rel; }
      virtual fmatvec::VecV getInflowFactor() {return fmatvec::VecV(1, fmatvec::INIT, -1.); }
      virtual fmatvec::VecV getOutflowFactor() {return fmatvec::VecV(1, fmatvec::INIT, 1.); }

      const fmatvec::VecV& evalQIn() { if(updQ) updateQ(); return QIn; }
      const fmatvec::VecV& evalQOut() { if(updQ) updateQ(); return QOut; }
      const fmatvec::VecV& getQIn(bool check=true) const { assert((not check) or (not updQ)); return QIn; }
      const fmatvec::VecV& getQOut(bool check=true) const { assert((not check) or (not updQ)); return QOut; }
      const fmatvec::MatV& getJacobian() const { return Jacobian; }

      virtual void updateQ() { }
      void updateM() override { M=Mlocal; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      virtual Element* getDependency() const { return nullptr; }

      void resetUpToDate() override { MBSim::Object::resetUpToDate(); updQ = true; }

    protected:
      HNode * nFrom;
      HNode * nTo;
      bool nFromRelative, nToRelative;
      fmatvec::VecV direction;
      fmatvec::SymMatV Mlocal;
      fmatvec::VecV QIn, QOut;
      fmatvec::MatV Jacobian;
      MBSim::Frame * frameOfReference;
      bool updQ;
    private:
      std::string saved_frameOfReference;
  };

  /*! RigidHLine */
  class RigidHLine : public HLine {
    public:
      RigidHLine(const std::string &name) : HLine(name), pressureLossGravity(0), length(0), updPLG(true) { }
      
      void setLength(double length_) {length=length_; }
      double getLength() const {return length; }
      void addInflowDependencyOnOutflow(RigidHLine* line);
      void addInflowDependencyOnInflow(RigidHLine* line);

      void calcqSize() override { qSize=0; }
      void calcuSize(int j=0) override { uSize[j]=getGeneralizedVelocitySize(); }
      fmatvec::Mat calculateJacobian(std::vector<RigidHLine*> dep_check);
      
      void updatePressureLossGravity();
      double evalPressureLossGravity() { if(updPLG) updatePressureLossGravity(); return pressureLossGravity; }

      void updateQ() override;
      void updateh(int j=0) override;
      void updateM() override;
      
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void plot() override;
      
      void resetUpToDate() override { HLine::resetUpToDate(); updPLG = true; }

    protected:
      double pressureLossGravity;
      double length;
      std::vector<RigidHLine*> dependencyOnOutflow, dependencyOnInflow;
      std::vector<std::string> refDependencyOnOutflowString, refDependencyOnInflowString;
      bool updPLG;
  };

  /*! ConstrainedLine */
  class ConstrainedLine : public HLine {
    public:
      ConstrainedLine(const std::string &name="") : HLine(name) { }
      
      void setQFunction(MBSim::Function<double(double)> * QFunction_) {
        QFunction=QFunction_;
        QFunction->setParent(this);
        QFunction->setName("Q");
      }

      void calcqSize() override { qSize=0; }
      void calcuSize(int j) override { uSize[j]=0; }
      
      void updateQ() override;
      
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      
    private:
      MBSim::Function<double(double)> * QFunction{nullptr};
  };

  /*! FluidPump */
  class FluidPump : public HLine {
    public:
      FluidPump(const std::string &name="") : HLine(name) { }
      
      void setQFunction(MBSim::Function<double(double)> * QFunction_) { QFunction=QFunction_; }

      void calcqSize() override { qSize=0; }
      void calcuSize(int j) override { uSize[j]=0; }
      
      void updateQ() override;
      
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      
    private:
      MBSim::Function<double(double)> *QFunction{nullptr};
  };

  /*! StatelessOrifice */
  class StatelessOrifice : public HLine {
    public:
      StatelessOrifice(const std::string &name="") : HLine(name) {}
      
      void setInflowFunction(MBSim::Function<double(double)> *inflowFunction_) { inflowFunction=inflowFunction_; }
      void setOutflowFunction(MBSim::Function<double(double)> *outflowFunction_) { outflowFunction=outflowFunction_; }
      void setOpeningFunction(MBSim::Function<double(double)> *openingFunction_) { openingFunction=openingFunction_; }
      void setDiameter(double diameter_) { diameter=diameter_; }
      void setAlpha(double alpha_) { alpha=alpha_; }
      void setCalcAreaModus(int calcAreaModus_) { calcAreaModus=calcAreaModus_; }

      void calcqSize() override { qSize=0; }
      void calcuSize(int j) override { uSize[j]=0; }
      
      void updateQ() override;
      
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void plot() override;
      
    private:
      MBSim::Function<double(double)> *inflowFunction{nullptr}, *outflowFunction{nullptr}, *openingFunction{nullptr};
      double diameter{0}, alpha{0.};
      int calcAreaModus{0};

      double pIn, pOut, dp, sign, opening, area, sqrt_dp;
  };

}

#endif   /* ----- #ifndef _HLINE_H_  ----- */

