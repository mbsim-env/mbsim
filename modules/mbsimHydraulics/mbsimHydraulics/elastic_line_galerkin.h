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

#ifndef  _ELASTIC_LINE_GALERKIN_H_
#define  _ELASTIC_LINE_GALERKIN_H_

#include "hline.h"

class ansatz_function;

namespace MBSimHydraulics {

  /*! ElasticLineGalerkin */
  class ElasticLineGalerkin : public HLine {

    public:
      
      enum AnsatzTypes {
        BSplineOrd4,
        BSplineOrd3,
        Polynom,
        Harmonic
      };

      /*! Constructor */
      ElasticLineGalerkin(const std::string &name="");

      void calcSize() override;

      /*! set initial pressure of the pipe fluid*/
      void setp0(double p0_) {p0=p0_; }
      /*! set the fracAir of the pipe (for E-Modulus calculation*/
      void setFracAir(double fracAir_) {fracAir=fracAir_; }
      /*! set delta_h of the pipe*/
      void setdh(double dh_) {delta_h=dh_; }
      /*! set damping meassure according Lehr, default=0*/
      void setDLehr(double DLehr_) {DLehr=DLehr_; }
      /*! set the method of the ansatz-function and its order; choose one of BSplineOrd3, BSplineOrd4, Polynom, Harmonic*/
      //void setAnsatzFunction(std::string method_, int nAnsatz_);
      void setAnsatzFunction(AnsatzTypes method_, int nAnsatz_);
      /*! set a 2dimension flow recognition*/
      void setFlow2D(bool flow2d=true){Flow2D=flow2d; }
      /*! select points in the pipe, for which a output should be created*/
      void setRelativePlotPoints(const fmatvec::Vec &rPP) {relPlotPoints=rPP; }
      void setDiameter(double d_) {d=d_; }
      void setLength(double l_) {l=l_; }

      void setQ0(double Q0_) {Q0=Q0_; }

      fmatvec::Vec getQIn() {return QIn; }
      fmatvec::Vec getQOut() {return QOut; }
      fmatvec::VecV getInflowFactor() override {return -Area*wE; }
      fmatvec::VecV getOutflowFactor() override {return Area*wA; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void calcqSize() override {qSize=mdim; }
      void calcuSize(int j) override {uSize[j]=mdim; }

      void updateQ() override;
      void updateh(int j=0) override;
      void updateT() override;
      void updateM() override;

      void plot() override;
      void plotParameters();

      void initializeUsingXML(xercesc::DOMElement * element) override;
    
    private:
      int mdim{0}, plotdim{0};
      double nu, g{0}, E{0}, k{0};
      fmatvec::Vec WInt, wA, wE, lambda;
      fmatvec::SymMat MatIntWWT, MatIntWSWST, K, D, N, Omega, MFac;
      fmatvec::SqrMat phi;
      ansatz_function * ansatz;
      fmatvec::Mat plotVecW, plotVecWS;
      double l{0}, d{0}, Area{0};

    protected:
      AnsatzTypes ansatzType;
      bool Flow2D{false};
      int nAnsatz{0};
      double p0{0}, Q0{0}, fracAir{0}, delta_h{0}, DLehr{0};
      fmatvec::Vec relPlotPoints;
  };
}


#endif   /* ----- #ifndef _HYDLINEGALERKIN_H_  ----- */

