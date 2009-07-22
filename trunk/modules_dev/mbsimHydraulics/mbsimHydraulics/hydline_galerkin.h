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

#ifndef  _HYDLINEGALERKIN_H_
#define  _HYDLINEGALERKIN_H_

#include "hydline.h"

class ansatz_function;

namespace MBSim {

  /** \brief Model for a multi-mass pipe*/
  class HydLineGalerkin : public HydLineAbstract {

    public:
      
      enum AnsatzTypes {
        BSplineOrd4,
        BSplineOrd3,
        Polynom,
        Harmonic
      };

      /*! Constructor */
      HydLineGalerkin(const std::string &name);
      /*! Destructor */
      ~HydLineGalerkin();
      virtual std::string getType() const { return "HydLineGalerkin"; }

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
      void setFlow2D(){Flow2D=true; }
      /*! select points in the pipe, for which a output should be created*/
      void setRelativePlotPoints(const fmatvec::Vec &rPP) {relPlotPoints=rPP; }

      void setQ0(double Q0_) {Q0=Q0_; }

      fmatvec::Vec getQIn(double t) {return QIn; }
      fmatvec::Vec getQOut(double t) {return QOut; }
      fmatvec::Vec getInflowFactor() {return -Area*wE; }
      fmatvec::Vec getOutflowFactor() {return Area*wA; }

      void init();
      void calcqSize();
      void calcuSize(int j);

      void updateStateDependentVariables(double t);
      void updateh(double t);
      void updateT(double t);
      void updateM(double t);

      void plot(double t, double dt);
      void initPlot();   
    
    private:
      int mdim, plotdim;
      double nu, g, E, k;
      fmatvec::Vec WInt, wA, wE, lambda;
      fmatvec::SymMat MatIntWWT, MatIntWSWST, K, D, N, Omega, MFac;
      fmatvec::SqrMat phi;
      ansatz_function * ansatz;
      fmatvec::Mat plotVecW, plotVecWS;
      fmatvec::Vec QIn, QOut;

    protected:
      AnsatzTypes ansatzType;
      bool Flow2D;
      int nAnsatz;
      double p0, Q0, fracAir, delta_h, DLehr;
      fmatvec::Vec relPlotPoints;
  };
}


#endif   /* ----- #ifndef _HYDLINEGALERKIN_H_  ----- */

