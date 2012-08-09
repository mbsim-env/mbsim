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

#ifndef  _ELASTIC_LINE_VARIATIONAL_H_
#define  _ELASTIC_LINE_VARIATIONAL_H_

#include "hline.h"

namespace MBSimHydraulics {

  /*! ElasticLineVariational 
    This line model is described in
    J. Makinen, R. Piche and A. Ellman (2000),
    Fluid Transmission Line Modeling Using a Variational Method, 
    ASME Journal of Dynamic Systems, Measurement, and Control,122 (1), 153-162.
    */
  class ElasticLineVariational : public HLine {

    public:

      enum WindowFunction {
        None,
        Hann,
        Hamming,
        Riemann,
        BlackmanHarris
      };
      
      /*! Constructor */
      ElasticLineVariational(const std::string &name);
      virtual std::string getType() const { return "ElasticLineVariational"; }

      /*! set initial pressure of the pipe fluid*/
      void setp0(double p0_) {p0=p0_; }
      /*! set the fracAir of the pipe (for E-Modulus calculation*/
      void setFracAir(double fracAir_) {fracAir=fracAir_; }
      /*! set diameter*/
      void setDiameter(double d_) {r=d_/2.; }
      /*! set length*/
      void setLength(double l_) {l=l_; }
      /*! select points in the pipe, for which a output should be created*/
      void setRelativePlotPoints(const fmatvec::Vec &rPP) {relPlotPoints=rPP; }
      /*! select window function (default Blackman-Harris) */
      void setWindowFunction(WindowFunction w) {window_function_type=w; }
      /*! number of harmonic ansatz functions (default n=9) */
      void setNumberOfAnsatzFunctions(unsigned int n_=4) {n=2*n_+1; }
      /*! print system state vector */
      void printLineStateSpace(bool print=true) {printStateSpace=print; }

      fmatvec::Vec getQIn() {return QIn; }
      fmatvec::Vec getQOut() {return QOut; }
      fmatvec::Vec getInflowFactor() {return wI; }
      fmatvec::Vec getOutflowFactor() {return wO; }

      void init(MBSim::InitStage stage);
      void calcqSize() {qSize=n-1; }
      void calcuSize(int j) {uSize[j]=n; }

      void updateStateDependentVariables(double t);
      void updateh(double t, int j=0);
      void updateT(double t) {T=Tlocal; }
      void updateM(double t, int j=0) {M[j]=Mlocal; }

      void plot(double t, double dt);
      void plotParameters();

      void initializeUsingXML(TiXmlElement * element);

    protected:
      double p0, fracAir, r, l;
      fmatvec::Vec relPlotPoints;
      WindowFunction window_function_type;
      int n;
      bool printStateSpace;

    private:
      fmatvec::Vec QIn, QOut, wO, wI;
      fmatvec::Vec hq, hu, hp0, cu, y;
      fmatvec::Mat Tlocal, relPlot;
      fmatvec::SymMat Mlocal;
      
      void doPrintStateSpace();
  };

}


#endif

