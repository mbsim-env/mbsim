/* Copyright (C) 2004-2006  Martin Förg, Mathias Bachmayer, Felix Kahr
 
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
 * Derived from data_interface_base Klass DataInterfaceBase. Here You'll find a bib
 * giving users the possibility of quick interaction with other objects.
 *
 * Feel free, implement Your own additional stuff derived from userfunction! 
 * The name of the game is user's function.   
 *
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _USERFUNCTION_H_
#define _USERFUNCTION_H_

#include<vector>
#include "eps.h"
#include "data_interface_base.h"

using namespace fmatvec;

namespace MBSim {
  class PPolynom;

  /*! Class for user defined functions
   * 
   * Spline interpolation or piecewise defined polynoms: ppolynom.h
   */
  class UserFunction : public DataInterfaceBase{
    private:
      /** tolerances for numerical differentiation */
      double delta, sqrtdelta;

    public:
      /*! Constructor */
      UserFunction() : delta(epsroot), sqrtdelta(sqrt(delta)) {}
      /*! Destructor */
      virtual ~UserFunction() {}
      
      /*! Function value */
//      virtual Vec operator()(double t);
      /*! Derivative */
      virtual Vec diff1(double t) {return (operator()(t+delta)-operator()(t-delta))/(2.0*delta);}
      /*! 2nd derivative */
      virtual Vec diff2(double t) {return (operator()(t+sqrtdelta)+operator()(t-sqrtdelta)-2*operator()(t))/(sqrtdelta*sqrtdelta);}    
  };

  /*! Linearly interpolated function table */
  class FuncTable : public UserFunction {
    protected:
      Mat table;
      int oldi;
      
    public:
      /*! Constructor */
      FuncTable() {oldi=0;}
      /*! Destructor */
      virtual ~FuncTable() {}
      
      /*! Function value */
      Vec operator()(double alpha); 
      /*! Initialise table */
      void setTable(const Mat &tab) {table = tab;}
      /*! Set given values manually */
      void setXY(const Vec& X,const Mat& Y);
      /*! Set given values by file */
      void setFile(const string &filename); 
  };

  class FuncTablePer : public UserFunction {
    protected:
      Mat table;
      
    public:
      /*! Constructor */
      FuncTablePer() {}
      /*! Destructor */
      virtual ~FuncTablePer() {}
      Vec operator()(double alpha) ; 
      void setFile(const string &filename); 
  };

  class FuncSum : public UserFunction {
    protected: 
      vector<DataInterfaceBase*> dib;
      vector<double> c;
      int outputdim;
    public:
      FuncSum() {}
      virtual ~FuncSum();
      void addInput(DataInterfaceBase* func_,double c_,int dim);
      void addInput(DataInterfaceBase* func_,double c_=1);
      Vec operator()(double x);      
  };
  
  /*! Constant function */
  class FuncConst : public UserFunction {
    protected:
      /** constant value */
      Vec c;
      
    public:
      /*! Constructor */
      FuncConst(const Vec& c_);
      /*! Destructor */
      virtual ~FuncConst() {}
      /*! Returns constant value */
      Vec operator()(double x) {return c;}
  };

  /*! Dynamic Step function (Used for e.g. Cosim) */
  class DynamicSteps : public UserFunction {
    protected:
      /** constant value */
      Vec c;
      
    public:
      /*! Constructor */
      DynamicSteps();
      /*! set new const value (current step) */
      void setValue(const Vec& c_) { c=c_; }
      /*! Destructor */
      virtual ~DynamicSteps() {}
      /*! Returns constant value */
      Vec operator()(double x) {return c;}
  };

  class FuncConstSwitchOff : public UserFunction {
    protected:
      Vec c;
      Vec Null;
      double xout;
    public:
      FuncConstSwitchOff(const Vec& c_, double xout_);
      Vec operator()(double x);  
  };


  
  /*! Linear function */
  class FuncLinear : public UserFunction {
    protected:
      Vec a,b;
      
    public:
      /*! Constructor */
      FuncLinear(const Vec& a_,const Vec& b_);
      Vec operator()(double x) {return a*x+b;}
  };
  
  /*! Quadratic function */
  class FuncQuadratic : public UserFunction {
    protected:
      Vec a,b,c;
      
    public:
      /*! Constructor */
      FuncQuadratic(const Vec& a_,const Vec& b_,const Vec& c_);
      Vec operator()(double x) {return a*x*x+b*x+c;}
  };
  
  /*! Harmonic function */
  class FuncHarmonic : public UserFunction {
    protected:
      Vec A,offset;
      double om, phi;
      
    public:
      FuncHarmonic(const Vec& A_,double om_, double phi_, const Vec& offset_) {A=A_; om=om_; phi=phi_; offset=offset_;}
      Vec operator()(double x) {return A*sin(om*x+phi)+offset;} 
  };
   
  class FuncGainOffset : public UserFunction {
    protected:
      double a;
      Vec b;
      DataInterfaceBase *f;
      
    public:
      /** gain*(f(x)+offset)*/
      FuncGainOffset(DataInterfaceBase *f_,double gain_,const Vec& offset_);
      Vec operator()(double x) {return a*((*f)(x)+b);} 
  };
  
  /*! Composed functions */
  class FuncFunction : public UserFunction {
    protected:
      /** outer function */
      DataInterfaceBase *f;
      /** inner function */
      DataInterfaceBase *g;
      
    public:
      /*! Constructor */
      FuncFunction(DataInterfaceBase *f_, DataInterfaceBase *g_);
      /*! Destructor */
      virtual ~FuncFunction() {}
      /*! Return function TODO: only for scalar-valued g*/
      Vec operator()(double x) {return (*f)(((*g)(x))(0));} 
  };
 
}

#endif
