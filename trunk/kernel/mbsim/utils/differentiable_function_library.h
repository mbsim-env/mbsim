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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef DIFFERENTIABLE_FUNCTION_LIBRARY_H_
#define DIFFERENTIABLE_FUNCTION_LIBRARY_H_

#include <mbsim/utils/function.h>

namespace MBSim {

  template<class Ret> class FirstDerivative;
  template<class Ret> class SecondDerivative;

  template<class Ret>
    class DifferentiableFunction : public Function1<Ret,double> {
      protected:
        DifferentiableFunction *firstDerivative, *secondDerivative;
      public:
        DifferentiableFunction() : firstDerivative(0), secondDerivative(0) {}

        virtual ~DifferentiableFunction() {
          delete firstDerivative;
          delete secondDerivative;
        }

        std::string getType() const { return "DifferentiableFunction"; }

        DifferentiableFunction* getFirstDerivative() {
          return firstDerivative ? firstDerivative : new FirstDerivative<Ret>(this);
        }

        DifferentiableFunction* getSecondDerivative() {
          return secondDerivative ? secondDerivative : new SecondDerivative<Ret>(this);
        }

        virtual Ret diff1(const double& x, const void * =NULL) {
          double dx = epsroot();
          return ((*this)(x+dx) - (*this)(x-dx))/(2.*dx);
        }
        virtual Ret diff2(const double& x, const void * =NULL) {
          double dx = epsroot();
          return ((*this).diff1(x+dx) - (*this).diff1(x-dx))/(2.*dx);
        }
    };

  template <class Ret>
    class FirstDerivative : public DifferentiableFunction<Ret> {
      protected:
        DifferentiableFunction<Ret> *f;
      public:
        FirstDerivative(DifferentiableFunction<Ret> *f_) : f(f_) {}
        Ret operator()(const double& x, const void * =NULL) {
          return (*f).diff1(x);
        }
        Ret diff1(const double& x, const void * =NULL) {
          return (*f).diff2(x);
        }
    };

  template <class Ret>
    class SecondDerivative : public DifferentiableFunction<Ret> {
      protected:
        DifferentiableFunction<Ret> *f;
      public:
        SecondDerivative(DifferentiableFunction<Ret> *f_) : f(f_) {}
        Ret operator()(const double& x, const void * =NULL) {
          return (*f).diff2(x);
        }
    };

  class Variable : public DifferentiableFunction<double> {
    public:
      Variable() {}
      double operator()(const double& x, const void * =NULL) {
        return x;
      }
      double diff1(const double& x, const void * =NULL) {
        return 1;
      }
      double diff2(const double& x, const void * =NULL) {
        return 0;
      }
  };

  class Zero : public DifferentiableFunction<double> {
    public:
      Zero() {}
      double operator()(const double& x, const void * =NULL) {
        return 0;
      }
      double diff1(const double& x, const void * =NULL) {
        return 0;
      }
      double diff2(const double& x, const void * =NULL) {
        return 0;
      }
  };

  class Constant : public DifferentiableFunction<double> {
    protected:
      double a;
    public:
      Constant(double a_) : a(a_) {}
      double operator()(const double& x, const void * =NULL) {
        return a;
      }
      double diff1(const double& x, const void * =NULL) {
        return 0;
      }
      double diff2(const double& x, const void * =NULL) {
        return 0;
      }
  };

  class Sinus : public DifferentiableFunction<double> {
    protected:
      DifferentiableFunction<double> *f;
    public:
      Sinus(DifferentiableFunction<double> *f_) : f(f_) {}
      double operator()(const double& x, const void * =NULL) {
        return sin((*f)(x));
      }
      double diff1(const double& x, const void * =NULL) {
        return cos((*f)(x))*(*f).diff1(x);
      }
      double diff2(const double& x, const void * =NULL) {
        return -sin((*f)(x))*(*f).diff1(x)*(*f).diff1(x) + cos((*f)(x))*(*f).diff2(x);
      }
  };

  class Product : public DifferentiableFunction<double> {
    protected:
      DifferentiableFunction<double> *f1, *f2;
    public:
      Product(DifferentiableFunction<double> *f1_, DifferentiableFunction<double> *f2_) : f1(f1_), f2(f2_){}
      double operator()(const double& x, const void * =NULL) {
        return (*f1)(x)*(*f2)(x);
      }
      double diff1(const double& x, const void * =NULL) {
        return (*f1).diff1(x)*(*f2)(x) + (*f1)(x)*(*f2).diff1(x);
      }
      double diff2(const double& x, const void * =NULL) {
        return (*f1).diff2(x)*(*f2)(x) + 2*(*f1).diff1(x)*(*f2).diff1(x) + (*f1)(x)*(*f2).diff2(x) ;
      }
  };

  class Sum : public DifferentiableFunction<double> {
    protected:
      DifferentiableFunction<double> *f1, *f2;
    public:
      Sum(DifferentiableFunction<double> *f1_, DifferentiableFunction<double> *f2_) : f1(f1_), f2(f2_){}
      double operator()(const double& x, const void * =NULL) {
        return (*f1)(x)+(*f2)(x);
      }
      double diff1(const double& x, const void * =NULL) {
        return (*f1).diff1(x)+(*f2).diff1(x);
      }
      double diff2(const double& x, const void * =NULL) {
        return (*f1).diff2(x)+(*f2).diff2(x);
      }
  };

  class LinearFunction : public DifferentiableFunction<double> {
    protected:
      double a, b;
    public:
      LinearFunction(double a_, double b_) : a(a_), b(b_) {}
      double operator()(const double& x, const void * =NULL) {
        return a*x+b;
      }
      double diff1(const double& x, const void * =NULL) {
        return a;
      }
      double diff2(const double& x, const void * =NULL) {
        return 0;
      }
  };

  template <class Col>
    class VectorFunction : public DifferentiableFunction<fmatvec::Vector<Col,double> > {
      protected:
        std::vector<DifferentiableFunction<double>*> f;
      public:
        VectorFunction() {}
        VectorFunction(std::vector<DifferentiableFunction<double>*> f_) : f(f_) {}
        void addFunction(DifferentiableFunction<double> *f_) {f.push_back(f_);}
        virtual fmatvec::Vector<Col,double> operator()(const double& x, const void * =NULL) {
          fmatvec::Vector<Col,double> r(f.size(),fmatvec::NONINIT);
          for(int i=0; i<r.size(); i++)
            r(i) = (*f[i])(x);
          return r;
        };
        virtual fmatvec::Vector<Col,double> diff1(const double& x, const void * =NULL) {
          fmatvec::Vector<Col,double> r(f.size(),fmatvec::NONINIT);
          for(int i=0; i<r.size(); i++)
            r(i) = (*f[i]).diff1(x);
          return r;
        }
        virtual fmatvec::Vector<Col,double> diff2(const double& x, const void * =NULL) {
          fmatvec::Vector<Col,double> r(f.size(),fmatvec::NONINIT);
          for(int i=0; i<r.size(); i++)
            r(i) = (*f[i]).diff2(x);
          return r;
        }
    };

}


#endif
