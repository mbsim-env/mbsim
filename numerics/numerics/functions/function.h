/* Copyright (C) 2004-2012 MBSim Development Team
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

#ifndef NUMERICSFUNCTION_H_
#define NUMERICSFUNCTION_H_

#include <fmatvec.h>

namespace MBSimNumerics {

  /*!
   * \brief template class for functions with one parameter
   * \author Markus Friedrich
   * \author Kilian Grundl
   * \date 2009-08-31 some comments (Thorsten Schindler)
   *
   * \todo: add XML support (?)
   * \todo: add mbsim-specific functions (?)
   */
  template<class Ret, class Arg>
    class Function1 {
      public:
        /**
         * \brief destructor
         */
        virtual ~Function1(){};

        /* INTERFACE FOR DERIVED CLASSES */
        /**
         * \return return value of function
         * \param argument of function
         * \param optional parameter class
         */
        virtual Ret operator()(const Arg& x, const void * =NULL)=0;
        /***************************************************/
    };

  /*!
   * \brief template class for functions with two parameters
   * \author Markus Friedrich
   * \date 2009-08-31 some comments (Thorsten Schindler)
   */
  template<class Ret, class Arg1, class Arg2>
    class Function2 {
      public:
        /**
         * \brief destructor
         */
        virtual ~Function2() {}

        /* INTERFACE FOR DERIVED CLASSES */
        /**
         * \return return value of function
         * \param argument of function
         * \param optional parameter class
         */
        virtual Ret operator()(const Arg1& p1, const Arg2& p2, const void * =NULL)=0;
        /***************************************************/
    };

  /*!
   * \brief template class for functions with three parameters
   * \author Markus Friedrich
   * \date 2009-08-31 some comments (Thorsten Schindler)
   */
  template<class Ret, class Arg1, class Arg2, class Arg3>
    class Function3 {
      public:
        /**
         * \brief destructor
         */
        virtual ~Function3() {}

        /* INTERFACE FOR DERIVED CLASSES */
        /**
         * \return return value of function
         * \param argument of function
         * \param optional parameter class
         */
        virtual Ret operator()(const Arg1& p1, const Arg2& p2, const Arg3& p3, const void * =NULL)=0;
        /***************************************************/
    };

  /*!
   * \brief template class for differentiable functions with one scalar parameter
   * \author Thorsten Schindler
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \date 2010-03-10 small correction: setDerivative(...,int) -> setDerivative(...,size_t) (Roland Zander)
   */
  template<class Ret>
    class DifferentiableFunction1 : public Function1<Ret,double> {
      public:
        /**
         * \brief constructor
         */
        DifferentiableFunction1() : Function1<Ret,double>(), order(0) {}

        /**
         * \brief destructor
         */
        virtual ~DifferentiableFunction1() { delete derivatives[0]; derivatives.erase(derivatives.begin()); }

        /* INHERITED INTERFACE OF FUNCTION1 */
        virtual Ret operator()(const double& x, const void * =NULL) {
          assert(derivatives.size()>0); return (getDerivative(order))(x); }
        /***************************************************/

        /**
         * \return derivative
         * \param degree of derivative
         */
        const Function1<Ret,double>& getDerivative(int degree) const { return *(derivatives[degree]); }

        /**
         * \return derivative
         * \param degree of derivative
         */
        Function1<Ret,double>& getDerivative(int degree) { return *(derivatives[degree]); }

        /**
         * \param highest derivative to add
         */
        void addDerivative(Function1<Ret,double> *diff) { derivatives.push_back(diff); }

        /**
         * \param derivative to add
         * \param degree of the derivative
         */
        void setDerivative(Function1<Ret,double> *diff,size_t degree) { derivatives.resize(max(derivatives.size(),degree+1)); derivatives[degree]=diff; }

        /**
         * \param orderOfDerivative
         */
        void setOrderOfDerivative(int i) { order=i; }
        /***************************************************/

      protected:
        /**
         * \brief vector of derivatives
         */
        std::vector<Function1<Ret,double>* > derivatives;
        int order;
    };

  /*!
   * \brief template class for constant functions with one parameter
   * \author Markus Friedrich
   * \date 2009-08-31 some comments (Thorsten Schindler)
   */
  template<class Ret, class Arg>
    class ConstantFunction1 : public Function1<Ret,Arg> {
      public:
        /**
         * \brief constructor
         */
        ConstantFunction1() {}

        /**
         * \brief constructor
         * \param constant return value
         */
        ConstantFunction1(Ret c_) : c(c_) {}

        virtual ~ConstantFunction1() {}

        /* INHERITED INTERFACE OF FUNCTION1 */
        virtual Ret operator()(const Arg& p, const void * =NULL) { return c; }
        /***************************************************/

        /* GETTER / SETTER */
        void setValue(Ret c_) { c=c_; }
        /***************************************************/

      protected:
        /**
         * \brief constant return value
         */
        Ret c;
    };

  /*!
   * \brief template class for constant functions with one parameter and scalar return value
   * \author Markus Friedrich
   * \date 2009-08-31 some comments (Thorsten Schindler)
   */
  template<class Arg>
    class ConstantFunction1<double, Arg> : public Function1<double,Arg> {
      public:
        /**
         * \brief constructor
         */
        ConstantFunction1() {}

        /**
         * \brief constructor
         * \param constant return value
         */
        ConstantFunction1(double c_) : c(c_) {}

        virtual ~ConstantFunction1() {}

        /* INHERITED INTERFACE OF FUNCTION1 */
        virtual double operator()(const Arg& p, const void * =NULL) { return c; }
        /***************************************************/

        /* GETTER / SETTER */
        void setValue(double c_) { c=c_; }
        /***************************************************/

      protected:
        /**
         * \brief constant return value
         */
        double c;
    };

  /*!
   * \brief template class for constant functions with two parameter
   * \author Markus Friedrich
   * \date 2009-08-31 some comments (Thorsten Schindler)
   */
  template<class Ret, class Arg1, class Arg2>
    class ConstantFunction2 : public Function2<Ret,Arg1,Arg2> {
      public:
        /**
         * \brief constructor
         */
        ConstantFunction2() {}

        /**
         * \brief constructor
         * \param constant return value
         */
        ConstantFunction2(Ret c_) : c(c_) {}

        virtual ~ConstantFunction2() {}

        /* INHERITED INTERFACE OF FUNCTION2 */
        virtual Ret operator()(const Arg1& p1, const Arg2& p2, const void * =NULL) { return c; }
        /***************************************************/

        /* GETTER / SETTER */
        void setValue(Ret c_) { c=c_; }
        /***************************************************/

      protected:
        /**
         * \brief constant return value
         */
        Ret c;
    };

  /*!
   * \brief template class for constant functions with two parameter and scalar return value
   * \author Markus Friedrich
   * \date 2009-08-31 some comments (Thorsten Schindler)
   */
  template<class Arg1, class Arg2>
    class ConstantFunction2<double, Arg1, Arg2> : public Function2<double,Arg1,Arg2> {
      public:
        /**
         * \brief constructor
         */
        ConstantFunction2() {}
        /**
         * \brief constructor
         * \param constant return value
         */
        ConstantFunction2(double c_) : c(c_) {}

        virtual ~ConstantFunction2() {}

        /* INHERITED INTERFACE OF FUNCTION2 */
        virtual double operator()(const Arg1& p1, const Arg2& p2, const void * =NULL) { return c; }
        /***************************************************/

        /* GETTER / SETTER */
        void setValue(double c_) { c=c_; }
        /***************************************************/

      protected:
        double c;
    };
}

#endif /* NUMERICSFUNCTION_H_ */
