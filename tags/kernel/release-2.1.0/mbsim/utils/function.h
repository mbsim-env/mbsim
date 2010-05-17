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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <mbsimtinyxml/tinyxml-src/tinyxml.h>
#include <mbsim/element.h>
#include <fmatvec.h>

namespace MBSim {

  /*! 
   * \brief template class for functions with one parameter
   * \author Markus Friedrich
   * \date 2009-08-31 some comments (Thorsten Schindler)
   */
  template<class Ret, class Arg>
    class Function1 {
      public:
        /**
         * \brief destructor
         */
        virtual ~Function1() {}

        /* INTERFACE FOR DERIVED CLASSES */
        /**
         * \return return value of function
         * \param argument of function
         * \param optional parameter class
         */
        virtual Ret operator()(const Arg& x, const void * =NULL)=0;

        /**
         * \brief initialize function with XML code
         * \param XML element
         */
        virtual void initializeUsingXML(TiXmlElement *element) {}
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

        /**
         * \brief initialize function with XML code
         * \param XML element
         */
        virtual void initializeUsingXML(TiXmlElement *element) {}
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

        /**
         * \brief initialize function with XML code
         * \param XML element
         */
        virtual void initializeUsingXML(TiXmlElement *element) {}
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

        /**
         * \brief initialize function with XML code
         * \param XML element
         */
        virtual void initializeUsingXML(TiXmlElement *element) {
          Function1<Ret,double>::initializeUsingXML(element);
          TiXmlElement * e;
          e=element->FirstChildElement(MBSIMNS"orderOfDerivative");
          if (e)
            setOrderOfDerivative(atoi(e->GetText()));
        }
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

        /* INHERITED INTERFACE OF FUNCTION1 */
        virtual Ret operator()(const Arg& p, const void * =NULL) { return c; }
        void initializeUsingXML(TiXmlElement *element) {
          Function1<Ret,Arg>::initializeUsingXML(element);
          TiXmlElement *e;
          e=element->FirstChildElement(MBSIMNS"value");
          c=Ret(e->GetText());
        }
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

        /* INHERITED INTERFACE OF FUNCTION1 */
        virtual double operator()(const Arg& p, const void * =NULL) { return c; }
        void initializeUsingXML(TiXmlElement *element) {
          Function1<double,Arg>::initializeUsingXML(element);
          TiXmlElement *e;
          e=element->FirstChildElement(MBSIMNS"value");
          c=Element::getDouble(e);
        }
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

        /* INHERITED INTERFACE OF FUNCTION2 */
        virtual Ret operator()(const Arg1& p1, const Arg2& p2, const void * =NULL) { return c; }
        void initializeUsingXML(TiXmlElement *element) {
          Function2<Ret,Arg1,Arg2>::initializeUsingXML(element);
          TiXmlElement *e;
          e=element->FirstChildElement(MBSIMNS"value");
          c=Ret(e->GetText());
        }
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

  /**
   * \brief function describing a linear relationship between the input relative distance / velocity and the output for a spring
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \todo put in function_library TODO
   */
  class LinearSpringDamperForce : public Function2<double,double,double> {
    public:
      /** 
       * \brief constructor
       */
      LinearSpringDamperForce() {}

      /** 
       * \brief constructor
       * \param stiffness
       * \param damping
       * \param undeformed length
       */
      LinearSpringDamperForce(double c_, double d_, double l0_) : c(c_), d(d_), l0(l0_) {}

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const double& g, const double& gd, const void * =NULL) { return c*(g-l0) + d*gd; }
      void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setParameters(double c_, double d_, double l0_) { c=c_; d=d_; l0=l0_; }
      /***************************************************/

    protected:
      /**
       * \brief stiffness, damping, undeformed length
       */
      double c, d, l0;
  };

  /**
   * \brief function describing a nonlinear relationship between the input relative distance / velocity and the output for a spring
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \todo put in function_library TODO
   */
  class NonlinearSpringDamperForce : public Function2<double,double,double> {
    public:
      /** 
       * \brief constructor
       */
      NonlinearSpringDamperForce() {}

      /** 
       * \brief constructor
       * \param distance depending force function
       * \param relative velocity depending force function
       */
      NonlinearSpringDamperForce(Function1<fmatvec::Vec, double> * gForceFun_, Function1<fmatvec::Vec, double> * gdForceFun_) : gForceFun(gForceFun_), gdForceFun(gdForceFun_) {}

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const double& g, const double& gd, const void * =NULL) { return (*gForceFun)(g)(0) + (*gdForceFun)(gd)(0); }
      void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setParameters(Function1<fmatvec::Vec, double> * gForceFun_, Function1<fmatvec::Vec, double> * gdForceFun_) { gForceFun=gForceFun_; gdForceFun=gdForceFun_; }
      /***************************************************/

    protected:
      /**
       * \brief distance depending force function
       */
      Function1<fmatvec::Vec, double> * gForceFun;

      /**
       * \brief relative velocity depending force function
       */
      Function1<fmatvec::Vec, double> * gdForceFun;
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

        /* INHERITED INTERFACE OF FUNCTION2 */
        virtual double operator()(const Arg1& p1, const Arg2& p2, const void * =NULL) { return c; }
        void initializeUsingXML(TiXmlElement *element) {
          Function2<double,Arg1,Arg2>::initializeUsingXML(element);
          TiXmlElement *e;
          e=element->FirstChildElement(MBSIMNS"value");
          c=Element::getDouble(e);
        }
        /***************************************************/

        /* GETTER / SETTER */
        void setValue(double c_) { c=c_; }
        /***************************************************/

      protected:
        double c;
    };

  /*! 
   * \brief function describing a linear relationship between the input relative distance / velocity and the output for a unilateral constraint
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \todo put in function_library TODO
   */
  class LinearRegularizedUnilateralConstraint: public Function2<double,double,double> {
    public:
      /**
       * \brief constructor
       */
      LinearRegularizedUnilateralConstraint() : c(0), d(0) {}

      /**
       * \brief constructor
       * \param stiffness
       * \param damping
       */
      LinearRegularizedUnilateralConstraint(double c_, double d_) : c(c_), d(d_) {}

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const double& g, const double& gd, const void * =NULL) { 
        if(g>0)
          return 0;
        else if(gd<0) 
          return -c*g - d*gd;
        else
          return -c*g;
      }
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setParameter(double c_, double d_) { c=c_; d=d_; }
      /***************************************************/

    private:
      /**
       * \brief stiffness, damping
       */
      double c, d;
  };

  /*! 
   * \brief function describing a linear relationship between the input relative distance / velocity and the output for a bilateral constraint
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \todo put in function_library TODO
   */
  class LinearRegularizedBilateralConstraint: public Function2<double,double,double> {
    public:
      /**
       * \brief constructor
       */
      LinearRegularizedBilateralConstraint() : c(0), d(0) {}

      /**
       * \brief constructor
       * \param stiffness
       * \param damping
       */
      LinearRegularizedBilateralConstraint(double c_, double d_) : c(c_), d(d_) {}

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const double& g, const double& gd, const void * =NULL) { 
        return -c*g - d*gd;
      }
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setParameter(double c_, double d_) { c=c_; d=d_; }
      /***************************************************/

    private:
      /**
       * \brief stiffness, damping
       */
      double c, d;
  };

  /*! 
   * \brief function describing a linear regularized relationship between the input relative velocity and the output for Coulomb friction
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \date 2010-01-09 beauty correction (Thorsten Schindler)
   * \todo put in function_library TODO
   */
  class LinearRegularizedCoulombFriction : public Function2<fmatvec::Vec,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       */
      LinearRegularizedCoulombFriction() : mu(0), gdLim(0.01) {}

      /**
       * \brief constructor
       * \param friction coefficient
       * \param border with respect to the relative velocity for the linear regularized increase of the friction force
       */
      LinearRegularizedCoulombFriction(double mu_, double gdLim_=0.01) : mu(mu_), gdLim(gdLim_) {}

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual fmatvec::Vec operator()(const fmatvec::Vec &gd, const double& laN, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setFrictionCoefficient(double mu_) { mu=mu_; }
      void setMarginalVelocity(double gdLim_) { gdLim=gdLim_; }
      /***************************************************/

    protected:
      /**
       * \brief friction coefficient, border with respect to the relative velocity for the linear regularized increase of the friction force
       */
      double mu, gdLim;
  };

  /**
   * \brief function describing a linear regularized relationship between the input relative velocity and the output for Stribeck friction
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \date 2010-01-09 beauty correction (Thorsten Schindler)
   * \todo put in function_library TODO
   */
  class LinearRegularizedStribeckFriction : public Function2<fmatvec::Vec,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       */
      LinearRegularizedStribeckFriction() : fmu(NULL), gdLim(0.01) {}

      /**
       * \brief constructor
       * \param function for friction coefficient depending on norm of relative velocity
       * \param border with respect to the relative velocity for the linear regularized increase of the friction force
       */
      LinearRegularizedStribeckFriction(Function1<double,double> *fmu_, double gdLim_=0.01) : fmu(fmu_), gdLim(gdLim_) {}

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual fmatvec::Vec operator()(const fmatvec::Vec &gd, const double& laN, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setFrictionFunction(Function1<double,double> *fmu_) { fmu=fmu_; }
      void setMarginalVelocity(double gdLim_) { gdLim=gdLim_; }
      /***************************************************/

    protected:
      /**
       * \brief friction coefficient function
       */
      Function1<double,double> *fmu;
      
      /**
       * \brief border with respect to the relative velocity for the linear regularized increase of the friction force
       */
      double gdLim;
  };

}

#endif /* FUNCTION_H_ */

