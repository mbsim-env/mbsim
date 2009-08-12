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
   */
  template<class Ret, class Arg>
    class Function1 {
      public:
        virtual ~Function1() {}
        virtual Ret operator()(const Arg& x)=0;
        virtual void initializeUsingXML(TiXmlElement *element) {}
    };

  /*! 
   * \brief template class for functions with two parameters
   */
  template<class Ret, class Arg1, class Arg2>
    class Function2 {
      public:
        virtual ~Function2() {}
        virtual Ret operator()(const Arg1& p1, const Arg2& p2)=0;
        virtual void initializeUsingXML(TiXmlElement *element) {}
    };

  /*! 
   * \brief template class for functions with three parameters
   */
  template<class Ret, class Arg1, class Arg2, class Arg3>
    class Function3 {
      public:
        virtual ~Function3() {}
        virtual Ret operator()(const Arg1& p1, const Arg2& p2, const Arg3& p3)=0;
        virtual void initializeUsingXML(TiXmlElement *element) {}
    };

  /*! 
   * \brief template class for differentiable functions with one scalar parameter
   */
  template<class Ret>
    class Differentiable1Function1 : public Function1<Ret,double> {
      public:
        Differentiable1Function1() : Function1<Ret,double>(), diff(0) {}
        Differentiable1Function1(Function1<Ret,double> *diff_) : Function1<Ret,double>(), diff(diff_) {}
        virtual ~Differentiable1Function1() { if(diff) delete diff; diff=0; }
        const Function1<Ret,double>& getDerivative() const { return *diff; }
        Function1<Ret,double>& getDerivative() { return *diff; }

      protected:
        Function1<Ret,double> *diff; // derivative
    };

  /*! 
   * \brief template class for two times differentiable functions with one scalar parameter
   */
  template<class Ret>
    class Differentiable2Function1 : public Function1<Ret,double> {
      public:
        Differentiable2Function1() : Function1<Ret,double>(), diff(0) {}
        Differentiable2Function1(Differentiable1Function1<Ret> *diff_) : Function1<Ret,double>(), diff(diff_) {}
        virtual ~Differentiable2Function1() { if(diff) delete diff; diff=0; }
        const Differentiable1Function1<Ret>& getDerivative() const { return *diff; }
        Differentiable1Function1<Ret>& getDerivative() { return *diff; }

      protected:
        Differentiable1Function1<Ret> *diff; // first and second derivative
    };

  template<class Ret, class Arg>
    class ConstantFunction1 : public Function1<Ret,Arg> {
      public:
        ConstantFunction1() {}
        ConstantFunction1(Ret c_) : c(c_) {}
        void setValue(Ret c_) { c=c_; }
        Ret operator()(const Arg& p) { return c; }
        void initializeUsingXML(TiXmlElement *element) {
          Function1<Ret,Arg>::initializeUsingXML(element);
          TiXmlElement *e;
          e=element->FirstChildElement(MBSIMNS"value");
          c=Ret(e->GetText());
        }

      protected:
        Ret c;
    };

  template<class Ret, class Arg1, class Arg2>
    class ConstantFunction2 : public Function2<Ret,Arg1,Arg2> {
      public:
        ConstantFunction2() {}
        ConstantFunction2(Ret c_) : c(c_) {}
        void setValue(Ret c_) { c=c_; }
        Ret operator()(const Arg1& p1, const Arg2& p2) { return c; }
        void initializeUsingXML(TiXmlElement *element) {
          Function2<Ret,Arg1,Arg2>::initializeUsingXML(element);
          TiXmlElement *e;
          e=element->FirstChildElement(MBSIMNS"value");
          c=Ret(e->GetText());
        }

      protected:
        Ret c;
    };

  class LinearSpringDamperForce : public Function2<double,double,double> {
    public:
      LinearSpringDamperForce() {}
      LinearSpringDamperForce(double c_, double d_, double l0_) : c(c_), d(d_), l0(l0_) {}
      void setParameters(double c_, double d_, double l0_) { c=c_; d=d_; l0=l0_; }
      double operator()(const double& g, const double& gd) { return c*(g-l0) + d*gd; }
      void initializeUsingXML(TiXmlElement *element);

    protected:
      double c, d, l0;
  };

  template<class Arg1, class Arg2>
    class ConstantFunction2<double, Arg1, Arg2> : public Function2<double,Arg1,Arg2> {
      public:
        ConstantFunction2() {}
        ConstantFunction2(double c_) : c(c_) {}
        void setValue(double c_) { c=c_; }
        double operator()(const Arg1& p1, const Arg2& p2) { return c; }
        void initializeUsingXML(TiXmlElement *element) {
          Function2<double,Arg1,Arg2>::initializeUsingXML(element);
          TiXmlElement *e;
          e=element->FirstChildElement(MBSIMNS"value");
          c=atof(e->GetText());
        }

      protected:
        double c;
    };

  class LinearRegularizedUnilateralConstraint: public Function2<double,double,double> {
    public:
      LinearRegularizedUnilateralConstraint() : c(0), d(0) {}
      LinearRegularizedUnilateralConstraint(double c_, double d_) : c(c_), d(d_) {}
      void setParameter(double c_, double d_) { c=c_; d=d_; }
      double operator()(const double& g, const double& gd) { 
        if(g>0)
          return 0;
        else if(gd<0) 
          return -c*g - d*gd;
        else
          return -c*g;
      }
      virtual void initializeUsingXML(TiXmlElement *element);

    private:
      double c, d;
  };

  class LinearRegularizedBilateralConstraint: public Function2<double,double,double> {
    public:
      LinearRegularizedBilateralConstraint() : c(0), d(0) {}
      LinearRegularizedBilateralConstraint(double c_, double d_) : c(c_), d(d_) {}
      void setParameter(double c_, double d_) { c=c_; d=d_; }
      double operator()(const double& g, const double& gd) { 
        return -c*g - d*gd;
      }
      virtual void initializeUsingXML(TiXmlElement *element);

    private:
      double c, d;
  };

  class LinearRegularizedCoulombFriction : public Function2<fmatvec::Vec,fmatvec::Vec,double> {
    public:
      LinearRegularizedCoulombFriction() : mu(0), gdLim(0.01) {}
      LinearRegularizedCoulombFriction(double mu_, double gdLim_=0.01) : mu(mu_), gdLim(gdLim_) {}
      void setFrictionCoeffizient(double mu_) { mu=mu_; }
      void setMarginalVelocity(double gdLim_) { gdLim=gdLim_; }
      virtual void initializeUsingXML(TiXmlElement *element);

    protected:
      double mu, gdLim;
  };

  class LinearRegularizedPlanarCoulombFriction : public LinearRegularizedCoulombFriction {
    public:
      LinearRegularizedPlanarCoulombFriction() : LinearRegularizedCoulombFriction() {}
      LinearRegularizedPlanarCoulombFriction(double mu_, double gdLim_=0.01) : LinearRegularizedCoulombFriction(mu_, gdLim_) {}
      fmatvec::Vec operator()(const fmatvec::Vec &gd, const double& laN) { 
        if(fabs(gd(0)) < gdLim)
          return fmatvec::Vec(1,fmatvec::INIT,-laN*mu*gd(0)/gdLim);
        else
          return fmatvec::Vec(1,fmatvec::INIT,gd(0)>0?-laN*mu:laN*mu);
      }
  };

  class LinearRegularizedSpatialCoulombFriction : public LinearRegularizedCoulombFriction {
    public:
      LinearRegularizedSpatialCoulombFriction() : LinearRegularizedCoulombFriction() {}
      LinearRegularizedSpatialCoulombFriction(double mu_, double gdLim_=0.01) : LinearRegularizedCoulombFriction(mu_, gdLim_) {}
      fmatvec::Vec operator()(const fmatvec::Vec &gd, const double& laN) { 
        double normgd = nrm2(gd);
        if(normgd < gdLim)
          return gd*(-laN*mu/gdLim);
        else
          return gd*(-laN*mu/normgd);
      }
  };

  class LinearRegularizedPlanarStribeckFriction : public Function2<fmatvec::Vec,fmatvec::Vec,double> {
    public:
      LinearRegularizedPlanarStribeckFriction() : gdLim(0.01), fmu(NULL) {}
      LinearRegularizedPlanarStribeckFriction(double gdLim_, Function1<double,double> *fmu_) : gdLim(gdLim_), fmu(fmu_) {}
      void setMarginalVelocity(double gdLim_) { gdLim=gdLim_; }
      void setFrictionFunction(Function1<double,double> *fmu_) { fmu=fmu_; }
      fmatvec::Vec operator()(const fmatvec::Vec &gd, const double& laN);
      virtual void initializeUsingXML(TiXmlElement *element);

    protected:
      double gdLim;
      Function1<double,double> *fmu;
  };

}

#endif /* FUNCTION_H_ */

