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

#ifndef _FUNCTION_LIBRARY_H_
#define _FUNCTION_LIBRARY_H_

#include "function.h"
#include "mbsim/objectfactory.h"
#include "mbsim/element.h"
#include "mbsim/utils/eps.h"

namespace MBSim {

  template<typename Sig> class StateDependentFunction;

  template <>
    class StateDependentFunction<fmatvec::Vec3(fmatvec::VecV,double)> : public fmatvec::Function<fmatvec::Vec3(fmatvec::VecV,double)> {
      private:
        fmatvec::Function<fmatvec::Vec3(fmatvec::VecV)> *f;
      public:
        StateDependentFunction(fmatvec::Function<fmatvec::Vec3(fmatvec::VecV)> *f_) : f(f_) { }
        typename fmatvec::Size<fmatvec::VecV>::type getArg1Size() const { return f->getArgSize();}
        typename fmatvec::Size<double>::type getArg2Size() const { return 0; }
        fmatvec::Vec3 operator()(const fmatvec::VecV &arg1, const double &arg2) {return (*f)(arg1); }
        typename fmatvec::Der<fmatvec::Vec3, fmatvec::VecV>::type parDer1(const fmatvec::VecV &arg1, const double &arg2) { return f->parDer(arg1); }
        typename fmatvec::Der<fmatvec::Vec3, double>::type parDer2(const fmatvec::VecV &arg1, const double &arg2) {return fmatvec::Vec3(); }
        typename fmatvec::Der<typename fmatvec::Der<fmatvec::Vec3, double>::type, double>::type parDer2ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Vec3(); }
        typename fmatvec::Der<fmatvec::Vec3, double>::type parDer2DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Vec3(); }
        typename fmatvec::Der<typename fmatvec::Der<fmatvec::Vec3, fmatvec::VecV>::type, double>::type parDer1ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Mat3xV(getArg1Size()); }
        typename fmatvec::Der<fmatvec::Vec3, fmatvec::VecV>::type parDer1DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return f->parDerDirDer(arg1Dir,arg1); }
    };

  template <>
    class StateDependentFunction<fmatvec::RotMat3(fmatvec::VecV,double)> : public fmatvec::Function<fmatvec::RotMat3(fmatvec::VecV,double)> {
      private:
        fmatvec::Function<fmatvec::RotMat3(fmatvec::VecV)> *f;
      public:
        StateDependentFunction(fmatvec::Function<fmatvec::RotMat3(fmatvec::VecV)> *f_) : f(f_) { }
        typename fmatvec::Size<fmatvec::VecV>::type getArg1Size() const { return f->getArgSize();}
        typename fmatvec::Size<double>::type getArg2Size() const { return 0; }
        fmatvec::RotMat3 operator()(const fmatvec::VecV &arg1, const double &arg2) {return (*f)(arg1); }
        typename fmatvec::Der<fmatvec::RotMat3, fmatvec::VecV>::type parDer1(const fmatvec::VecV &arg1, const double &arg2) { return f->parDer(arg1); }
        typename fmatvec::Der<fmatvec::RotMat3, double>::type parDer2(const fmatvec::VecV &arg1, const double &arg2) {return fmatvec::Vec3(); }
        typename fmatvec::Der<typename fmatvec::Der<fmatvec::RotMat3, double>::type, double>::type parDer2ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Vec3(); }
        typename fmatvec::Der<fmatvec::RotMat3, double>::type parDer2DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Vec3(); }
        typename fmatvec::Der<typename fmatvec::Der<fmatvec::RotMat3, fmatvec::VecV>::type, double>::type parDer1ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Mat3xV(getArg1Size()); }
        typename fmatvec::Der<fmatvec::RotMat3, fmatvec::VecV>::type parDer1DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return f->parDerDirDer(arg1Dir,arg1); }
    };

  template<typename Sig> class TimeDependentFunction;

  template <>
    class TimeDependentFunction<fmatvec::Vec3(fmatvec::VecV,double)> : public fmatvec::Function<fmatvec::Vec3(fmatvec::VecV,double)> {
      private:
        fmatvec::Function<fmatvec::Vec3(double)> *f;
      public:
        TimeDependentFunction(fmatvec::Function<fmatvec::Vec3(double)> *f_) : f(f_) { }
        typename fmatvec::Size<fmatvec::VecV>::type getArg1Size() const { return 0;}
        typename fmatvec::Size<double>::type getArg2Size() const { return 1; }
        fmatvec::Vec3 operator()(const fmatvec::VecV &arg1, const double &arg2) {return (*f)(arg2); }
        typename fmatvec::Der<fmatvec::Vec3, fmatvec::VecV>::type parDer1(const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Mat3xV(); }
        typename fmatvec::Der<fmatvec::Vec3, double>::type parDer2(const fmatvec::VecV &arg1, const double &arg2) {return f->parDer(arg2); }
        typename fmatvec::Der<typename fmatvec::Der<fmatvec::Vec3, double>::type, double>::type parDer2ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return f->parDerParDer(arg2); }
        typename fmatvec::Der<fmatvec::Vec3, double>::type parDer2DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Vec3(); }
        typename fmatvec::Der<typename fmatvec::Der<fmatvec::Vec3, fmatvec::VecV>::type, double>::type parDer1ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Mat3xV(); }
        typename fmatvec::Der<fmatvec::Vec3, fmatvec::VecV>::type parDer1DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Mat3xV(); }
    };

  template <>
    class TimeDependentFunction<fmatvec::RotMat3(fmatvec::VecV,double)> : public fmatvec::Function<fmatvec::RotMat3(fmatvec::VecV,double)> {
      private:
        fmatvec::Function<fmatvec::RotMat3(double)> *f;
      public:
        TimeDependentFunction(fmatvec::Function<fmatvec::RotMat3(double)> *f_) : f(f_) { }
        typename fmatvec::Size<fmatvec::VecV>::type getArg1Size() const { return 0;}
        typename fmatvec::Size<double>::type getArg2Size() const { return 1; }
        fmatvec::RotMat3 operator()(const fmatvec::VecV &arg1, const double &arg2) {return (*f)(arg2); }
        typename fmatvec::Der<fmatvec::RotMat3, fmatvec::VecV>::type parDer1(const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Mat3xV(); }
        typename fmatvec::Der<fmatvec::RotMat3, double>::type parDer2(const fmatvec::VecV &arg1, const double &arg2) {return f->parDer(arg2); }
        typename fmatvec::Der<typename fmatvec::Der<fmatvec::RotMat3, double>::type, double>::type parDer2ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return f->parDerParDer(arg2); }
        typename fmatvec::Der<fmatvec::RotMat3, double>::type parDer2DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Vec3(); }
        typename fmatvec::Der<typename fmatvec::Der<fmatvec::RotMat3, fmatvec::VecV>::type, double>::type parDer1ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Mat3xV(); }
        typename fmatvec::Der<fmatvec::RotMat3, fmatvec::VecV>::type parDer1DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return fmatvec::Mat3xV(); }
    };


  template<typename Sig> class NestedFunction; 

  template<typename Ret, typename Argo, typename Argi> 
    class NestedFunction<Ret(Argo(Argi))> : public fmatvec::Function<Ret(Argi)> {
      public:
       NestedFunction(fmatvec::Function<Ret(Argo)> *fo_=0, fmatvec::Function<Argo(Argi)> *fi_=0) : fo(fo_), fi(fi_) { }
       typename fmatvec::Size<Argi>::type getArgSize() const { return fi->getArgSize();}
        Ret operator()(const Argi &arg) {return (*fo)((*fi)(arg));}
        typename fmatvec::Der<Ret, Argi>::type parDer(const Argi &arg) { return fo->parDer((*fi)(arg))*fi->parDer(arg); }
        typename fmatvec::Der<Ret, Argi>::type parDerDirDer(const Argi &argDir, const Argi &arg) { return fo->parDerDirDer(fi->parDer(arg)*argDir,(*fi)(arg))*fi->parDer(arg) + fo->parDer((*fi)(arg))*fi->parDerDirDer(argDir,arg); }
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"outerFunction");
          fo=ObjectFactory<fmatvec::FunctionBase>::create<fmatvec::Function<Ret(Argo)> >(e->FirstChildElement());
          fo->initializeUsingXML(e->FirstChildElement());
          e=element->FirstChildElement(MBSIMNS"innerFunction");
          fi=ObjectFactory<fmatvec::FunctionBase>::create<fmatvec::Function<Argo(Argi)> >(e->FirstChildElement());
          fi->initializeUsingXML(e->FirstChildElement());
        }
        MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; } 
      private:
        fmatvec::Function<Ret(Argo)> *fo;
        fmatvec::Function<Argo(Argi)> *fi;
    };

  template<typename Ret, typename Argo> 
    class NestedFunction<Ret(Argo(double))> : public fmatvec::Function<Ret(double)> {
      public:
       NestedFunction(fmatvec::Function<Ret(Argo)> *fo_=0, fmatvec::Function<Argo(double)> *fi_=0) : fo(fo_), fi(fi_) { }
       typename fmatvec::Size<double>::type getArgSize() const { return fi->getArgSize();}
        Ret operator()(const double &arg) {return (*fo)((*fi)(arg));}
        typename fmatvec::Der<Ret, double>::type parDer(const double &arg) { return fo->parDer((*fi)(arg))*fi->parDer(arg); }
        typename fmatvec::Der<Ret, double>::type parDerDirDer(const double &argDir, const double &arg) { return fo->parDerDirDer(fi->parDer(arg)*argDir,(*fi)(arg))*fi->parDer(arg) + fo->parDer((*fi)(arg))*fi->parDerDirDer(argDir,arg); }
//        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &arg) { return fo->parDerParDer((*fi)(arg))*fi->parDer(arg)*fi->parDer(arg) + fo->parDer((*fi)(arg))*fi->parDerParDer(arg); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &arg) { return fo->parDerDirDer(fi->parDer(arg),(*fi)(arg))*fi->parDer(arg) + fo->parDer((*fi)(arg))*fi->parDerParDer(arg); }
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"outerFunction");
          fo=ObjectFactory<fmatvec::FunctionBase>::create<fmatvec::Function<Ret(Argo)> >(e->FirstChildElement());
          fo->initializeUsingXML(e->FirstChildElement());
          e=element->FirstChildElement(MBSIMNS"innerFunction");
          fi=ObjectFactory<fmatvec::FunctionBase>::create<fmatvec::Function<Argo(double)> >(e->FirstChildElement());
          fi->initializeUsingXML(e->FirstChildElement());
        }
        MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; } 
      private:
        fmatvec::Function<Ret(Argo)> *fo;
        fmatvec::Function<Argo(double)> *fi;
    };


  /**
   * \brief function describing a linear relationship between the input relative distance / velocity and the output for a spring
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \todo put in function_library TODO
   */
  class LinearSpringDamperForce : public fmatvec::Function<double(double,double)> {
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
      virtual double operator()(const double& g, const double& gd) { return c*(g-l0) + d*gd; }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
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
  class NonlinearSpringDamperForce : public fmatvec::Function<double(double,double)> {
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
      NonlinearSpringDamperForce(Function<fmatvec::Vec(double)> * gForceFun_, Function<fmatvec::Vec(double)> * gdForceFun_) : gForceFun(gForceFun_), gdForceFun(gdForceFun_) {}

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const double& g, const double& gd) { return (*gForceFun)(g)(0) + (*gdForceFun)(gd)(0); }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setParameters(Function<fmatvec::Vec(double)> * gForceFun_, Function<fmatvec::Vec(double)> * gdForceFun_) { gForceFun=gForceFun_; gdForceFun=gdForceFun_; }
      /***************************************************/

    protected:
      /**
       * \brief distance depending force function
       */
      Function<fmatvec::Vec(double)> * gForceFun;

      /**
       * \brief relative velocity depending force function
       */
      Function<fmatvec::Vec(double)> * gdForceFun;
  };

  /*! 
   * \brief function describing a linear relationship between the input relative distance / velocity and the output for a unilateral constraint
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \todo put in function_library TODO
   */
  class LinearRegularizedUnilateralConstraint: public fmatvec::Function<double(double,double)> {
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
      virtual double operator()(const double& g, const double& gd) { 
        if(g>0)
          return 0;
        else if(gd<0) 
          return -c*g - d*gd;
        else
          return -c*g;
      }
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
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
  class LinearRegularizedBilateralConstraint: public fmatvec::Function<double(double,double)> {
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
      virtual double operator()(const double& g, const double& gd) { 
        return -c*g - d*gd;
      }
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);

      virtual std::string getType() const { return "LinearRegularizedBilateralConstraint"; }
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
  class LinearRegularizedCoulombFriction : public fmatvec::Function<fmatvec::Vec(fmatvec::Vec,double)> {
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
      virtual fmatvec::Vec operator()(const fmatvec::Vec &gd, const double& laN);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
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
  class LinearRegularizedStribeckFriction : public fmatvec::Function<fmatvec::Vec(fmatvec::Vec,double)> {
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
      LinearRegularizedStribeckFriction(fmatvec::Function<double(double)> *fmu_, double gdLim_=0.01) : fmu(fmu_), gdLim(gdLim_) {}

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual fmatvec::Vec operator()(const fmatvec::Vec &gd, const double& laN);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setFrictionFunction(fmatvec::Function<double(double)> *fmu_) { fmu=fmu_; }
      void setMarginalVelocity(double gdLim_) { gdLim=gdLim_; }
      /***************************************************/

    protected:
      /**
       * \brief friction coefficient function
       */
      Function<double(double)> *fmu;
      
      /**
       * \brief border with respect to the relative velocity for the linear regularized increase of the friction force
       */
      double gdLim;
  };

  /**
   * \brief function describing the influence between the deformations on a body
   */
  class InfluenceFunction : public fmatvec::Function<double(fmatvec::Vec2,fmatvec::Vec2)> {
    public:
      InfluenceFunction(){}
      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const fmatvec::Vec2& firstContourLagrangeParameter, const fmatvec::Vec2& secondContourLagrangeParameter)=0;
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/
  };

  /*
   * \brief Influence function for flexibility of contour with no influence to other contours (or contour points)
   */
  class FlexibilityInfluenceFunction : public InfluenceFunction {
    public:
      FlexibilityInfluenceFunction() : flexibility(0) {
      }
      FlexibilityInfluenceFunction(const std::string& ContourName_, const double & flexibility_) :
          flexibility(flexibility_) {
      }
      virtual ~FlexibilityInfluenceFunction() {}
      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const fmatvec::Vec2& firstContourLagrangeParameter, const fmatvec::Vec2& secondContourLagrangeParameter) {
        if(nrm2(firstContourLagrangeParameter - secondContourLagrangeParameter) < macheps())
          return flexibility;
        else
          return 0;
      }
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

    protected:
      double flexibility;
  };

  /*
   * \brief a class for Influence-Functions with constant coupling
   */
  class ConstantInfluenceFunction : public InfluenceFunction {
    public:
      ConstantInfluenceFunction() : couplingValue(0) {
    }
      ConstantInfluenceFunction(const double & couplingValue_) :
          couplingValue(couplingValue_) {
      }
      virtual ~ConstantInfluenceFunction() {}
      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const fmatvec::Vec2& firstContourLagrangeParameter, const fmatvec::Vec2& secondContourLagrangeParameter) {
        return couplingValue;
      }
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

    protected:
      double couplingValue;
  };

  class Function_SS_from_VS : public fmatvec::Function<double(double)> {
    public:
      Function_SS_from_VS() : fun(NULL) {}
      Function_SS_from_VS(fmatvec::Function<fmatvec::Vec(double)> * fun_) : fun(fun_) {assert((*fun)(0).size()==1); }
      void setFunction(fmatvec::Function<fmatvec::Vec(double)> * fun_) {fun=fun_; assert((*fun)(0).size()==1); }
      double operator()(const double& x) {return (*fun)(x)(0); }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      Function<fmatvec::Vec(double)> * fun;
  };

  template<class Col>
  class Function_VS_from_SS : public fmatvec::Function<fmatvec::Vector<Col,double>(double)> {
    public:
      Function_VS_from_SS() : fun(NULL), vec(0) {}
      Function_VS_from_SS(fmatvec::Function<double(double)> * fun_, fmatvec::Vector<Col,double> v) : fun(fun_), vec(v) {vec/=nrm2(v); }
      void setFunction(fmatvec::Function<double(double)> * fun_) {fun=fun_; }
      void setVector(fmatvec::Vector<Col,double> v) {vec=v; vec/=nrm2(v); }
      fmatvec::Vector<Col,double> operator()(const double& x) {return (*fun)(x)*vec; }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    private:
      fmatvec::Function<double(double)> * fun;
      fmatvec::Vector<Col,double> vec;
  };

  class TabularFunction_SSS: public fmatvec::Function<double(double,double)> {
    public:
      TabularFunction_SSS();
      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual double operator()(const double& x, const double& y);
      /***************************************************/
      /* GETTER / SETTER */
      void setXValues(fmatvec::Vec x_);
      void setYValues(fmatvec::Vec y_);
      void setXYMat(fmatvec::Mat XY_);
      /***************************************************/

    private:
      fmatvec::Vec xVec;
      fmatvec::Vec yVec;
      fmatvec::Mat XY;

      int xSize;
      int ySize;
      int x0Index,x1Index;
      int y0Index,y1Index;

      fmatvec::Vec func_value;
      fmatvec::Vec xy;
      fmatvec::Vec XYval;
      fmatvec::Mat XYfac;

      void calcIndex(const double * x, fmatvec::Vec X, int * xSize, int * xIndexMinus, int * xIndexPlus);
  };

  template<class Col>
  void Function_VS_from_SS<Col>::initializeUsingXML(MBXMLUtils::TiXmlElement * element) {
    MBXMLUtils::TiXmlElement * e;
    e=element->FirstChildElement(MBSIMNS"function");
    fmatvec::Function<double(double)> * f=ObjectFactory<fmatvec::FunctionBase>::create<fmatvec::Function<double(double)> >(e->FirstChildElement());
    f->initializeUsingXML(e->FirstChildElement());
    setFunction(f);
    e=element->FirstChildElement(MBSIMNS"direction");
    setVector(Element::getVec(e));
  }

  template <class Arg>
    class ToDouble {
    };

  template <>
    class ToDouble<double> {
      public:
        static double cast(const double &x) {
          return x;
        }
    };

  template <class Col>
    class ToDouble<fmatvec::Vector<Col,double> > {
      public:
        static double cast(const fmatvec::Vector<Col,double> &x) {
          return x.e(0); 
        }
    };

  template <class Ret>
  class FromMatStr {
    public:
      static Ret cast(const char *x) {
//        throw std::runtime_error("FromMatStr::cast not implemented for current type.");
        return Ret(x);
      }
  };

  template <>
  class FromMatStr<double> {
    public:
      static double cast(const char *x) {
        return atof(x);
      }
  };

  template<typename Sig> class ConstantFunction;

  template<typename Ret, typename Arg>
    class ConstantFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
      protected:
        Ret a;
      public:
        ConstantFunction() {}
        ConstantFunction(const Ret &a_) : a(a_) {}
        Ret operator()(const Arg &arg) { return a; }
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"value");
          a=FromMatStr<Ret>::cast(e->GetText());
        }
        MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; } 
    };

  template<typename Ret, typename Arg1, typename Arg2>
    class ConstantFunction<Ret(Arg1,Arg2)> : public fmatvec::Function<Ret(Arg1,Arg2)> {
      protected:
        Ret a;
      public:
        ConstantFunction() {}
        ConstantFunction(const Ret &a_) : a(a_) {}
        Ret operator()(const Arg1 &arg1, const Arg2 &arg2) { return a; }
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"value");
          a=FromMatStr<Ret>::cast(e->GetText());
        }
        MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; } 
    };

  template<typename Sig> class LinearFunction;

  template<typename Ret, typename Arg>
    class LinearFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
      private:
        typename fmatvec::Der<Ret, Arg>::type A;
        Ret b;
      public:
        LinearFunction() { }
        LinearFunction(const typename fmatvec::Der<Ret, Arg>::type &A_, const Ret &b_) : A(A_), b(b_) {}
        LinearFunction(const typename fmatvec::Der<Ret, Arg>::type &A_) : A(A_), b(A.rows()) {}
        typename fmatvec::Size<Arg>::type getArgSize() const { return A.cols(); }
        Ret operator()(const Arg &arg) { return A*arg+b; }
        typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &arg) { return A; }
        typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &arg1Dir, const Arg &arg1) { return typename fmatvec::Der<Ret, Arg>::type(A.rows(),A.cols()); }
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"slope");
          A=FromMatStr<typename fmatvec::Der<Ret, Arg>::type>::cast(e->GetText());
          e=element->FirstChildElement(MBSIMNS"intercept");
          if(e) b=FromMatStr<Ret>::cast(e->GetText());
        }
        MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; } 
        void setSlope(const typename fmatvec::Der<Ret, Arg>::type &A_) { A = A_; }
        void setIntercept(const Ret &b_) { b = b_; }
    };

  template<>
    class LinearFunction<double(double)> : public fmatvec::Function<double(double)> {
      private:
        typename fmatvec::Der<double, double>::type A;
        double b;
      public:
        LinearFunction() {}
        LinearFunction(const typename fmatvec::Der<double, double>::type &A_, const double &b_) : A(A_), b(b_) {}
        typename fmatvec::Size<double>::type getArgSize() const { return 1; }
        double operator()(const double &arg) { return A*arg+b; }
        typename fmatvec::Der<double, double>::type parDer(const double &arg) { return A; }
        typename fmatvec::Der<double, double>::type parDerDirDer(const double &arg1Dir, const double &arg1) { return 0; }
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"slope");
          A=Element::getDouble(e);
          e=element->FirstChildElement(MBSIMNS"intercept");
          b=Element::getDouble(e);
        }
        MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; } 
    };

 template<class Col>
  class QuadraticFunction : public fmatvec::Function<fmatvec::Vector<Col,double>(double)> {
    private:
       fmatvec::Vector<Col,double> a0, a1, a2;
    public:
      QuadraticFunction() { }
      QuadraticFunction(const fmatvec::Vector<Col,double> &a0_, const fmatvec::Vector<Col,double> &a1_, const fmatvec::Vector<Col,double> &a2_) : a0(a0_), a1(a1_), a2(a2_) { }
      fmatvec::Vector<Col,double> operator()(const double &x) {  
        fmatvec::Vector<Col,double> y(a0.size(), fmatvec::NONINIT);
        for (int i=0; i<y.size(); i++)
          y(i)=a0(i)+(a1(i)+a2(i)*x)*x;
        return y;
      }
      fmatvec::Vector<Col,double> parDer(const double &x) {  
        fmatvec::Vector<Col,double> y(a0.size(), fmatvec::NONINIT);
        for (int i=0; i<y.size(); i++)
          y(i)=a1(i)+2.*a2(i)*x;
        return y;
      }
      fmatvec::Vector<Col,double> parDerParDer(const double &x) {  
        fmatvec::Vector<Col,double> y(a0.size(), fmatvec::NONINIT);
        for (int i=0; i<y.size(); i++)
          y(i)=2.*a2(i);
        return y;
      }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
        MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"a0");
        a0=Element::getVec(e);
        e=element->FirstChildElement(MBSIMNS"a1");
        a1=Element::getVec(e, a0.size());
        e=element->FirstChildElement(MBSIMNS"a2");
        a2=Element::getVec(e, a0.size());
      }
  };

  template<class Col>
  class SinusFunction : public fmatvec::Function<fmatvec::Vector<Col,double>(double)> {
    protected:
      fmatvec::Vector<Col,double> A, f, phi0, y0;
    public:
      SinusFunction() { }
      SinusFunction(const fmatvec::Vector<Col,double> &A_, const fmatvec::Vector<Col,double> &f_, const fmatvec::Vector<Col,double> &phi0_, const fmatvec::Vector<Col,double> &y0_) : A(A_), f(f_), phi0(phi0_), y0(y0_) { }
      fmatvec::Vector<Col,double> operator()(const double &x) {  
        fmatvec::Vector<Col,double> y(A.size(), fmatvec::NONINIT);
        for (int i=0; i<y.size(); i++)
          y(i)=y0(i)+A(i)*sin(2.*M_PI*f(i)*x+phi0(i));
        return y;
      }
      fmatvec::Vector<Col,double> parDer(const double &x) {  
        fmatvec::Vector<Col,double> y(A.size(), fmatvec::NONINIT);
        for (int i=0; i<y.size(); i++) {
          double om = 2.*M_PI*f(i);
          y(i)=A(i)*om*cos(om*x+phi0(i));
        }
        return y;
      }
      fmatvec::Vector<Col,double> parDerParDer(const double &x) {  
        fmatvec::Vector<Col,double> y(A.size(), fmatvec::NONINIT);
        for (int i=0; i<y.size(); i++) {
          double om = 2.*M_PI*f(i);
          y(i)=-A(i)*om*om*sin(om*x+phi0(i));
        }
        return y;
      }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
        MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"amplitude");
        A=Element::getVec(e);
        e=element->FirstChildElement(MBSIMNS"frequency");
        f=Element::getVec(e, A.size());
        e=element->FirstChildElement(MBSIMNS"phase");
        phi0=Element::getVec(e, A.size());
        e=element->FirstChildElement(MBSIMNS"offset");
        if (e)
          y0=Element::getVec(e, A.size());
        else
          y0=fmatvec::Vector<Col,double>(A.size());
      }
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) {
        MBXMLUtils::TiXmlElement *ele0 = fmatvec::Function<fmatvec::Vector<Col,double>(double)>::writeXMLFile(parent);
        addElementText(ele0,MBSIMNS"amplitude",A);
        addElementText(ele0,MBSIMNS"frequency",f);
        addElementText(ele0,MBSIMNS"phase",phi0);
        addElementText(ele0,MBSIMNS"offset",y0);
        return ele0;
      }
  };

  template<class Col>
  class PositiveSinusFunction : public SinusFunction<Col> {
    public:
      PositiveSinusFunction() { }
      PositiveSinusFunction(const fmatvec::Vector<Col,double> &A, const fmatvec::Vector<Col,double> &f, const fmatvec::Vector<Col,double> &phi0, const fmatvec::Vector<Col,double> &y0) : SinusFunction<Col>(A, f, phi0, y0) { }
      fmatvec::Vector<Col,double> operator()(const double &x) {
        fmatvec::Vector<Col,double> y=SinusFunction<Col>::operator()(x);
        for (int i=0; i<SinusFunction<Col>::A.size(); i++)
          if (y(i)<0)
            y(i)=0;
        return y;
      }
  };

   template<class Col>
   class StepFunction : public fmatvec::Function<fmatvec::Vector<Col,double>(double)> {
   private:
      fmatvec::Vector<Col,double> stepTime, stepSize;
    public:
      StepFunction() {}
      StepFunction(const fmatvec::Vector<Col,double> &stepTime_, const fmatvec::Vector<Col,double> &stepSize_) : stepTime(stepTime_), stepSize(stepSize_) { }
      fmatvec::Vector<Col,double> operator()(const double &x) {
        fmatvec::Vector<Col,double> y(stepTime.size());
        for (int i=0; i<y.size(); i++)
          if (x>=stepTime(i))
            y(i)=stepSize(i);
        return y;
      }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
        MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"time");
        stepTime=Element::getVec(e);
        e=element->FirstChildElement(MBSIMNS"size");
        stepSize=Element::getVec(e);
      }
  };

  class SummationFunction : public fmatvec::Function<fmatvec::Vector<fmatvec::Ref,double>(double)> {
    public:
      SummationFunction() { }
      void addFunction(Function<fmatvec::Vector<fmatvec::Ref,double>(double)> *function, double factor=1.) {
        functions.push_back(function);
        factors.push_back(factor);
      }
      fmatvec::Vector<fmatvec::Ref,double> operator()(const double &x) {
        fmatvec::Vec y=factors[0]*(*(functions[0]))(x);
        for (unsigned int i=1; i<functions.size(); i++)
          y+=factors[i]*(*(functions[i]))(x);
        return y;
      }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
        MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"function");
        while (e && e->ValueStr()==MBSIMNS"function") {
          MBXMLUtils::TiXmlElement *ee = e->FirstChildElement();
          typedef Function<fmatvec::Vector<fmatvec::Ref,double>(double)> typevec;
          Function<fmatvec::Vector<fmatvec::Ref,double>(double)> *f=ObjectFactory<fmatvec::FunctionBase>::create<Function<fmatvec::Vector<fmatvec::Ref,double>(double)> >(ee);
          f->initializeUsingXML(ee);
          ee=e->FirstChildElement(MBSIMNS"factor");
          double factor=Element::getDouble(ee);
          addFunction(f, factor);
          e=e->NextSiblingElement();
        }
      }
    private:
      std::vector<Function<fmatvec::Vector<fmatvec::Ref,double>(double)> *> functions;
      std::vector<double> factors;
  };

    class Polynom : public fmatvec::Function<double(double)> {
      protected:
       std::vector<Function<double(double)>* > derivatives;
       void addDerivative(Function<double(double)>* diff) { derivatives.push_back(diff); }
      public:
        Polynom() { }

        double operator()(const double &x) { return (*derivatives[0])(x); }
        class Polynom_Evaluation : public Function<double(double)> {
          public:
            Polynom_Evaluation(const fmatvec::Vec &a_) : a(a_) {}
            double operator()(const double &x) {
              double value=a(a.size()-1);
              for (int i=a.size()-2; i>-1; i--)
                value=value*x+a(i);
              return value;
            }
        private:
            fmatvec::Vec a;
        };

        void setCoefficients(fmatvec::Vec a) {
          for (int i=0; i<a.size(); i++) {
            addDerivative(new Polynom::Polynom_Evaluation(a.copy()));
            fmatvec::Vec b(a.size()-1);
            for (int j=0; j<b.size(); j++)
              b(j)=a(j+1)*(j+1.);
            a.resize(a.size()-1);
            a=b;
          }
          addDerivative(new Polynom::Polynom_Evaluation(a.copy()));
        }

        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"coefficients");
          setCoefficients(MBSim::Element::getVec(e));
        }
    };

  template<class Row, class Col>
  class TabularFunction : public fmatvec::Function<fmatvec::Vector<Col,double>(double)> {
    public:
      TabularFunction() : xIndexOld(0) {}
      TabularFunction(const fmatvec::Vector<Row,double> &x_, const fmatvec::Matrix<fmatvec::General,Row,Col,double> &y_) : x(x_), y(y_), xIndexOld(0) {
        check();
      }
      fmatvec::Vector<Col,double> operator()(const double& xVal);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    protected:
      fmatvec::Vector<Row,double> x;
      fmatvec::Matrix<fmatvec::General,Row,Col,double> y;
    private:
      int xIndexOld, xSize;
      void check();
  };

  class PeriodicTabularFunction : public TabularFunction<fmatvec::Ref,fmatvec::Ref> {
    public:
      PeriodicTabularFunction() {}
      PeriodicTabularFunction(fmatvec::Vec x_, fmatvec::Mat y_) : TabularFunction<fmatvec::Ref,fmatvec::Ref>(x_, y_) {
        check();
      }
      fmatvec::Vec operator()(const double& xVal);
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
        TabularFunction<fmatvec::Ref,fmatvec::Ref>::initializeUsingXML(element);
        check();
      }
    private:
      double xMin, xMax, xDelta;
      void check() {
        xMin=x(0);
        xMax=x(x.size()-1);
        xDelta=xMax-xMin;
      }
  };

  template<class Arg> 
    class RotationAboutXAxis : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Vec3 a;
      public:
        RotationAboutXAxis() { a(0) = 1; A(0,0) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 1;
        }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double alpha = ToDouble<Arg>::cast(q);
          const double cosq=cos(alpha);
          const double sinq=sin(alpha);
          A(1,1) = cosq;
          A(2,1) = sinq;
          A(1,2) = -sinq;
          A(2,2) = cosq;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          return a;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          return typename fmatvec::Der<fmatvec::RotMat3, Arg>::type(1);
        }
        const fmatvec::Vec3& getAxisOfRotation() const { return a; }
        void setAxisOfRotation(const fmatvec::Vec3 &a_) { a = a_; }
    };

  template<class Arg> 
    class RotationAboutZAxis : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Vec3 a;
      public:
        RotationAboutZAxis() { a(2) = 1; A(2,2) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 1;
        }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double alpha = ToDouble<Arg>::cast(q);
          const double cosq=cos(alpha);
          const double sinq=sin(alpha);
          A(0,0) = cosq;
          A(1,0) = sinq;
          A(0,1) = -sinq;
          A(1,1) = cosq;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          return a;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          return typename fmatvec::Der<fmatvec::RotMat3, Arg>::type(1);
        }
        const fmatvec::Vec3& getAxisOfRotation() const { return a; }
        void setAxisOfRotation(const fmatvec::Vec3 &a_) { a = a_; }
    };

  template<class Arg> 
    class RotationAboutFixedAxis : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Vec3 a;
      public:
        RotationAboutFixedAxis() { }
        RotationAboutFixedAxis(const fmatvec::Vec3 &a_) : a(a_) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 1;
        }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double alpha = ToDouble<Arg>::cast(q);
          const double cosq=cos(alpha);
          const double sinq=sin(alpha);
          const double onemcosq=1-cosq;
          const double a0a1=a.e(0)*a.e(1);
          const double a0a2=a.e(0)*a.e(2);
          const double a1a2=a.e(1)*a.e(2);
          A.e(0,0) = cosq+onemcosq*a.e(0)*a.e(0);
          A.e(1,0) = onemcosq*a0a1+a.e(2)*sinq;
          A.e(2,0) = onemcosq*a0a2-a.e(1)*sinq;
          A.e(0,1) = onemcosq*a0a1-a.e(2)*sinq;
          A.e(1,1) = cosq+onemcosq*a.e(1)*a.e(1);
          A.e(2,1) = onemcosq*a1a2+a.e(0)*sinq;
          A.e(0,2) = onemcosq*a0a2+a.e(1)*sinq;
          A.e(1,2) = onemcosq*a1a2-a.e(0)*sinq;
          A.e(2,2) = cosq+onemcosq*a.e(2)*a.e(2);
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          return a;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          return typename fmatvec::Der<fmatvec::RotMat3, Arg>::type(1);
        }
        const fmatvec::Vec3& getAxisOfRotation() const { return a; }
        void setAxisOfRotation(const fmatvec::Vec3 &a_) { a = a_; }
        void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
          MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"axisOfRotation");
          a=FromMatStr<fmatvec::Vec3>::cast(e->GetText());
        }
        MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; } 
    };

  template<class Arg> 
    class RotationAboutAxesXY : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        RotationAboutAxesXY() : J(2), Jd(2) { J.e(0,0) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 2;
        }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double a=q(0);
          double b=q(1);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          double sinb = sin(b);

          A.e(0,0) = cosb;
          A.e(1,0) = sina*sinb;
          A.e(2,0) = -cosa*sinb;
          A.e(1,1) = cosa;
          A.e(2,1) = sina;
          A.e(0,2) = sinb;
          A.e(1,2) = -sina*cosb;
          A.e(2,2) = cosa*cosb;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          double a = q(0);
          J.e(1,1) = cos(a);
          J.e(2,1) = sin(a);
          return J;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          double a = q(0);
          double ad = qd(0);
          double bd = qd(1);
          Jd.e(1) = -sin(a)*ad*bd;
          Jd.e(2) = cos(a)*ad*bd;
          return Jd;
        }
    };

  template<class Arg> 
    class RotationAboutAxesXYZ : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        RotationAboutAxesXYZ() : J(3), Jd(3) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double a=q.e(0);
          double b=q.e(1);
          double g=q.e(2);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          double sinb = sin(b);
          double cosg = cos(g);
          double sing = sin(g);
          A.e(0,0) = cosb*cosg;
          A.e(1,0) = sina*sinb*cosg+cosa*sing;
          A.e(2,0) = -cosa*sinb*cosg+sina*sing;
          A.e(0,1) = -cosb*sing;
          A.e(1,1) = -sing*sinb*sina+cosa*cosg;
          A.e(2,1) = cosa*sinb*sing+sina*cosg;
          A.e(0,2) = sinb;
          A.e(1,2) = -sina*cosb;
          A.e(2,2) = cosa*cosb;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          double a = q.e(0);
          double b = q.e(1);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          J.e(0,0) = 1;
          //J.e(0,1) = 0;
          J.e(0,2) = sin(b);
          //J.e(1,0) = 0;
          J.e(1,1) = cosa;
          J.e(1,2) = -sina*cosb;
          //J.e(2,0) = 0;
          J.e(2,1) = sina;
          J.e(2,2) = cosa*cosb;
          return J;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          double a = q.e(0);
          double b = q.e(1);
          double ad = qd.e(0);
          double bd = qd.e(1);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          double sinb = sin(b);
          //Jd.e(0,0) = 0;
          //Jd.e(0,1) = 0;
          Jd.e(0,2) = cosb*bd;
          //Jd.e(1,0) = 0;
          Jd.e(1,1) = -sina*ad;
          Jd.e(1,2) = -cosa*cosb*ad + sina*sinb*bd;
          //Jd.e(2,0) = 0;
          Jd.e(2,1) = cosa*ad;
          Jd.e(2,2) = -sina*cosb*ad - cosa*sinb*bd;
          return Jd;
        }
    };
  
  template<class Arg> 
    class RotationAboutAxesZXZ : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        RotationAboutAxesZXZ() : J(3), Jd(3) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double psi=q.e(0);
          double theta=q.e(1);
          double phi=q.e(2);
          double spsi = sin(psi);
          double stheta = sin(theta);
          double sphi = sin(phi);
          double cpsi = cos(psi);
          double ctheta = cos(theta);
          double cphi = cos(phi);
          A.e(0,0) = cpsi*cphi-spsi*ctheta*sphi;
          A.e(1,0) = spsi*cphi+cpsi*ctheta*sphi;
          A.e(2,0) = stheta*sphi;
          A.e(0,1) = -cpsi*sphi-spsi*ctheta*cphi;
          A.e(1,1) = -spsi*sphi+cpsi*ctheta*cphi;
          A.e(2,1) = stheta*cphi;
          A.e(0,2) = spsi*stheta;
          A.e(1,2) = -cpsi*stheta;
          A.e(2,2) = ctheta;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          throw std::runtime_error("RotationAboutAxesZXZ::parDer() not yet implemented.");
          return J;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          throw std::runtime_error("RotationAboutAxesZXZ::parDerDirDer() not yet implemented.");
          return Jd;
        }
    };

  template<class Arg> 
    class TCardanAngles : public fmatvec::Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        TCardanAngles() : T(3,3,fmatvec::Eye()) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
        fmatvec::MatV operator()(const Arg &q) {
          double alpha = q.e(0);
          double beta = q.e(1);
          double cos_beta = cos(beta);
          double sin_beta = sin(beta);
          double cos_alpha = cos(alpha);
          double sin_alpha = sin(alpha);
          double tan_beta = sin_beta/cos_beta;
          T.e(0,1) = tan_beta*sin_alpha;
          T.e(0,2) = -tan_beta*cos_alpha;
          T.e(1,1) = cos_alpha;
          T.e(1,2) = sin_alpha;
          T.e(2,1) = -sin_alpha/cos_beta;
          T.e(2,2) = cos_alpha/cos_beta;
          return T;
        }
    };

  template<class Arg> 
    class TCardanAngles2 : public fmatvec::Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        TCardanAngles2() : T(3,3,fmatvec::Eye()) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
        fmatvec::MatV operator()(const Arg &q) {
          double beta = q.e(1);
          double gamma = q.e(2);
          double cos_beta = cos(beta);
          double sin_beta = sin(beta);
          double cos_gamma = cos(gamma);
          double sin_gamma = sin(gamma);
          double tan_beta = sin_beta/cos_beta;
          T.e(0,0) = cos_gamma/cos_beta;
          T.e(0,1) = -sin_gamma/cos_beta;
          T.e(1,0) = sin_gamma;
          T.e(1,1) = cos_gamma;
          T.e(2,0) = -cos_gamma*tan_beta;
          T.e(2,1) = sin_gamma*tan_beta;
          return T;
        }
    };

  template<class Arg> 
    class TEulerAngles : public fmatvec::Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        TEulerAngles() : T(3,3) { T(0,2) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
        fmatvec::MatV operator()(const Arg &q) {
          double psi = q(0);
          double theta = q(1);
          double cos_theta = cos(theta);
          double sin_theta = sin(theta);
          double cos_psi = cos(psi);
          double sin_psi = sin(psi);
          double tan_theta = sin_theta/cos_theta;

          T(0,0) = -sin_psi/tan_theta;
          T(0,1) = cos_psi/tan_theta;
          T(1,0) = cos_psi;
          T(1,1) = sin_psi;
          T(2,0) = sin_psi/sin_theta;
          T(2,1) = -cos_psi/sin_theta;

          return T;
        }
    };

  template<class Arg> 
    class TEulerAngles2 : public fmatvec::Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        TEulerAngles2() : T(3,3,fmatvec::Eye()) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
        fmatvec::MatV operator()(const Arg &q) {
          double theta = q(1);
          double phi = q(2);
          double cos_theta = cos(theta);
          double sin_theta = sin(theta);
          double cos_phi = cos(phi);
          double sin_phi = sin(phi);
          double tan_theta = sin_theta/cos_theta;

          T(0,0) = sin_phi/sin_theta;
          T(0,1) = cos_phi/sin_theta;
          T(1,0) = cos_phi;
          T(1,1) = -sin_phi;
          T(2,0) = -sin_phi/tan_theta;
          T(2,1) = -cos_phi/tan_theta;

          return T;
        }
    };

  template<class Row, class Col>
    void TabularFunction<Row,Col>::initializeUsingXML(MBXMLUtils::TiXmlElement * element) {
      MBXMLUtils::TiXmlElement *e=element->FirstChildElement(MBSIMNS"x");
      if (e) {
        fmatvec::Vector<Row,double> x_=Element::getVec(e);
        x=x_;
        e=element->FirstChildElement(MBSIMNS"y");
        fmatvec::Matrix<fmatvec::General,Row,Col,double> y_=Element::getMat(e, x.size(), 0);
        y=y_;
      }
      e=element->FirstChildElement(MBSIMNS"xy");
      if (e) {
        fmatvec::MatV xy=Element::getMat(e);
        assert(xy.cols()>1);
        x=xy.col(0);
        y=xy(fmatvec::Range<fmatvec::Var,fmatvec::Var>(0, xy.rows()-1), fmatvec::Range<fmatvec::Var,fmatvec::Var>(1, xy.cols()-1));
      }
      check();
    }

  template<class Row, class Col>
    fmatvec::Vector<Col,double> TabularFunction<Row,Col>::operator()(const double& xVal) {
      int i=xIndexOld;
      if (xVal<=x(0)) {
        xIndexOld=0;
        return trans(y.row(0));
      }
      else if (xVal>=x(xSize-1)) {
        xIndexOld=xSize-1;
        return trans(y.row(xSize-1));
      }
      else if (xVal<=x(i)) {
        while (xVal<x(i))
          i--;
      }
      else {
        do
          i++;
        while (xVal>x(i));
        i--;
      }
      xIndexOld=i;
      fmatvec::RowVector<Col,double> m=(y.row(i+1)-y.row(i))/(x(i+1)-x(i));
      return trans(y.row(i)+(xVal-x(i))*m);
    }

  template<class Row, class Col>
    void TabularFunction<Row,Col>::check() {
      for (int i=1; i<x.size(); i++)
        assert(x(i)>x(i-1));
      assert(x.size()==y.rows());
      xSize=x.size();
    }

}

#endif /* _FUNCTION_LIBRARY_H_ */

