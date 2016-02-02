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

#ifndef _KINETIC_FUNCTIONS_H_
#define _KINETIC_FUNCTIONS_H_

#include "mbsim/functions/function.h"
#include "mbsim/element.h"
#include "mbsim/utils/eps.h"

namespace MBSim {

  class Contour;
  class ContourPointData;

  /**
   * \brief function describing a linear relationship between the input relative distance / velocity and the output for a spring
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   */
  class LinearSpringDamperForce : public Function<double(double,double)> {
    public:
      /** 
       * \brief constructor
       */
      LinearSpringDamperForce() : l0(0) {}

      /** 
       * \brief constructor
       * \param stiffness
       * \param damping
       */
      LinearSpringDamperForce(double c_, double d_) : c(c_), d(d_), l0(0) {}

      /** 
       * \brief constructor
       * \param stiffness
       * \param damping
       * \param undeformed length
       */
      LinearSpringDamperForce(double c_, double d_, double l0_); 

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const double& g, const double& gd) { return c*(g-l0) + d*gd; }
      void initializeUsingXML(xercesc::DOMElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setStiffnessCoefficient(double c_) { c=c_; }
      void setDampingCoefficient(double d_) { d=d_; }
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
   */
  class NonlinearSpringDamperForce : public Function<double(double,double)> {
    public:
      /** 
       * \brief constructor
       */
      NonlinearSpringDamperForce() {}

      /** 
       * \brief destructor
       */
      ~NonlinearSpringDamperForce() {
        delete gForceFun;
        delete gdForceFun;
      }

      /** 
       * \brief constructor
       * \param distance depending force function
       * \param relative velocity depending force function
       */
      NonlinearSpringDamperForce(Function<double(double)> * gForceFun_, Function<double(double)> * gdForceFun_) : gForceFun(gForceFun_), gdForceFun(gdForceFun_) {
        gForceFun->setParent(this);
        gdForceFun->setParent(this);
      }

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const double& g, const double& gd) { return (*gForceFun)(g) + (*gdForceFun)(gd); }
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(Element::InitStage stage) {
        Function<double(double,double)>::init(stage);
        gForceFun->init(stage);
        gdForceFun->init(stage);
      }
      /***************************************************/

      /* GETTER / SETTER */
      void setDistanceFunction(Function<double(double)> * gForceFun_) {
        gForceFun=gForceFun_;
        gForceFun->setParent(this);
        gForceFun->setName("Distance");
      }

      void setVelocityFunction(Function<double(double)> * gdForceFun_) {
        gdForceFun=gdForceFun_;
        gdForceFun->setParent(this);
        gdForceFun->setName("Velocity");
      }
      /***************************************************/

    protected:
      /**
       * \brief distance depending force function
       */
      Function<double(double)> * gForceFun;

      /**
       * \brief relative velocity depending force function
       */
      Function<double(double)> * gdForceFun;
  };

  /*! 
   * \brief function describing a linear relationship between the input relative distance / velocity and the output for a unilateral constraint
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \todo put in function_library TODO
   */
  class LinearRegularizedUnilateralConstraint: public Function<double(double,double)> {
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
      virtual void initializeUsingXML(xercesc::DOMElement *element);
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
  class LinearRegularizedBilateralConstraint: public Function<double(double,double)> {
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
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent);

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
  class LinearRegularizedCoulombFriction : public Function<fmatvec::Vec(fmatvec::Vec,double)> {
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
      virtual void initializeUsingXML(xercesc::DOMElement *element);
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
   * \todo delete function pointer
   */
  class LinearRegularizedStribeckFriction : public Function<fmatvec::Vec(fmatvec::Vec,double)> {
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
      LinearRegularizedStribeckFriction(Function<double(double)> *fmu_, double gdLim_=0.01) : fmu(fmu_), gdLim(gdLim_) {
        fmu->setParent(this);
      }

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual fmatvec::Vec operator()(const fmatvec::Vec &gd, const double& laN);
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual void init(Element::InitStage stage) {
        Function<fmatvec::Vec(fmatvec::Vec,double)>::init(stage);
        fmu->init(stage);
      }
      /***************************************************/

      /* GETTER / SETTER */
      void setFrictionFunction(Function<double(double)> *fmu_) {
        fmu=fmu_;
        fmu->setParent(this);
        fmu->setName("Friction");
      }
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
  class InfluenceFunction : public Function<double(std::pair<Contour*, ContourPointData>, std::pair<Contour*, ContourPointData>)> {
    public:
      InfluenceFunction(){}
      virtual ~InfluenceFunction() {}
      /* INHERITED INTERFACE OF FUNCTION2 */
      void setTime(double t_) { t = t_; }
      virtual double operator()(const std::pair<Contour*, ContourPointData>& firstContourInfo, const std::pair<Contour*, ContourPointData>& secondContourInfo)=0;
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      /***************************************************/
    protected:
      double t;
      fmatvec::Vec2 getContourParameters(double t, const std::pair<Contour*, ContourPointData>& contourInfo);
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
      virtual double operator()(const std::pair<Contour*, ContourPointData>& firstContourInfo, const std::pair<Contour*, ContourPointData>& secondContourInfo) {
        if(nrm2(getContourParameters(t,firstContourInfo)- getContourParameters(t,secondContourInfo)) < macheps())
          return flexibility;
        else
          return 0;
      }
      virtual void initializeUsingXML(xercesc::DOMElement *element);
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
      virtual double operator()(const std::pair<Contour*, ContourPointData>& firstContourInfo, const std::pair<Contour*, ContourPointData>& secondContourInfo) {
        return couplingValue;
      }
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      /***************************************************/

    protected:
      double couplingValue;
  };

}

#endif
