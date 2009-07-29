/* Copyright (C) 2004-2009 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _CONSTITUTIVE_LAWS_H_
#define _CONSTITUTIVE_LAWS_H_

#include "fmatvec.h"
#include "mbsim/userfunction.h"
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"
#include <fstream>

namespace MBSim {

  /**
   * \brief basic force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class GeneralizedForceLaw {
    public:
      /**
       * \brief constructor
       */
      GeneralizedForceLaw() {};

      /**
       * \brief destructor
       */
      virtual ~GeneralizedForceLaw() {};

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \param relative distance
       * \param tolerance
       * \return flag, if force law is active
       */
      virtual bool isActive(double g, double gTol) { return true; }
      virtual bool remainsActive(double s, double sTol) { return true; }
      virtual double project(double la, double gdn, double r) { return 0; }
      virtual fmatvec::Vec diff(double la, double gdn, double r) { return fmatvec::Vec(2); }
      virtual double solve(double G, double gdn) { return 0; }

      /**
       * \param contact force parameter
       * \param contact relative velocity
       * \param tolerance for contact force parameters
       * \param tolerance for relative velocity
       * \return flag if the force law is valid given the parameters
       */
      virtual bool isFulfilled(double la,  double gdn, double tolla, double tolgd) { return true; }

      /**
       * \param relative distance
       * \param relative velocity
       * \return force law value
       */
      virtual double operator()(double g,  double gd) { return 0; }

      /**
       * \return flag if the force law is setvalued
       */
      virtual bool isSetValued() const = 0;

      /**
       * \brief initialize the force law using XML
       * \param XML element
       */
      virtual void initializeUsingXML(TiXmlElement *element) {}
      /***************************************************/
  };

  /**
   * \brief basic unilateral force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class UnilateralConstraint : public GeneralizedForceLaw {
    public:
      /**
       * \brief constructor
       */
      UnilateralConstraint() {};

      /**
       * \brief destructor
       */
      virtual ~UnilateralConstraint() {};

      /* INHERITED INTERFACE */
      virtual bool isActive(double g, double gTol) { return g<=gTol; }
      virtual double project(double la, double gdn, double r);
      virtual fmatvec::Vec diff(double la, double gdn, double r);
      virtual double solve(double G, double gdn);
      virtual bool isFulfilled(double la,  double gdn, double tolla, double tolgd);
      virtual bool isSetValued() const { return true; }
      /***************************************************/

      bool remainsClosed(double s, double sTol) { return s<=sTol; }  // s = gd/gdd
  };

  /**
   * \brief basic bilateral force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class BilateralConstraint : public GeneralizedForceLaw {
    public:
      /**
       * \brief constructor
       */
      BilateralConstraint() {};

      /**
       * \brief destructor
       */
      virtual ~BilateralConstraint() {};

      /* INHERITED INTERFACE */
      virtual bool isActive(double g, double gTol) { return true; }
      virtual double project(double la, double gdn, double r);
      virtual fmatvec::Vec diff(double la, double gdn, double r);
      virtual double solve(double G, double gdn);
      virtual bool isFulfilled(double la,  double gdn, double tolla, double tolgd);
      virtual bool isSetValued() const { return true; }
      /***************************************************/

      bool remainsClosed(double s, double sTol) { return true; }
  };

  /**
   * \brief basic force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class GeneralizedImpactLaw {
    public:
      /**
       * \brief constructor
       */
      GeneralizedImpactLaw() {};

      /**
       * \brief destructor
       */
      virtual ~GeneralizedImpactLaw() {};

      /* INTERFACE FOR DERIVED CLASSES */
      virtual double project(double la, double gdn, double gda, double r) = 0;
      virtual fmatvec::Vec diff(double la, double gdn, double gda, double r) = 0;
      virtual double solve(double G, double gdn, double gda) = 0;
      virtual bool isFulfilled(double la,  double gdn, double gda, double tolla, double tolgd) = 0;
      virtual void initializeUsingXML(TiXmlElement *element) {}
      /***************************************************/
  };

  /**
   * \brief basic unilateral force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class UnilateralNewtonImpact : public GeneralizedImpactLaw {
    public:
      /**
       * \brief constructor
       */
      UnilateralNewtonImpact() : epsilon(0), gd_limit(1e-2) {};

      /**
       * \brief constructor
       */
      UnilateralNewtonImpact(double epsilon_) : epsilon(epsilon_), gd_limit(1e-2) {};

      /**
       * \brief constructor
       */
      UnilateralNewtonImpact(double epsilon_, double gd_limit_) : epsilon(epsilon_), gd_limit(gd_limit_) {};

      /**
       * \brief destructor
       */
      virtual ~UnilateralNewtonImpact() {};

      /* INHERITED INTERFACE */
      virtual double project(double la, double gdn, double gda, double r);
      virtual fmatvec::Vec diff(double la, double gdn, double gda, double r);
      virtual double solve(double G, double gdn, double gda);
      virtual bool isFulfilled(double la,  double gdn, double gda, double tolla, double tolgd);
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

    protected:
      double epsilon, gd_limit;
  };

  /**
   * \brief basic bilateral force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class BilateralImpact : public GeneralizedImpactLaw {
    public:
      /**
       * \brief constructor
       */
      BilateralImpact() {};

      /**
       * \brief destructor
       */
      virtual ~BilateralImpact() {};

      /* INHERITED INTERFACE */
      virtual double project(double la, double gdn, double gda, double r);
      virtual fmatvec::Vec diff(double la, double gdn, double gda, double r);
      virtual double solve(double G, double gdn, double gda);
      virtual bool isFulfilled(double la,  double gdn, double gda, double tolla, double tolgd);
      /***************************************************/
  };

  /**
   * \brief basic friction force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class FrictionForceLaw {
    public:
      /**
       * \brief constructor
       */
      FrictionForceLaw(double gdLim_ = 0.01) : gdLim(gdLim_) {};

      /**
       * \brief destructor
       */
      virtual ~FrictionForceLaw() {};

      /* INTERFACE FOR DERIVED CLASSES */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r) { return fmatvec::Vec(2); }
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r) { return fmatvec::Mat(2,2); }
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, double laN) { return fmatvec::Vec(2); }
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double tolla, double tolgd) { return true; }
      virtual fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd, double laN) { return fmatvec::Vec(2); }
      virtual fmatvec::Vec operator()(const fmatvec::Vec &gd, double laN) { return fmatvec::Vec(2); }
      virtual int getFrictionDirections() = 0;
      virtual bool isSticking(const fmatvec::Vec& s, double sTol) = 0;
      virtual double getFrictionCoefficient(double gd) { return 0; }
      virtual bool isSetValued() const = 0;
      virtual void initializeUsingXML(TiXmlElement *element) {}
      /***************************************************/

      void setMarginalVelocity(double gdLim_) { gdLim = gdLim_; }

    protected:
      double gdLim;
  };

  /**
   * \brief basic planar friction force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class PlanarCoulombFriction : public FrictionForceLaw {
    public:
      /**
       * \brief constructor
       */
      PlanarCoulombFriction() : mu(0) {};

      /**
       * \brief constructor
       */
      PlanarCoulombFriction(double mu_) : mu(mu_) {};

      /**
       * \brief destructor
       */
      virtual ~PlanarCoulombFriction() {}

      /* INHERITED INTERFACE */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r);
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r);
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, double laN);
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double tolla, double tolgd);
      virtual int getFrictionDirections() { return 1; }
      virtual bool isSticking(const fmatvec::Vec& s, double sTol) { return fabs(s(0)) <= sTol; }
      virtual fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd, double laN);
      virtual bool isSetValued() const { return true; }
      /***************************************************/

      void setFrictionCoefficient(double mu_) { mu = mu_; }
      double getFrictionCoefficient(double gd) { return mu; }

    protected:
      double mu;
  };

  /**
   * \brief basic spatial friction force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class SpatialCoulombFriction : public FrictionForceLaw {
    public:
      /**
       * \brief constructor
       */
      SpatialCoulombFriction() : mu(0) {};

      /**
       * \brief constructor
       */
      SpatialCoulombFriction(double mu_) : mu(mu_) {};

      /**
       * \brief destructor
       */
      virtual ~SpatialCoulombFriction() {}

      /* INHERITED INTERFACE */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r);
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r);
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, double laN);
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double tolla, double tolgd);
      virtual int getFrictionDirections() { return 2; }
      virtual bool isSticking(const fmatvec::Vec& s, double sTol) { return nrm2(s(0,1)) <= sTol; }
      virtual fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd, double laN);
      virtual bool isSetValued() const { return true; }
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      void setFrictionCoefficient(double mu_) { mu = mu_; }
      double getFrictionCoefficient(double gd) { return mu; }

    protected:
      double mu;
  };

  /**
   * \brief basic friction force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class FrictionImpactLaw {
    public:
      /**
       * \brief constructor
       */
      FrictionImpactLaw() {};

      /**
       * \brief destructor
       */
      virtual ~FrictionImpactLaw() {};

      /* INTERFACE FOR DERIVED CLASSES */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r) = 0;
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r) = 0;
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN) = 0;
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double tolla, double tolgd) = 0;
      virtual int getFrictionDirections() = 0;
      virtual void initializeUsingXML(TiXmlElement *element) {}
      /***************************************************/
  };

  /**
   * \brief basic planar friction force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class PlanarCoulombImpact : public FrictionImpactLaw {
    public:
      /**
       * \brief constructor
       */
      PlanarCoulombImpact() : mu(0) {};

      /**
       * \brief constructor
       */
      PlanarCoulombImpact(double mu_) : mu(mu_) {};

      /**
       * \brief destructor
       */
      virtual ~PlanarCoulombImpact() {}

      /* INHERITED INTERFACE */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r);
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r);
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN);
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double tolla, double tolgd);
      virtual int getFrictionDirections() { return 1; }
      /***************************************************/

      void setFrictionCoefficient(double mu_) { mu = mu_; }
      double getFrictionCoefficient(double gd) { return mu; }

    protected:
      double mu;
  };

  /**
   * \brief basic spatial friction force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class SpatialCoulombImpact : public FrictionImpactLaw {
    public:
      /**
       * \brief constructor
       */
      SpatialCoulombImpact() : mu(0) {};

      /**
       * \brief constructor
       */
      SpatialCoulombImpact(double mu_) : mu(mu_) {};

      /**
       * \brief destructor
       */
      virtual ~SpatialCoulombImpact() {}

      /* INHERITED INTERFACE */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r);
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r);
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN);
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double tolla, double tolgd);
      virtual int getFrictionDirections() { return 2; }
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      void setFrictionCoefficient(double mu_) { mu = mu_; }
      double getFrictionCoefficient(double gd) { return mu; }

    protected:
      double mu;
  };

  /**
   * \brief basic regularized unilateral force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class RegularizedUnilateralConstraint : public GeneralizedForceLaw {
    public:
      /**
       * \brief constructor
       */
      RegularizedUnilateralConstraint() {};

      /**
       * \brief destructor
       */
      virtual ~RegularizedUnilateralConstraint() {};

      /* INHERITED INTERFACE */
      virtual bool isActive(double g, double gTol) { return g<=gTol; }
      virtual bool remainsActive(double s, double sTol) { return s<=sTol; }
      virtual bool isSetValued() const { return false; }
      /***************************************************/
  };

  /**
   * \brief basic regularized bilateral force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \todo operator should be replaced by a function class TODO
   */
  class RegularizedBilateralConstraint : public GeneralizedForceLaw {
    public:
      /**
       * \brief constructor
       */
      RegularizedBilateralConstraint() {};

      /**
       * \brief destructor
       */
      virtual ~RegularizedBilateralConstraint() {};

      /* INHERITED INTERFACE */
      virtual bool isActive(double g, double gTol) { return true; }
      virtual bool remainsActive(double s, double sTol) { return true; }
      virtual bool isSetValued() const { return false; }
      /***************************************************/
  };

  /**
   * \todo delete after new function concept TODO
   */
  class LinearRegularizedUnilateralConstraint: public RegularizedUnilateralConstraint {
    public:
      LinearRegularizedUnilateralConstraint() : c(0), d(0) {};
      LinearRegularizedUnilateralConstraint(double c_, double d_) : c(c_), d(d_) {};
      virtual ~LinearRegularizedUnilateralConstraint() {};
      double operator()(double g,  double gd) { 
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

  /**
   * \todo delete after new function concept TODO
   */
  class LinearRegularizedBilateralConstraint: public RegularizedBilateralConstraint {
    public:
      LinearRegularizedBilateralConstraint() : c(0), d(0) {};
      LinearRegularizedBilateralConstraint(double c_, double d_) : c(c_), d(d_) {};
      virtual ~LinearRegularizedBilateralConstraint() {};
      double operator()(double g,  double gd) { 
        return -c*g - d*gd;
      }
      virtual void initializeUsingXML(TiXmlElement *element);

    private:
      double c, d;
  };

  /**
   * \todo delete after new function concept TODO
   */
  class LinearRegularizedPlanarCoulombFriction : public FrictionForceLaw {
    public:
      LinearRegularizedPlanarCoulombFriction() : mu(0) {};
      LinearRegularizedPlanarCoulombFriction(double mu_, double gdLim=0.01) : FrictionForceLaw(gdLim), mu(mu_) {};
      virtual ~LinearRegularizedPlanarCoulombFriction() {}
      void setFrictionCoefficient(double mu_) { mu = mu_; }
      int getFrictionDirections() { return 1; }
      bool isSticking(const fmatvec::Vec& s, double sTol) { return fabs(s(0)) <= sTol; }
      fmatvec::Vec operator()(const fmatvec::Vec &gd, double laN) { 
        if(fabs(gd(0)) < gdLim)
          return fmatvec::Vec(1,fmatvec::INIT,-laN*mu*gd(0)/gdLim);
        else
          return fmatvec::Vec(1,fmatvec::INIT,gd(0)>0?-laN*mu:laN*mu);
      }
      bool isSetValued() const { return false; }

    private:
      double mu;
  };

  /**
   * \todo delete after new function concept TODO
   */
  class LinearRegularizedSpatialCoulombFriction : public FrictionForceLaw {
    public:
      LinearRegularizedSpatialCoulombFriction() : mu(0) {};
      LinearRegularizedSpatialCoulombFriction(double mu_, double gdLim=0.01) : FrictionForceLaw(gdLim), mu(mu_) {};
      virtual ~LinearRegularizedSpatialCoulombFriction() {}
      void setFrictionCoefficient(double mu_) { mu = mu_; }
      int getFrictionDirections() { return 2; }
      bool isSticking(const fmatvec::Vec& s, double sTol) { return nrm2(s(0,1)) <= sTol; }
      fmatvec::Vec operator()(const fmatvec::Vec &gd, double laN) { 
        double normgd = nrm2(gd);
        if(normgd < gdLim)
          return gd*(-laN*mu/gdLim);
        else
          return gd*(-laN*mu/normgd);
      }
      bool isSetValued() const { return false; }
      virtual void initializeUsingXML(TiXmlElement *element);

    private:
      double mu;
  };

  /**
   * \todo delete after new function concept TODO
   */
  class LinearRegularizedStribeckFriction : public FrictionForceLaw {
    public:
      LinearRegularizedStribeckFriction() : fmu(0) {};
      LinearRegularizedStribeckFriction(UserFunction *fmu_) : fmu(fmu_) {};
      virtual ~LinearRegularizedStribeckFriction() {};
      void setFrictionCharacteristics(UserFunction *fmu_) { fmu = fmu_; }
      bool isSticking(const fmatvec::Vec& s, double sTol) { return nrm2(s(0,1)) <= sTol; }
      fmatvec::Vec operator()(const fmatvec::Vec &gd, double laN) { 
        int nFric = gd.size();
        fmatvec::Vec la(nFric,fmatvec::NONINIT);
        double normgd = nrm2(gd(0,nFric-1));
        if(normgd < gdLim) {
          double mu0 = (*fmu)(0)(0);
          la(0,nFric-1) = gd(0,nFric-1)*(-laN*mu0/gdLim);
        } else {
          double mu = (*fmu)(nrm2(gd(0,nFric-1))-gdLim)(0); //oder (*fmu)(nrm2(gd(0,nFric-1)))(0)
          la(0,nFric-1) = gd(0,nFric-1)*(-laN*mu/normgd);
        }
        return la;
      }
      bool isSetValued() const { return false; }

    private:
      UserFunction *fmu;
  };

}

#endif /* _CONSTITUTIVE_LAWS_H_ */

