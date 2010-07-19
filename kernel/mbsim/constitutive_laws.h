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
#include "mbsim/utils/function.h"

namespace MBSim {

  /**
   * \brief basic force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   */
  class GeneralizedForceLaw {
    public:
      /**
       * \brief constructor
       */
      GeneralizedForceLaw() : forceFunc(NULL) {};

      GeneralizedForceLaw(Function2<double,double,double> *forceFunc_) : forceFunc(forceFunc_) {};

      /**
       * \brief destructor
       */
      virtual ~GeneralizedForceLaw() { if(forceFunc) delete forceFunc; forceFunc = NULL; };

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \param relative distance
       * \param tolerance
       * \return flag, if force law is active
       */
      virtual bool isActive(double g, double gTol) { return true; }
      virtual bool remainsActive(double s, double sTol) { return true; }
      virtual double project(double la, double gdn, double r, double laMin=0) { return 0; }
      virtual fmatvec::Vec diff(double la, double gdn, double r, double laMin=0) { return fmatvec::Vec(2, fmatvec::INIT, 0); }
      virtual double solve(double G, double gdn) { return 0; }

      /**
       * \param contact force parameter
       * \param contact relative velocity
       * \param tolerance for contact force parameters
       * \param tolerance for relative velocity
       * \return flag if the force law is valid given the parameters
       */
      virtual bool isFulfilled(double la,  double gdn, double tolla, double tolgd, double laMin=0) { return true; }

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
      
      double operator()(double g, double gd, const void * additional=NULL) { assert(forceFunc); return (*forceFunc)(g,gd,additional); }

      /** \brief Set the force function for use in regularisized constitutive laws
       * The first input parameter to the force function is g.
       * The second input parameter to the force function is gd.
       * The return value is the force.
       */
      void setForceFunction(Function2<double,double,double> *forceFunc_) { forceFunc=forceFunc_; }

    protected:
      Function2<double,double,double> *forceFunc;
  };

  /**
   * \brief basic unilateral force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
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
      virtual double project(double la, double gdn, double r, double laMin=0);
      virtual fmatvec::Vec diff(double la, double gdn, double r, double laMin=0);
      virtual double solve(double G, double gdn);
      virtual bool isFulfilled(double la,  double gdn, double tolla, double tolgd, double laMin=0);
      virtual bool isSetValued() const { return true; }
      /***************************************************/

      bool remainsClosed(double s, double sTol) { return s<=sTol; }  // s = gd/gdd
  };

  /**
   * \brief basic bilateral force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   * \date 2010-07-06 isSticking added for impact laws (Robert Huber)
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
      virtual double project(double la, double gdn, double r, double laMin=0);
      virtual fmatvec::Vec diff(double la, double gdn, double r, double laMin=0);
      virtual double solve(double G, double gdn);
      virtual bool isFulfilled(double la,  double gdn, double tolla, double tolgd, double laMin=0);
      virtual bool isSetValued() const { return true; }
      /***************************************************/

      bool remainsClosed(double s, double sTol) { return true; }
  };

  /**
   * \brief basic force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
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
      virtual double project(double la, double gdn, double gda, double r, double laMin=0) = 0;
      virtual fmatvec::Vec diff(double la, double gdn, double gda, double r, double laMin=0) = 0;
      virtual double solve(double G, double gdn, double gda) = 0;
      virtual bool isFulfilled(double la,  double gdn, double gda, double tolla, double tolgd, double laMin=0) = 0;
      virtual void initializeUsingXML(TiXmlElement *element) {}
      /***************************************************/
  };

  /**
   * \brief basic unilateral force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
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
      virtual double project(double la, double gdn, double gda, double r, double laMin=0);
      virtual fmatvec::Vec diff(double la, double gdn, double gda, double r, double laMin=0);
      virtual double solve(double G, double gdn, double gda);
      virtual bool isFulfilled(double la,  double gdn, double gda, double tolla, double tolgd, double laMin=0);
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

    protected:
      double epsilon, gd_limit;
  };

  /**
   * \brief basic bilateral force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
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
      virtual double project(double la, double gdn, double gda, double r, double laMin=0);
      virtual fmatvec::Vec diff(double la, double gdn, double gda, double r, double laMin=0);
      virtual double solve(double G, double gdn, double gda);
      virtual bool isFulfilled(double la,  double gdn, double gda, double tolla, double tolgd, double laMin=0);
      /***************************************************/
  };

  /**
   * \brief basic friction force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   */
  class FrictionForceLaw {
    public:
      /**
       * \brief constructor
       */
      FrictionForceLaw() : frictionForceFunc(NULL) {};

      FrictionForceLaw(Function2<fmatvec::Vec,fmatvec::Vec,double> *frictionForceFunc_) : frictionForceFunc(frictionForceFunc_) {};

      /**
       * \brief destructor
       */
      virtual ~FrictionForceLaw() { if(frictionForceFunc) delete frictionForceFunc; frictionForceFunc = NULL; };

      /* INTERFACE FOR DERIVED CLASSES */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r) { return fmatvec::Vec(2); }
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r) { return fmatvec::Mat(2,2); }
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, double laN) { return fmatvec::Vec(2); }
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double tolla, double tolgd) { return true; }
      virtual fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd, double laN) { return fmatvec::Vec(2); }
      virtual int getFrictionDirections() = 0;
      virtual bool isSticking(const fmatvec::Vec& s, double sTol) = 0;
      virtual double getFrictionCoefficient(double gd) { return 0; }
      virtual bool isSetValued() const = 0;
      virtual void initializeUsingXML(TiXmlElement *element) {}
      /***************************************************/
      
      fmatvec::Vec operator()(const fmatvec::Vec &gd, double laN) { assert(frictionForceFunc); return (*frictionForceFunc)(gd,laN); }

      /** \brief Set the friction force function for use in regularisized constitutive friction laws
       * The first input parameter to the friction force function is gd.
       * The second input parameter to the friction force function is laN.
       * The return value is the force vector.
       */
      void setFrictionForceFunction(Function2<fmatvec::Vec,fmatvec::Vec,double> *frictionForceFunc_) { frictionForceFunc=frictionForceFunc_; }

    protected:
      Function2<fmatvec::Vec,fmatvec::Vec,double> *frictionForceFunc;
  };

  /**
   * \brief basic planar friction force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
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
      virtual fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd, double laN);
      virtual int getFrictionDirections() { return 1; }
      virtual bool isSticking(const fmatvec::Vec& s, double sTol) { return fabs(s(0)) <= sTol; }
      virtual double getFrictionCoefficient(double gd) { return mu; }
      virtual bool isSetValued() const { return true; }
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      void setFrictionCoefficient(double mu_) { mu = mu_; }

    protected:
      double mu;
  };

  /**
   * \brief basic spatial friction force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
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
      virtual fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd, double laN);
      virtual int getFrictionDirections() { return 2; }
      virtual bool isSticking(const fmatvec::Vec& s, double sTol) { return nrm2(s(0,1)) <= sTol; }
      virtual double getFrictionCoefficient(double gd) { return mu; }
      virtual bool isSetValued() const { return true; }
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      void setFrictionCoefficient(double mu_) { mu = mu_; }

    protected:
      double mu;
  };

  /**
   * \brief planar Stribeck friction force law on acceleration level for constraint description
   * \author Thorsten Schindler
   * \date 2009-09-02 inital commit (Thorsten Schindler)
   * \todo high oscillations in normal relative velocity TODO
   */
  class PlanarStribeckFriction : public FrictionForceLaw {
    public:
      /**
       * \brief constructor
       */
      PlanarStribeckFriction() : fmu(0) {};

      /**
       * \brief constructor
       */
      PlanarStribeckFriction(Function1<double,double> *fmu_) : fmu(fmu_) {};

      /**
       * \brief destructor
       */
      virtual ~PlanarStribeckFriction() { if(fmu) delete fmu; fmu=0; }

      /* INHERITED INTERFACE */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r);
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r);
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, double laN);
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double tolla, double tolgd);
      virtual fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd, double laN);
      virtual int getFrictionDirections() { return 1; }
      virtual bool isSticking(const fmatvec::Vec& s, double sTol) { return fabs(s(0)) <= sTol; }
      virtual double getFrictionCoefficient(double gd) { return (*fmu)(gd); }
      virtual bool isSetValued() const { return true; }
      /***************************************************/

    protected:
      /**
       * friction coefficient function
       */
      Function1<double,double> *fmu;
  };

  /**
   * \brief spatial Stribeck friction force law on acceleration level for constraint description
   * \author Thorsten Schindler
   * \date 2009-09-02 initial commit (Thorsten Schindler)
   * \todo high oscillations in normal relative velocity TODO
   */
  class SpatialStribeckFriction : public FrictionForceLaw {
    public:
      /**
       * \brief constructor
       */
      SpatialStribeckFriction() : fmu(0) {};

      /**
       * \brief constructor
       */
      SpatialStribeckFriction(Function1<double,double> *fmu_) : fmu(fmu_) {};

      /**
       * \brief destructor
       */
      virtual ~SpatialStribeckFriction() { if(fmu) delete fmu; fmu=0; }

      /* INHERITED INTERFACE */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r);
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r);
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, double laN);
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double tolla, double tolgd);
      virtual fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd, double laN);
      virtual int getFrictionDirections() { return 2; }
      virtual bool isSticking(const fmatvec::Vec& s, double sTol) { return nrm2(s(0,1)) <= sTol; }
      virtual double getFrictionCoefficient(double gd) { return (*fmu)(gd); }
      virtual bool isSetValued() const { return true; }
      /***************************************************/

    protected:
      /**
       * friction coefficient function
       */
      Function1<double,double> *fmu;
  };

  /**
   * \brief basic friction force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
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
      virtual int isSticking(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double laTol, double gdTol) = 0;
      virtual int getFrictionDirections() = 0;
      virtual void initializeUsingXML(TiXmlElement *element) {}
      /***************************************************/
  };

  /**
   * \brief basic planar friction force law on velocity level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
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
      virtual int isSticking(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double laTol, double gdTol);
      virtual int getFrictionDirections() { return 1; }
      virtual void initializeUsingXML(TiXmlElement *element);
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
      virtual int isSticking(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double laTol, double gdTol);
      virtual int getFrictionDirections() { return 2; }
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      void setFrictionCoefficient(double mu_) { mu = mu_; }
      double getFrictionCoefficient(double gd) { return mu; }

    protected:
      double mu;
  };

  /**
   * \brief planar Stribeck friction force law on velocity level for constraint description
   * \author Thorsten Schindler
   * \date 2009-09-02 initial commit (Thorsten Schindler)
   * \todo high oscillations in normal relative velocity TODO
   */
  class PlanarStribeckImpact : public FrictionImpactLaw {
    public:
      /**
       * \brief constructor
       */
      PlanarStribeckImpact() : fmu(0) {};

      /**
       * \brief constructor
       */
      PlanarStribeckImpact(Function1<double,double> *fmu_) : fmu(fmu_) {};

      /**
       * \brief destructor
       */
      virtual ~PlanarStribeckImpact() { if(fmu) delete fmu; fmu=0; }

      /* INHERITED INTERFACE */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r);
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r);
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN);
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double tolla, double tolgd);
      virtual int isSticking(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double laTol, double gdTol);
      virtual int getFrictionDirections() { return 1; }
      /***************************************************/

      double getFrictionCoefficient(double gd) { return (*fmu)(gd); }

    protected:
      /**
       * friction coefficient function
       */
      Function1<double,double> *fmu;
  };

  /**
   * \brief spatial Stribeck friction force law on velocity level for constraint description
   * \author Thorsten Schindler
   * \date 2009-09-02 initial commit (Thorsten Schindler)
   * \todo high oscillations in normal relative velocity TODO
   */
  class SpatialStribeckImpact : public FrictionImpactLaw {
    public:
      /**
       * \brief constructor
       */
      SpatialStribeckImpact() : fmu(0) {};

      /**
       * \brief constructor
       */
      SpatialStribeckImpact(Function1<double,double> *fmu_) : fmu(fmu_) {};

      /**
       * \brief destructor
       */
      virtual ~SpatialStribeckImpact() { if(fmu) delete fmu; fmu=0; }

      /* INHERITED INTERFACE */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r);
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double r);
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN);
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double tolla, double tolgd);
      virtual int isSticking(const fmatvec::Vec& la, const fmatvec::Vec& gdn, const fmatvec::Vec& gda, double laN, double laTol, double gdTol);
      virtual int getFrictionDirections() { return 2; }
      /***************************************************/

      double getFrictionCoefficient(double gd) { return (*fmu)(gd); }

    protected:
      /**
       * friction coefficient function
       */
      Function1<double,double> *fmu;
  };

  /**
   * \brief basic regularized unilateral force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   */
  class RegularizedUnilateralConstraint : public GeneralizedForceLaw {
    public:
      /**
       * \brief constructor
       */
      RegularizedUnilateralConstraint() {};

      RegularizedUnilateralConstraint(Function2<double,double,double> *forceFunc_) : GeneralizedForceLaw(forceFunc_) {};

      /**
       * \brief destructor
       */
      virtual ~RegularizedUnilateralConstraint() {};

      /* INHERITED INTERFACE */
      virtual bool isActive(double g, double gTol) { return g<=gTol; }
      virtual bool remainsActive(double s, double sTol) { return s<=sTol; }
      virtual bool isSetValued() const { return false; }
      /***************************************************/

      virtual void initializeUsingXML(TiXmlElement *element);
  };

  /**
   * \brief basic regularized bilateral force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   */
  class RegularizedBilateralConstraint : public GeneralizedForceLaw {
    public:
      /**
       * \brief constructor
       */
      RegularizedBilateralConstraint() {};

      RegularizedBilateralConstraint(Function2<double,double,double> *forceFunc_) : GeneralizedForceLaw(forceFunc_) {};

      /**
       * \brief destructor
       */
      virtual ~RegularizedBilateralConstraint() {};

      /* INHERITED INTERFACE */
      virtual bool isActive(double g, double gTol) { return true; }
      virtual bool remainsActive(double s, double sTol) { return true; }
      virtual bool isSetValued() const { return false; }
      /***************************************************/

      virtual void initializeUsingXML(TiXmlElement *element);
  };

  class RegularizedPlanarFriction : public FrictionForceLaw {
    public:
      RegularizedPlanarFriction() {};
      RegularizedPlanarFriction(Function2<fmatvec::Vec,fmatvec::Vec,double> *frictionForceFunc_) : FrictionForceLaw(frictionForceFunc_) {};
      virtual ~RegularizedPlanarFriction() {}
      int getFrictionDirections() { return 1; }
      bool isSticking(const fmatvec::Vec& s, double sTol) { return fabs(s(0)) <= sTol; }
      bool isSetValued() const { return false; }
      virtual void initializeUsingXML(TiXmlElement *element);
  };

  class RegularizedSpatialFriction : public FrictionForceLaw {
    public:
      RegularizedSpatialFriction() {};
      RegularizedSpatialFriction(Function2<fmatvec::Vec,fmatvec::Vec,double> *frictionForceFunc_) : FrictionForceLaw(frictionForceFunc_) {};
      virtual ~RegularizedSpatialFriction() {}
      int getFrictionDirections() { return 2; }
      bool isSticking(const fmatvec::Vec& s, double sTol) { return nrm2(s(0,1)) <= sTol; }
      bool isSetValued() const { return false; }
      virtual void initializeUsingXML(TiXmlElement *element);
  };

}

#endif /* _CONSTITUTIVE_LAWS_H_ */

