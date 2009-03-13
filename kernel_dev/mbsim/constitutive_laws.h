/* Copyright (C) 2004-2008  Martin Förg
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _CONSTITUTIVE_LAWS_H_
#define _CONSTITUTIVE_LAWS_H_

#include <fmatvec.h>
#include <fstream>
#include <mbsim/userfunction.h>

using namespace fmatvec;

namespace MBSim {

  class GeneralizedForceLaw {
    public:
      GeneralizedForceLaw() {};
      virtual ~GeneralizedForceLaw() {};
      virtual bool isActive(double g, double gTol) {return true;}
      virtual bool remainsActive(double s, double sTol) {return true;}
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      virtual double project(double la, double gdn, double r) {return 0;}
      virtual Vec diff(double la, double gdn, double r) {return Vec(2);}
      virtual double solve(double G, double gdn) {return 0;}
      virtual bool isFullfield(double la,  double gdn, double tolla, double tolgd) {return true;}
      virtual double operator()(double g,  double gd) {return 0;}
      virtual bool isSetValued() const = 0;
  };

  class UnilateralConstraint : public GeneralizedForceLaw {
    protected:
    public:
      UnilateralConstraint() {};
      virtual ~UnilateralConstraint() {};
      bool isActive(double g, double gTol) {return g<=gTol;}
      bool remainsClosed(double s, double sTol) {return s<=sTol;}  // s = gd/gdd
      void load(const string& path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);
      double project(double la, double gdn, double r);
      Vec diff(double la, double gdn, double r);
      double solve(double G, double gdn);
      bool isFullfield(double la,  double gdn, double tolla, double tolgd);
      bool isSetValued() const {return true;}
  };

  class BilateralConstraint : public GeneralizedForceLaw {
    public:
      BilateralConstraint() {};
      virtual ~BilateralConstraint() {};
      bool isActive(double g, double gTol) {return true;}
      bool remainsClosed(double s, double sTol) {return true;}
      //void load(const string& path, ifstream &inputfile);
      //void save(const string &path, ofstream &outputfile);
      double project(double la, double gdn, double r);
      Vec diff(double la, double gdn, double r);
      double solve(double G, double gdn);
      bool isFullfield(double la,  double gdn, double tolla, double tolgd);
      bool isSetValued() const {return true;}
  };

  class GeneralizedImpactLaw {
    public:
      GeneralizedImpactLaw() {};
      virtual ~GeneralizedImpactLaw() {};
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      virtual double project(double la, double gdn, double gda, double r) = 0;
      virtual Vec diff(double la, double gdn, double gda, double r) = 0;
      virtual double solve(double G, double gdn, double gda) = 0;
      virtual bool isFullfield(double la,  double gdn, double gda, double tolla, double tolgd) = 0;
  };

  class UnilateralNewtonImpact : public GeneralizedImpactLaw {
    protected:
      double epsilon, gd_limit;
    public:
      UnilateralNewtonImpact() : epsilon(0), gd_limit(1e-2) {};
      UnilateralNewtonImpact(double epsilon_) : epsilon(epsilon_), gd_limit(1e-2) {};
      UnilateralNewtonImpact(double epsilon_, double gd_limit_) : epsilon(epsilon_), gd_limit(gd_limit_) {};
      virtual ~UnilateralNewtonImpact() {};
      void load(const string& path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);
      double project(double la, double gdn, double gda, double r);
      Vec diff(double la, double gdn, double gda, double r);
      double solve(double G, double gdn, double gda);
      bool isFullfield(double la,  double gdn, double gda, double tolla, double tolgd);
  };

  class BilateralImpact : public GeneralizedImpactLaw {
    public:
      BilateralImpact() {};
      virtual ~BilateralImpact() {};
      double project(double la, double gdn, double gda, double r);
      Vec diff(double la, double gdn, double gda, double r);
      double solve(double G, double gdn, double gda);
      bool isFullfield(double la,  double gdn, double gda, double tolla, double tolgd);
  };

  class FrictionForceLaw {
    protected:
      double gdLim;
    public:
      FrictionForceLaw(double gdLim_ = 0.01) : gdLim(gdLim_) {};
      virtual ~FrictionForceLaw() {};
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      virtual Vec project(const Vec& la, const Vec& gdn, double laN, double r) {return Vec(2);}
      virtual Mat diff(const Vec& la, const Vec& gdn, double laN, double r) {return Mat(2,2);}
      virtual Vec solve(const SqrMat& G, const Vec& gdn, double laN) {return Vec(2);}
      virtual bool isFullfield(const Vec& la, const Vec& gdn, double laN, double tolla, double tolgd) {return true;}
      virtual Vec dlaTdlaN(const Vec& gd, double laN) {return Vec(2);}
      virtual Vec operator()(const Vec &gd, double laN) {return Vec(2);}
      virtual int getFrictionDirections() = 0;
      virtual bool isSticking(const Vec& s, double sTol) = 0;
      virtual double getFrictionCoefficient(double gd) {return 0;}
      virtual bool isSetValued() const = 0;
      void setMarginalVelocity(double gdLim_) {gdLim = gdLim_;}
  };

  class PlanarCoulombFriction : public FrictionForceLaw {
    protected:
      double mu;
    public:
      PlanarCoulombFriction() : mu(0) {};
      PlanarCoulombFriction(double mu_) : mu(mu_) {};
      virtual ~PlanarCoulombFriction() {}
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      void setFrictionCoefficient(double mu_) {mu = mu_;}
      double getFrictionCoefficient(double gd) {return mu;}
      Vec project(const Vec& la, const Vec& gdn, double laN, double r);
      Mat diff(const Vec& la, const Vec& gdn, double laN, double r);
      Vec solve(const SqrMat& G, const Vec& gdn, double laN);
      bool isFullfield(const Vec& la, const Vec& gdn, double laN, double tolla, double tolgd);
      int getFrictionDirections() {return 1;}
      bool isSticking(const Vec& s, double sTol) {return abs(s(0)) <= sTol;}
      Vec dlaTdlaN(const Vec& gd, double laN);
      bool isSetValued() const {return true;}
  };

  class SpatialCoulombFriction : public FrictionForceLaw {
    protected:
      double mu;
    public:
      SpatialCoulombFriction() : mu(0) {};
      SpatialCoulombFriction(double mu_) : mu(mu_) {};
      virtual ~SpatialCoulombFriction() {}
      void setFrictionCoefficient(double mu_) {mu = mu_;}
      double getFrictionCoefficient(double gd) {return mu;}
      Vec project(const Vec& la, const Vec& gdn, double laN, double r);
      Mat diff(const Vec& la, const Vec& gdn, double laN, double r);
      Vec solve(const SqrMat& G, const Vec& gdn, double laN);
      bool isFullfield(const Vec& la, const Vec& gdn, double laN, double tolla, double tolgd);
      int getFrictionDirections() {return 2;}
      bool isSticking(const Vec& s, double sTol) {return nrm2(s(0,1)) <= sTol;}
      Vec dlaTdlaN(const Vec& gd, double laN);
      bool isSetValued() const {return true;}
  };

  class FrictionImpactLaw {
    public:
      FrictionImpactLaw() {};
      virtual ~FrictionImpactLaw() {};
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      virtual Vec project(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) = 0;
      virtual Mat diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) = 0;
      virtual Vec solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN) = 0;
      virtual bool isFullfield(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double tolla, double tolgd) = 0;
      virtual int getFrictionDirections() = 0;
  };

  class PlanarCoulombImpact : public FrictionImpactLaw {
    protected:
      double mu;
    public:
      PlanarCoulombImpact() : mu(0) {};
      PlanarCoulombImpact(double mu_) : mu(mu_) {};
      virtual ~PlanarCoulombImpact() {}
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      void setFrictionCoefficient(double mu_) {mu = mu_;}
      double getFrictionCoefficient(double gd) {return mu;}
      Vec project(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Mat diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Vec solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN);
      bool isFullfield(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double tolla, double tolgd);
      int getFrictionDirections() {return 1;}
  };

  class SpatialCoulombImpact : public FrictionImpactLaw {
    protected:
      double mu;
    public:
      SpatialCoulombImpact() : mu(0) {};
      SpatialCoulombImpact(double mu_) : mu(mu_) {};
      virtual ~SpatialCoulombImpact() {}
      void setFrictionCoefficient(double mu_) {mu = mu_;}
      double getFrictionCoefficient(double gd) {return mu;}
      Vec project(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Mat diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Vec solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN);
      bool isFullfield(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double tolla, double tolgd);
      int getFrictionDirections() {return 2;}
  };

  class RegularizedUnilateralConstraint : public GeneralizedForceLaw {
    public:
      RegularizedUnilateralConstraint() {};
      virtual ~RegularizedUnilateralConstraint() {};
      bool isActive(double g, double gTol) {return g<=gTol;}
      bool remainsActive(double s, double sTol) {return s<=sTol;}
      bool isSetValued() const {return false;}
  };

  class RegularizedBilateralConstraint : public GeneralizedForceLaw {
    public:
      RegularizedBilateralConstraint() {};
      virtual ~RegularizedBilateralConstraint() {};
      bool isActive(double g, double gTol) {return true;}
      bool remainsActive(double s, double sTol) {return true;}
      bool isSetValued() const {return false;}
  };

  class LinearRegularizedUnilateralConstraint: public RegularizedUnilateralConstraint {
    private:
      double c, d;
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
  };

  class LinearRegularizedBilateralConstraint: public RegularizedBilateralConstraint {
    private:
      double c, d;
    public:
      LinearRegularizedBilateralConstraint() : c(0), d(0) {};
      LinearRegularizedBilateralConstraint(double c_, double d_) : c(c_), d(d_) {};
      virtual ~LinearRegularizedBilateralConstraint() {};
      double operator()(double g,  double gd) { 
	return -c*g - d*gd;
      }
  };

  class LinearRegularizedPlanarCoulombFriction : public FrictionForceLaw {
    private:
      double mu;
    public:
      LinearRegularizedPlanarCoulombFriction() : mu(0) {};
      LinearRegularizedPlanarCoulombFriction(double mu_, double gdLim=0.01) : FrictionForceLaw(gdLim), mu(mu_) {};
      virtual ~LinearRegularizedPlanarCoulombFriction() {}
      void setFrictionCoefficient(double mu_) {mu = mu_;}
      int getFrictionDirections() {return 1;}
      bool isSticking(const Vec& s, double sTol) {return abs(s(0)) <= sTol;}
      Vec operator()(const Vec &gd, double laN) { 
	if(fabs(gd(0)) < gdLim)
	  return Vec(1,INIT,-laN*mu*gd(0)/gdLim);
	else
	  return Vec(1,INIT,gd(0)>0?-laN*mu:laN*mu);
      }
      bool isSetValued() const {return false;}
  };

  class LinearRegularizedSpatialCoulombFriction : public FrictionForceLaw {
    private:
      double mu;
    public:
      LinearRegularizedSpatialCoulombFriction() : mu(0) {};
      LinearRegularizedSpatialCoulombFriction(double mu_, double gdLim=0.01) : FrictionForceLaw(gdLim), mu(mu_) {};
      virtual ~LinearRegularizedSpatialCoulombFriction() {}
      void setFrictionCoefficient(double mu_) {mu = mu_;}
      int getFrictionDirections() {return 2;}
      bool isSticking(const Vec& s, double sTol) {return nrm2(s(0,1)) <= sTol;}
      Vec operator()(const Vec &gd, double laN) { 
	double normgd = nrm2(gd);
	if(normgd < gdLim)
	  return gd*(-laN*mu/gdLim);
	else
	  return gd*(-laN*mu/normgd);
      }
      bool isSetValued() const {return false;}
  };

  class LinearRegularizedStribeckFriction : public FrictionForceLaw {
    private:
      UserFunction *fmu;
    public:
      LinearRegularizedStribeckFriction() : fmu(0) {};
      LinearRegularizedStribeckFriction(UserFunction *fmu_) : fmu(fmu_) {};
      virtual ~LinearRegularizedStribeckFriction() {};
      void setFrictionCharacteristics(UserFunction *fmu_) {fmu = fmu_;}
      bool isSticking(const Vec& s, double sTol) {return nrm2(s(0,1)) <= sTol;}
      Vec operator()(const Vec &gd, double laN) { 
	int nFric = gd.size();
	Vec la(nFric,NONINIT);
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
      bool isSetValued() const {return false;}
  };

}
#endif
