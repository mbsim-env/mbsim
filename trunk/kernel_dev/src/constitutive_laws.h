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
#include "userfunction.h"

using namespace fmatvec;

namespace MBSim {

  class ConstraintLaw {
    public:
      ConstraintLaw() {};
      virtual ~ConstraintLaw() {};
      virtual bool isClosed(double g, double gTol) = 0;
      virtual bool remainsClosed(double s, double sTol) = 0;
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      virtual double operator()(double la, double gdn, double r) = 0;
      virtual Vec diff(double la, double gdn, double r) = 0;
      virtual double solve(double G, double gdn) = 0;
      virtual bool isFullfield(double la,  double gdn, double tolla, double tolgd) = 0;
      //virtual bool isClosed(const Vec& g) {return g(0)<=0;}
      //virtual Vec operator()(const Vec& la, const Vec& gdn, const Vec& r) = 0;
      //virtual Vec diff(const Vec& la, const Vec& gdn, const Vec& r) = 0;
      //virtual Vec solve(const SqrMat& G, const Vec& gdn) = 0;
      //virtual bool isFullfield(const Vec& la,  const Vec& gdn, double tolla, double tolgd) = 0;
  };

  class UnilateralConstraint : public ConstraintLaw {
    protected:
    public:
      UnilateralConstraint() {};
      virtual ~UnilateralConstraint() {};
      bool isClosed(double g, double gTol) {return g<=gTol;}
      bool remainsClosed(double s, double sTol) {return s<=sTol;}  // s = gd/gdd
      void load(const string& path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);
      double operator()(double la, double gdn, double r);
      Vec diff(double la, double gdn, double r);
      double solve(double G, double gdn);
      bool isFullfield(double la,  double gdn, double tolla, double tolgd);
      //Vec operator()(const Vec& la,  const Vec& gdn, const Vec& r);
      //Vec diff(const Vec& la,  const Vec& gdn, const Vec& r);
      //Vec solve(const SqrMat& G, const Vec& gdn);
      //bool isFullfield(const Vec& la,  const Vec& gdn, double tolla, double tolgd);
  };

  class BilateralConstraint : public ConstraintLaw {
    public:
      BilateralConstraint() {};
      virtual ~BilateralConstraint() {};
      bool isClosed(double g, double gTol) {return true;}
      bool remainsClosed(double s, double sTol) {return true;}
      //void load(const string& path, ifstream &inputfile);
      //void save(const string &path, ofstream &outputfile);
      double operator()(double la, double gdn, double r);
      Vec diff(double la, double gdn, double r);
      double solve(double G, double gdn);
      bool isFullfield(double la,  double gdn, double tolla, double tolgd);
      //bool isClosed(const Vec& g) {return true;}
      //Vec operator()(const Vec& la,  const Vec& gdn, const Vec& r);
      //Vec diff(const Vec& la,  const Vec& gdn, const Vec& r);
      //Vec solve(const SqrMat& G, const Vec& gdn);
      //bool isFullfield(const Vec& la,  const Vec& gdn, double tolla, double tolgd);
  };

  class NormalImpactLaw {
    public:
      NormalImpactLaw() {};
      virtual ~NormalImpactLaw() {};
      //virtual bool isClosed(double g) = 0;
      //virtual bool remainsClosed(double gd) = 0;
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      virtual double operator()(double la, double gdn, double gda, double r) = 0;
      virtual Vec diff(double la, double gdn, double gda, double r) = 0;
      virtual double solve(double G, double gdn, double gda) = 0;
      virtual bool isFullfield(double la,  double gdn, double gda, double tolla, double tolgd) = 0;
  };

  class UnilateralNewtonImpact : public NormalImpactLaw {
    protected:
      double epsilon, gd_limit;
    public:
      UnilateralNewtonImpact() : epsilon(0), gd_limit(1e-2) {};
      UnilateralNewtonImpact(double epsilon_) : epsilon(epsilon_), gd_limit(1e-2) {};
      UnilateralNewtonImpact(double epsilon_, double gd_limit_) : epsilon(epsilon_), gd_limit(gd_limit_) {};
      virtual ~UnilateralNewtonImpact() {};
      //bool isClosed(double g) {return g<=0;}
      //bool remainsClosed(double g) {return g<=0;}
      void load(const string& path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);
      double operator()(double la, double gdn, double gda, double r);
      Vec diff(double la, double gdn, double gda, double r);
      double solve(double G, double gdn, double gda);
      bool isFullfield(double la,  double gdn, double gda, double tolla, double tolgd);
      //Vec operator()(const Vec& la,  const Vec& gdn, const Vec& gda, const Vec& r);
      //Vec diff(const Vec& la,  const Vec& gdn, const Vec& gda, const Vec& r);
      //Vec solve(const SqrMat& G, const Vec& gdn, const Vec& gda);
      //bool isFullfield(const Vec& la,  const Vec& gdn, const Vec& gda, double tolla, double tolgd);
  };

  class BilateralImpact : public NormalImpactLaw {
    public:
      BilateralImpact() {};
      virtual ~BilateralImpact() {};
      //bool isClosed(double g) {return true;}
      //void load(const string& path, ifstream &inputfile);
      //void save(const string &path, ofstream &outputfile);
      double operator()(double la, double gdn, double gda, double r);
      Vec diff(double la, double gdn, double gda, double r);
      double solve(double G, double gdn, double gda);
      bool isFullfield(double la,  double gdn, double gda, double tolla, double tolgd);
  };

  class FrictionLaw {
    protected:
      double gdEps;
    public:
      FrictionLaw() : gdEps(1e-8) {};
      virtual ~FrictionLaw() {};
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      //virtual Vec operator()(const Vec& la, const Vec& gdn, const Vec& r) = 0;
      //virtual Mat diff(const Vec& la, const Vec& gdn, const Vec& r) = 0;
      //virtual Vec solve(const SqrMat& G, const Vec& laN, const Vec& gdn) = 0;
      //virtual bool isFullfield(const Vec& la, const Vec& gdn, double tolla, double tolgd) = 0;
      virtual Vec operator()(const Vec& la, const Vec& gdn, double laN, double r) = 0;
      virtual Mat diff(const Vec& la, const Vec& gdn, double laN, double r) = 0;
      virtual Vec solve(const SqrMat& G, const Vec& gdn, double laN) = 0;
      virtual bool isFullfield(const Vec& la, const Vec& gdn, double laN, double tolla, double tolgd) = 0;
      virtual int getFrictionDirections() = 0;
      virtual Vec dlaTdlaN(const Vec& gd, double laN) = 0;
  };

  class DryFriction : public FrictionLaw {
    public:
      DryFriction() {};
      virtual ~DryFriction() {};
      virtual double getFrictionCoefficient(double gd) = 0; 
      virtual bool isSticking(const Vec& s, double sTol) = 0;
  };

  class PlanarCoulombFriction : public DryFriction {
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
      Vec operator()(const Vec& la, const Vec& gdn, double laN, double r);
      Mat diff(const Vec& la, const Vec& gdn, double laN, double r);
      Vec solve(const SqrMat& G, const Vec& gdn, double laN);
      bool isFullfield(const Vec& la, const Vec& gdn, double laN, double tolla, double tolgd);
      int getFrictionDirections() {return 1;}
      bool isSticking(const Vec& s, double sTol) {return abs(s(0)) <= sTol;}
      Vec dlaTdlaN(const Vec& gd, double laN);
  };

  class SpatialCoulombFriction : public DryFriction {
    protected:
      double mu;
    public:
      SpatialCoulombFriction() : mu(0) {};
      SpatialCoulombFriction(double mu_) : mu(mu_) {};
      virtual ~SpatialCoulombFriction() {}
      void setFrictionCoefficient(double mu_) {mu = mu_;}
      double getFrictionCoefficient(double gd) {return mu;}
      //Vec operator()(const Vec& la, const Vec& gdn, const Vec& r); 
      //Mat diff(const Vec& la, const Vec& gdn, const Vec& r);
      //Vec solve(const SqrMat& G, const Vec& laN, const Vec& gdn);
      //bool isFullfield(const Vec& la, const Vec& gdn, double tolla, double tolgd);
      Vec operator()(const Vec& la, const Vec& gdn, double laN, double r);
      Mat diff(const Vec& la, const Vec& gdn, double laN, double r);
      Vec solve(const SqrMat& G, const Vec& gdn, double laN);
      bool isFullfield(const Vec& la, const Vec& gdn, double laN, double tolla, double tolgd);
      int getFrictionDirections() {return 2;}
      bool isSticking(const Vec& s, double sTol) {return nrm2(s(0,1)) <= sTol;}
      Vec dlaTdlaN(const Vec& gd, double laN);
  };

  class TangentialImpactLaw {
    public:
      TangentialImpactLaw() {};
      virtual ~TangentialImpactLaw() {};
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      virtual Vec operator()(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) = 0;
      virtual Mat diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) = 0;
      virtual Vec solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN) = 0;
      virtual bool isFullfield(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double tolla, double tolgd) = 0;
      virtual int getFrictionDirections() = 0;
  };

 class DryFrictionImpact : public TangentialImpactLaw {
    public:
      DryFrictionImpact() {};
      virtual ~DryFrictionImpact() {};
      virtual double getFrictionCoefficient(double gd) = 0; 
  };

  class PlanarCoulombImpact : public DryFrictionImpact {
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
      Vec operator()(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Mat diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Vec solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN);
      bool isFullfield(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double tolla, double tolgd);
      int getFrictionDirections() {return 1;}
  };

  class SpatialCoulombImpact : public DryFrictionImpact {
    protected:
      double mu;
    public:
      SpatialCoulombImpact() : mu(0) {};
      SpatialCoulombImpact(double mu_) : mu(mu_) {};
      virtual ~SpatialCoulombImpact() {}
      void setFrictionCoefficient(double mu_) {mu = mu_;}
      double getFrictionCoefficient(double gd) {return mu;}
      Vec operator()(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Mat diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Vec solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN);
      bool isFullfield(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double tolla, double tolgd);
      int getFrictionDirections() {return 2;}
  };

  class RegularizedConstraintLaw {
    public:
      RegularizedConstraintLaw() {};
      virtual ~RegularizedConstraintLaw() {};
      virtual double operator()(double g,  double gd) = 0;
      virtual bool isClosed(double gN) = 0;
      virtual bool remainsClosed(double gd) = 0;
  };

  class RegularizedUnilateralConstraint : public RegularizedConstraintLaw {
    public:
      RegularizedUnilateralConstraint() {};
      virtual ~RegularizedUnilateralConstraint() {};
      bool isClosed(double g, double gTol) {return g<=gTol;}
      bool remainsClosed(double s, double sTol) {return s<=sTol;}
  };

  class RegularizedBilateralConstraint : public RegularizedConstraintLaw {
    public:
      RegularizedBilateralConstraint() {};
      virtual ~RegularizedBilateralConstraint() {};
      bool isClosed(double g, double gTol) {return true;}
      bool remainsClosed(double s, double sTol) {return true;}
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

  class RegularizedFrictionLaw {
    protected:
      double gdT_grenz;
    public:
      RegularizedFrictionLaw() : gdT_grenz(0.1) {};
      virtual ~RegularizedFrictionLaw() {};
      virtual Vec operator()(const Vec &gd, double laN) = 0; 
      virtual int getFrictionDirections() = 0;
      virtual bool isSticking(const Vec& s, double sTol) = 0;
  };

  class RegularizedDryFriction : public RegularizedFrictionLaw {
    protected:
    public:
      RegularizedDryFriction() {};
      virtual ~RegularizedDryFriction() {};
 };

  class LinearRegularizedPlanarCoulombFriction : public RegularizedDryFriction {
    private:
      double mu;
    public:
      LinearRegularizedPlanarCoulombFriction() {};
      LinearRegularizedPlanarCoulombFriction(double mu_) : mu(mu_) {};
      virtual ~LinearRegularizedPlanarCoulombFriction() {}
      void setFrictionCoefficient(double mu_) {mu = mu_;}
      int getFrictionDirections() {return 1;}
      bool isSticking(const Vec& s, double sTol) {return abs(s(0)) <= sTol;}
      Vec operator()(const Vec &gd, double laN) { 
	if(fabs(gd(0)) < gdT_grenz)
	  return Vec(1,INIT,-laN*mu*gd(0)/gdT_grenz);
	else
	  return Vec(1,INIT,gd(0)>0?-laN*mu:laN*mu);
      }
  };
  class LinearRegularizedSpatialCoulombFriction : public RegularizedDryFriction {
    private:
      double mu;
    public:
      LinearRegularizedSpatialCoulombFriction() {};
      LinearRegularizedSpatialCoulombFriction(double mu_) : mu(mu_) {};
      virtual ~LinearRegularizedSpatialCoulombFriction() {}
      void setFrictionCoefficient(double mu_) {mu = mu_;}
      int getFrictionDirections() {return 2;}
      bool isSticking(const Vec& s, double sTol) {return nrm2(s(0,1)) <= sTol;}
      Vec operator()(const Vec &gd, double laN) { 
	double norm_gdT = nrm2(gd);
	if(norm_gdT < gdT_grenz)
	  return gd*(-laN*mu/gdT_grenz);
	else
	  return gd*(-laN*mu/norm_gdT);
      }
  };

  class LinearRegularizedStribeckFriction : public RegularizedDryFriction {
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
	double norm_gdT = nrm2(gd(0,nFric-1));
	if(norm_gdT < gdT_grenz) {
	  double mu0 = (*fmu)(0)(0);
	  la(0,nFric-1) = gd(0,nFric-1)*(-laN*mu0/gdT_grenz);
	} else {
	  double mu = (*fmu)(nrm2(gd(0,nFric-1))-gdT_grenz)(0); //oder (*fmu)(nrm2(gd(0,nFric-1)))(0)
	  la(0,nFric-1) = gd(0,nFric-1)*(-laN*mu/norm_gdT);
	}
	return la;
      }
  };

}
#endif
