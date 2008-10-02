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

  class ContactLaw {
    public:
      ContactLaw() {};
      virtual ~ContactLaw() {};
      virtual bool isActive(double g) = 0;
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      virtual double operator()(double la, double gdn, double gda, double r) = 0;
      virtual Vec diff(double la, double gdn, double gda, double r) = 0;
      virtual double solve(double G, double gdn, double gda) = 0;
      virtual bool isFullfield(double la,  double gdn, double gda, double tolla, double tolgd) = 0;
      //virtual bool isActive(const Vec& g) {return g(0)<=0;}
      //virtual Vec operator()(const Vec& la, const Vec& gdn, const Vec& gda, const Vec& r) = 0;
      //virtual Vec diff(const Vec& la, const Vec& gdn, const Vec& gda, const Vec& r) = 0;
      //virtual Vec solve(const SymMat& G, const Vec& gdn, const Vec& gda) = 0;
      //virtual bool isFullfield(const Vec& la,  const Vec& gdn, const Vec& gda, double tolla, double tolgd) = 0;
  };

  class UnilateralContact : public ContactLaw {
    protected:
      double epsilon, gd_limit;
    public:
      UnilateralContact() : epsilon(0), gd_limit(1e-2) {};
      UnilateralContact(double epsilon_) : epsilon(epsilon_), gd_limit(1e-2) {};
      UnilateralContact(double epsilon_, double gd_limit_) : epsilon(epsilon_), gd_limit(gd_limit_) {};
      virtual ~UnilateralContact() {};
      bool isActive(double g) {return g<=0;}
      void load(const string& path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);
      double operator()(double la, double gdn, double gda, double r);
      Vec diff(double la, double gdn, double gda, double r);
      double solve(double G, double gdn, double gda);
      bool isFullfield(double la,  double gdn, double gda, double tolla, double tolgd);
      //Vec operator()(const Vec& la,  const Vec& gdn, const Vec& gda, const Vec& r);
      //Vec diff(const Vec& la,  const Vec& gdn, const Vec& gda, const Vec& r);
      //Vec solve(const SymMat& G, const Vec& gdn, const Vec& gda);
      //bool isFullfield(const Vec& la,  const Vec& gdn, const Vec& gda, double tolla, double tolgd);
  };

  class BilateralContact : public ContactLaw {
    public:
      BilateralContact() {};
      virtual ~BilateralContact() {};
      bool isActive(double g) {return true;}
      void load(const string& path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);
      double operator()(double la, double gdn, double gda, double r);
      Vec diff(double la, double gdn, double gda, double r);
      double solve(double G, double gdn, double gda);
      bool isFullfield(double la,  double gdn, double gda, double tolla, double tolgd);
      //bool isActive(const Vec& g) {return true;}
      //Vec operator()(const Vec& la,  const Vec& gdn, const Vec& gda, const Vec& r);
      //Vec diff(const Vec& la,  const Vec& gdn, const Vec& gda, const Vec& r);
      //Vec solve(const SymMat& G, const Vec& gdn, const Vec& gda);
      //bool isFullfield(const Vec& la,  const Vec& gdn, const Vec& gda, double tolla, double tolgd);
  };

  class FrictionLaw {
    public:
      FrictionLaw() {};
      virtual ~FrictionLaw() {};
      virtual void load(const string& path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      //virtual Vec operator()(const Vec& la, const Vec& gdn, const Vec& gda, const Vec& r) = 0;
      //virtual Mat diff(const Vec& la, const Vec& gdn, const Vec& gda, const Vec& r) = 0;
      //virtual Vec solve(const SymMat& G, const Vec& laN, const Vec& gdn, const Vec& gda) = 0;
      //virtual bool isFullfield(const Vec& la, const Vec& gdn, const Vec &gda, double tolla, double tolgd) = 0;
      virtual Vec operator()(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) = 0;
      virtual Mat diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) = 0;
      virtual Vec solve(const SymMat& G, const Vec& gdn, const Vec& gda, double laN) = 0;
      virtual bool isFullfield(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double tolla, double tolgd) = 0;
      virtual int getFrictionDirections() = 0;
  };

  class DryFriction : public FrictionLaw {
    public:
      DryFriction() {};
      virtual ~DryFriction() {};
      virtual double getFrictionCoefficient(double gd) = 0; 
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
      Vec operator()(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Mat diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Vec solve(const SymMat& G, const Vec& gdn, const Vec& gda, double laN);
      bool isFullfield(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double tolla, double tolgd);
      int getFrictionDirections() {return 1;}
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
      //Vec operator()(const Vec& la, const Vec& gdn, const Vec& gda, const Vec& r); 
      //Mat diff(const Vec& la, const Vec& gdn, const Vec& gda, const Vec& r);
      //Vec solve(const SymMat& G, const Vec& laN, const Vec& gdn, const Vec& gda);
      //bool isFullfield(const Vec& la, const Vec& gdn, const Vec& gda, double tolla, double tolgd);
      Vec operator()(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Mat diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r);
      Vec solve(const SymMat& G, const Vec& gdn, const Vec& gda, double laN);
      bool isFullfield(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double tolla, double tolgd);
      int getFrictionDirections() {return 2;}
  };

  class RegularizedContactLaw {
    public:
      RegularizedContactLaw() {};
      virtual ~RegularizedContactLaw() {};
      virtual double operator()(double g,  double gd) = 0;
      virtual bool isActive(double gN) = 0;
  };

  class RegularizedUnilateralContact : public RegularizedContactLaw {
    public:
      RegularizedUnilateralContact() {};
      virtual ~RegularizedUnilateralContact() {};
      virtual bool isActive(double gN) {return gN<=0;}
  };

  class RegularizedBilateralContact : public RegularizedContactLaw {
    public:
      RegularizedBilateralContact() {};
      virtual ~RegularizedBilateralContact() {};
      virtual bool isActive(double gN) {return true;}
  };

  class LinearRegularizedUnilateralContact: public RegularizedUnilateralContact {
    private:
      double c, d;
    public:
      LinearRegularizedUnilateralContact() : c(0), d(0) {};
      LinearRegularizedUnilateralContact(double c_, double d_) : c(c_), d(d_) {};
      virtual ~LinearRegularizedUnilateralContact() {};
      double operator()(double g,  double gd) { 
	if(g>0)
	  return 0;
	else if(gd<0) 
	  return -c*g - d*gd;
	else
	  return -c*g;
      }
  };

  class LinearRegularizedBilateralContact: public RegularizedBilateralContact {
    private:
      double c, d;
    public:
      LinearRegularizedBilateralContact() : c(0), d(0) {};
      LinearRegularizedBilateralContact(double c_, double d_) : c(c_), d(d_) {};
      virtual ~LinearRegularizedBilateralContact() {};
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

 // class StiffConnection {
 //   public:
 //     StiffConnection() {};
 //     virtual ~StiffConnection() {};
 //     virtual Vec operator()(const Vec &la ,const Vec &g, const Vec& r);
 //     virtual Vec diff(const Vec& la, const Vec &gdn, const Vec& r);
 //     virtual double solve(const SymMat& G, const Vec& gdn);
 //     virtual bool isFullfield(const Vec& la, const Vec& gdn, double tolgd);
 // };

  class RegularizedConnection {
    public:
      RegularizedConnection() {};
      virtual ~RegularizedConnection() {};
      virtual Vec operator()(const Vec &g, const Vec &gd) = 0;
  };

  class LinearRegularizedConnection : public RegularizedConnection {
    public:
      SqrMat C, D;
      LinearRegularizedConnection() {};
      LinearRegularizedConnection(const SqrMat &C_, const SqrMat &D_) : C(C_), D(D_) {};
      virtual ~LinearRegularizedConnection() {};
      Vec operator()(const Vec &g, const Vec &gd) {
	return -C*g - D*gd;
      }
  };

  class SpringDamperConnection : public RegularizedConnection {
    public:
      double c, d;
      SpringDamperConnection() {};
      SpringDamperConnection(double c_, double d_) : c(c_), d(d_) {};
      virtual ~SpringDamperConnection() {};
      void setStiffness(double c_) {c = c_;}
      void setDamping(double d_) {d = d_;}
      Vec operator()(const Vec &g, const Vec &gd) {
	return -c*g - d*gd;
      }
  };

}
#endif
