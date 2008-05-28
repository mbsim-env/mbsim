/* Copyright (C) 2004-2006  Martin Förg
 
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

#ifndef _OBJECT_H_
#define _OBJECT_H_

#include<string>
#include<vector>
#include "element.h"

using namespace std;

namespace MBSim {

  class Link;
  class LinkPort;
  class LinkContour;
  class Port;
  class Constraint;
  class ForceElement;
  class ContactFlexible;
  class ContactRigid;
  class Contour;

  /*! \brief Node class for all Objects having own dynamics and mass */
  class Object : public Element {

    friend class MultiBodySystem;

    protected:
	    struct LinkPortData {
	      LinkPort* link;
	      int ID;
	      int objectID;
	    };
	    
	    struct LinkContourData {
	      LinkContour* link;
	      int ID;
	      int objectID;
	    };
	    
		/** link data vectors */
	    vector<LinkPortData> linkSingleValuedPortData;
	    vector<LinkPortData> linkSetValuedPortData;
	    vector<LinkContourData> linkSingleValuedContourData;
	    vector<LinkContourData> linkSetValuedContourData;	
		/** set-valued and single-valued links */
	    vector<Link*> linkSetValued;
	    vector<Link*> linkSingleValued;
		/** ports */
	    vector<Port*> port;
	    /** contours */
	    vector<Contour*> contour;
	
	    /** size of object positions */
	    int qSize;
	    /** size of object velocities */
	    int uSize;
	    /** size of object order one parameters */
	    int xSize;
	    /** indices of q, u, x */
	    int qInd, uInd, xInd;
	    /** indices for velocities and order one parameters */
	    Index Iu, Ix;
	
	    /** Object positions, velocities, order one parameters */
	    Vec q, u, x;
	    /** initial values */
	    Vec q0,u0,x0;
	    /** velocities */
	    Vec qd,ud,xd;
	    /** smooth, set-valued and control vector */
	    Vec h,r,f;
	
	    /** linear relation matrix of position and velocity parameters */
	    Mat T;
	    /** mass matrix */
	    SymMat M;
	    /** LU-decomposition of mass matrix */
	    SymMat LLM;
	    /** force direction matrix */
	    vector<Mat> W;
	    /** for explicit time depending kinematical excitations */
	    vector<Vec> w;
	    
	    /** centre of hit sphere */
	    Vec WrOHitSphere;
	    /** radius of hit sphere */
	    double RHitSphere;
	    
		/*! Write positions to file for preintegration */
	    virtual void writeq();
	    /*! Read positions from file for preintegration */
	    virtual void readq0();
	    /*! Write velocities to file for preintegration */
	    virtual void writeu();
	    /*! Read velocities from file for preintegration */
	    virtual void readu0();
	    /*! Write order one parameters to file for preintegration */
	    virtual void writex();
	    /*! Write order one parameters from file for preintegration */
	    virtual void readx0();
	        
	    /*! References to state of multibody system mbs */
	    virtual void updatezRef();
	    /*! References to differentiated state of multibody system mbs */
	    virtual void updatezdRef();
	    /*! References to positions of multibody system mbs */
	    virtual void updateqRef();
	    /*! References to differentiated positions of multibody system mbs */
	    virtual void updateqdRef();
	    /*! References to velocities of multibody system mbs */
	    virtual void updateuRef();
	    /*! References to differentiated velocities of multibody system mbs */
	    virtual void updateudRef();
	    /*! References to order one parameters of multibody system mbs */
	    virtual void updatexRef();
	    /*! References to differentiated order one parameters of multibody system mbs */
	    virtual void updatexdRef();
	    /*! References to smooth vector of multibody system mbs */
	    virtual void updatehRef();
	    /*! References to set valued vector of multibody system mbs */
	    virtual void updaterRef();
	    /*! References to control vector of multibody system mbs */
	    virtual void updatefRef();
	    /*! References to T-matrix of multibody system mbs */
	    virtual void updateTRef();
	    /*! References to mass matrix of multibody system mbs */
	    virtual void updateMRef();
	    /*! References to cholesky decomposition of mass matrix of multibody system mbs */
	    virtual void updateLLMRef();
	
	    /*! Initialize object at start of simulation with respect to contours and ports */
	    virtual void init();
	    /*! Initialize state of object at start of simulation */
	    virtual void initz();
		/*! Updates mass matrix */
	    virtual void updateM(double t) {};
		/*! Updates kinematics */
	    virtual void updateKinematics(double t) = 0;
	    /*! Update Delassus matrix and vector for constrained equations */
	    virtual void updateGb(double t);
	    /*! Updates force direction matrix for absolute parametrisation */
	    virtual void updateW(double t);
	    /*! Updates explicit time depending kinematical excitations for absolute parametrisation */
	    virtual void updatew(double t);
	    /*! Updates force direction matrix for relative parametrisation */
	    virtual void updateWj(double t) {};
	    /*! Updates explicit time depending kinematical excitations for absolute parametrisation */
	    virtual void updatewj(double t) {};
	    /*! Update relation matrix between position and velocity parameters */
	    virtual void updateT(double t) {};
	    /*! Updates smooth right hand side */
	    virtual void updateh(double t) {};
	    /*! Updates set-valued right hand side */
	    virtual void updater(double t);
	    /*! Updates differentiated state */
	    virtual void updatezd(double t) = 0;
	    /*! Updates velocity gaps */
	    virtual void updatedu(double t, double dt) = 0;
	    /*! Updates position gaps */
	    virtual void updatedq(double t, double dt) = 0;
	    /*! Updates order one parameter gaps */
	    virtual void updatedx(double t, double dt) {};
	
	    /*! Perform LL-decomposition of mass martix */
	    virtual void facLLM();
		/*! Return self-reference */
	    virtual Object* getResponsible() {return this;}
	
    public:
	    /*! Constructor */
	    Object(const string &name);
	    /*! Destructor */
	    virtual ~Object();
	
		/*! Set indices of q */
	    void setqInd(int qInd_) { qInd = qInd_; }
	    /*! Set indices of u */
	    void setuInd(int uInd_) { uInd = uInd_; Iu = Index(uInd,uInd+uSize-1); }
	    /*! Set indices of x */
	    void setxInd(int xInd_) { xInd = xInd_; Ix = Index(xInd,xInd+xSize-1); }
	    /*! Get indices of q */
	    int getqInd() { return qInd; }
	    /*! Get indices of u */
	    int getuInd() { return uInd; }
	    /*! Get indices of x */
	    int getxInd() { return xInd; }
	    /*! Returns index for velocities */
	    const Index& getuIndex() const { return Iu;}
	    /*! Returns index for order one parameters */
	    const Index& getxIndex() const { return Ix;}
	
	    /*! Get size of position vector */
	    int getqSize() const { return qSize; }
	    /*! Get size of velocity vector */
	    int getuSize() const { return uSize; }
	    /*! Get size of order one parameter vector */
	    int getxSize() const { return xSize; }
	    /*! Get number of state variables */
	    int getzSize() const { return qSize + uSize + xSize; }
	    
	    /*! Get smooth force vector */
	    const Vec& geth() const {return h;};
	    /*! Get smooth force vector */
	    Vec& geth() {return h;};
	    /*! Get setvalued force vector */
	    const Vec& getr() const {return r;};
		/*! Get control vector */
	    const Vec& getf() const {return f;};
	    /*! Get control vector */
	    Vec& getf() {return f;};
	    /*! Get mass matrix */
	    const SymMat& getM() const {return M;};
	    /*! Get mass matrix */
	    SymMat& getM() {return M;};
	    /*! Get T-matrix */
	    const Mat& getT() const {return T;};
	    /*! Get T-matrix */
	    Mat& getT() {return T;};
	    /*! Get Cholesky decomposition of the mass matrix */
	    const SymMat& getLLM() const {return LLM;};
	    /*! Get Cholesky decomposition of the mass matrix */
	    SymMat& getLLM() {return LLM;};
		/*! Get positions */
	    const Vec& getq() const {return q;};
	    /*! Get velocities */
	    const Vec& getu() const {return u;};
	    /*! Get order one parameters */
	    const Vec& getx() const {return x;};
		/*! Get positions */
	    Vec& getq() {return q;};
	    /*! Get velocities */
	    Vec& getu() {return u;};
	    /*! Get order one parameters */
	    Vec& getx() {return x;};
		/*! Get initial positions */
	    const Vec& getq0() const {return q0;};
	    /*! Get initial velocities */
	    const Vec& getu0() const {return u0;};
	    /*! Set initial order one parameters */
	    const Vec& getx0() const {return x0;};
		/*! Get initial positions */
	    Vec& getq0() {return q0;};
	    /*! Get initial velocities */
	    Vec& getu0() {return u0;};
	    /*! Set initial order one parameters */
	    Vec& getx0() {return x0;};
		/*! Get initial differentiated positions */
	    const Vec& getqd() const {return qd;};
	    /*! Get initial differentiated velocities */
	    const Vec& getud() const {return ud;};
	    /*! Get initial order one parameters */
	    const Vec& getxd() const {return xd;};
		/*! Get initial differentiated positions */
	    Vec& getqd() {return qd;};
	    /*! Get initial differentiated velocities */
	    Vec& getud() {return ud;};
	    /*! Get initial order one parameters */
	    Vec& getxd() {return xd;};
		/*! Set positions */
	    void setq(Vec q_) { q = q_; }
	    /*! Set velocities */
	    void setu(Vec u_) { u = u_; }
	    /*! Set order one parameters */
	    void setx(Vec x_) { x = x_; }	
	    /*! Set initial positions */
	    void setq0(Vec q0_) { q0 = q0_; }
	    /*! Set initial velocities */
	    void setu0(Vec u0_) { u0 = u0_; }
	    /*! Set initial order one parameters */
	    void setx0(Vec x0_) { x0 = x0_; }
		/*! Set initial positions */
	    void setq0(double q0_) { q0 = Vec(1,INIT,q0_); }
	    /*! Set initial velocities */
	    void setu0(double u0_) { u0 = Vec(1,INIT,u0_); }
	    /*! Set initial order one parameters */
	    void setx0(double x0_) { x0 = Vec(1,INIT,x0_); }
		/*! Get column dimension of W */
		virtual int getWSize() const { return uSize; }
		/*! Get W */
	    vector<Mat>& getWs() {return W;}
	    
	    /*! Plots intereseting data */ 
	    void plot(double t, double dt = 1);
	    /*! Initialises plot files */ 
	    void initPlotFiles();
		/*! Add link */
	    virtual void addLink(LinkPort *link, Port *port, int objectID);
	    /*! Add link */
	    virtual void addLink(LinkContour *link, Contour *contour, int objectID);
		/*! Add port */
	    virtual void addPort(Port * port);
	    /*! Add contour */
	    virtual void addContour(Contour* contour);
		/*! Get port */
	    virtual Port* getPort(const string &name, bool check=true);
	    /*! Get contour */
	    virtual Contour* getContour(const string &name, bool check=true);
		/*! Set centre of hit sphere */
	    void setWrHitSphere(const Vec &WrOS) {WrOHitSphere = WrOS;}
	    /*! Get centre of hit sphere */
	    const Vec& getWrHitSphere() const {return WrOHitSphere;}
	    /*! Set radius of hit sphere */
	    void setRadiusHitSphere(const double &R) {RHitSphere = R;}
	    /*! Get radius of hit sphere */
	    const double& getRadiusHitSphere() const {return RHitSphere;}
		/*! Calculates dimension of state */
	    virtual void calcSize() {};
	
	    /*! Compute kinetic energy, which is the quadratic form \f$\frac{1}{2}\boldsymbol{u}^T\boldsymbol{M}\boldsymbol{u}\f$ for all bodies */
	    virtual double computeKineticEnergy();
	    /*! Compute potential energy */
	    virtual double computePotentialEnergy() {return 0; }
	    /*! Compute Jacobian \f$\boldsymbol{J}={\partial\boldsymbol{h}}/{\partial\boldsymbol{z}}\f$ of generalized force vector */
	    virtual void updateJh(double t);
  };

}

#endif /* _OBJECT_H_ */
