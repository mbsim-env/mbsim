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

  class CoordinateSystem;
  class Contour;
  class Subsystem;

  /*! \brief Node class for all Objects having own dynamics and mass */
  class Object : public Element {

    friend class MultiBodySystem;

    protected:
    Subsystem* parent;

    public: 

    vector<CoordinateSystem*> port;
    vector<Contour*> contour;

    protected:

    /** Size of object positions */
    int qSize;
    /** Size of object velocities */
    int uSize;
    /** Size of object order one parameters */
    int xSize;
    /** Size of object h-vector (columns of J) */
    int hSize;

    int qInd, uInd, xInd, hInd;

    /** Object positions */
    Vec q;
    /** Object velocities */
    Vec u;
    /** Object order one parameters */
    Vec x;
    Vec q0,u0,x0;
    Vec qd,ud,xd;
    Vec h,r,f;

    /** linear relation matrix \f$\boldsymbol{T}\f$ of position and velocity parameters */
    Mat T;
    /** mass matrix \f$\boldsymbol{M}\f$*/
    SymMat M;
    /** LU-decomposition of mass matrix \f$\boldsymbol{M}\f$*/
    SymMat LLM;
    Mat W;
    Index Iu, Ix, Ih;
    SymMat G;
    Vec b;
    Mat Jh;

    public: // TODO nur zum Testen

    virtual void writeq();
    virtual void readq0();
    virtual void writeu();
    virtual void readu0();
    virtual void writex();
    virtual void readx0();
    Vec  WrOHitSphere;
    double RHitSphere;
    virtual void updatezRef(); // references to parent data
    virtual void updatezdRef();
    virtual void updateqRef();
    virtual void updateqdRef();
    virtual void updateuRef();
    virtual void updateudRef();
    virtual void updatexRef();
    virtual void updatexdRef();
    virtual void updatehRef();
    virtual void updaterRef();
    virtual void updatefRef();
    virtual void updateTRef();
    virtual void updateMRef();
    virtual void updateLLMRef();

    virtual void updateT(double t) {};
    virtual void updateh(double t) {};
    virtual void updater(double t) {};
    virtual void updateM(double t) {};
    virtual void updateKinematics(double t) = 0;
    virtual void updatedx(double t, double dt) {};
    virtual void updatedq(double t, double dt);
    virtual void updatezd(double t);
    virtual void updatedu(double t, double dt);

    virtual void setParent(Subsystem *p) {parent = p;}

    /*! Initialize object at start of simulation with respect to contours and ports */
    virtual void init();
    
    /*! Initialize state of object at start of simulation */
    virtual void initz();

    /*! Perform LL-decomposition of mass martix \f$\boldsymbol{M}\f$*/
    virtual void facLLM();

    //virtual Object* getResponsible() {return this;}

////    /*! compute Jacobian of right-hand side for parts depending only on this bodies coordinates
////    */
////    //virtual void updateJh_internal(double t);
////    /*! compute Jacobian of right-hand side for single-valued links
////    */
////    //virtual void updateJh_links(double t);
////	/* ************************************************************************/
	
    public:
    /*! Constructor */
    Object(const string &name);
    /*! Destructor */
    virtual ~Object();
  
    void setqSize(int qSize_) { qSize = qSize_; }
    void setuSize(int uSize_) { uSize = uSize_; }
    void setxSize(int xSize_) { xSize = xSize_; }
    void sethSize(int hSize_) { hSize = hSize_; }
    void setqInd(int qInd_) { qInd = qInd_; }
    void setuInd(int uInd_) { uInd = uInd_; }
    void setxInd(int xInd_) { xInd = xInd_; }
    void sethInd(int hInd_) { hInd = hInd_; }
    int  getqInd() { return qInd; }
    int  getuInd() { return uInd; }
    int  getxInd() { return xInd; }
    int  gethInd() { return hInd; }

    Mat& getJh() {return Jh;}
    const Vec& getb() const {return b;}
    Vec& getb() {return b;}
    const Mat& getW() const {return W;}
    Mat& getW() {return W;}

    /*! Get size of position vector Object::q \return Object::qSize */
    int getqSize() const { return qSize; }
    /*! Get size of velocity vector Object::u \return Object::uSize */
    int getuSize() const { return uSize; }
    /*! Get size of order one parameter vector Object::x \return Object::xSize */
    int getxSize() const { return xSize; }
    /*! Get number of state variables \return Object::qSize + Object::uSize + Object::xSize */
    int getzSize() const { return qSize + uSize + xSize; }
    /*! Get size of Jacobian matrix Object::h \return Object::hSize */
    int gethSize() const { return hSize; }
    virtual int getWSize() const { return uSize; }

    const Index& getuIndex() const { return Iu;}
    const Index& getxIndex() const { return Ix;}
    const Index& gethIndex() const { return Ih;}
    /*! Get smooth force vector */
    const Vec& geth() const {return h;};
    /*! Get smooth force vector */
    Vec& geth() {return h;};
    /*! Get setvalued force vector */
    const Vec& getr() const {return r;};
    /*! Get setvalued force vector */
    Vec& getr() {return r;};

    const Vec& getf() const {return f;};
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

    const Vec& getq() const {return q;};
    const Vec& getu() const {return u;};
    const Vec& getx() const {return x;};

    Vec& getq() {return q;};
    Vec& getu() {return u;};
    Vec& getx() {return x;};

    const Vec& getq0() const {return q0;};
    const Vec& getu0() const {return u0;};
    const Vec& getx0() const {return x0;};

    Vec& getq0() {return q0;};
    Vec& getu0() {return u0;};
    Vec& getx0() {return x0;};

    const Vec& getqd() const {return qd;};
    const Vec& getud() const {return ud;};
    const Vec& getxd() const {return xd;};

    Vec& getqd() {return qd;};
    Vec& getud() {return ud;};
    Vec& getxd() {return xd;};

    void setq(Vec q_) { q = q_; }
    void setu(Vec u_) { u = u_; }
    void setx(Vec x_) { x = x_; }
	
    /*! Set initial positions with \param q0_ */
    void setq0(Vec q0_) { q0 = q0_; }
    /*! Set initial velocities with \param u0_ */
    void setu0(Vec u0_) { u0 = u0_; }
    void setx0(Vec x0_) { x0 = x0_; }

    void setq0(double q0_) { q0 = Vec(1,INIT,q0_); }
    void setu0(double u0_) { u0 = Vec(1,INIT,u0_); }
    void setx0(double x0_) { x0 = Vec(1,INIT,x0_); }

    //vector<Mat>& getWj() {return Wj;}

    //void plot(double t); 
    void plot(double t, double dt = 1); 
    void initPlotFiles();
    void plotParameters();
    void load(const string &path, ifstream &inputfile);

    virtual void addCoordinateSystem(CoordinateSystem * port);
    virtual void addContour(Contour* contour);

    int portIndex(const CoordinateSystem *port_) const;

    string getFullName() const; 

    virtual CoordinateSystem* getCoordinateSystem(const string &name, bool check=true);
    const vector<CoordinateSystem*>& getCoordinateSystems() const {return port;}
    virtual Contour* getContour(const string &name, bool check=true);
    const vector<Contour*>& getContours() const {return contour;}

    void   setWrHitSphere(const Vec &WrOS)     {WrOHitSphere = WrOS;}
    const Vec&    getWrHitSphere()  const      {return WrOHitSphere;}
    void   setRadiusHitSphere(const double &R) {RHitSphere = R;}
    const double& getRadiusHitSphere() const   {return RHitSphere;}

    virtual void calcSize() {};
    virtual void calchSize() {};

    /*! Compute kinetic energy, which is the quadratic form \f$\frac{1}{2}\boldsymbol{u}^T\boldsymbol{M}\boldsymbol{u}\f$ for all bodies */
    virtual double computeKineticEnergy();
    /*! Compute potential energy */
    virtual double computePotentialEnergy() {return 0; }
    /*! Compute Jacobian \f$\boldsymbol{J}={\partial\boldsymbol{h}}/{\partial\boldsymbol{z}}\f$ of generalized force vector */
    virtual void updateJh(double t);

    virtual string getType() const {return "Object";}
    virtual MultiBodySystem* getMultiBodySystem(); 
  };

}

#endif
