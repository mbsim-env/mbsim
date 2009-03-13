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
#include <mbsim/element.h>
#include <mbsim/interfaces.h>

using namespace std;

namespace MBSim {

  class Frame;
  class Contour;
  class Subsystem;

  /*! \brief Node class for all Objects having own dynamics and mass */
  class Object : public Element, public ObjectInterface {

//    friend class MultiBodySystem;

    public: 

    vector<Frame*> port;
    vector<Contour*> contour;

    protected:

    Subsystem* parent;

    /** Size of object positions */
    int qSize;
    /** Size of object velocities */
    int uSize[2];
    /** Size of object h-vector (columns of J) */
    int hSize[2];

    int qInd, uInd[2], hInd[2];

    /** Object positions */
    Vec q;
    /** Object velocities */
    Vec u;
    /** Object order one parameters */
    Vec q0,u0;
    Vec qd,ud;
    Vec h, r;

    /** linear relation matrix \f$\boldsymbol{T}\f$ of position and velocity parameters */
    Mat T;
    /** mass matrix \f$\boldsymbol{M}\f$*/
    SymMat M;
    /** LU-decomposition of mass matrix \f$\boldsymbol{M}\f$*/
    SymMat LLM;
    Index Iu, Ih;
    
    public: // TODO nur zum Testen

    virtual void writeq();
    virtual void readq0();
    virtual void writeu();
    virtual void readu0();
    virtual void writex();
    virtual void readx0();

    virtual void updateqRef(const Vec& ref);
    virtual void updateqdRef(const Vec& ref);
    virtual void updateuRef(const Vec& ref);
    virtual void updateudRef(const Vec& ref);
    virtual void updatehRef(const Vec& ref, int i=0);
    virtual void updaterRef(const Vec& ref);
    virtual void updateTRef(const Mat &ref);
    virtual void updateMRef(const SymMat &ref, int i=0);
    virtual void updateLLMRef(const SymMat &ref, int i=0);

    void updateT(double t) {};
    void updateh(double t) {};
    void updateM(double t) {};
    void updatedq(double t, double dt);
    void updatedu(double t, double dt);
    void updateud(double t);
    void updateqd(double t);
    void updatezd(double t);

    /*! Initialize object at start of simulation with respect to contours and ports */
    virtual void init();

    /*! Initialize object at start of simulation with respect to contours and ports */
    virtual void preinit();
    
    /*! Initialize state of object at start of simulation */
    virtual void initz();

    /*! Perform LL-decomposition of mass martix \f$\boldsymbol{M}\f$*/
    virtual void facLLM();

    public:
    /*! Constructor */
    Object(const string &name);
    /*! Destructor */
    virtual ~Object();
  
    Subsystem* getParent() {return parent;}
    void setParent(Subsystem* sys) {parent = sys;}

    void setqSize(int qSize_) { qSize = qSize_; }
    void setuSize(int uSize_, int i=0) { uSize[i] = uSize_; }
    void sethSize(int hSize_, int i=0);// { hSize = hSize_; }
    void setqInd(int qInd_) { qInd = qInd_; }
    void setuInd(int uInd_, int i=0) { uInd[i] = uInd_; }
    void sethInd(int hInd_, int i=0); // { hInd = hInd_; }
    int  getqInd() { return qInd; }
    int  getuInd(int i=0) { return uInd[i]; }
    int  gethInd(int i=0) { return hInd[i]; }

    int gethInd(Subsystem* sys,int i=0); 

    /*! Get size of position vector Object::q \return Object::qSize */
    int getqSize() const { return qSize; }
    /*! Get size of velocity vector Object::u \return Object::uSize */
    int getuSize(int i=0) const { return uSize[i]; }
    /*! Get number of state variables \return Object::qSize + Object::uSize */
    int getzSize() const { return qSize + uSize[0]; }
    /*! Get size of Jacobian matrix Object::h \return Object::hSize */
    int gethSize(int i=0) const { return hSize[i]; }

    const Index& getuIndex() const { return Iu;}
    const Index& gethIndex() const { return Ih;}
    /*! Get smooth force vector */
    const Vec& geth() const {return h;};
    /*! Get smooth force vector */
    Vec& geth() {return h;};
    /*! Get setvalued force vector */
    const Vec& getr() const {return r;};
    /*! Get setvalued force vector */
    Vec& getr() {return r;};


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

    Vec& getq() {return q;};
    Vec& getu() {return u;};

    const Vec& getq0() const {return q0;};
    const Vec& getu0() const {return u0;};

    Vec& getq0() {return q0;};
    Vec& getu0() {return u0;};

    const Vec& getqd() const {return qd;};
    const Vec& getud() const {return ud;};

    Vec& getqd() {return qd;};
    Vec& getud() {return ud;};

    void setq(const Vec &q_) { q = q_; }
    void setu(const Vec &u_) { u = u_; }
	
    /*! Set initial positions with \param q0_ */
    void setq0(const Vec &q0_) { q0 = q0_; }
    /*! Set initial velocities with \param u0_ */
    void setu0(const Vec &u0_) { u0 = u0_; }

    void setq0(double q0_) { q0 = Vec(1,INIT,q0_); }
    void setu0(double u0_) { u0 = Vec(1,INIT,u0_); }

    void load(const string &path, ifstream &inputfile);
    void save(const string &path, ofstream &outputfile);

    virtual void addFrame(Frame * port);
    virtual void addContour(Contour* contour);

    int portIndex(const Frame *port_) const;

    //string getFullName() const; 

    virtual Frame* getFrame(const string &name, bool check=true);
    const vector<Frame*>& getFrames() const {return port;}
    virtual Contour* getContour(const string &name, bool check=true);
    const vector<Contour*>& getContours() const {return contour;}

    virtual void calcqSize() {};
    virtual void calcuSize(int j) {};
    virtual void calchSize(int j);

    /*! Compute kinetic energy, which is the quadratic form \f$\frac{1}{2}\boldsymbol{u}^T\boldsymbol{M}\boldsymbol{u}\f$ for all bodies */
    virtual double computeKineticEnergy();
    /*! Compute potential energy */
    virtual double computePotentialEnergy() {return 0; }
    /*! Compute Jacobian \f$\boldsymbol{J}={\partial\boldsymbol{h}}/{\partial\boldsymbol{z}}\f$ of generalized force vector */

    virtual void resizeJacobians(int j) {}
    virtual void checkForConstraints() {}

    virtual string getType() const {return "Object";}
    //virtual MultiBodySystem* getMultiBodySystem(); 

    void setMultiBodySystem(MultiBodySystem *sys);
    void setFullName(const string &str);

    virtual void plot(double t, double dt = 1, bool top=true); 
    virtual void initPlot(bool top=true);
    virtual void closePlot();
    H5::Group *getPlotGroup() { return plotGroup; }
    PlotFeatureStatus getPlotFeature(PlotFeature fp) { return Element::getPlotFeature(fp); };
    PlotFeatureStatus getPlotFeatureForChildren(PlotFeature fp) { return Element::getPlotFeatureForChildren(fp); };
  };

}

#endif
