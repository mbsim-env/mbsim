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

#include <string>
#include <vector>
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

  /*! 
   *  \brief base class for all Objects having own dynamics and mass
   *
   * */
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

    protected:

    vector<LinkPortData> linkSingleValuedPortData;
    vector<LinkPortData> linkSetValuedPortData;
    vector<LinkContourData> linkSingleValuedContourData;
    vector<LinkContourData> linkSetValuedContourData;

    vector<Link*> linkSetValued;

    vector<Link*> links;
    vector<Port*> port;
    vector<Contour*> contour;

    /** size of object positions */
    int qSize;
    /** size of object velocities */
    int uSize;
    /** size of object order one parameters */
    int xSize;
    int qInd, uInd, xInd;

    /** object positions */
    Vec q;
    /** object velocities */
    Vec u;
    /** object order one parameters */
    Vec x;
    Vec q0,u0,x0;
    Vec qd,ud,xd;
    Vec h,r,f;

    Mat T;
    SymMat M;
    SymMat LLM;
    vector<Mat> W;
    vector<Vec> w;
    Index Iu, Ix;

    virtual void writeq();
    virtual void readq0();
    virtual void writeu();
    virtual void readu0();
    virtual void writex();
    virtual void readx0();
    Vec  WrOHitSphere;
    double RHitSphere;
    virtual void updatezRef();
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

    /*! initialize object at start of simulation */
    virtual void init();
    virtual void initz();

    virtual void updateM(double t) {};

    virtual void updateKinematics(double t) = 0;
    virtual void updateG(double t);
    virtual void updateW(double t) {};
    virtual void updateh(double t) {};
    virtual void updater(double t);
    virtual void updatezd(double t) = 0;
    virtual void updatedu(double t, double dt) = 0;
    virtual void updatedq(double t, double dt) = 0;
    virtual void updatedx(double t, double dt) {};

    public:
    Object(const string &name);
    virtual ~Object();

    void setqInd(int qInd_) { qInd = qInd_; }
    void setuInd(int uInd_) { uInd = uInd_; Iu = Index(uInd,uInd+uSize-1); }
    void setxInd(int xInd_) { xInd = xInd_; Ix = Index(xInd,xInd+xSize-1); }

    int getqSize() const { return qSize; }
    int getuSize() const { return uSize; }
    int getxSize() const { return xSize; }
    int getzSize() const { return qSize + uSize + xSize; }
    virtual int getWSize() const { return uSize; }

    const Index& getuIndex() const { return Iu;}
    const Index& getxIndex() const { return Ix;}

    const Vec& geth() const {return h;};
    const Vec& getr() const {return r;};
    Vec& geth() {return h;};

    const Vec& getf() const {return f;};
    Vec& getf() {return f;};

    const SymMat& getM() const {return M;};
    SymMat& getM() {return M;};

    const Mat& getT() const {return T;};
    Mat& getT() {return T;};

    const SymMat& getLLM() const {return LLM;};
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

    void setq0(Vec q0_) { q0 = q0_; }
    void setu0(Vec u0_) { u0 = u0_; }
    void setx0(Vec x0_) { x0 = x0_; }

    void setq0(double q0_) { q0 = Vec(1,INIT,q0_); }
    void setu0(double u0_) { u0 = Vec(1,INIT,u0_); }
    void setx0(double x0_) { x0 = Vec(1,INIT,x0_); }

    vector<Mat>& getWs() {return W;}

    //void plot(double t); 
    void plot(double t, double dt = 1); 
    void initPlotFiles();

    virtual void addLink(LinkPort *link, Port *port, int objectID);
    virtual void addLink(LinkContour *link, Contour *contour, int objectID);

    virtual void addPort(Port * port);
    virtual void addContour(Contour* contour);

    Port* getPort(const string &name, bool check=true);
    Contour* getContour(const string &name, bool check=true);

    void   setWrHitSphere(const Vec &WrOS)     {WrOHitSphere = WrOS;}
    const Vec&    getWrHitSphere()  const      {return WrOHitSphere;}
    void   setRadiusHitSphere(const double &R) {RHitSphere = R;}
    const double& getRadiusHitSphere() const   {return RHitSphere;}

    virtual void calcSize() {};

    /*! compute kinetic energy, which is the quadratic form \f$\frac{1}{2}\boldsymbol{u}^T\boldsymbol{M}\boldsymbol{u}\f$ for all bodies
    */
    virtual double computeKineticEnergy() { return 0.5*trans(u)*M*u; }

    /*! compute potential energy, holding every potential!!!
    */
    virtual double computePotentialEnergy() {return 0; }
  };

}

#endif
