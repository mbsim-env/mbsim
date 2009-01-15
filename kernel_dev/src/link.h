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

#ifndef _LINK_H_
#define _LINK_H_

#include <vector>
#include "element.h"
#include "interfaces.h"

#ifdef HAVE_AMVIS
namespace AMVis {class Arrow;}
#endif

namespace MBSim {
  class CoordinateSystem;
  class Contour;
  class HitSphereLink;
  class UserFunction;
  /*! 
   *  \brief This is a general link to one or more objects.
   * 
   * */
  class Link : public Element, public LinkInterface {

    protected:

      Subsystem* parent;

      /** Internal integrable State Variables */
      Vec x;
      /** Internal integrable State Variables velocities ud \see updatedu(double t, double dt), updateud(double t) */
      Vec xd;
      Vec x0;

      int xSize;
      int xInd;

      int svSize;
      Vec sv;
      int svInd;
      Vector<int> jsv;

      bool setValued;

      int gSize, gInd;
      int gdSize, gdInd;
      int laSize, laInd;
      int laIndMBS;

      vector<Vec> WF, WM;
      vector<Mat> fF, fM;

      int rFactorSize, rFactorInd;
      Vec g, gd, la, res;

      Vec rFactor;
      Vector<int> rFactorUnsure;

      //bool active;
      Index Ig, Ila;
      Vec la0;

      double scaleTolQ,scaleTolp;
      double gdTol, gddTol, laTol, LaTol, rMax;

      HitSphereLink* HSLink;
      bool checkHSLink;

      vector<Mat> W;
      vector<Mat> V;
      vector<Vec> h;
      vector<Vec> r;
      Vec wb;

      /*! Array in which all ports are listed, connecting bodies via a Link.
      */
      vector<CoordinateSystem*> port;

      /** Array in which all contours linked by LinkContour are managed.*/
      vector<Contour*> contour;

#ifdef HAVE_AMVIS
      vector<AMVis::Arrow*> arrowAMVis;
      vector<double> arrowAMVisScale;
      vector<int> arrowAMVisID;
      vector<bool> arrowAMVisMoment;
      vector<UserFunction*> arrowAMVisUserFunctionColor;
#endif

    public:

      virtual void updater(double t);
      virtual void updatewb(double t) {};
      virtual void updateW(double t) {};
      virtual void updateV(double t) {};
      virtual void updateh(double t) {};
      virtual void updatedx(double t, double dt) {}
      virtual void updatexd(double t) {}
      virtual void updateStopVector(double t) {}

      virtual void updateWRef(const Mat& ref, int i=0);
      virtual void updateVRef(const Mat& ref, int i=0);
      virtual void updatehRef(const Vec &ref, int i=0);
      virtual void updaterRef(const Vec &ref);
      virtual void updatewbRef(const Vec &ref);
      virtual void updatefRef(const Vec &ref) {};

      Link(const string &name, bool setValued);
      Link(const string &name);
      ~Link();

      Subsystem* getParent() {return parent;}
      void setParent(Subsystem* sys) {parent = sys;}

      const vector<Mat>& getW() const {return W;}
      const vector<Mat>& getV() const {return V;}
      const vector<Vec>& geth() const {return h;}

      void setxInd(int xInd_) {xInd = xInd_;};
      void setsvInd(int svInd_) {svInd = svInd_;};

      int getlaIndMBS() const {return laIndMBS;}
      void setlaIndMBS(int laIndParent) { laIndMBS = laInd + laIndParent; }

      int getxSize() const {return xSize;}
      int getsvSize() const {return svSize;}

      /*! activate HitSphereLink-check for this Link before updateStage1(), only usefull for Contacts
       * 	\param checkHSLink_, true for checks, false for no check
       */
      void setHitSphereCheck(bool checkHSLink_) {checkHSLink=checkHSLink_;}
      /*! \return HSLink for MultiBodySystem
       *  */
      bool getHitSphereCheck() {return checkHSLink;}

      virtual void calcxSize() {xSize = 0;}
      virtual void calcsvSize() {svSize = 0;}
      virtual void calclaSize() {laSize = 0;}
      virtual void calcgSize() {gSize = 0;}
      virtual void calcgdSize() {gdSize = 0;}
      virtual void calcrFactorSize() {rFactorSize = 0;}

      virtual void init();
      virtual void preinit() {}
      virtual void initz();


      /*! compute potential energy, holding every potential!!!
      */
      virtual double computePotentialEnergy() {return 0;}


      const Vec& getx() const {return x;}
      const Vec& getxd() const {return xd;}
      /*! Sets the internal states of a Link.*/
      void setx(const Vec &x_) {x = x_;}

      void plot(double t, double dt=1);
      void initPlotFiles();

      //bool isSetValued() const {return setValued;} 
      virtual bool isSetValued() const {return false;}

      const Vec& getla() const {return la;}
      Vec& getla() {return la;}
      const Vec& getg() const {return g;}
      Vec& getg() {return g;}
      //void setg(const Vec& g_) {g = g_;}
      //void setgd(const Vec& gd_) {gd = gd_;}
      int getgSize() const {return gSize;} 
      int getgdSize() const {return gdSize;} 
      int getlaSize() const {return laSize;} 
      int getgdInd() const {return gdInd;} 
      int getlaInd() const {return laInd;} 
      int getrFactorSize() const {return rFactorSize;} 
      const Index& getgIndex() const {return Ig;}
      const Index& getlaIndex() const {return Ila;}
      virtual bool isActive() const = 0; 
      virtual bool gActiveChanged() = 0;

      void savela();
      void initla();

      const Vector<int>& getrFactorUnsure() const {return rFactorUnsure;}

      virtual void updatexRef(const Vec& ref);
      virtual void updatexdRef(const Vec& ref);
      virtual void updatelaRef(const Vec& ref);
      virtual void updategRef(const Vec& ref);
      virtual void updategdRef(const Vec& ref);
      virtual void updateresRef(const Vec& ref);
      virtual void updaterFactorRef(const Vec& ref);
      virtual void updatesvRef(const Vec &sv);
      virtual void updatejsvRef(const Vector<int> &jsvParent);

      void setgInd(int gInd_) {gInd = gInd_;Ig=Index(gInd,gInd+gSize-1);} 
      void setgdInd(int gdInd_) {gdInd = gdInd_;} 
      void setlaInd(int laInd_) {laInd = laInd_;Ila=Index(laInd,laInd+laSize-1); } 
      void setrFactorInd(int rFactorInd_) {rFactorInd = rFactorInd_; } 

      virtual void solveImpactsFixpointSingle() { cout << "\nsolveImpactsFixpointSingle not implemented." << endl; throw 50; }
      virtual void solveConstraintsFixpointSingle() { cout << "\nsolveConstraintsFixpointSingle not implemented." << endl; throw 50; }
      virtual void solveImpactsGaussSeidel() { cout << "\nsolveImpactsGaussSeidel not implemented." << endl; throw 50; }
      virtual void solveConstraintsGaussSeidel() { cout << "\nsolveConstraintsGaussSeidel not implemented." << endl; throw 50; }
      virtual void solveConstraintsRootFinding() { cout << "\nsolveConstraintsRootFinding not implemented." << endl; throw 50; }
      virtual void solveImpactsRootFinding() { cout << "\nsolveImpactsRootFinding not implemented." << endl; throw 50; }
      virtual void jacobianConstraints() { cout << "\njacobianConstraints not implemented." << endl; throw 50; }
      virtual void jacobianImpacts() { cout << "\njacobianConstraints not implemented." << endl; throw 50; }
      virtual void updaterFactors() { cout << "\nupdaterFactors not implemented." << endl; throw 50; }

      virtual std::string getTerminationInfo(double dt) {return ("No Convergence within " + getName());}

      void decreaserFactors();

      virtual void checkConstraintsForTermination() { cout << "\ncheckConstraintsForTermination not implemented." << endl; throw 50; }
      virtual void checkImpactsForTermination() { cout << "\ncheckImpactForTermination not implemented." << endl; throw 50; }

      /*! Defines the maximum error radius lambdas have to match. */  
      virtual void setlaTol(double tol) {laTol = tol;}
      virtual void setLaTol(double tol) {LaTol = tol;}
      virtual void setgdTol(double tol) {gdTol = tol;}
      virtual void setgddTol(double tol) {gddTol = tol;}
      virtual void setScaleTolQ(double scaleTolQ_) {scaleTolQ = scaleTolQ_;}
      virtual void setScaleTolp(double scaleTolp_) {scaleTolp = scaleTolp_;}

      /*! Defines the maximal r-factor. */  
      virtual void setrMax(double rMax_) {rMax = rMax_;}

      virtual void connect(CoordinateSystem *port1, int id);

      /*! Adds contours of other bodies, as constraints for ports connected to a LinkContour. */
      virtual void connect(Contour *port1, int id);

      string getType() const {return "Link";}

      void load(const string& path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);

      virtual void checkActiveg() {}
      virtual void checkActivegd() {}
      virtual void checkActivegdn() {}
      virtual void checkActivegdd() {}
      virtual void checkAllgd() {}
      virtual void updateCondition() {}

      void updateJacobians(double t) {}

      virtual void resizeJacobians(int j) {}

      /*! \brief Set AMVisbody Arrow do display the link load (fore or Moment)
       * @param scale scalefactor (default=1) scale=1 means 1KN or 1KNM is equivalent to arrowlength one
       * @param ID ID of load and corresponding CoordinateSystem/Contour (ID=0 or 1)
       * @param funcColor Userfunction to manipulate Color of Arrow at each TimeStep
       * default: Red arrow for Forces and green one for Moments
       * */
#ifdef HAVE_AMVIS
      virtual void addAMVisForceArrow(AMVis::Arrow *arrow,double scale=1, int ID=0, UserFunction *funcColor=0);
      virtual void addAMVisMomentArrow(AMVis::Arrow *arrow,double scale=1, int ID=0, UserFunction *funcColor=0);
#endif
  };
}

#endif
