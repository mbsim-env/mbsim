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
#include "contour_pdata.h"

#ifdef HAVE_AMVIS
namespace AMVis {class Arrow;}
#endif

namespace MBSim {
  //class Object;
  class Port;
  class Contour;
  class MultiBodySystem;
  class HitSphereLink;
  class UserFunction;
  struct ContourPointData;
  /*! 
   *  \brief This is a general link to one or more objects.
   * 
   * */
  class Link : public Element {

    protected:
      /*! Internal integrable State Variables x and its velocities ud \see updatedu(double t, double dt), updateud(double t) */
      Vec x, xd;

      int xSize;
      int xInd;

      int svSize;
      Vec sv;
      int svInd;
      Vector<int> jsv;

      bool setValued;

      int gSize, gInd;
      int laSize, laInd;

      vector<Vec> load;
      vector<Mat> loadDir;

      int rFactorSize, rFactorInd;
      Vec g, gd, gdn, la, s, res;

      Vec rFactor;
      Vector<int> rFactorUnsure;

      bool active;
      Index Ig, Ila;
      Vec la0;

      double scaleTolQ,scaleTolp;
      double gdTol, laTol, rMax;

      HitSphereLink* HSLink;
      bool checkHSLink;

     // vector<Object*> object;

#ifdef HAVE_AMVIS
      vector<AMVis::Arrow*> arrowAMVis;
      vector<double> arrowAMVisScale;
      vector<int> arrowAMVisID;
      vector<bool> arrowAMVisMoment;
      vector<UserFunction*> arrowAMVisUserFunctionColor;
#endif

    public:

      virtual void updatexRef();
      virtual void updatexdRef();
      virtual void updatesvRef();
      virtual void updatejsvRef();

      Link(const string &name, bool setValued);
      ~Link();

     // Object* getObject(int id) { return object[id]; }
     // void addObject(Object* obj) { object.push_back(obj); }
     // int getNumObjects() const { return object.size(); }

      void setxInd(int xInd_) {xInd = xInd_;};
      void setsvInd(int svInd_) {svInd = svInd_;};

      int getxSize() const {return xSize;}
      int getsvSize() const {return svSize;}

      /*! activate HitSphereLink-check for this Link before updateStage1(), only usefull for Contacts
       * 	\param checkHSLink_, true for checks, false for no check
       */
      void setHitSphereCheck(bool checkHSLink_) {checkHSLink=checkHSLink_;}
      /*! \return HSLink for MultiBodySystem
       *  */
      bool getHitSphereCheck() {return checkHSLink;}
      virtual void calcSize() {}
      virtual void init();

      virtual void updateStage1(double t) = 0;
      virtual void updateStage2(double t) {}

      /*! compute potential energy, holding every potential!!!
      */
      virtual double computePotentialEnergy() {return 0;}

      /*! Supplies time variation of x to a fixed step solver.*/
      virtual void updatedx(double t, double dt) {};
      /*! Supplies the time derivative of x to a variable step solver.*/
      virtual void updatexd(double t) {};

      virtual void updateStopVector(double t) {}

      const Vec& getx() const {return x;}
      const Vec& getxd() const {return xd;}
      /* Sets the internal states of a Link.*/
      void setx(const Vec &x_) {x = x_;}

      void plot(double t, double dt=1);
      void initPlotFiles();

      //bool isSetValued() const {return setValued;} 
      bool isSetValued() const; 

      /*! Returns the actual load supplied by the Link to the Port connected by it*/
      const Vec& getLoad(int id) const { return load[id];}
      const Mat& getLoadDirections(int id) const {return loadDir[id];}

      const Vec& getla() const {return la;}
      Vec& getla() {return la;}
      const Vec& getg() const {return g;}
      Vec& getg() {return g;}
      //void setg(const Vec& g_) {g = g_;}
      //void setgd(const Vec& gd_) {gd = gd_;}
      int getgSize() const {return gSize;} 
      int getlaSize() const {return laSize;} 
      int getlaInd() const {return laInd;} 
      int getrFactorSize() const {return rFactorSize;} 
      const Index& getgIndex() const {return Ig;}
      const Index& getlaIndex() const {return Ila;}
      bool isActive() const {return active;}
      void savela();
      void initla();

      const Vector<int>& getrFactorUnsure() const {return rFactorUnsure;}

      virtual void updatelaRef();
      virtual void updategRef();
      virtual void updategdRef();
      virtual void updatesRef();
      virtual void updateresRef();
      virtual void updaterFactorRef();
      virtual void updateRef();

      void setgInd(int gInd_) {gInd = gInd_;Ig=Index(gInd,gInd+gSize-1);} 
      void setlaInd(int laInd_) {laInd = laInd_;Ila=Index(laInd,laInd+laSize-1); } 
      void setrFactorInd(int rFactorInd_) {rFactorInd = rFactorInd_; } 

      virtual void projectJ(double dt) { cout << "\nprojectJ not implemented." << endl; throw 50; }
      virtual void projectGS(double dt) { cout << "\nprojectGS not implemented." << endl; throw 50; }
      virtual void solveGS(double dt) { cout << "\nsolveGS not implemented." << endl; throw 50; }

      virtual void residualProj(double dt) { cout << "\nresidualProj not implemented." << endl; throw 50; }
      virtual void checkForTermination(double dt) { cout << "\ncheckForTermination not implemented." << endl; throw 50; }
      virtual void residualProjJac(double dt) { cout << "\nresidualProjJac not implemented." << endl; throw 50; }

      virtual void updaterFactors() { cout << "\nupdaterFactors not implemented." << endl; throw 50; }
      void decreaserFactors();

      /*! Defines the maximum error radius lambdas have to match. */  
      virtual void setlaTol(double tol) {laTol = tol;}
      virtual void setgdTol(double tol) {gdTol = tol;}
      virtual void setScaleTolQ(double scaleTolQ_) {scaleTolQ = scaleTolQ_;}
      virtual void setScaleTolp(double scaleTolp_) {scaleTolp = scaleTolp_;}

      /*! Defines the maximal r-factor. */  
      virtual void setrMax(double rMax_) {rMax = rMax_;}

      /*! \brief Set AMVisbody Arrow do display the link load (fore or Moment)
       * @param scale scalefactor (default=1) scale=1 means 1KN or 1KNM is equivalent to arrowlength one
       * @param ID ID of load and corresponding Port/Contour (ID=0 or 1)
       * @param funcColor Userfunction to manipulate Color of Arrow at each TimeStep
       * default: Red arrow for Forces and green one for Moments
       * */

#ifdef HAVE_AMVIS
      virtual void addAMVisForceArrow(AMVis::Arrow *arrow,double scale=1, int ID=0, UserFunction *funcColor=0);
      virtual void addAMVisMomentArrow(AMVis::Arrow *arrow,double scale=1, int ID=0, UserFunction *funcColor=0);
#endif
  };


  /*! 
   *  \brief This is a general link via Port to one or more objects.
   * 
   * */
  class LinkPort : public Link {

    protected:
      /*! Array in which all ports are listed, connecting bodies via a Link.
      */
      vector<Port*> port;

    public:
      LinkPort(const string &name, bool setValued);

      virtual void connect(Port *port1, int id);
      void plot(double t, double dt=1);

      vector<Port*> getPorts() { return port; }
  };

  /*! 
   *  \brief This is a general Link via Contour to one or more objects.
   * 
   * */
  class LinkContour : public Link {

    protected:
      /** Array in which all contours linked by LinkContour are managed.*/
      vector<Contour*> contour;

      vector<ContourPointData> cpData;

    public:
      LinkContour(const string &name, bool setValued);
      void plot(double t, double dt=1);

      void init();

      /*! Adds contours of other bodies, as constraints for ports connected to a LinkContour. */
      virtual void connect(Contour *port1, int id);
      Contour* getContour(int id) { return contour[id]; }
      vector<Contour*> getContours() { return contour; }

      /*! Returns the actual vector from world inertia to Contourpoint. */
      const Vec& getWrOC(int id) const { return cpData[id].WrOC; }
      const Vec& getalpha(int id) const { return cpData[id].alpha; }
      /*! Returns the actual data to Contourpoint, ContourPointData holding position-vector, contour-parameter etc... */
      const ContourPointData& getContourPointData(int id) const { return cpData[id]; }
  };

}

#endif
