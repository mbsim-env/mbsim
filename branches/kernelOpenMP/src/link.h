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

#include<vector>
#include "element.h"
#include "contour_pdata.h"

#ifdef HAVE_AMVIS
namespace AMVis {class Arrow;}
#endif

namespace MBSim {
  class Port;
  class Contour;
  class MultiBodySystem;
  class HitSphereLink;
  class UserFunction;
  struct ContourPointData;

  /*! \brief A general link to one or more objects */
  class Link : public Element {
    protected:
      /** Internal integrable state variables */
      Vec x;
      /** internal integrable state variable velocities */
      Vec xd;
      /** dimension of internal Link state */
      int xSize;
      /** start index of internal Link state */
      int xInd;

      int svSize;
      Vec sv;
      int svInd;
      Vector<int> jsv;
      /** set valued or functional link? */
      bool setValued;

      /** dimension and start index of gap parameters */
      int gSize, gInd;
      /** dimension and start index of force parameters */
      int laSize, laInd;
      /** load vector */
      vector<Vec> load;
      /** load directions */
      vector<Mat> loadDir;
      /** vector of time explicit kinematical excitations */
      vector<Vec> w;
      /** number and start index of rFactors */
      int rFactorSize, rFactorInd;
      /** gap, gap velocity, gap normal velocity, constraint forces, velocities for constraint iteration, Newton residuum */
      Vec g, gd, gdn, la, s, res;
      /** rFactors */
      Vec rFactor;
      Vector<int> rFactorUnsure;

      /** is a link active such that is has to be included in the kinetical balance */
      bool active;
      /** Index of gaps and loads */
      Index Ig, Ila;
      /** saved force parameters */
      Vec la0;

      /** scale factor for flow and pressure quantity tolerances */
      double scaleTolQ,scaleTolp;
      /** tolerances and maximum rFactor */
      double gdTol, laTol, rMax;
      /** HitSphere for checking approximately intersection of partners */
      HitSphereLink* HSLink;
      /** activate hit sphere check */
      bool checkHSLink;

      /** AMVis visualisation of loads */
#ifdef HAVE_AMVIS
      vector<AMVis::Arrow*> arrowAMVis;
      vector<double> arrowAMVisScale;
      vector<int> arrowAMVisID;
      vector<bool> arrowAMVisMoment;
      vector<UserFunction*> arrowAMVisUserFunctionColor;
      vector<Vec> arrowAMVisToPoint;
      vector<int> AMVisInstance;
#endif

    public:
      /*! Constructor */
      Link(const string &name, bool setValued);
      /*! Destructor */
      virtual ~Link();
      /*! Update internal Link state */
      virtual void updatexRef();
      /*! Update internal Link state velocities */
      virtual void updatexdRef();
      virtual void updatesvRef();
      virtual void updatejsvRef();
      /*! Set start index of internal Link state */
      void setxInd(int xInd_) {xInd = xInd_;};
      void setsvInd(int svInd_) {svInd = svInd_;};
      /*! Get dimension of internal Link state */
      int getxSize() const {return xSize;}
      int getsvSize() const {return svSize;}
      /*! Activate HitSphereLink-check for the Link before updateStage1(), only usefull for Contacts */
      void setHitSphereCheck(bool checkHSLink_) {checkHSLink=checkHSLink_;}
      /*! Get HSLink for MultiBodySystem */
      bool getHitSphereCheck() const {return checkHSLink;}
      /*! Calculates dimension of data administration */
      virtual void calcSize() {}
      /*! Initialises Link representation */
      virtual void init();
      /*! Updates necessary data for further calculations in a first step */
      virtual void updateStage1(double t) = 0;
      /*! Updates necessary data for further calculations in a second step */
      virtual void updateStage2(double t) {}
      /*! Update explicit time kinematical excitation */
      virtual void updatew(double t) {}
      /*! Compute potential energy, holding every potential */
      virtual double computePotentialEnergy() {return 0;}
      /*! Supplies time variation of x to a fixed step solver */
      virtual void updatedx(double t, double dt) {};
      /*! Supplies the time derivative of x to a variable step solver */
      virtual void updatexd(double t) {};

      virtual void updateStopVector(double t) {}
      /*! Get internal state variables of a Link */
      const Vec& getx() const {return x;}
      /*! Get internal state velocities of a Link */
      const Vec& getxd() const {return xd;}
      /*! Sets the internal states of a Link */
      void setx(const Vec &x_) {x = x_;}
      /*! Plots interesting data */
      void plot(double t, double dt=1);
      /*! Initialises plot files */
      void initPlotFiles();
      /*! Returns FLAG for setvalued or functional relationship */
      bool isSetValued() const; 
      /*! Returns the actual load supplied by the Link to the Port connected by it */
      const Vec& getLoad(int id) const { return load[id];}
      /*! Returns the load directions */
      const Mat& getLoadDirections(int id) const {return loadDir[id];}
      /*! Get explicit time kinematical excitation */
      const Vec& getw(int id) const {return w[id];}
      /*! Get loads */
      const Vec& getla() const {return la;}
      /*! Get loads */
      Vec& getla() {return la;}
      /*! Get gaps */
      const Vec& getg() const {return g;}
      /*! Get gaps */
      Vec& getg() {return g;}
      /*! Get number of gaps */
      int getgSize() const {return gSize;}
      /*! Get number of loads */
      int getlaSize() const {return laSize;}
      /*! Get start index of loads */
      int getlaInd() const {return laInd;}
      /*! Get number of rFactors */
      int getrFactorSize() const {return rFactorSize;}
      /*! Get index of gaps */ 
      const Index& getgIndex() const {return Ig;}
      /*! Get index of loads */
      const Index& getlaIndex() const {return Ila;}
      /*! Get FLAG if Link is active */
      bool isActive() const {return active;}
      /*! Saves force parameter */
      void savela();
      /*! Initialises force parameter to saved value */
      void initla();

      const Vector<int>& getrFactorUnsure() const {return rFactorUnsure;}
      /*! Get loads from MBS */
      virtual void updatelaRef();
      /*! Get gaps from MBS */
      virtual void updategRef();
      /*! Get gap velocities from MBS */
      virtual void updategdRef();
      /*! Get relative velocities for constraint iteration from MBS */
      virtual void updatesRef();
      /*! Get Newton residuum from MBS */
      virtual void updateresRef();
      /*! Get rFactors from MBS */
      virtual void updaterFactorRef();
      /*! Get everything necessary from MBS */
      virtual void updateRef();
      /*! Set index of gaps */
      void setgInd(int gInd_) {gInd = gInd_;Ig=Index(gInd,gInd+gSize-1);}
      /*! Set index of loads */
      void setlaInd(int laInd_) {laInd = laInd_;Ila=Index(laInd,laInd+laSize-1); }
      /*! Set start index of rFactors */
      void setrFactorInd(int rFactorInd_) {rFactorInd = rFactorInd_; } 
      /*! Abstract Jacobian solver */
      virtual void projectJ(double dt) { cout << "\nprojectJ not implemented." << endl; throw 50; }
      /*! Abstract Gauss-Seidel solver for 3D*/
      virtual void projectGS(double dt) { cout << "\nprojectGS not implemented." << endl; throw 50; }
      /*! Abstract Gauss-Seidel solver for 2D */
      virtual void solveGS(double dt) { cout << "\nsolveGS not implemented." << endl; throw 50; }
      /*! Abstract rootFinding with numerical Jacobian */
      virtual void residualProj(double dt) { cout << "\nresidualProj not implemented." << endl; throw 50; }
      /*! Test if there is convergence in constraint solver */
      virtual void checkForTermination(double dt) { cout << "\ncheckForTermination not implemented." << endl; throw 50; }
      /*! Information about constraint solver convergence */
      virtual std::string getTerminationInfo(double dt) {return ("No Convergence within " + getFullName());}
      /*! Abstract rootFinding with analytical Jacobian */
      virtual void residualProjJac(double dt) { cout << "\nresidualProjJac not implemented." << endl; throw 50; }
      /*! Abstract update of rFactors */
      virtual void updaterFactors() { cout << "\nupdaterFactors not implemented." << endl; throw 50; }
      /*! Automatically decreases r-factors for Link */
      void decreaserFactors();

      /*! Defines the maximum error radius lambdas have to match */  
      virtual void setlaTol(double tol) {laTol = tol;}
      /*! Defines the maximum error radius gap velocities have to match */  
      virtual void setgdTol(double tol) {gdTol = tol;}
      /*! Set scale factor for flow quantity tolerances */
      virtual void setScaleTolQ(double scaleTolQ_) {scaleTolQ = scaleTolQ_;}
      /*! set scale factor for pressure quantity tolerances */
      virtual void setScaleTolp(double scaleTolp_) {scaleTolp = scaleTolp_;}

      /*! Defines the maximal r-factor. */  
      virtual void setrMax(double rMax_) {rMax = rMax_;}

      /*! \brief Set AMVisbody Arrow to display the link load (force or torque)
       * @param scale scalefactor (default=1) scale=1 means 1KN or 1KNM is equivalent to arrowlength one (arrowlength = scale*force/1000N)
       * @param ID ID of load and corresponding Port/Contour-partner (ID=0 or 1)
       * @param funcColor Userfunction to manipulate color of Arrow at each time step
       * default: Red arrow for forces and green one for moments
       */
#ifdef HAVE_AMVIS
      /*! AMVis visualisation of force */
      virtual void addAMVisForceArrow(AMVis::Arrow *arrow,double scale=1, int ID=0, UserFunction *funcColor=0, int instance=0);
      /*! AMVis visualisation of torque */
      virtual void addAMVisMomentArrow(AMVis::Arrow *arrow,double scale=1, int ID=0, UserFunction *funcColor=0, int instance=0);
#endif
  };

  /*! \brief A general link via Port to one or more objects */
  class LinkPort : public Link {
    protected:
      /*! Array in which all ports are listed, connecting bodies via a Link. */
      vector<Port*> port;

    public:
      /*! Constructor */
      LinkPort(const string &name, bool setValued);
      /*! Destructor */
      virtual ~LinkPort() {}
      /*! Adds ports of other bodies connected to a LinkPort */
      virtual void connect(Port *port1, int id);
      /*! Writing to plot-file */
      void plot(double t, double dt=1);
      /*! Returns the complete port vector */
      vector<Port*> getPorts() const { return port; }
  };

  /*! \brief A general Link via Contour to one or more objects */
  class LinkContour : public Link {
    protected:
      /** Array in which all contours linked by LinkContour are managed */
      vector<Contour*> contour;
      /** Array of ContourPoint Datas to manage the location of the link on a partner */
      vector<ContourPointData> cpData;

    public:
      /*! Constructor */
      LinkContour(const string &name, bool setValued);
      /*! Destructor */
      virtual ~LinkContour() {}
      /*! Writing to plot-file */
      void plot(double t, double dt=1);
      /*! Calls the init-method of Link */
      void init();
      /*! Adds contours of other bodies connected to a LinkContour */
      virtual void connect(Contour *port1, int id);
      /*! Returns a contour with identifiaction number */
      Contour* getContour(int id) const { return contour[id]; }
      /*! Returns the complete contour vector */
      vector<Contour*> getContours() const { return contour; }
      /*! Returns the actual vector from world inertia to contourpoint */
      const Vec& getWrOC(int id) const { return cpData[id].WrOC; }
      /*! Returns the actual involved contour parameter */
      const Vec& getalpha(int id) const { return cpData[id].alpha; }
      /*! Returns the actual data to Contourpoint */
      const ContourPointData& getContourPointData(int id) const { return cpData[id]; }
  };

}

#endif
