/* Copyright (C) 2004-2009 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _LINK_H_
#define _LINK_H_

#include "mbsim/element.h"
#include "mbsim/interfaces.h"
#include "mbsim/mbsim_event.h"
#include "hdf5serie/vectorserie.h"
#include <vector>

namespace H5 {
  class Group;
}

#ifdef HAVE_AMVIS
namespace AMVis {class Arrow;}
#endif

namespace MBSim {
  class Frame;
  class Contour;
  class HitSphereLink;
  class UserFunction;

  /** 
   * \brief general link to one or more objects
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   */
  class Link : public Element, public LinkInterface {
    public:
      /**
       * \brief constructor
       * \param name of link
       */
      Link(const string &name);

      /**
       * \brief destructor
       */
      virtual ~Link();

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updater(double t);
      virtual void updatewb(double t) {};
      virtual void updateW(double t) {};
      virtual void updateV(double t) {};
      virtual void updateh(double t) {};
      virtual void updatedx(double t, double dt) {}
      virtual void updatexd(double t) {}
      virtual void updateStopVector(double t) {}
      virtual void updateJacobians(double t) {}
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      void load(const string& path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);
      string getType() const { return "Link"; }
      virtual void plot(double t, double dt = 1);
      virtual void closePlot();
      /***************************************************/

      /* INTERFACE TO BE DEFINED IN DERIVED CLASS */
      /**
       * \brief references to contact force direction matrix of subsystem parent
       */
      virtual void updateWRef(const Mat& ref, int i=0);

      /**
       * \brief references to condensed contact force direction matrix of subsystem parent
       */
      virtual void updateVRef(const Mat& ref, int i=0);

      /**
       * \brief references to smooth force vector of subsystem parent
       */
      virtual void updatehRef(const Vec &ref, int i=0);

      /**
       * \brief references to nonsmooth force vector of subsystem parent
       */
      virtual void updaterRef(const Vec &ref);

      /**
       * \brief references to TODO of subsystem parent
       */
      virtual void updatewbRef(const Vec &ref);

      /**
       * \brief references to TODO of subsystem parent
       */
      virtual void updatefRef(const Vec &ref) {};

      /**
       * \brief references to order one parameter of subsystem parent
       */
      virtual void updatexRef(const Vec& ref);

      /**
       * \brief references to order one parameter derivatives of subsystem parent
       */
      virtual void updatexdRef(const Vec& ref);

      /**
       * \brief references to contact force parameter of subsystem parent
       */
      virtual void updatelaRef(const Vec& ref);

      /**
       * \brief references to contact relative distances of subsystem parent
       */
      virtual void updategRef(const Vec& ref);

      /**
       * \brief references to contact relative velocities of subsystem parent
       */
      virtual void updategdRef(const Vec& ref);

      /**
       * \brief references to residuum of nonlinear contact equations of subsystem parent
       */
      virtual void updateresRef(const Vec& ref);

      /**
       * \brief references to rfactors of subsystem parent
       */
      virtual void updaterFactorRef(const Vec& ref);

      /**
       * \brief references to stopvector of subsystem parent (root function for event driven integration)
       */
      virtual void updatesvRef(const Vec &sv);

      /**
       * \brief references to stopvector evaluation of subsystem parent (root detection with corresponding bool array by event driven integrator)
       */
      virtual void updatejsvRef(const Vector<int> &jsvParent);

      /**
       * \brief calculates size of order one parameters
       */
      virtual void calcxSize() { xSize = 0; }

      /**
       * \brief calculates size of contact force parameters
       */
      virtual void calclaSize() { laSize = 0; }

      /**
       * \brief calculates size of active contact force parameters
       */
      virtual void calclaSizeForActiveg() { laSize = 0; }

      /**
       * \brief calculates size of relative distances
       */
      virtual void calcgSize() { gSize = 0; }

      /**
       * \brief calculates size of active relative distances
       */
      virtual void calcgSizeActive() { gSize = 0; }

      /**
       * \brief calculates size of relative velocities
       */
      virtual void calcgdSize() { gdSize = 0; }

      /**
       * \brief calculates size of active relative velocities
       */
      virtual void calcgdSizeActive() { gdSize = 0; }

      /**
       * \brief calculates size of rfactors
       */
      virtual void calcrFactorSize() { rFactorSize = 0; }

      /**
       * \brief calculates size of stopvector (root function for event driven integration)
       */
      virtual void calcsvSize() { svSize = 0; }

      /**
       * \brief initialise link
       */
      virtual void init();

      /**
       * \brief do tasks before initialisation 
       */
      virtual void preinit() {}

      /**
       * \brief initialise order one parameters TODO necessary?
       */
      virtual void initz();

      /**
       * \brief plots time series header
       */
      virtual void initPlot();

      /**
       * \return set valued force laws used?
       */
      virtual bool isSetValued() const { return false; }

      /**
       * \return are link equations active?
       */
      virtual bool isActive() const = 0;

      /**
       * \return has the relative distance vector changed?
       */
      virtual bool gActiveChanged() = 0;

      /**
       * solve contact equations of motion with single step fixed point scheme on velocity level
       */
      virtual void solveImpactsFixpointSingle() { throw MBSimError("ERROR (Link::solveImpactsFixpointSingle): Not implemented."); }

      /**
       * solve contact equations of motion with single step fixed point scheme on acceleration level
       */
      virtual void solveConstraintsFixpointSingle() { throw MBSimError("ERROR (Link::solveConstraintsFixpointSingle): Not implemented."); }

      /**
       * solve contact equations of motion with Gauss-Seidel scheme on velocity level
       */
      virtual void solveImpactsGaussSeidel() { throw MBSimError("ERROR (Link::solveImpactsGaussSeidel): Not implemented."); }

      /**
       * solve contact equations of motion with Gauss-Seidel scheme on acceleration level
       */
      virtual void solveConstraintsGaussSeidel() { throw MBSimError("ERROR (Link::solveConstraintsGaussSeidel): Not implemented."); }

      /**
       * solve contact equations of motion with Newton scheme on velocity level
       */
      virtual void solveImpactsRootFinding() { throw MBSimError("ERROR (Link::solveImpactsRootFinding): Not implemented."); }

      /**
       * solve contact equations of motion with Newton scheme on acceleration level
       */
      virtual void solveConstraintsRootFinding() { throw MBSimError("ERROR (Link::solveConstraintsRootFinding): Not implemented."); }

      /**
       * \brief computes JACOBIAN and mass action matrix of nonlinear contact equations on acceleration level
       */
      virtual void jacobianConstraints() { throw MBSimError("ERROR (Link::jacobianConstraints): Not implemented."); }

      /**
       * \brief computes JACOBIAN and mass action matrix of nonlinear contact equations on velocity level
       */
      virtual void jacobianImpacts() { throw MBSimError("ERROR (Link::jacobianImpacts): Not implemented."); }

      /**
       * \brief updates rfactor relaxation for contact equations
       */
      virtual void updaterFactors() { throw MBSimError("ERROR (Link::updaterFactors): Not implemented."); }

      /**
       * \brief verify underlying force laws on velocity level concerning given tolerances
       */
      virtual void checkImpactsForTermination() { throw MBSimError("ERROR (Link::checkImpactsForTermination): Not implemented."); }
      
      /**
       * \brief verify underlying force laws on acceleration level concerning given tolerances
       */
      virtual void checkConstraintsForTermination() { throw MBSimError("ERROR (Link::checkConstraintsForTermination): Not implemented."); }

      /**
       * \param frame to add to link frame vector
       */
      virtual void connect(Frame *frame_);
      
      /**
       * \param contour to add to link contour vector
       */
      virtual void connect(Contour *contour_);

      /**
       * \brief set possible attribute for active relative distance in derived classes 
       */
      virtual void checkActiveg() {}

      /**
       * \brief set possible attribute for active relative velocity in derived classes for initialising event driven simulation 
       */
      virtual void checkActivegd() {}

      /**
       * \brief set possible attribute for active relative velocity in derived classes for updating event driven simulation after an impact
       */
      virtual void checkActivegdn() {}

      /**
       * \brief set possible attribute for active relative acceleration in derived classes for updating event driven simulation after an impact
       */
      virtual void checkActivegdd() {}

      /**
       * \brief set possible attribute for active relative velocity in derived classes for updating event driven and time-stepping simulation before an impact
       */
      virtual void checkAllgd() {}

      /**
       * \brief set possible attributes for active relative kinematics in derived classes for updating event driven simulation before case study
       */
      virtual void updateCondition() {}

      /**
       * \brief compute potential energy
       */
      virtual double computePotentialEnergy() { return 0; }

      /**
       * \brief TODO
       */
      virtual void resizeJacobians(int j) {}

      /* GETTER / SETTER */
      virtual void setlaTol(double tol) { laTol = tol; }
      virtual void setLaTol(double tol) { LaTol = tol; }
      virtual void setgdTol(double tol) { gdTol = tol; }
      virtual void setgddTol(double tol) { gddTol = tol; }
      virtual void setrMax(double rMax_) { rMax = rMax_; }
      virtual void setScaleTolQ(double scaleTolQ_) { scaleTolQ = scaleTolQ_; }
      virtual void setScaleTolp(double scaleTolp_) { scaleTolp = scaleTolp_; }

      /**
       * \param arrow do display the link load (force or moment)
       * \param scale scalefactor (default=1): scale=1 means 1KN or 1KNM is equivalent to arrowlength one
       * \param ID of load and corresponding frame/contour (ID=0 or 1)
       * \param userfunction to manipulate color of arrow at each timestep (default: red arrow for forces and green one for moments)
       */
#ifdef HAVE_AMVIS
      virtual void addAMVisForceArrow(AMVis::Arrow *arrow,double scale=1, int ID=0, UserFunction *funcColor=0);
      virtual void addAMVisMomentArrow(AMVis::Arrow *arrow,double scale=1, int ID=0, UserFunction *funcColor=0);
#endif
      /***************************************************/

      /* GETTER / SETTER */
      Subsystem* getParent() { return parent; }
      void setParent(Subsystem* sys) { parent = sys; }

      const vector<Mat>& getW() const { return W; }
      const vector<Mat>& getV() const { return V; }
      const vector<Vec>& geth() const { return h; }

      void setx(const Vec &x_) { x = x_; }
      const Vec& getx() const { return x; }
      const Vec& getxd() const { return xd; }
      void setxInd(int xInd_) { xInd = xInd_; };
      int getxSize() const { return xSize; }
      
      void setsvInd(int svInd_) { svInd = svInd_; };
      int getsvSize() const { return svSize; }

      const Vec& getla() const { return la; }
      Vec& getla() { return la; }
      void setlaInd(int laInd_) { laInd = laInd_;Ila=Index(laInd,laInd+laSize-1); } 
      int getlaInd() const { return laInd; } 
      int getlaSize() const { return laSize; } 
      const Index& getlaIndex() const { return Ila; }
      int getlaIndMBS() const { return laIndMBS; }
      void setlaIndMBS(int laIndParent) { laIndMBS = laInd + laIndParent; }

      const Vec& getg() const { return g; }
      Vec& getg() { return g; }
      void setgInd(int gInd_) { gInd = gInd_; Ig=Index(gInd,gInd+gSize-1); } 
      void setgdInd(int gdInd_) { gdInd = gdInd_; } 
      int getgdInd() const { return gdInd; } 
      int getgSize() const { return gSize; } 
      int getgdSize() const { return gdSize; } 
      const Index& getgIndex() const { return Ig; }
      
      void setrFactorInd(int rFactorInd_) { rFactorInd = rFactorInd_; } 
      int getrFactorSize() const { return rFactorSize; } 
      
      const Vector<int>& getrFactorUnsure() const { return rFactorUnsure; }

      /**
       * \brief saves contact force parameters for use as starting value in next time step
       */
      void savela();

      /**
       * \brief load contact force parameters for use as starting value
       */
      void initla();

      /**
       * \brief decrease rfactor if mass action matrix is not diagonal dominant (cf. Foerg: Dissertation, page 80 et seq.) 
       */
      void decreaserFactors();

    protected:
      /**
       * \brief parent of link 
       */
      Subsystem* parent;

      /** 
       * \brief order one parameters
       */
      Vec x;

      /** 
       * \brief differentiated order one parameters 
       */
      Vec xd;

      /**
       * \brief order one initial value
       */
      Vec x0;

      /**
       * \brief size  and local index of order one parameters
       */
      int xSize, xInd;

      /**
       * \brief stop vector for event driven integration (root function)
       */
      Vec sv;

      /**
       * \brief evaluation of roots of stop vector with a boolean vector
       */
      Vector<int> jsv;

      /**
       * \brief size and local index of stop vector
       */
      int svSize, svInd;

      /**
       * \brief relative distance, relative velocity, contact force parameters
       */
      Vec g, gd, la;
      
      /**
       * \brief size and local index of relative distances
       */
      int gSize, gInd;

      /**
       * \brief size and local index of relative velocities
       */
      int gdSize, gdInd;

      /**
       * \brief size and local index of contact force parameters
       */
      int laSize, laInd;
      
      /**
       * \brief local index of relative distances and contact force parameters
       */
      Index Ig, Ila;
      
      /**
       * \brief tolerance for relative velocity, relative acceleration, force and impact  
       */
      double gdTol, gddTol, laTol, LaTol;
      
      /**
       * \brief attribute to save contact force parameter of previous time step
       */
      Vec la0;

      /**
       * \brief vector of rfactors for relaxation of contact equations
       */
      Vec rFactor;

      /**
       * \brief boolean vector defining if rfactor belongs to not diagonal dominant mass action matrix (cf. Foerg Dissertation, page 80 et seq.)
       */
      Vector<int> rFactorUnsure;

      /**
       * \brief size and local index of rfactors
       */
      int rFactorSize, rFactorInd;
      
      /**
       * \brief maximum r-factor
       */
      double rMax;

      /**
       * \brief TODO
       */
      int laIndMBS;
      
      /**
       * residuum of nonlinear contact equations
       */
      Vec res;


      /** 
       * \brief force direction matrix for nonsmooth right hand side
       */
      vector<Mat> W;

      /**
       * \brief reduced force direction matrix for nonsmooth right hand side
       */
      vector<Mat> V;
      
      /**
       * \brief smooth right hand side
       */
      vector<Vec> h;
      
      /**
       * \brief nonsmooth right hand side
       */
      vector<Vec> r;
      
      /**
       * \brief TODO
       */
      Vec wb;

      /** 
       * \brief force and moment direction for smooth right hand side
       */
      vector<Vec> WF, WM;
      
      /**
       * \brief force and moment direction matrix for nonsmooth right hand side
       */
      vector<Mat> fF, fM;

      /**
       * \brief scale factor for flow and pressure quantity tolerances tolQ/tolp=tol*scaleTolQ/scaleTolp TODO necessary?
       */
      double scaleTolQ, scaleTolp;

      /**
       * \brief array in which all frames are listed, connecting bodies via a link
       */
      vector<Frame*> port;

      /** 
       * \brief array in which all contours are listed, connecting bodies via link
       */
      vector<Contour*> contour;

#ifdef HAVE_AMVIS
      vector<AMVis::Arrow*> arrowAMVis;
      vector<double> arrowAMVisScale;
      vector<int> arrowAMVisID;
      vector<bool> arrowAMVisMoment;
      vector<UserFunction*> arrowAMVisUserFunctionColor;
#endif
  };
}

#endif /* _LINK_H_ */

