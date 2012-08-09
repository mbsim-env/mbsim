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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _LINK_H_
#define _LINK_H_

#include "mbsim/element.h"
//#include "mbsim/link_interface.h"
//#include "mbsim/extradynamic_interface.h"
#include "mbsim/mbsim_event.h"

namespace H5 {
  class Group;
}

namespace MBSim {

  class Frame;
  class Object;
  class ExtraDynamic;
  class Contour;
  class DynamicSystem;

  /** 
   * \brief general link to one or more objects
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   * \date 2009-04-06 ExtraDynamicInterface included / LinkMechanics added (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-27 enhanced structure for implicit integration (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   * \date 2009-12-14 revised inverse kinetics (Martin Foerg)
   * \date 2010-07-06 added LinkStatus and LinearImpactEstimation for timestepper ssc (Robert Huber)
   * \date 2012-05-08 added LinkStatusReg for AutoTimeSteppingSSCIntegrator (Jan Clauberg)
   * 
   */
  //class Link : public Element, public LinkInterface, public ExtraDynamicInterface {
  class Link : public Element {
    public:
      /**
       * \brief constructor
       * \param name of link
       */
      Link(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Link() {}

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updateg(double t) = 0;
      virtual void updategd(double t) = 0;
      virtual void updatewb(double t, int i=0) {};
      virtual void updateW(double t, int i=0) {};
      virtual void updateV(double t, int i=0) {};
      virtual void updateh(double t, int i=0) {};
      virtual void updateStopVector(double t) {}
      virtual void updateLinkStatus(double t) {}
      virtual void updateLinkStatusReg(double t) {}
      virtual void updateJacobians(double t, int j=0) {}
      virtual void updateb(double t) {};
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void updatedx(double t, double dt) {}
      virtual void updatexd(double t) {}
      virtual void calcxSize() { xSize = 0; }
      virtual const fmatvec::Vec& getx() const { return x; }
      virtual fmatvec::Vec& getx() { return x; }
      virtual void setxInd(int xInd_) { xInd = xInd_; };
      virtual int getxSize() const { return xSize; }
      virtual void updatexRef(const fmatvec::Vec& ref);
      virtual void updatexdRef(const fmatvec::Vec& ref);
      virtual void updatebRef(const fmatvec::Mat &hRef);
      virtual void init(InitStage stage);
      virtual void initz();
      /***************************************************/

      virtual void setbInd(int bInd_) { bInd = bInd_; };

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Link"; }
      virtual void plot(double t, double dt = 1);
      virtual void closePlot();
      /***************************************************/

      /* INTERFACE TO BE DEFINED IN DERIVED CLASS */
      /**
       * \brief references to contact force direction matrix of dynamic system parent
       */
      virtual void updateWRef(const fmatvec::Mat& ref, int i=0) = 0;

      /**
       * \brief references to condensed contact force direction matrix of dynamic system parent
       */
      virtual void updateVRef(const fmatvec::Mat& ref, int i=0) = 0;

      /**
       * \brief references to complete and link smooth force vector of dynamic system parent
       */
      virtual void updatehRef(const fmatvec::Vec &hRef, int i=0) = 0;

      /**
       * \brief references to nonsmooth force vector of dynamic system parent
       */
      virtual void updaterRef(const fmatvec::Vec &ref, int i=0) = 0;

      /**
       * \brief references to TODO of dynamic system parent
       */
      virtual void updatewbRef(const fmatvec::Vec &ref);

      /**
       * \brief references to TODO of dynamic system parent
       */
      virtual void updatefRef(const fmatvec::Vec &ref) {};

      /**
       * \brief references to contact force parameter of dynamic system parent
       */
      virtual void updatelaRef(const fmatvec::Vec& ref);

      /**
       * \brief delete reference to contact force parameter of dynamic system parent
       */
      virtual void deletelaRef();

      /**
       * \brief references to contact relative distances of dynamic system parent
       */
      virtual void updategRef(const fmatvec::Vec& ref);

      /**
       * \brief references to contact relative velocities of dynamic system parent
       */
      virtual void updategdRef(const fmatvec::Vec& ref);

      /**
       * \brief references to residuum of nonlinear contact equations of dynamic system parent
       */
      virtual void updateresRef(const fmatvec::Vec& ref);

      /**
       * \brief references to rfactors of dynamic system parent
       */
      virtual void updaterFactorRef(const fmatvec::Vec& ref);

      /**
       * \brief references to stopvector of dynamic system parent (root function for event driven integration)
       */
      virtual void updatesvRef(const fmatvec::Vec &sv);

      /**
       * \brief references to stopvector evaluation of dynamic system parent (root detection with corresponding bool array by event driven integrator)
       */
      virtual void updatejsvRef(const fmatvec::Vector<int> &jsvParent);

      /**
       * \brief reference to vector of link status (for set valued links with piecewise link equations)
       */
       virtual void updateLinkStatusRef(const fmatvec::Vector<int> &LinkStatusParent);

      /**
       * \brief reference to vector of link status (for single-valued links)
       */
       virtual void updateLinkStatusRegRef(const fmatvec::Vector<int> &LinkStatusRegParent);

      /**
       * \brief calculates size of contact force parameters
       */
      virtual void calclaSize(int j) { laSize = 0; }

      /**
       * \brief calculates size of relative distances
       */
      virtual void calcgSize(int j) { gSize = 0; }

      /**
       * \brief calculates size of relative velocities
       */
      virtual void calcgdSize(int j) { gdSize = 0; }

      /**
       * \brief calculates size of rfactors
       */
      virtual void calcrFactorSize(int j) { rFactorSize = 0; }

      /**
       * \brief calculates size of rfactors
       */
      virtual void calcbSize() { bSize = 0; }

      /**
       * \brief calculates size of stopvector (root function for event driven integration)
       */
      virtual void calcsvSize() { svSize = 0; }
      
      /**
       * \brief calculates size of vector LinkStatus
       */
      virtual void calcLinkStatusSize() { LinkStatusSize =0;}

      /**
       * \brief calculates size of vector LinkStatusReg
       */
      virtual void calcLinkStatusRegSize() { LinkStatusRegSize =0;}

      /**
       * \return set valued force laws used?
       */
      virtual bool isSetValued() const { return false; }

      /**
       * \return updateh is needed if set valued force law is not active?
       */
      virtual bool hasSmoothPart() const { return false; }

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
      virtual void solveImpactsFixpointSingle(double dt) { throw MBSimError("ERROR in "+getName()+" (Link::solveImpactsFixpointSingle): Not implemented."); }

      /**
       * solve contact equations of motion with single step fixed point scheme on acceleration level
       */
      virtual void solveConstraintsFixpointSingle() { throw MBSimError("ERROR in "+getName()+" (Link::solveConstraintsFixpointSingle): Not implemented."); }

      /**
       * solve contact equations of motion with Gauss-Seidel scheme on velocity level
       */
      virtual void solveImpactsGaussSeidel(double dt) { throw MBSimError("ERROR in "+getName()+" (Link::solveImpactsGaussSeidel): Not implemented."); }

      /**
       * solve contact equations of motion with Gauss-Seidel scheme on acceleration level
       */
      virtual void solveConstraintsGaussSeidel() { throw MBSimError("ERROR in "+getName()+" (Link::solveConstraintsGaussSeidel): Not implemented."); }

      /**
       * solve contact equations of motion with Newton scheme on velocity level
       */
      virtual void solveImpactsRootFinding(double dt) { throw MBSimError("ERROR in "+getName()+" (Link::solveImpactsRootFinding): Not implemented."); }

      /**
       * solve contact equations of motion with Newton scheme on acceleration level
       */
      virtual void solveConstraintsRootFinding() { throw MBSimError("ERROR in "+getName()+" (Link::solveConstraintsRootFinding): Not implemented."); }

      /**
       * \brief computes JACOBIAN and mass action matrix of nonlinear contact equations on acceleration level
       */
      virtual void jacobianConstraints() { throw MBSimError("ERROR in "+getName()+" (Link::jacobianConstraints): Not implemented."); }

      /**
       * \brief computes JACOBIAN and mass action matrix of nonlinear contact equations on velocity level
       */
      virtual void jacobianImpacts() { throw MBSimError("ERROR in "+getName()+" (Link::jacobianImpacts): Not implemented."); }

      /**
       * \brief updates rfactor relaxation for contact equations
       */
      virtual void updaterFactors() { throw MBSimError("ERROR in "+getName()+" (Link::updaterFactors): Not implemented."); }

      /**
       * \brief verify underlying force laws on velocity level concerning given tolerances
       */
      virtual void checkImpactsForTermination(double dt) { throw MBSimError("ERROR in "+getName()+" (Link::checkImpactsForTermination): Not implemented."); }
      
      /**
       * \brief verify underlying force laws on acceleration level concerning given tolerances
       */
      virtual void checkConstraintsForTermination() { throw MBSimError("ERROR in "+getName()+" (Link::checkConstraintsForTermination): Not implemented."); }

      /**
       * \brief sets the state of a set-valued link, e.g. contact stays closed
       * or contact will slide
       */
      virtual void checkActive(int j) {}

      /**
       * \brief compute potential energy
       */
      virtual double computePotentialEnergy() { return 0; }

      /* GETTER / SETTER */
      virtual void setlaTol(double tol) { laTol = tol; }
      virtual void setLaTol(double tol) { LaTol = tol; }
      virtual void setgTol(double tol) { gTol = tol; }
      virtual void setgdTol(double tol) { gdTol = tol; }
      virtual void setgddTol(double tol) { gddTol = tol; }
      virtual void setrMax(double rMax_) { rMax = rMax_; }
      /***************************************************/

      /* GETTER / SETTER */
      const std::vector<fmatvec::Mat>& getW(int i=0) const { return W[i]; }
      const std::vector<fmatvec::Mat>& getV(int i=0) const { return V[i]; }
      const std::vector<fmatvec::Vec>& geth(int i=0) const { return h[i]; }

      void setx(const fmatvec::Vec &x_) { x = x_; }
      const fmatvec::Vec& getxd() const { return xd; }
      
      void setsvInd(int svInd_) { svInd = svInd_; };
      int getsvSize() const { return svSize; }

      void setLinkStatusInd(int LinkStatusInd_) { LinkStatusInd = LinkStatusInd_; };
      int getLinkStatusSize() const { return LinkStatusSize; }

      void setLinkStatusRegInd(int LinkStatusRegInd_) { LinkStatusRegInd = LinkStatusRegInd_; };
      int getLinkStatusRegSize() const { return LinkStatusRegSize; }

      const fmatvec::Vec& getla() const { return la; }
      fmatvec::Vec& getla() { return la; }
      void setlaInd(int laInd_) { laInd = laInd_;Ila=fmatvec::Index(laInd,laInd+laSize-1); } 
      int getlaInd() const { return laInd; } 
      int getlaSize() const { return laSize; } 
      int getbSize() const { return bSize; }
      const fmatvec::Index& getlaIndex() const { return Ila; }

      const fmatvec::Vec& getg() const { return g; }
      fmatvec::Vec& getg() { return g; }
      void setgInd(int gInd_) { gInd = gInd_; Ig=fmatvec::Index(gInd,gInd+gSize-1); } 
      const fmatvec::Vec& getgd() const { return gd; }
      fmatvec::Vec& getgd() { return gd; }
      void setgdInd(int gdInd_) { gdInd = gdInd_; } 
      int getgdInd() const { return gdInd; } 
      int getgSize() const { return gSize; } 
      int getgdSize() const { return gdSize; } 
      const fmatvec::Index& getgIndex() const { return Ig; }
      
      void setrFactorInd(int rFactorInd_) { rFactorInd = rFactorInd_; } 
      int getrFactorSize() const { return rFactorSize; } 
      
      const fmatvec::Vector<int>& getrFactorUnsure() const { return rFactorUnsure; }

      /**
       * \brief saves contact force parameters for use as starting value in next time step
       */
      void savela(double dt=1.0);

      /**
       * \brief load contact force parameters for use as starting value
       */
      void initla(double dt=1.0);

      /**
       * \brief decrease rfactor if mass action matrix is not diagonal dominant (cf. Foerg: Dissertation, page 80 et seq.) 
       */
      void decreaserFactors();

      /**
       * \return a general element access
       */
      virtual Element* getByPathSearch(std::string path);

      /**
       * \brief get gap distance and calculate gap velocity of unilateral links to estimate impacts within the next step
       * \param gInActive gap distance of inactive links (return)
       * \param gdInActive gap velocities of inactive links (return)
       * \param IndInActive index for gInActive/gdInActive; incremented with size after storage (return and input)
       * \param gAct gap distance of active links (return)
       * \param IndActive index for gActive; incremented with size after storage (return and input)
      */
      virtual void LinearImpactEstimation(fmatvec::Vec &gInActive_,fmatvec::Vec &gdInActive_,int *IndInActive_,fmatvec::Vec &gAct_,int *IndActive_){};
      
      /**
       * \brief calculates the number of active and inactive unilateral constraints and increments sizeActive/sizeInActive
       */
      virtual void SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) {};

      virtual void updatecorr(int j) { corr.init(0); }
      virtual void updatecorrRef(const fmatvec::Vec &ref);
      virtual void calccorrSize(int j) { corrSize = 0; }
      void setcorrInd(int corrInd_) { corrInd = corrInd_; } 
      int getcorrSize() const { return corrSize; } 
      virtual void checkRoot() {};

    protected:
      /** 
       * \brief order one parameters
       */
      fmatvec::Vec x;

      /** 
       * \brief differentiated order one parameters 
       */
      fmatvec::Vec xd;

      /**
       * \brief order one initial value
       */
      fmatvec::Vec x0;

      /**
       * \brief size  and local index of order one parameters
       */
      int xSize, xInd;

      /**
       * \brief stop vector for event driven integration (root function)
       */
      fmatvec::Vec sv;

      /**
       * \brief evaluation of roots of stop vector with a boolean vector
       */
      fmatvec::Vector<int> jsv;

      /**
       * \brief size and local index of stop vector
       */
      int svSize, svInd;
             
      /**
       * for set valued links with piecewise link equation (e.g. unilateral contacts or coulomb friction)
       * \brief status of link (default 0) describing which pice of the equation is valid (e.g. stick or slip)
       */
      fmatvec::Vector<int> LinkStatus;

      /**
       * \brief size and local index of link status vector (set-valued)
       */
       int LinkStatusSize, LinkStatusInd;
    
       /**
       * for single valued links
       * \brief status of link
       */
      fmatvec::Vector<int> LinkStatusReg;

      /**
       * \brief size and local index of single-valued link status vector
       */
       int LinkStatusRegSize, LinkStatusRegInd; 

      /**
       * \brief relative distance, relative velocity, contact force parameters
       */
      fmatvec::Vec g, gd, la;
      
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
       * \brief size and local index of contact force parameters
       */
      int bSize, bInd;

      /**
       * \brief local index of relative distances and contact force parameters
       */
      fmatvec::Index Ig, Ila;
      
      /**
       * \brief tolerance for relative velocity, relative acceleration, force and impact  
       */
      double gTol, gdTol, gddTol, laTol, LaTol;
      
      /**
       * \brief attribute to save contact force parameter of previous time step
       */
      fmatvec::Vec la0;

      /**
       * \brief vector of rfactors for relaxation of contact equations
       */
      fmatvec::Vec rFactor;

      /**
       * \brief boolean vector defining if rfactor belongs to not diagonal dominant mass action matrix (cf. Foerg Dissertation, page 80 et seq.)
       */
      fmatvec::Vector<int> rFactorUnsure;

      /**
       * \brief size and local index of rfactors
       */
      int rFactorSize, rFactorInd;
      
      /**
       * \brief maximum r-factor
       */
      double rMax;

      /**
       * residuum of nonlinear contact equations
       */
      fmatvec::Vec res;

      /** 
       * \brief force direction matrix for nonsmooth right hand side
       */
      std::vector<fmatvec::Mat> W[2];

      /**
       * \brief reduced force direction matrix for nonsmooth right hand side
       */
      std::vector<fmatvec::Mat> V[2];
      
      /**
       * \brief smooth complete and link right hand side
       */
      std::vector<fmatvec::Vec> h[2];
      
      /**
       * \brief smooth Jacobians for implicit integration
       */
      std::vector<fmatvec::Mat> dhdq;
      std::vector<fmatvec::Mat> dhdu;
      std::vector<fmatvec::Vec> dhdt;
      
      /**
       * \brief nonsmooth right hand side
       */
      std::vector<fmatvec::Vec> r[2];
      
      /**
       * \brief TODO
       */
      fmatvec::Vec wb;

      /**
       * \brief TODO
       */
      fmatvec::Mat b;

      int corrSize, corrInd;
      fmatvec::Vec corr;
  };
}

#endif /* _LINK_H_ */

