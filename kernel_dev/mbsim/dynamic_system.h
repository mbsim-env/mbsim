/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _DYNAMIC_SYSTEM_H_
#define _DYNAMIC_SYSTEM_H_

#include "mbsim/element.h"
#include "mbsim/interfaces.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/frame.h"
#ifdef HAVE_AMVISCPPINTERFACE
#include "amviscppinterface/group.h"
#endif

namespace H5 {
  class Group;
}

namespace MBSim {
  class Frame;
  class Contour;
  class OrderOneDynamics;
  class DataInterfaceBase;
  class Object;
  class Link;

  // TODO delete compatibility classes 
  class TreeRigid;
  class BodyRigid;

  /**
   * \brief dynamic system as topmost hierarchical level
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   * \date 2009-04-06 ExtraDynamicInterface included (Thorsten Schindler)
   */
  class DynamicSystem : public Element, public ObjectInterface, public LinkInterface, public ExtraDynamicInterface {
    public:
      /** 
       * \brief constructor
       */
      DynamicSystem(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~DynamicSystem();

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateT(double t); 
      virtual void updateh(double t); 
      virtual void updateM(double t);
      virtual void updateJacobians(double t) = 0;
      virtual void updatedq(double t, double dt); 
      virtual void updateud(double t) { throw new MBSimError("ERROR (DynamicSystem::updateud): Not implemented!"); }
      virtual void updateqd(double t) { throw new MBSimError("ERROR (DynamicSystem::updateud): Not implemented!"); }
      virtual void sethSize(int hSize_, int i=0);
      virtual int gethSize(int i=0) const { return hSize[i]; }
      virtual int getqSize() const { return qSize; }
      virtual int getuSize(int i=0) const { return uSize[i]; }
      virtual void calcqSize();
      virtual void calcuSize(int j=0);
      virtual void setqInd(int qInd_) { qInd = qInd_; }
      virtual void setuInd(int uInd_, int i=0) { uInd[i] = uInd_; }
      virtual int gethInd(DynamicSystem* sys, int i=0); 
      virtual H5::Group *getPlotGroup() { return plotGroup; }
      virtual PlotFeatureStatus getPlotFeature(PlotFeature fp) { return Element::getPlotFeature(fp); };
      virtual PlotFeatureStatus getPlotFeatureForChildren(PlotFeature fp) { return Element::getPlotFeatureForChildren(fp); };
#ifdef HAVE_AMVISCPPINTERFACE
      virtual AMVis::Group* getAMVisGrp() { return amvisGrp; }
#endif
      /** 
       * Return the full path of the object.
       * \param pathDelim The delimiter of the path
       */
      std::string getPath(char pathDelim='.') { return parent?parent->getPath()+pathDelim+name:name; }
      /*****************************************************/

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updater(double t); 
      virtual void updatewb(double t); 
      virtual void updateW(double t); 
      virtual void updateV(double t); 
      virtual void updateg(double t);
      virtual void updategd(double t);
      virtual void updateStopVector(double t); 
      /*****************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void updatedx(double t, double dt); 
      virtual void updatexd(double t);
      virtual void calcxSize();
      const fmatvec::Vec& getx() const { return x; };
      fmatvec::Vec& getx() { return x; };
      void setxInd(int xInd_) { xInd = xInd_; }
      int getxSize() const { return xSize; }
      void updatexRef(const fmatvec::Vec &ref);
      void updatexdRef(const fmatvec::Vec &ref);
      virtual void init();
      virtual void preinit();
      virtual void initz();
      /*****************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      /** DEPRECATED */
      virtual std::string getType() const { return "DynamicSystem"; }
      virtual void setDynamicSystemSolver(DynamicSystemSolver* sys);
      virtual void plot(double t, double dt);
      virtual void closePlot();
      /*****************************************************/

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \brief compute Cholesky decomposition of mass matrix TODO necessary?
       */
      virtual void facLLM() = 0;

      /**
       * \brief solve contact equations with single step fixed point scheme on acceleration level 
       */
      virtual int solveConstraintsFixpointSingle();

      /**
       * \brief solve contact equations with single step fixed point scheme on velocity level 
       */
      virtual int solveImpactsFixpointSingle();

      /**
       * \brief solve contact equations with Gauss-Seidel scheme on acceleration level 
       */
      virtual int solveConstraintsGaussSeidel();

      /**
       * \brief solve contact equations with Gauss-Seidel scheme on velocity level 
       */
      virtual int solveImpactsGaussSeidel();

      /**
       * \brief solve contact equations with Newton scheme on acceleration level 
       */
      virtual int solveConstraintsRootFinding();

      /**
       * \brief solve contact equations with Newton scheme on velocity level 
       */
      virtual int solveImpactsRootFinding();

      /**
       * \brief compute JACOBIAN of contact equations on acceleration level 
       */
      virtual int jacobianConstraints();

      /**
       * \brief compute JACOBIAN of contact equations on velocity level
       */
      virtual int jacobianImpacts();

      /**
       * \brief validate force laws concerning given tolerances on acceleration level
       */
      virtual void checkConstraintsForTermination();

      /**
       * \brief validate force laws concerning given tolerances on velocity level
       */
      virtual void checkImpactsForTermination();

      /**
       * \brief update relaxation factors for contact equations
       */
      virtual void updaterFactors();

      /**
       * \brief plots time series headers and manages HDF5 files of dynamic systems
       */
      virtual void initPlot();

      /**
       * \param name of the frame
       * \param check for existence of frame
       * \return frame
       */
      virtual FrameInterface* getFrame(const std::string &name, bool check=true);

      /**
       * \param name of the contour
       * \param check for existence of contour
       * \return contour
       */
      virtual Contour* getContour(const std::string &name, bool check=true);
      /*****************************************************/

      /* GETTER / SETTER */
      DynamicSystem* getParent() { return parent; }
      void setParent(DynamicSystem* sys) { parent = sys; }

      const fmatvec::Vec& getq() const { return q; };
      const fmatvec::Vec& getu() const { return u; };
      const fmatvec::Vec& getxd() const { return xd; };
      fmatvec::Vec& getxd() { return xd; };
      const fmatvec::Vec& getx0() const { return x0; };
      fmatvec::Vec& getx0() { return x0; };

      const fmatvec::Mat& getT() const { return T; };
      const fmatvec::SymMat& getM() const { return M; };
      const fmatvec::SymMat& getLLM() const { return LLM; };
      const fmatvec::Vec& geth() const { return h; };
      const fmatvec::Vec& getf() const { return f; };
      fmatvec::Vec& getf() { return f; };

      const fmatvec::Mat& getW() const { return W; }
      fmatvec::Mat& getW() { return W; }
      const fmatvec::Mat& getV() const { return V; }
      fmatvec::Mat& getV() { return V; }

      const fmatvec::Vec& getla() const { return la; }
      fmatvec::Vec& getla() { return la; }
      const fmatvec::Vec& getg() const { return g; }
      fmatvec::Vec& getg() { return g; }
      const fmatvec::Vec& getgd() const { return gd; }
      fmatvec::Vec& getgd() { return gd; }
      const fmatvec::Vec& getrFactor() const { return rFactor; }
      fmatvec::Vec& getrFactor() { return rFactor; }
      fmatvec::Vec& getsv() { return sv; }
      const fmatvec::Vec& getsv() const { return sv; }
      fmatvec::Vector<int>& getjsv() { return jsv; }
      const fmatvec::Vector<int>& getjsv() const { return jsv; }
      const fmatvec::Vec& getres() const { return res; }
      fmatvec::Vec& getres() { return res; }

      void setx(const fmatvec::Vec& x_) { x = x_; }
      void setx0(const fmatvec::Vec &x0_) { x0 = x0_; }
      void setx0(double x0_) { x0 = fmatvec::Vec(1,fmatvec::INIT,x0_); }

      int getxInd() { return xInd; }
      int getlaInd() const { return laInd; } 

      int getqInd() { return qInd; }
      int getuInd(int i=0) { return uInd[i]; }
      int gethInd(int i=0) { return hInd[i]; }
      void sethInd(int hInd_, int i=0) { hInd[i] = hInd_; }
      void setlaInd(int ind) { laInd = ind; }
      void setgInd(int ind) { gInd = ind; }
      void setgdInd(int ind) { gdInd = ind; }
      void setrFactorInd(int ind) { rFactorInd = ind; }
      void setsvInd(int svInd_) { svInd = svInd_; };

      int getzSize() const { return qSize + uSize[0] + xSize; }

      void setqSize(int qSize_) { qSize = qSize_; }
      void setuSize(int uSize_, int i=0) { uSize[i] = uSize_; }
      void setxSize(int xSize_) { xSize = xSize_; }

      int getlaSize() const { return laSize; } 
      int getgSize() const { return gSize; } 
      int getgdSize() const { return gdSize; } 
      int getrFactorSize() const { return rFactorSize; } 
      int getsvSize() const { return svSize; }
      /*****************************************************/

      /**
       * \brief references to positions of dynamic system parent
       * \param vector to be referenced
       */
      void updateqRef(const fmatvec::Vec &ref); 

      /**
       * \brief references to differentiated positions of dynamic system parent
       * \param vector to be referenced
       */
      void updateqdRef(const fmatvec::Vec &ref);

      /**
       * \brief references to velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updateuRef(const fmatvec::Vec &ref);

      /**
       * \brief references to differentiated velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updateudRef(const fmatvec::Vec &ref);

      /**
       * \brief references to smooth right hand side of dynamic system parent
       * \param vector to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updatehRef(const fmatvec::Vec &ref, int i=0);

      /**
       * \brief references to order one right hand side of dynamic system parent
       * \param vector to be referenced
       */
      void updatefRef(const fmatvec::Vec &ref);

      /**
       * \brief references to nonsmooth right hand side of dynamic system parent
       * \param vector to be referenced
       */
      void updaterRef(const fmatvec::Vec &ref);

      /**
       * \brief references to linear transformation matrix between differentiated positions and velocities of dynamic system parent
       * \param matrix to be referenced
       */
      void updateTRef(const fmatvec::Mat &ref);

      /**
       * \brief references to mass matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateMRef(const fmatvec::SymMat &ref, int i=0);

      /**
       * \brief references to Cholesky decomposition of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateLLMRef(const fmatvec::SymMat &ref, int i=0);

      /**
       * \brief references to relative distances of dynamic system parent
       * \param vector to be referenced
       */
      void updategRef(const fmatvec::Vec &ref);

      /**
       * \brief references to relative velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updategdRef(const fmatvec::Vec &ref);

      /**
       * \brief references to contact force parameters of dynamic system parent
       * \param vector to be referenced
       */
      void updatelaRef(const fmatvec::Vec &ref);

      /**
       * \brief references to TODO of dynamic system parent
       * \param vector to be referenced
       */      
      void updatewbRef(const fmatvec::Vec &ref);

      /**
       * \brief references to contact force direction matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateWRef(const fmatvec::Mat &ref, int i=0);

      /**
       * \brief references to condensed contact force direction matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateVRef(const fmatvec::Mat &ref, int i=0);

      /**
       * \brief references to stopvector (rootfunction for event driven integrator) of dynamic system parent
       * \param vector to be referenced
       */
      void updatesvRef(const fmatvec::Vec& ref);

      /**
       * \brief references to boolean evaluation of stopvector concerning roots of dynamic system parent
       * \param vector to be referenced
       */
      void updatejsvRef(const fmatvec::Vector<int> &ref);

      /**
       * \brief references to residuum of contact equations of dynamic system parent
       * \param vector to be referenced
       */
      void updateresRef(const fmatvec::Vec &ref);

      /**
       * \brief references to relaxation factors for contact equations of dynamic system parent
       * \param vector to be referenced
       */
      void updaterFactorRef(const fmatvec::Vec &ref);

      /**
       * \brief TODO
       */
      virtual void buildListOfObjects(std::vector<Object*> &obj, bool recursive=false);
      /**
       * \brief TODO
       */
      virtual void buildListOfLinks(std::vector<Link*> &lnk, bool recursive=false);
      /**
       * \brief TODO
       */
      virtual void buildListOfOrderOneDynamics(std::vector<OrderOneDynamics*> &ood, bool recursive=false);

      /**
       * \brief set possible attribute for active relative kinematics for updating event driven simulation before case study
       */
      void updateCondition();

      /**
       * \brief change JACOBIAN of contact size concerning use of inverse kinetics
       * \param index of normal usage and inverse kinetics
       */
      void resizeJacobians(int j);

      /**
       * \brief analyse constraints of dynamic systems for usage in inverse kinetics
       */
      void checkForConstraints();

      /**
       * \brief distribute links to set- and single valued container
       */
      void setUpLinks();

      /**
       * \return flag, if vector of active relative distances has changed
       */
      bool gActiveChanged();

      /**
       * \brief calculates size of stop vector
       */
      void calcsvSize();

      /**
       * \brief calculates size of contact force parameters
       */
      void calclaSize();

      /**
       * \brief calculates size of active contact force parameters
       */
      void calclaSizeForActiveg();

      /**
       * \brief calculates size of relative distances
       */
      void calcgSize();

      /**
       * \brief calculates size of active relative distances
       */
      void calcgSizeActive();

      /**
       * \brief calculates size of relative velocities
       */
      void calcgdSize();

      /**
       * \brief calculates size of active relative velocities
       */
      void calcgdSizeActive();

      /**
       * \brief calculates size of relaxation factors for contact equations
       */
      void calcrFactorSize();

      /** 
       * \brief rearrange vector of active setvalued links
       */
      void checkActiveLinks();

      /**
       * \brief set possible attribute for active relative distance in derived classes 
       */
      void checkActiveg();

      /**
       * \brief set possible attribute for active relative velocity in derived classes for initialising event driven simulation 
       */
      void checkActivegd();

      /**
       * \brief set possible attribute for active relative velocity in derived classes for updating event driven simulation after an impact
       */
      void checkActivegdn();

      /**
       * \brief set possible attribute for active relative acceleration in derived classes for updating event driven simulation after an impact
       */
      void checkActivegdd();

      /**
       * \brief set possible attribute for active relative velocity in derived classes for updating event driven and time-stepping simulation before an impact
       */
      void checkAllgd();

      /**
       * \param tolerance for relative velocity
       */
      void setgdTol(double tol);

      /**
       * \param tolerance for relative acceleration
       */
      void setgddTol(double tol);

      /**
       * \param tolerance for contact force
       */
      void setlaTol(double tol);

      /**
       * \param tolerance for impact
       */
      void setLaTol(double tol);

      /**
       * \param maximum relaxation factor for contact equations
       */
      void setrMax(double rMax);

      /**
       * \brief TODO
       */
      void setlaIndDS(int laIndParent);

      /**
       * \param frame to add
       * \param relative position of frame
       * \param relative orientation of frame
       * \param relation frame
       */
      void addFrame(Frame *frame_, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const FrameInterface* refFrame=0);

      /**
       * \param name of frame to add
       * \param relative position of frame
       * \param relative orientation of frame
       * \param relation frame
       */
      void addFrame(const std::string &str, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const FrameInterface* refFrame=0);

      /**
       * \param contour to add
       * \param relative position of contour
       * \param relative orientation of contour
       * \param relation frame
       */
      void addContour(Contour* contour, const fmatvec::Vec &RrRC, const fmatvec::SqrMat &ARC, const FrameInterface* refFrame=0);

      /**
       * \param contour to add
       * \param relative position of contour
       * \param relation frame
       */
      void addContour(Contour* contour, const fmatvec::Vec &RrRC, const FrameInterface* refFrame=0) { addContour(contour,RrRC,fmatvec::SqrMat(3,fmatvec::EYE),refFrame); }

      /**
       * \param frame
       * \return index of frame TODO renaming
       */
      int frameIndex(const Frame *frame_) const;

      /**
       * \param name of the dynamic system
       * \param check for existence of dynamic system
       * \return dynamic system
       */
      DynamicSystem* getDynamicSystem(const std::string &name,bool check=true);

      /**
       * \param object to add
       */
      void addObject(Object *object);

      /**
       * \param name of the object
       * \param check for existence of object
       * \return object
       */
      Object* getObject(const std::string &name,bool check=true);

      /**
       * \param link to add
       */
      void addLink(Link *link);

      /**
       * \param name of the link
       * \param check for existence of link
       * \return link
       */
      Link* getLink(const std::string &name,bool check=true);

      /**
       * \param order one dynamics to add
       */
      void addOrderOneDynamics(OrderOneDynamics *ood_);

      /**
       * \param name of the order one dynamics
       * \param check for existence of order one dynamics
       * \return order one dynamics
       */
      OrderOneDynamics* getOrderOneDynamics(const std::string &name,bool check=true);

      /**
       * \param data interface base to add
       */
      void addDataInterfaceBase(DataInterfaceBase* dib_);

      /**
       * \param name of the data interface interface
       * \param check for existence of data interface interface
       * \return data interface interface
       */
      DataInterfaceBase* getDataInterfaceBase(const std::string &name, bool check=true);

    protected:
      /**
       * \brief parent dynamic system
       */
      DynamicSystem *parent;

      /**
       * \brief frame of reference of the dynamic system
       */
      StationaryFrame frameOfReference;

      /** 
       * \brief container for possible ingredients
       */
      std::vector<Object*> object;
      std::vector<Link*> link;
      std::vector<OrderOneDynamics*> orderOneDynamics;
      std::vector<DataInterfaceBase*> DIB;
      std::vector<DynamicSystem*> dynamicsystem;
      std::vector<Link*> linkSingleValued;
      std::vector<Link*> linkSetValued;
      std::vector<Link*> linkSetValuedActive;

      /** 
       * \brief linear relation matrix of position and velocity parameters
       */
      fmatvec::Mat T;

      /**
       * \brief mass matrix
       */
      fmatvec::SymMat M;

      /** 
       * \brief Cholesky decomposition of mass matrix
       */
      fmatvec::SymMat LLM;

      /**
       * \brief positions, differentiated positions, initial positions
       */
      fmatvec::Vec q, qd, q0;

      /**
       * \brief velocities, differentiated velocities, initial velocities
       */
      fmatvec::Vec u, ud, u0;

      /**
       * \brief order one parameters, differentiated order one parameters, initial order one parameters
       */
      fmatvec::Vec x, xd, x0;

      /**
       * \brief smooth, nonsmooth and order one right hand side
       */
      fmatvec::Vec h, r, f;

      /**
       * \brief 
       */
      fmatvec::Mat W, V;

      /**
       * \brief contact force parameters
       */
      fmatvec::Vec la;

      /** 
       * \brief relative distances and velocities
       */
      fmatvec::Vec g, gd;

      /**
       * \brief TODO
       */
      fmatvec::Vec wb;

      /**
       * \brief residuum of nonlinear contact equations for Newton scheme
       */
      fmatvec::Vec res;

      /**
       * \brief rfactors for relaxation nonlinear contact equations
       */
      fmatvec::Vec rFactor;

      /**
       * \brief stop vector (root functions for event driven integration
       */
      fmatvec::Vec sv;

      /**
       * \brief boolean evaluation of stop vector concerning roots
       */
      fmatvec::Vector<int> jsv;

      /** 
       * \brief size and local start index of positions
       */
      int qSize, qInd;

      /** 
       * \brief size and local start index of velocities
       */
      int uSize[2], uInd[2];

      /** 
       * \brief size and local start index of order one parameters
       */
      int xSize, xInd;

      /** 
       * \brief size and local start index of order smooth right hand side
       */
      int hSize[2], hInd[2];

      /** 
       * \brief size and local start index of relative distances
       */
      int gSize, gInd;

      /** 
       * \brief size and local start index of relative velocities
       */
      int gdSize, gdInd;

      /** 
       * \brief size and local start index of contact force parameters
       */
      int laSize, laInd;

      /** 
       * \brief size and local start index of rfactors
       */
      int rFactorSize, rFactorInd;

      /** 
       * \brief size and local start index of stop vector
       */
      int svSize, svInd;

      /**
       * \brief inertial position of frames, contours and dynamic systems (see group.h / tree.h)
       */
      std::vector<fmatvec::Vec> IrOF, IrOC, IrOD;

      /**
       * \brief orientation to inertial frame of frames, contours and dynamic systems (see group.h / tree.h)
       */
      std::vector<fmatvec::SqrMat> AIF, AIC, AID;

      /**
       * \brief vector of frames and contours
       */
      std::vector<Frame*> frame;
      std::vector<Contour*> contour;

#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Group* amvisGrp;
#endif

      /**
       * \param frame to add
       */
      void addFrame(Frame *frame_);

      /**
       * \param contour to add
       */
      void addContour(Contour* contour);

      /**
       * \param dynamic system to add
       */
      void addDynamicSystem(DynamicSystem *dynamicsystem);
  };
}

#endif

