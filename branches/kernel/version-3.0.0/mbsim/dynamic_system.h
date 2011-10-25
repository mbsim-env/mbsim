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
#include "mbsim/link_interface.h"
//#include "mbsim/object_interface.h"
#include "mbsim/extradynamic_interface.h"
#include "mbsim/mbsim_event.h"

namespace H5 {
  class Group;
}

namespace MBSim {
  class Frame;
  class Contour;
  class ExtraDynamic;
  class Object;
  class Link;
  class ModellingInterface;
  class Contact;
  class InverseKineticsJoint;

  /**
   * \brief dynamic system as topmost hierarchical level
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   * \date 2009-04-06 ExtraDynamicInterface included (Thorsten Schindler)
   * \date 2009-06-14 OpenMP (Thorsten Schindler)
   * \date 2009-07-08 relative dynamic system location (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-27 implicit integration improvement (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   * \date 2009-12-14 revised inverse kinetics (Martin Foerg)
   * \date 2010-07-06 modifications for timestepper ssc, e.g LinkStatus and buildListOfSetValuedLinks (Robert Huber)
   * \todo OpenMP only static scheduling with intelligent reordering of vectors by dynamic test runs
   */
  //class DynamicSystem : public Element, public ObjectInterface, public LinkInterface, public ExtraDynamicInterface {
  class DynamicSystem : public Element, public LinkInterface, public ExtraDynamicInterface {
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
      virtual void updateh(double t, int i=0); 
      virtual void updateh0Fromh1(double t); 
      virtual void updateW0FromW1(double t); 
      virtual void updateV0FromV1(double t); 
      virtual void updateStateDependentVariables(double t) = 0;
      virtual void updateStateDerivativeDependentVariables(double t);
      virtual void updatedhdz(double t);
      virtual void updateM(double t, int i=0);
      virtual void updateJacobians(double t, int j=0) = 0;
      virtual void updatedq(double t, double dt); 
      virtual void updateud(double t, int i=0) { throw MBSimError("ERROR (DynamicSystem::updateud): Not implemented!"); }
      virtual void updatezd(double t) = 0;
      virtual void updatedu(double t, double dt) = 0;
      virtual void updateqd(double t);
      virtual void sethSize(int hSize_, int i=0);
      virtual int gethSize(int i=0) const { return hSize[i]; }
      virtual int getqSize() const { return qSize; }
      virtual int getuSize(int i=0) const { return uSize[i]; }
      virtual void calcqSize();
      virtual void calcuSize(int j=0);
      //virtual int getqInd(DynamicSystem* sys);
      virtual int getuInd(int i=0) { return uInd[i]; }
      //virtual int getuInd(DynamicSystem* sys, int i=0);
      //virtual void setqInd(int qInd_) { qInd = qInd_; }
      //virtual void setuInd(int uInd_, int i=0) { uInd[i] = uInd_; }
      virtual void setqInd(int qInd_);
      virtual void setuInd(int uInd_, int i=0);
      virtual void sethInd(int hInd_, int i=0);
      virtual void setxInd(int xInd_);
      //virtual int gethInd(DynamicSystem* sys, int i=0); 
      virtual const fmatvec::Vec& getq() const { return q; };
      virtual fmatvec::Vec& getq() { return q; };
      virtual const fmatvec::Vec& getu() const { return u; };
      virtual fmatvec::Vec& getu() { return u; };
      virtual H5::Group *getPlotGroup() { return plotGroup; }
      virtual PlotFeatureStatus getPlotFeature(PlotFeature fp) { return Element::getPlotFeature(fp); };
      virtual PlotFeatureStatus getPlotFeatureForChildren(PlotFeature fp) { return Element::getPlotFeatureForChildren(fp); };
#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual OpenMBV::Group* getOpenMBVGrp();
#endif
      /** 
       * Return the full path of the object.
       * \param pathDelim The delimiter of the path
       */
      std::string getPath(char pathDelim='.') { return parent?parent->getPath()+pathDelim+name:name; }
      /*****************************************************/

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updatewb(double t, int j=0); 
      virtual void updateW(double t, int j=0); 
      virtual void updateV(double t, int j=0); 
      virtual void updateg(double t);
      virtual void updategd(double t);
      virtual void updateStopVector(double t); 
      virtual void updateLinkStatus(double t);

      virtual void updategInverseKinetics(double t); 
      virtual void updategdInverseKinetics(double t);
      virtual void updateWInverseKinetics(double t, int j=0); 
      virtual void updatehInverseKinetics(double t, int j=0); 
      virtual void updateJacobiansInverseKinetics(double t, int j=0); 
      virtual void updatebInverseKinetics(double t); 
      /*****************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void updatedx(double t, double dt); 
      virtual void updatexd(double t);
      virtual void calcxSize();
      const fmatvec::Vec& getx() const { return x; };
      fmatvec::Vec& getx() { return x; };
      int getxSize() const { return xSize; }
      void updatexRef(const fmatvec::Vec &ref);
      void updatexdRef(const fmatvec::Vec &ref);
      virtual void init(InitStage stage);
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
      virtual void facLLM(int i=0) = 0;

      /**
       * \brief solve contact equations with single step fixed point scheme on acceleration level 
       */
      virtual int solveConstraintsFixpointSingle();

      /**
       * \brief solve contact equations with single step fixed point scheme on velocity level 
       */
      virtual int solveImpactsFixpointSingle(double dt);

      /**
       * \brief solve contact equations with Gauss-Seidel scheme on acceleration level 
       */
      virtual int solveConstraintsGaussSeidel();

      /**
       * \brief solve contact equations with Gauss-Seidel scheme on velocity level 
       */
      virtual int solveImpactsGaussSeidel(double dt);

      /**
       * \brief solve contact equations with Newton scheme on acceleration level 
       */
      virtual int solveConstraintsRootFinding();

      /**
       * \brief solve contact equations with Newton scheme on velocity level 
       */
      virtual int solveImpactsRootFinding(double dt);

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
      virtual void checkImpactsForTermination(double dt);

      /**
       * \brief update relaxation factors for contact equations
       */
      virtual void updaterFactors();

      /**
       * \param name of the frame
       * \param check for existence of frame
       * \return frame
       */
      virtual Frame* getFrame(const std::string &name, bool check=true);

      /**
       * \param name of the contour
       * \param check for existence of contour
       * \return contour
       */
      virtual Contour* getContour(const std::string &name, bool check=true);
      /*****************************************************/

      /* GETTER / SETTER */
      void setPosition(const fmatvec::Vec& PrPF_) { PrPF = PrPF_; }
      void setOrientation(const fmatvec::SqrMat& APF_) { APF = APF_; }
      void setFrameOfReference(Frame *frame) { frameParent = frame; };

      const fmatvec::Vec& getxd() const { return xd; };
      fmatvec::Vec& getxd() { return xd; };
      const fmatvec::Vec& getx0() const { return x0; };
      fmatvec::Vec& getx0() { return x0; };

      const fmatvec::Mat& getT() const { return T; };
      const fmatvec::SymMat& getM(int i=0) const { return M[i]; };
      const fmatvec::SymMat& getLLM(int i=0) const { return LLM[i]; };
      const fmatvec::Vec& geth(int j=0) const { return h[j]; };
      const fmatvec::Vec& getf() const { return f; };
      fmatvec::Vec& getf() { return f; };
      fmatvec::Mat getdhdq() const { return dhdqObject + dhdqLink; }
      fmatvec::SqrMat getdhdu() const { return dhduObject + dhduLink; }
      fmatvec::Vec getdhdt() const { return dhdtObject + dhdtLink; }

      const fmatvec::Mat& getW(int i=0) const { return W[i]; }
      fmatvec::Mat& getW(int i=0) { return W[i]; }
      const fmatvec::Mat& getV(int i=0) const { return V[i]; }
      fmatvec::Mat& getV(int i=0) { return V[i]; }

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
      fmatvec::Vector<int>& getLinkStatus() { return LinkStatus; }
      const fmatvec::Vector<int>& getLinkStatus() const { return LinkStatus; }
      const fmatvec::Vec& getres() const { return res; }
      fmatvec::Vec& getres() { return res; }

      void setx(const fmatvec::Vec& x_) { x = x_; }
      void setx0(const fmatvec::Vec &x0_) { x0 = x0_; }
      void setx0(double x0_) { x0 = fmatvec::Vec(1,fmatvec::INIT,x0_); }

      int getxInd() { return xInd; }
      int getlaInd() const { return laInd; } 

      int gethInd(int i=0) { return hInd[i]; }
      void setlaInd(int ind) { laInd = ind; }
      void setgInd(int ind) { gInd = ind; }
      void setgdInd(int ind) { gdInd = ind; }
      void setrFactorInd(int ind) { rFactorInd = ind; }
      void setsvInd(int svInd_) { svInd = svInd_; };
      void setLinkStatusInd(int LinkStatusInd_) {LinkStatusInd = LinkStatusInd_;};      

      int getzSize() const { return qSize + uSize[0] + xSize; }

      void setqSize(int qSize_) { qSize = qSize_; }
      void setuSize(int uSize_, int i=0) { uSize[i] = uSize_; }
      void setxSize(int xSize_) { xSize = xSize_; }

      int getlaSize() const { return laSize; } 
      int getgSize() const { return gSize; } 
      int getgdSize() const { return gdSize; } 
      int getrFactorSize() const { return rFactorSize; } 
      int getsvSize() const { return svSize; }
      int getLinkStatusSize() const { return LinkStatusSize; }
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
       * \brief references to velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updateuallRef(const fmatvec::Vec &ref);

      /**
       * \brief references to differentiated velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updateudRef(const fmatvec::Vec &ref, int i=0);

      /**
       * \brief references to velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updateudallRef(const fmatvec::Vec &ref, int i=0);

      /**
       * \brief references to smooth right hand side of dynamic system parent
       * \param complete vector to be referenced
       * \param vector concerning objects to be referenced
       * \param vector concerning links to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updatehRef(const fmatvec::Vec &hRef, int i=0);

      /**
       * \brief references to order one right hand side of dynamic system parent
       * \param vector to be referenced
       */
      void updatefRef(const fmatvec::Vec &ref);

      /**
       * \brief references to nonsmooth right hand side of dynamic system parent
       * \param vector to be referenced
       */
      void updaterRef(const fmatvec::Vec &ref, int j=0);

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

      void updatelaInverseKineticsRef(const fmatvec::Vec &ref);
      void updatebInverseKineticsRef(const fmatvec::Mat &ref);

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

      void updateWnVRefObjects();

      /**
       * \brief references to contact force direction matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateWInverseKineticsRef(const fmatvec::Mat &ref, int i=0);

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
       * \brief references to status vector of set valued links with piecewise link equations (which piece is valid)
       * \param vector to be referenced 
       */
      void updateLinkStatusRef(const fmatvec::Vector<int> &LinkStatusParent);

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
       * \brief build flat list of objects
       * \param list of objects
       * \param flag for recursive
       */
      virtual void buildListOfObjects(std::vector<Object*> &obj, bool recursive=false);

      /**
       * \brief build flat list of links
       * \param list of links
       * \param flag for recursive
       */
      virtual void buildListOfLinks(std::vector<Link*> &lnk, bool recursive=false);

      /**
       * \brief build flat list of all setvalued links
       * \param list of links
       * \param flag for recursive
       */
      virtual void buildListOfSetValuedLinks(std::vector<Link*> &lnk, bool recursive=false);

      /**
       * \brief build flat list of frames
       * \param list of frames
       * \param flag for recursive
       */
      virtual void buildListOfFrames(std::vector<Frame*> &frm, bool recursive);

      /**
       * \brief build flat list of contours
       * \param list of contours
       * \param flag for recursive
       */
      virtual void buildListOfContours(std::vector<Contour*> &cnt, bool recursive);

      /**
       * \brief build flat list of extra dynamic
       * \param list of extra dynamic
       * \param flag for recursive
       */
      virtual void buildListOfExtraDynamic(std::vector<ExtraDynamic*> &ed, bool recursive=false);

      /**
       * \brief build flat list of models
       * \param list of models
       * \param flag for recursive
       */
      void buildListOfModels(std::vector<ModellingInterface*> &model, bool recursive = true);

      /**
       * \brief set possible attribute for active relative kinematics for updating event driven simulation before case study
       */
      void updateCondition();
      void checkState();

      /**
       * \brief analyse constraints of dynamic systems for usage in inverse kinetics
       */
      void setUpInverseKinetics();

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
       * \brief calculates size of link status vector
       */
      void calcLinkStatusSize();

      /**
       * \brief calculates size of contact force parameters
       */
      void calclaInverseKineticsSize();

      /**
       * \brief calculates size of contact force parameters
       */
      void calcbInverseKineticsSize();

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
      void setgTol(double tol);

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
       * \param frame to add
       * \param relative position of frame
       * \param relative orientation of frame
       * \param relation frame name
       */
      void addFrame(Frame *frame_, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const std::string& refFrameName);

      /**
       * \param frame to add
       * \param relative position of frame
       * \param relative orientation of frame
       * \param relation frame
       */
      void addFrame(Frame *frame_, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const Frame* refFrame=0);

      /**
       * \param name of frame to add
       * \param relative position of frame
       * \param relative orientation of frame
       * \param relation frame
       */
      void addFrame(const std::string &str, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const Frame* refFrame=0);

      /**
       * \param contour to add
       * \param relative position of contour
       * \param relative orientation of contour
       * \param relation frame name
       */
      void addContour(Contour* contour, const fmatvec::Vec &RrRC, const fmatvec::SqrMat &ARC, const std::string& refFrameName);

      /**
       * \param contour to add
       * \param relative position of contour
       * \param relative orientation of contour
       * \param relation frame
       */
      void addContour(Contour* contour, const fmatvec::Vec &RrRC, const fmatvec::SqrMat &ARC, const Frame* refFrame=0);

      /**
       * \param contour to add
       * \param relative position of contour
       * \param relation frame
       */
      void addContour(Contour* contour, const fmatvec::Vec &RrRC, const Frame* refFrame=0) { addContour(contour,RrRC,fmatvec::SqrMat(3,fmatvec::EYE),refFrame); }

      /**
       * \param frame
       * \return index of frame TODO renaming
       */
      int frameIndex(const Frame *frame_) const;

      /**
       * \param dynamic system to add
       */
      void addGroup(DynamicSystem *dynamicsystem);

      /**
       * \param name of the dynamic system
       * \param check for existence of dynamic system
       * \return dynamic system
       */
      DynamicSystem* getGroup(const std::string &name,bool check=true);

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
       * \param add link for inverse kinetics
       */
      void addInverseKineticsLink(Link *link);

      /**
       * \param name of the link
       * \param check for existence of link
       * \return link
       */
      Link* getLink(const std::string &name,bool check=true);

      /**
       * \param extra dynamic to add
       */
      void addExtraDynamic(ExtraDynamic *ed_);

      /**
       * \param name of the extra dynamic
       * \param check for existence of extra dynamic
       * \return extra dynamic
       */
      ExtraDynamic* getExtraDynamic(const std::string &name,bool check=true);

      /**
       * \param modell to add
       */
      void addModel(ModellingInterface *modell);

      /**
       * \param name of the model
       * \param check for existence of model
       * \return modelling interface
       */
      ModellingInterface* getModel(const std::string &name, bool check=true);

      /** Return frame "I" */
      Frame *getFrameI() { return I; }

      virtual Element *getByPathSearch(std::string path);

    protected:
      /**
       * \brief parent frame
       */
      Frame *frameParent;

      /**
       * \brief relative translation with respect to parent frame
       */
      fmatvec::Vec PrPF;

      /**
       * \brief relative rotation with respect to parent frame
       */
      fmatvec::SqrMat APF;

      /** 
       * \brief container for possible ingredients
       */
      std::vector<Object*> object;
      std::vector<Link*> link;
      std::vector<Link*> linkSingleValued;
      std::vector<Link*> linkSetValued;
      std::vector<Link*> linkSetValuedActive;
      std::vector<Link*> linkSetValuedNotActiveWithSmoothPart;
      std::vector<ExtraDynamic*> extraDynamic;
      std::vector<ModellingInterface*> model;
      std::vector<DynamicSystem*> dynamicsystem;
      std::vector<Link*> inverseKineticsLink;

      /** 
       * \brief linear relation matrix of position and velocity parameters
       */
      fmatvec::Mat T;

      /**
       * \brief mass matrix
       */
      fmatvec::SymMat M[2];

      /** 
       * \brief Cholesky decomposition of mass matrix
       */
      fmatvec::SymMat LLM[2];

      /**
       * \brief positions, differentiated positions, initial positions
       */
      fmatvec::Vec q, qd, q0;

      /**
       * \brief velocities, differentiated velocities, initial velocities
       */
      fmatvec::Vec u, ud[2], u0;

      /**
       * \brief order one parameters, differentiated order one parameters, initial order one parameters
       */
      fmatvec::Vec x, xd, x0;

      /**
       * \brief smooth, smooth with respect to objects, smooth with respect to links, nonsmooth and order one right hand side
       */
      fmatvec::Vec h[2], r[2], f;

      /**
       * \brief matrices for implicit integration
       */
      fmatvec::Mat    dhdqObject, dhdqLink;
      fmatvec::SqrMat dhduObject, dhduLink;
      fmatvec::Vec    dhdtObject, dhdtLink;

      /**
       * \brief 
       */
      fmatvec::Mat W[2], V[2];

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
       * \brief status of set valued links 
       */
      fmatvec::Vector<int> LinkStatus;

      /** 
       * \brief size and local start index of positions relative to parent
       */
      int qSize, qInd;

      /** 
       * \brief size and local start index of velocities relative to parent
       */
      int uSize[2], uInd[2];

      /** 
       * \brief size and local start index of order one parameters relative to parent
       */
      int xSize, xInd;

      /** 
       * \brief size and local start index of order smooth right hand side relative to parent
       */
      int hSize[2], hInd[2];

      /** 
       * \brief size and local start index of relative distances relative to parent
       */
      int gSize, gInd;

      /** 
       * \brief size and local start index of relative velocities relative to parent
       */
      int gdSize, gdInd;

      /** 
       * \brief size and local start index of contact force parameters relative to parent
       */
      int laSize, laInd;

      /** 
       * \brief size and local start index of rfactors relative to parent
       */
      int rFactorSize, rFactorInd;

      /** 
       * \brief size and local start index of stop vector relative to parent
       */
      int svSize, svInd;

      /**
       * \brief size and local start index of link status vector relative to parent
       */
      int LinkStatusSize, LinkStatusInd;

      /**
       * \brief inertial position of frames, contours (see group.h / tree.h)
       */
      std::vector<fmatvec::Vec> IrOF, IrOC;

      /**
       * \brief orientation to inertial frame of frames, contours (see group.h / tree.h)
       */
      std::vector<fmatvec::SqrMat> AIF, AIC;

      /**
       * \brief vector of frames and contours
       */
      std::vector<Frame*> frame;
      std::vector<Contour*> contour;

#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group* openMBVGrp;
#endif

      /**
       * \param frame to add
       */
      void addFrame(Frame *frame_);

      /**
       * \param contour to add
       */
      void addContour(Contour* contour);

      /** A pointer to frame "I" */
      Frame *I;

      /** 
       * \brief size of contact force parameters of special links relative to parent
       */
      int laInverseKineticsSize, bInverseKineticsSize;

      fmatvec::Mat WInverseKinetics[2], bInverseKinetics;
      fmatvec::Vec laInverseKinetics;

    private:
      std::vector<std::string> saved_refFrameF, saved_refFrameC;
      std::vector<fmatvec::Vec> saved_RrRF, saved_RrRC;
      std::vector<fmatvec::SqrMat> saved_ARF, saved_ARC;
  };
}

#endif

