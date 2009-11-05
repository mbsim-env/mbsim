/* Copyright (C) 2004-2009 MBSim Development Team
 *
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

#ifndef _OBJECT_H_
#define _OBJECT_H_

#include "mbsim/element.h"
#include "mbsim/object_interface.h"
#include "mbsim/frame.h"
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"
#include <string>
#include <vector>

namespace MBSim {

  class DynamicSystem;
  class Contour;
  class Link;

  /** 
   * \brief class for all objects having own dynamics and mass
   * \author Martin Foerg
   * \date 2009-03-24 plot energy added (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-27 implicit integration improvement (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   */
  class Object : public Element, public ObjectInterface {
    public: 
      /**
       * \brief constructor
       */
      Object(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Object();

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateT(double t) {};
      virtual void updateh(double t) {};
      virtual void updateM(double t) {};
      virtual void updatedhdz(double t);
      virtual void updatedq(double t, double dt);
      virtual void updatedu(double t, double dt);
      virtual void updateud(double t);
      virtual void updateqd(double t);
      virtual void updatezd(double t);
      virtual void sethSize(int hSize_, int i=0);
      virtual int gethSize(int i=0) const { return hSize[i]; }
      virtual int getqSize() const { return qSize; }
      virtual int getuSize(int i=0) const { return uSize[i]; }
      virtual void calcqSize() {};
      virtual void calcuSize(int j) {};
      virtual int getqInd(DynamicSystem* sys);
      virtual int getuInd(int i=0) { return uInd[i]; }
      virtual int getuInd(DynamicSystem* sys, int i=0);
      virtual void setqInd(int qInd_) { qInd = qInd_; }
      virtual void setuInd(int uInd_, int i=0) { uInd[i] = uInd_; }
      virtual int gethInd(DynamicSystem* sys,int i=0); 
      virtual const fmatvec::Vec& getq() const { return q; };
      virtual const fmatvec::Vec& getu() const { return u; };
      virtual H5::Group *getPlotGroup() { return plotGroup; }
      virtual PlotFeatureStatus getPlotFeature(PlotFeature fp) { return Element::getPlotFeature(fp); };
      virtual PlotFeatureStatus getPlotFeatureForChildren(PlotFeature fp) { return Element::getPlotFeatureForChildren(fp); };
      /*******************************************************/ 

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt = 1); 
      virtual void closePlot();
      virtual std::string getType() const { return "Object"; }
      virtual void setDynamicSystemSolver(DynamicSystemSolver *sys);
      /*******************************************************/ 

      /**
       * \brief references to positions of dynamic system parent
       * \param vector to be referenced
       */
      virtual void updateqRef(const fmatvec::Vec& ref);

      /**
       * \brief references to differentiated positions of dynamic system parent
       * \param vector to be referenced
       */
      virtual void updateqdRef(const fmatvec::Vec& ref);

      /**
       * \brief references to velocities of dynamic system parent
       * \param vector to be referenced
       */
      virtual void updateuRef(const fmatvec::Vec& ref);

      /**
       * \brief references to differentiated velocities of dynamic system parent
       * \param vector to be referenced
       */
      virtual void updateudRef(const fmatvec::Vec& ref);

      /**
       * \brief references to smooth force vector of dynamic system parent
       * \param vector to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updatehRef(const fmatvec::Vec& hRef, const fmatvec::Vec& hObjectRef, int i=0);

      /**
       * \brief references to object Jacobian for implicit integration of dynamic system parent regarding positions
       * \param matrix concerning links to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updatedhdqRef(const fmatvec::Mat& ref, int i=0);

      /**
       * \brief references to object Jacobian for implicit integration of dynamic system parent regarding velocities
       * \param matrix concerning links to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updatedhduRef(const fmatvec::SqrMat& ref, int i=0);

      /**
       * \brief references to object Jacobian for implicit integration of dynamic system parent regarding time
       * \param matrix concerning links to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updatedhdtRef(const fmatvec::Vec& ref, int i=0);

      /**
       * \brief references to nonsmooth force vector of dynamic system parent
       * \param vector to be referenced
       */
      virtual void updaterRef(const fmatvec::Vec& ref);

      /**
       * \brief references to linear transformation matrix between differentiated positions and velocities of dynamic system parent
       * \param matrix to be referenced
       */
      virtual void updateTRef(const fmatvec::Mat &ref);

      /**
       * \brief references to mass matrix of dynamic system parent
       * \param vector to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updateMRef(const fmatvec::SymMat &ref, int i=0);

      /**
       * \brief references to Cholesky decomposition of dynamic system parent
       * \param vector to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updateLLMRef(const fmatvec::SymMat &ref, int i=0);

      /**
       * \brief initialize object at start of simulation with respect to contours and frames
       */
      virtual void init(InitStage stage);

      /**
       * initialize state of object at start of simulation
       */
      virtual void initz();

      /**
       * \brief perform Cholesky decomposition of mass martix
       */
      virtual void facLLM();

      /**
       * \brief checks dependency on other objects.
       * \return a vector of objects the calling objects depends on.
       */
      std::vector<Object*> getObjectsDependingOn() const { return dependency; }

      void addDependency(Object* obj) { dependency.push_back(obj); }
      /**
       * \brief computes the length of the pathes in the graph that represents
       * the dependencies between all objects. The function also cuts
       * all dependencies except the one associated with the longest path.
       * \return the length of the longest path in the graph.
       */
      int cutDependencies();
      int computeLevel();

      /**
       * \brief calculates size of right hand side
       * \param index of normal usage and inverse kinetics TODO
       */
      virtual void calcSize(int j) {}

      /**
       * \return kinetic energy 
       */
      virtual double computeKineticEnergy() { return 0.5*trans(u)*M*u; }

      /**
       * \return potential energy
       */
      virtual double computePotentialEnergy() { return 0; }

      /**
       * \brief TODO
       */
      virtual void resizeJacobians(int j) {}

      /**
       * \brief TODO
       */
      virtual void checkForConstraints() {}

      /**
       * \return frame of reference
       */
      virtual Frame *getFrameOfReference() { return frameOfReference; }

      /**
       * \return frame of reference
       */
      virtual const Frame *getFrameOfReference() const { return frameOfReference; }

      /**
       * \param frame of reference
       */
      virtual void setFrameOfReference(Frame *frame) { frameOfReference = frame; }
      /*******************************************************/ 

      /* GETTER / SETTER */
      DynamicSystem* getParent() { return parent; }
      void setParent(DynamicSystem* sys) { parent = sys; }

      void setqSize(int qSize_) { qSize = qSize_; }
      void setuSize(int uSize_, int i=0) { uSize[i] = uSize_; }
      int getzSize() const { return qSize + uSize[0]; }

      void sethInd(int hInd_, int i=0); 
      int gethInd(int i=0) { return hInd[i]; }

      const fmatvec::Index& getuIndex() const { return Iu;}
      const fmatvec::Index& gethIndex() const { return Ih;}

      const fmatvec::Vec& geth() const { return h; };
      fmatvec::Vec& geth() { return h; };
      const fmatvec::Vec& getr() const { return r; };
      fmatvec::Vec& getr() { return r; };
      const fmatvec::SymMat& getM() const { return M; };
      fmatvec::SymMat& getM() { return M; };
      const fmatvec::Mat& getT() const { return T; };
      fmatvec::Mat& getT() { return T; };
      const fmatvec::SymMat& getLLM() const { return LLM; };
      fmatvec::SymMat& getLLM() { return LLM; };

      fmatvec::Vec& getq() { return q; };
      fmatvec::Vec& getu() { return u; };

      const fmatvec::Vec& getq0() const { return q0; };
      const fmatvec::Vec& getu0() const { return u0; };
      fmatvec::Vec& getq0() { return q0; };
      fmatvec::Vec& getu0() { return u0; };

      const fmatvec::Vec& getqd() const { return qd; };
      const fmatvec::Vec& getud() const { return ud; };
      fmatvec::Vec& getqd() { return qd; };
      fmatvec::Vec& getud() { return ud; };

      void setq(const fmatvec::Vec &q_) { q = q_; }
      void setu(const fmatvec::Vec &u_) { u = u_; }

      void setInitialGeneralizedPosition(const fmatvec::Vec &q0_) { q0 = q0_; }
      void setInitialGeneralizedVelocity(const fmatvec::Vec &u0_) { u0 = u0_; }
      void setInitialGeneralizedPosition(double q0_) { q0 = fmatvec::Vec(1,fmatvec::INIT,q0_); }
      void setInitialGeneralizedVelocity(double u0_) { u0 = fmatvec::Vec(1,fmatvec::INIT,u0_); }

      /** 
       * \return full path of the object
       * \param delimiter of the path
       */
      std::string getPath(char pathDelim='.');
      /*******************************************************/ 

      virtual void initializeUsingXML(TiXmlElement *element);
      virtual Frame *getFrameByPath(std::string path);
      virtual Contour *getContourByPath(std::string path);
      virtual Link *getLinkByPath(std::string path);

    protected:
      /**
       * \brief dynamic system, object belongs to
       */
      DynamicSystem * parent;

      /**
       * \brief frame of reference of the object
       */
      Frame * frameOfReference;

      /**
       * \brief size of object positions
       */
      int qSize;

      /** 
       * \brief size of object velocities
       */
      int uSize[2];

      /**
       * \brief size of object h-vector (columns of J)
       */
      int hSize[2];

      /**
       * \brief indices of positions, velocities, right hand side
       */
      int qInd, uInd[2], hInd[2];

      /** 
       * \brief positions, velocities
       */
      fmatvec::Vec q, u;

      /**
       * \brief initial position, velocity
       */
      fmatvec::Vec q0,u0;

      /**
       * \brief differentiated positions, velocities
       */
      fmatvec::Vec qd,ud;

      /** 
       * \brief complete and object smooth and nonsmooth right hand side
       */
      fmatvec::Vec h, hObject, r;

      /** 
       * \brief Jacobians of h
       */
      fmatvec::Mat    dhdq;
      fmatvec::SqrMat dhdu;
      fmatvec::Vec    dhdt;

      /** 
       * \brief linear relation matrix of differentiated position and velocity parameters
       */
      fmatvec::Mat T;

      /** 
       * \brief mass matrix 
       */
      fmatvec::SymMat M;

      /**
       * \brief LU-decomposition of mass matrix 
       */
      fmatvec::SymMat LLM;

      /**
       * \brief indices for velocities and right hand side
       */
      fmatvec::Index Iu, Ih;

      /**
       * \brief vector containing all dependencies.
       */
      std::vector<Object*> dependency;

    private:
      std::string saved_frameOfReference;
  };

}

#endif /* _OBJECT_H_ */

