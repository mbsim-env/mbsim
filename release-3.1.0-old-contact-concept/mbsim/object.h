/* Copyright (C) 2004-2010 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _OBJECT_H_
#define _OBJECT_H_

#include "mbsim/element.h"
//#include "mbsim/object_interface.h"

namespace MBSim {

  class DynamicSystem;
  class Link;

  /** 
   * \brief class for all objects having own dynamics and mass
   * \author Martin Foerg
   * \date 2009-03-24 plot energy added (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-27 implicit integration improvement (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   * \date 2009-12-14 revised inverse kinetics (Martin Foerg)
   * \date 2010-06-20 revision of doxygen comments: add parameter names (Roland Zander)
   */
  //class Object : public Element, public ObjectInterface {
  class Object : public Element {
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
      virtual void updateh(double t, int j=0) {};
      virtual void updateh0Fromh1(double t);
      virtual void updateW0FromW1(double t);
      virtual void updateV0FromV1(double t);
      virtual void updateM(double t, int i=0) {};
      virtual void updatedhdz(double t);
      virtual void updatedq(double t, double dt);
      virtual void updatedu(double t, double dt);
      virtual void updateud(double t, int i=0);
      virtual void updateqd(double t);
      virtual void updatezd(double t);
      virtual void sethSize(int hSize_, int i=0);
      virtual int gethSize(int i=0) const { return hSize[i]; }
      virtual int getqSize() const { return qSize; }
      virtual int getuSize(int i=0) const { return uSize[i]; }
      virtual void calcqSize() {};
      virtual void calcuSize(int j) {};
      //virtual int getqInd(DynamicSystem* sys);
      virtual int getuInd(int i=0) { return uInd[i]; }
      //virtual int getuInd(DynamicSystem* sys, int i=0);
      virtual void setqInd(int qInd_) { qInd = qInd_; }
      virtual void setuInd(int uInd_, int i=0) { uInd[i] = uInd_; }
      //virtual int gethInd(DynamicSystem* sys,int i=0); 
      virtual const fmatvec::Vec& getq() const { return q; };
      virtual const fmatvec::Vec& getu() const { return u; };
      virtual H5::Group *getPlotGroup() { return plotGroup; }
      virtual PlotFeatureStatus getPlotFeature(PlotFeature fp) { return Element::getPlotFeature(fp); };
      virtual PlotFeatureStatus getPlotFeatureForChildren(PlotFeature fp) { return Element::getPlotFeatureForChildren(fp); };
      virtual void updateStateDependentVariables(double t) = 0;
      virtual void updateStateDerivativeDependentVariables(double t) {};
      virtual void updateJacobians(double t, int j=0) = 0;
      virtual void updatehInverseKinetics(double t, int i=0) {};
      /*******************************************************/ 

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt = 1); 
      virtual void closePlot();
      virtual std::string getType() const { return "Object"; }
      //virtual void setDynamicSystemSolver(DynamicSystemSolver *sys);
      /*******************************************************/ 

      /**
       * \brief references to positions of dynamic system parent
       * \param qRef vector to be referenced
       */
      virtual void updateqRef(const fmatvec::Vec& qRef);

      /**
       * \brief references to differentiated positions of dynamic system parent
       * \param qdRef vector to be referenced
       */
      virtual void updateqdRef(const fmatvec::Vec& qdRef);

      /**
       * \brief references to velocities of dynamic system parent
       * \param uRef vector to be referenced
       */
      virtual void updateuRef(const fmatvec::Vec& uRef);

      /**
       * \brief references to velocities of dynamic system parent
       * \param uallRef vector to be referenced
       */
      virtual void updateuallRef(const fmatvec::Vec& uallRef);

      /**
       * \brief references to differentiated velocities of dynamic system parent
       * \param udRef vector to be referenced
       */
      virtual void updateudRef(const fmatvec::Vec& udRef, int i=0);

      /**
       * \brief references to differentiated velocities of dynamic system parent
       * \param udallRef vector to be referenced
       */
      virtual void updateudallRef(const fmatvec::Vec& udallRef, int i=0);

      /**
       * \brief references to smooth force vector of dynamic system parent
       * \param hRef vector to be referenced
       * \param i    index of normal usage and inverse kinetics
       */
      virtual void updatehRef(const fmatvec::Vec& hRef, int i=0);

      virtual void updateWRef(const fmatvec::Mat& WRef, int i=0);
      virtual void updateVRef(const fmatvec::Mat& VRef, int i=0);

      /**
       * \brief references to object Jacobian for implicit integration of dynamic system parent regarding positions
       * \param dhdqRef matrix concerning links to be referenced
       * \param i       index of normal usage and inverse kinetics
       */
      virtual void updatedhdqRef(const fmatvec::Mat& dhdqRef, int i=0);

      /**
       * \brief references to object Jacobian for implicit integration of dynamic system parent regarding velocities
       * \param dhduRef matrix concerning links to be referenced
       * \param i       index of normal usage and inverse kinetics
       */
      virtual void updatedhduRef(const fmatvec::SqrMat& dhduRef, int i=0);

      /**
       * \brief references to object Jacobian for implicit integration of dynamic system parent regarding time
       * \param dhdtRef matrix concerning links to be referenced
       * \param i       index of normal usage and inverse kinetics
       */
      virtual void updatedhdtRef(const fmatvec::Vec& dhdtRef, int i=0);

      /**
       * \brief references to nonsmooth force vector of dynamic system parent
       * \param vector to be referenced
       */
      virtual void updaterRef(const fmatvec::Vec& ref, int i=0);

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
      virtual void facLLM(int i=0);

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
       * \param j index of normal usage and inverse kinetics TODO
       */
      virtual void calcSize(int j) {}

      /**
       * \return kinetic energy 
       */
      virtual double computeKineticEnergy() { return 0.5*u.T()*M[0]*u; }

      /**
       * \return potential energy
       */
      virtual double computePotentialEnergy() { return 0; }

      /**
       * \brief TODO
       */
      virtual void setUpInverseKinetics() {}
      /*******************************************************/ 

      /* GETTER / SETTER */
      void setqSize(int qSize_) { qSize = qSize_; }
      void setuSize(int uSize_, int i=0) { uSize[i] = uSize_; }
      int getzSize() const { return qSize + uSize[0]; }

      virtual void sethInd(int hInd_, int i=0); 
      int gethInd(int i=0) { return hInd[i]; }

      const fmatvec::Vec& geth(int i=0) const { return h[i]; };
      fmatvec::Vec& geth(int i=0) { return h[i]; };
      const fmatvec::Vec& getr(int i=0) const { return r[i]; };
      fmatvec::Vec& getr(int i=0) { return r[i]; };
      const fmatvec::SymMat& getM(int i=0) const { return M[i]; };
      fmatvec::SymMat& getM(int i=0) { return M[i]; };
      const fmatvec::Mat& getT() const { return T; };
      fmatvec::Mat& getT() { return T; };
      const fmatvec::SymMat& getLLM(int i=0) const { return LLM[i]; };
      fmatvec::SymMat& getLLM(int i=0) { return LLM[i]; };

      fmatvec::Vec& getq() { return q; };
      fmatvec::Vec& getu() { return u; };

      const fmatvec::Vec& getq0() const { return q0; };
      const fmatvec::Vec& getu0() const { return u0; };
      fmatvec::Vec& getq0() { return q0; };
      fmatvec::Vec& getu0() { return u0; };

      const fmatvec::Vec& getqd() const { return qd; };
      const fmatvec::Vec& getud(int i=0) const { return ud[i]; };
      fmatvec::Vec& getqd() { return qd; };
      fmatvec::Vec& getud(int i=0) { return ud[i]; };

      void setq(const fmatvec::Vec &q_) { q = q_; }
      void setu(const fmatvec::Vec &u_) { u = u_; }

      void setInitialGeneralizedPosition(const fmatvec::Vec &q0_) { q0 = q0_; }
      void setInitialGeneralizedVelocity(const fmatvec::Vec &u0_) { u0 = u0_; }
      void setInitialGeneralizedPosition(double q0_) { q0 = fmatvec::Vec(1,fmatvec::INIT,q0_); }
      void setInitialGeneralizedVelocity(double u0_) { u0 = fmatvec::Vec(1,fmatvec::INIT,u0_); }

      /*******************************************************/ 

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);

      virtual Element* getByPathSearch(std::string path);

    protected:
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
      fmatvec::Vec q, u, uall;

      /**
       * \brief initial position, velocity
       */
      fmatvec::Vec q0, u0;

      /**
       * \brief differentiated positions, velocities
       */
      fmatvec::Vec qd, ud[2], udall[2];

      /** 
       * \brief complete and object smooth and nonsmooth right hand side
       */
      fmatvec::Vec h[2], r[2];

      fmatvec::Mat W[2], V[2];

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
      fmatvec::SymMat M[2];

      /**
       * \brief LU-decomposition of mass matrix 
       */
      fmatvec::SymMat LLM[2];

      /**
       * \brief indices for velocities and right hand side
       */
      //fmatvec::Index Iu, Ih;

      /**
       * \brief vector containing all dependencies.
       */
      std::vector<Object*> dependency;
  };

}

#endif /* _OBJECT_H_ */

