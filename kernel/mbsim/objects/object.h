/* Copyright (C) 2004-2014 MBSim Development Team
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
      virtual void updateT() { }
      virtual void updateh(int j=0) { }
      virtual void updateM() { }
      virtual void updatedhdz();
      virtual void updatedq();
      virtual void updatedu();
      virtual void updateud();
      virtual void updateqd();
      virtual void sethSize(int hSize_, int i=0);
      virtual int gethSize(int i=0) const { return hSize[i]; }
      virtual int getqSize() const { return qSize; }
      virtual int getuSize(int i=0) const { return uSize[i]; }
      virtual void calcqSize() {};
      virtual void calcuSize(int j) {};
      //virtual int getqInd(DynamicSystem* sys);
      virtual int getuInd(int i=0) { return uInd[i]; }
      //virtual int getuInd(DynamicSystem* sys, int i=0)
      virtual void setqInd(int qInd_) { qInd = qInd_; }
      virtual void setuInd(int uInd_, int i=0) { uInd[i] = uInd_; }
      //virtual int gethInd(DynamicSystem* sys,int i=0); 
      virtual const fmatvec::Vec& getq() const { return q; }
      virtual const fmatvec::Vec& getu() const { return u; }
      virtual H5::GroupBase *getPlotGroup() { return plotGroup; }
      virtual PlotFeatureStatus getPlotFeature(PlotFeature fp) { return Element::getPlotFeature(fp); }
      virtual PlotFeatureStatus getPlotFeatureForChildren(PlotFeature fp) { return Element::getPlotFeatureForChildren(fp); }
      /*******************************************************/ 

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot();
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
      virtual void updateudRef(const fmatvec::Vec& udRef);

      /**
       * \brief references to differentiated velocities of dynamic system parent
       * \param udallRef vector to be referenced
       */
      virtual void updateudallRef(const fmatvec::Vec& udallRef);

      /**
       * \brief references to smooth force vector of dynamic system parent
       * \param hRef vector to be referenced
       * \param i    index of normal usage and inverse kinetics
       */
      virtual void updatehRef(const fmatvec::Vec& hRef, int i=0);

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
       * \brief references to nonsmooth force vector of dynamic system parent
       * \param vector to be referenced
       */
      virtual void updaterdtRef(const fmatvec::Vec& ref);

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
      virtual void updateMRef(const fmatvec::SymMat &ref);

      /**
       * \brief references to Cholesky decomposition of dynamic system parent
       * \param vector to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updateLLMRef(const fmatvec::SymMat &ref);

      /**
       * \brief initialize object at start of simulation with respect to contours and frames
       */
      virtual void init(InitStage stage);

      /**
       * initialize state of object at start of simulation
       */
      virtual void initz();

      /*!
       * \brief writes its z-Vector to a subgroup of the given group
       */
      virtual void writez(H5::GroupBase *group);

      /*!
       * \brief reads the z-Vector of a subgroup of the given group
       */
      virtual void readz0(H5::GroupBase *group);

      /**
       * \brief perform Cholesky decomposition of mass martix
       */
      virtual void updateLLM() { LLM = facLL(evalM()); }

      /**
       * \return kinetic energy 
       */
      virtual double evalKineticEnergy() { return 0.5*u.T()*M*u; }

      /**
       * \return potential energy
       */
      virtual double evalPotentialEnergy() { return 0; }

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

      const fmatvec::Vec& geth(int i=0, bool check=true) const;
      fmatvec::Vec& geth(int i=0, bool check=true);

      fmatvec::Vec& getq() { return q; };
      fmatvec::Vec& getu() { return u; };

      const fmatvec::Vec& getq0() const { return q0; };
      const fmatvec::Vec& getu0() const { return u0; };
      fmatvec::Vec& getq0() { return q0; };
      fmatvec::Vec& getu0() { return u0; };

      const fmatvec::Mat& evalT();
      const fmatvec::Vec& evalh(int i=0);
      const fmatvec::SymMat& evalM();
      const fmatvec::SymMat& evalLLM();
      const fmatvec::Vec& evalr(int i=0);
      const fmatvec::Vec& evalrdt();
      const fmatvec::Vec& evaludall();

      void setq(const fmatvec::Vec &q_) { q = q_; }
      void setu(const fmatvec::Vec &u_) { u = u_; }

      /*******************************************************/ 

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

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
      fmatvec::Vec qd, ud, udall;

      /** 
       * \brief complete and object smooth and nonsmooth right hand side
       */
      fmatvec::Vec h[2], r[2], rdt;

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
      fmatvec::SymMat M;

      /**
       * \brief LU-decomposition of mass matrix 
       */
      fmatvec::SymMat LLM;

  };

}

#endif /* _OBJECT_H_ */
