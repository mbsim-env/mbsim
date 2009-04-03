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

#include <string>
#include <vector>
#include <mbsim/element.h>
#include <mbsim/interfaces.h>
#include <mbsim/dynamic_system.h>
#ifdef HAVE_AMVISCPPINTERFACE
#include <amviscppinterface/body.h>
#endif

namespace MBSim {

  class Frame;
  class Contour;
  class DynamicSystem;

  /** 
   * \brief class for all objects having own dynamics and mass
   * \author Martin Foerg
   * \date 2009-03-24 plot energy added (Thorsten Schindler)
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
      void updateT(double t) {};
      void updateh(double t) {};
      void updateM(double t) {};
      void updatedq(double t, double dt);
      void updatedu(double t, double dt);
      void updateud(double t);
      void updateqd(double t);
      void updatezd(double t);
      void sethSize(int hSize_, int i=0);
      int gethSize(int i=0) const { return hSize[i]; }
      int getqSize() const { return qSize; }
      int getuSize(int i=0) const { return uSize[i]; }
      virtual void calcqSize() {};
      virtual void calcuSize(int j) {};
      void setqInd(int qInd_) { qInd = qInd_; }
      void setuInd(int uInd_, int i=0) { uInd[i] = uInd_; }
      int gethInd(DynamicSystem* sys,int i=0); 
      H5::Group *getPlotGroup() { return plotGroup; }
      PlotFeatureStatus getPlotFeature(PlotFeature fp) { return Element::getPlotFeature(fp); };
      PlotFeatureStatus getPlotFeatureForChildren(PlotFeature fp) { return Element::getPlotFeatureForChildren(fp); };
      /*******************************************************/ 

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt = 1); 
      virtual void initPlot();
      virtual void closePlot();
      virtual std::string getType() const {return "Object";}
      void setDynamicSystemSolver(DynamicSystemSolver *sys);
      void setFullName(const std::string &str);
      void load(const std::string &path, std::ifstream &inputfile);
      void save(const std::string &path, std::ofstream &outputfile);
      /*******************************************************/ 

      /* INTERFACE */
      virtual void writeq(); // TODO the following: only testing
      virtual void readq0();
      virtual void writeu();
      virtual void readu0();
      virtual void writex();
      virtual void readx0();

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
      virtual void updatehRef(const fmatvec::Vec& ref, int i=0);

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
      virtual void init();

      /**
       * \brief initialize object at start of simulation with respect to contours and frames TODO
       */
      virtual void preinit();

      /**
       * initialize state of object at start of simulation
       */
      virtual void initz();

      /**
       * \brief perform Cholesky decomposition of mass martix
       */
      virtual void facLLM();

      /**
       * \brief calculates size of right hand side
       * \param index of normal usage and inverse kinetics TODO
       */
      virtual void calchSize(int j) {}

      /**
       * \return kinetic energy 
       */
      virtual double computeKineticEnergy();

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
       * \param contour to add
       */
      virtual void addContour(Contour* contour);

      /**
       * \param frame to add
       */
      virtual void addFrame(Frame * port);

      /**
       * \param name of the contour
       * \param flag for checking existence
       * \return contour
       */
      virtual Contour* getContour(const std::string &name, bool check=true);

      /**
       * \param name of the frame
       * \param flag for checking existence
       * \return frame
       */
      virtual Frame* getFrame(const std::string &name, bool check=true);
      /*****************************************************/

      /* GETTER / SETTER */
      DynamicSystem* getParent() { return parent; }
      void setParent(DynamicSystem* sys) { parent = sys; }

      void setqSize(int qSize_) { qSize = qSize_; }
      void setuSize(int uSize_, int i=0) { uSize[i] = uSize_; }
      int getzSize() const { return qSize + uSize[0]; }

      void sethInd(int hInd_, int i=0); 
      int getqInd() { return qInd; }
      int getuInd(int i=0) { return uInd[i]; }
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

      const fmatvec::Vec& getq() const { return q; };
      const fmatvec::Vec& getu() const { return u; };
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

      void setq0(const fmatvec::Vec &q0_) { q0 = q0_; }
      void setu0(const fmatvec::Vec &u0_) { u0 = u0_; }
      void setq0(double q0_) { q0 = fmatvec::Vec(1,fmatvec::INIT,q0_); }
      void setu0(double u0_) { u0 = fmatvec::Vec(1,fmatvec::INIT,u0_); }

      const std::vector<Frame*>& getFrames() const { return port; }
      const std::vector<Contour*>& getContours() const { return contour; }
      /*****************************************************/

      /**
       * \param frame
       * \return index of frame TODO rename
       */
      int portIndex(const Frame *port_) const;

      /**
       * \param contour
       * \return index of contour TODO rename
       */
      int contourIndex(const Contour *contour_) const;

      /** 
       * Return the full path of the object. This function replaces the getFullName
       * which is deprecated.
       * \param pathDelim The delimiter of the path
       */
      std::string getPath(char pathDelim='.') { return parent?parent->getPath()+pathDelim+name:name; }

#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Group* getAMVisGrp() { return amvisGrp; }
#endif

    protected:
      /**
       * \brief dynamic system, object belongs to
       */
      DynamicSystem* parent;

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
       * \brief smooth and nonsmooth right hand side
       */
      fmatvec::Vec h, r;

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
       * \brief vector of frames and contours
       */
      std::vector<Frame*> port;
      std::vector<Contour*> contour;

#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Body* amvisBody;
      AMVis::Group* amvisGrp;
#endif
  };

}

#endif

