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
#ifdef HAVE_AMVISCPPINTERFACE
#include <amviscppinterface/body.h>
#endif

using namespace std;

namespace MBSim {

  class Frame;
  class Contour;
  class Subsystem;

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
      Object(const string &name);

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
      int gethInd(Subsystem* sys,int i=0); 
      H5::Group *getPlotGroup() { return plotGroup; }
      PlotFeatureStatus getPlotFeature(PlotFeature fp) { return Element::getPlotFeature(fp); };
      PlotFeatureStatus getPlotFeatureForChildren(PlotFeature fp) { return Element::getPlotFeatureForChildren(fp); };
      /*******************************************************/ 

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt = 1); 
      virtual void initPlot();
      virtual void closePlot();
      virtual string getType() const {return "Object";}
      void setMultiBodySystem(MultiBodySystem *sys);
      void setFullName(const string &str);
      void load(const string &path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);
      /*******************************************************/ 

      /* INTERFACE */
      virtual void writeq(); // TODO the following: only testing
      virtual void readq0();
      virtual void writeu();
      virtual void readu0();
      virtual void writex();
      virtual void readx0();

      /**
       * \brief references to positions of subsystem parent
       * \param vector to be referenced
       */
      virtual void updateqRef(const Vec& ref);

      /**
       * \brief references to differentiated positions of subsystem parent
       * \param vector to be referenced
       */
      virtual void updateqdRef(const Vec& ref);

      /**
       * \brief references to velocities of subsystem parent
       * \param vector to be referenced
       */
      virtual void updateuRef(const Vec& ref);

      /**
       * \brief references to differentiated velocities of subsystem parent
       * \param vector to be referenced
       */
      virtual void updateudRef(const Vec& ref);

      /**
       * \brief references to smooth force vector of subsystem parent
       * \param vector to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updatehRef(const Vec& ref, int i=0);

      /**
       * \brief references to nonsmooth force vector of subsystem parent
       * \param vector to be referenced
       */
      virtual void updaterRef(const Vec& ref);

      /**
       * \brief references to linear transformation matrix between differentiated positions and velocities of subsystem parent
       * \param matrix to be referenced
       */
      virtual void updateTRef(const Mat &ref);

      /**
       * \brief references to mass matrix of subsystem parent
       * \param vector to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updateMRef(const SymMat &ref, int i=0);

      /**
       * \brief references to Cholesky decomposition of subsystem parent
       * \param vector to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updateLLMRef(const SymMat &ref, int i=0);


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
      virtual Contour* getContour(const string &name, bool check=true);

      /**
       * \param name of the frame
       * \param flag for checking existence
       * \return frame
       */
      virtual Frame* getFrame(const string &name, bool check=true);
      /*****************************************************/

      /* GETTER / SETTER */
      Subsystem* getParent() { return parent; }
      void setParent(Subsystem* sys) { parent = sys; }

      void setqSize(int qSize_) { qSize = qSize_; }
      void setuSize(int uSize_, int i=0) { uSize[i] = uSize_; }
      int getzSize() const { return qSize + uSize[0]; }

      void sethInd(int hInd_, int i=0); 
      int getqInd() { return qInd; }
      int getuInd(int i=0) { return uInd[i]; }
      int gethInd(int i=0) { return hInd[i]; }

      const Index& getuIndex() const { return Iu;}
      const Index& gethIndex() const { return Ih;}

      const Vec& geth() const { return h; };
      Vec& geth() { return h; };
      const Vec& getr() const { return r; };
      Vec& getr() { return r; };
      const SymMat& getM() const { return M; };
      SymMat& getM() { return M; };
      const Mat& getT() const { return T; };
      Mat& getT() { return T; };
      const SymMat& getLLM() const { return LLM; };
      SymMat& getLLM() { return LLM; };

      const Vec& getq() const { return q; };
      const Vec& getu() const { return u; };
      Vec& getq() { return q; };
      Vec& getu() { return u; };

      const Vec& getq0() const { return q0; };
      const Vec& getu0() const { return u0; };
      Vec& getq0() { return q0; };
      Vec& getu0() { return u0; };

      const Vec& getqd() const { return qd; };
      const Vec& getud() const { return ud; };
      Vec& getqd() { return qd; };
      Vec& getud() { return ud; };

      void setq(const Vec &q_) { q = q_; }
      void setu(const Vec &u_) { u = u_; }

      void setq0(const Vec &q0_) { q0 = q0_; }
      void setu0(const Vec &u0_) { u0 = u0_; }
      void setq0(double q0_) { q0 = Vec(1,INIT,q0_); }
      void setu0(double u0_) { u0 = Vec(1,INIT,u0_); }

      const vector<Frame*>& getFrames() const { return port; }
      const vector<Contour*>& getContours() const { return contour; }
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

#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Group* getAMVisGrp() { return amvisGrp; }
#endif

    protected:
      /**
       * \brief subsystem, object belongs to
       */
      Subsystem* parent;

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
      Vec q, u;

      /**
       * \brief initial position, velocity
       */
      Vec q0,u0;

      /**
       * \brief differentiated positions, velocities
       */
      Vec qd,ud;

      /** 
       * \brief smooth and nonsmooth right hand side
       */
      Vec h, r;

      /** 
       * \brief linear relation matrix of differentiated position and velocity parameters
       */
      Mat T;

      /** 
       * \brief mass matrix 
       */
      SymMat M;

      /**
       * \brief LU-decomposition of mass matrix 
       */
      SymMat LLM;

      /**
       * \brief indices for velocities and right hand side
       */
      Index Iu, Ih;

      /**
       * \brief vector of frames and contours
       */
      vector<Frame*> port;
      vector<Contour*> contour;

#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Body* amvisBody;
      AMVis::Group* amvisGrp;
#endif
  };

}

#endif

