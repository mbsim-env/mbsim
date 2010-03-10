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
 *          rzander@users.berlios.de
 */

#ifndef _ELEMENT_H_
#define _ELEMENT_H_

#include "fmatvec.h"
#include <string>
#include <vector>
#include <hdf5serie/vectorserie.h>
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"

#define MBSIMNS "{http://mbsim.berlios.de/MBSim}"

#ifdef NO_ISO_14882
#include<fstream.h>
#else 
#include<fstream>
#endif

namespace H5 {
  class Group;
}

/**
 * \brief namespace MBSim
 */
namespace MBSim {

  class ObjectInterface;

  /** \brief Plot feature status */
  enum PlotFeatureStatus {
    enabled, /*!< set the feature enabled */
    disabled, /*!< set the feature disabled */
    unset /*!< use the default value for the feature */
  };

  // NOTE!!! When adding a new PlotFeature here, the default setting for this feature must
  // be specified in dynamic_system_solver.cc:DynamicSystemSolver::constructor() and the
  // new feature must also be added in
  // element.cc:Element::initializeUsingXML(TiXmlElement *element)
  // and in
  // mbsimxml/schema/mbsim.xsd.in
  /** \brief Plot Features */
  enum PlotFeature {
    plotRecursive=0, /*!< enables/disables all plotting beyond this hierarchy */
    separateFilePerGroup, /*!< create a separate h5 file for the Group */
    state, /*!< plot the state vector (x, q, u) */
    stateDerivative, /*!< plot the derivative of the state vector (xd, qd, ud) */
    rightHandSide, /*!< plot the smooth and non smooth right hand size (h, r=W*l) */
    globalPosition, /*!< plot some global (world) positions/orientation */
    globalVelocity, /*!< plot some global (world) velocities */
    energy, /*!< plot the energy */
    openMBV, /*!< plot the OpenMBV part */
    generalizedLinkForce, /*!< plot the smooth/non smooth generalized link force (l) */
    linkKinematics, /*!< plot the kinematic of links (g, gd) */
    stopVector, /*!< plot the stop vector (sv) */
    LASTPLOTFEATURE
  };

  /** \brief The stages of the initialization
   *
   * see also DynamicSystemSolver::init()
   */
  enum InitStage {
    modelBuildup=0, /*< build up complex internal models */
    resolveXMLPath, /*!< resolve the path (given by the XML ref* attributes) to the corrosponding pointer */
    preInit, /*!< Make some early initialization. TODO This should be split into detailed stages. */
    resize, /*!< Do the resizing of all vectors, matrices and containers, ... */
    plot, /*!< Build/initialize the plot structure */
    relativeFrameContourLocation, /*!< Set the relative position and orientation of all Frame's/Contour's in all Group's/RigidBody's */
    worldFrameContourLocation, /*!< Set the world position and orientation of all Frame's/Contour's in all Group's */
    reorganizeHierarchy, /*!< Reorganize the hierarchy (build invisible tree structur) */
    unknownStage, /*!< Init all the rest. TODO This should be split into detailed stages. */
    calculateLocalInitialValues, /*!< calculation of non-linear initial values in complex internal models */
    LASTINITSTAGE
  };

  class DynamicSystemSolver;

  /** 
   * \brief basic class of MBSim mainly for plotting
   * \author Martin Foerg
   * \date 2009-03-24 plot feature energy (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   */
  class Element {
    public:
      /**
       * \brief constructor
       */	
      Element(const std::string &name);

      /** 
       * \brief destructor
       */
      virtual ~Element();

      /* INTERFACE */
      /**
       * \brief TODO
       */
      virtual void initDataInterfaceBase(DynamicSystemSolver *parentds) {};

      /**
       * \return std::string representation
       */
      virtual std::string getType() const { return "Element"; }

      /**
       * \param TODO
       */
      virtual void setDynamicSystemSolver(DynamicSystemSolver *sys) { ds = sys; }

      /**
       * \brief plots time dependent data
       * \param simulation time
       * \param simulation time step size for derivative calculation
       */
      virtual void plot(double t, double dt = 1);

      /**
       * \brief closes plot file
       */
      virtual void closePlot();
      /***************************************************/

      /**
       * \return element name
       */
      const std::string& getName() const { return name; }

      /**
       * \param element name
       */
      void setName(const std::string &str) { name = str; }

      /**
       * \return dynamic system solver
       */
      DynamicSystemSolver* getDynamicSystemSolver() { return ds; }

      /**
       * \brief plots time series header
       * \param invocing parent class
       */
      void init(InitStage stage, ObjectInterface* parent); 

      /**
       * \return associated plot group
       */
      H5::Group *getPlotGroup() { return plotGroup; }

      /**
       * \brief Set a plot feature
       * 
       * Set the plot feature pf of this object to enabled, disabled or unset.
       * If unset, this object uses the value of the plot feature pf of its parent object.
       */
      void setPlotFeature(PlotFeature pf, PlotFeatureStatus value) { plotFeature[pf]=value; }

      /**
       * \brief Set a plot feature for the children of this object
       * 
       * Set the plot feature pf of all children which plot feature is unset to enabled, disabled or unset.
       */
      void setPlotFeatureForChildren(PlotFeature pf, PlotFeatureStatus value) { plotFeatureForChildren[pf]=value; }

      /**
       * \brief Set a plot feature for this object and the children of this object.
       * 
       * This is a convenience function. It simply calls setPlotFeature and setPlotFeatureForChildren.
       */
      void setPlotFeatureRecursive(PlotFeature pf, PlotFeatureStatus value) { plotFeature[pf]=value; plotFeatureForChildren[pf]=value; }

      /**
       * \return plot feature
       */
      PlotFeatureStatus getPlotFeature(PlotFeature pf) { return plotFeature[pf]; }

      /**
       * \return plot feature for derived classes
       */
      PlotFeatureStatus getPlotFeatureForChildren(PlotFeature pf) { return plotFeatureForChildren[pf]; }

      virtual void initializeUsingXML(TiXmlElement *element);

      /**
       * \brief a general element access
       */
      template<class T> 
        T* getByPath(std::string path) {
          Element * e = getByPathSearch(path);
          if (dynamic_cast<T*>(e))
            return (T*)(e);
          else {
            std::cout << "Element \"" << path << "\" not found or not of wanted type. Sorry." << std::endl;
            throw(123);
          }
        }

      /**
       * \brief a general element search by path
       */
      virtual Element* getByPathSearch(std::string path) {return 0; }
      
      // some convenience function for XML
      static double getDouble(TiXmlElement *e);
      static int getInt(TiXmlElement *e);
      static bool getBool(TiXmlElement *e);
      static fmatvec::Vec getVec(TiXmlElement *e, int rows=0);
      static fmatvec::Mat getMat(TiXmlElement *e, int rows=0, int cols=0);
      static fmatvec::SqrMat getSqrMat(TiXmlElement *e, int size=0);
      static fmatvec::SymMat getSymMat(TiXmlElement *e, int size=0);

    protected:
      /** 
       * \brief name of element 
       */
      std::string name;

      /**
       * \brief dynamic system
       */
      DynamicSystemSolver *ds;

      /**
       * \brief time series
       */
      H5::VectorSerie<double> *plotVectorSerie;

      /**
       * \brief one entry of time series
       */
      std::vector<double> plotVector;

      /**
       * \brief columns of time series
       */
      std::vector<std::string> plotColumns;

      /**
       * \brief associated plot group
       */
      H5::Group *plotGroup;

      void updatePlotFeatures(ObjectInterface* parent);

    private:
      /**
       * \brief plot feature
       */
      PlotFeatureStatus plotFeature[LASTPLOTFEATURE], plotFeatureForChildren[LASTPLOTFEATURE];
  };

}

#endif
