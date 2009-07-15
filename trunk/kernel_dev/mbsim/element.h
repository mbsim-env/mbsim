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

  enum PlotFeatureStatus {
    enabled, disabled, unset
  };

  // NOTE!!! When adding a new PlotFeature here, the default setting for this feature must
  // be specified in dynamic_system_solver.cc:DynamicSystemSolver::constructor() and the
  // new feature must also be added in
  // element.cc:Element::initializeUsingXML(TiXmlElement *element)
  // and in
  // mbsimxml/schema/mbsim.xsd.in
  enum PlotFeature {
    plotRecursive=0, separateFilePerDynamicSystem, state, stateDerivative, rightHandSide, globalPosition, contact, energy, openMBV, LASTPLOTFEATURE
  };

  class DynamicSystemSolver;

  /** 
   * \brief basic class of MBSim mainly for plotting
   * \author Martin Foerg
   * \date 2009-03-24 plot feature energy (Thorsten Schindler)
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
       * \return dynamic system
       */
      DynamicSystemSolver* getDynamicSystemSolver() { return ds; }

      /**
       * \param name of data interface base
       */
      void addDataInterfaceBaseRef(const std::string& DIBRef_);

      /**
       * \brief plots time series header
       * \param invocing parent class
       */
      void initPlot(ObjectInterface* parent); 

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

    protected:
      /** 
       * \brief name of element 
       */
      std::string name;

      /** 
       * \brief vector for data interface base references
       */
      std::vector<std::string> DIBRefs;

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
