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

  enum PlotFeature {
    plotRecursive=0, separateFilePerDynamicSystem, state, stateDerivative, rightHandSide, globalPosition, contact, energy, amvis, LASTPLOTFEATURE
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
       * \param element fullname DEPRECATED
       */
      virtual void setFullName(const std::string &str) { fullName = str; }

      /**
       * \brief load topology
       * \param path TODO
       * \param inputfile
       */
      virtual void load(const std::string &path, std::ifstream &inputfile);

      /**
       * \brief save topology
       * \param path TODO
       * \param outputfile
       */
      virtual void save(const std::string &path, std::ofstream &outputfile);

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
       * \param order of plot invocations
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
       * \return element fullname
       */
      const std::string& getFullName() const { return fullName; }

      /** 
       * \param input file
       * \return number of elements in topology of input file
       */
      static int getNumberOfElements(std::ifstream &inputfile);

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

    protected:
      /** 
       * \brief name of element 
       */
      std::string name;

      /** 
       * \brief fullname of element
       */
      std::string fullName;

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
