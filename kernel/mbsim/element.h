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

#ifndef _ELEMENT_H_
#define _ELEMENT_H_

#include "fmatvec/fmatvec.h"
#include "fmatvec/atom.h"
#include "mbsim/objectfactory.h"
#include "mbsim/utils/plotfeatureenum.h"
#include "mbsim/utils/initconfigenum.h"
#include "mbsim/namespace.h"
#include "mbsim/mbsim_event.h"
#include <hdf5serie/vectorserie.h>
#include <boost/core/demangle.hpp>

namespace OpenMBV {
  class Group;
}

namespace H5 {
  class Group;
}

/**
 * \brief namespace MBSim
 */
namespace MBSim {

  extern const PlotFeatureEnum plotRecursive, openMBV, debug;

  class DynamicSystemSolver;
  class Frame;

  /**
   * \brief basic class of MBSim mainly for plotting
   * \author Martin Foerg
   * \date 2009-03-24 plot feature energy (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   */
  class Element : virtual public fmatvec::Atom {
    public:
      /** \brief The stages of the initialization
       *
       * see also DynamicSystemSolver::init()
       */
      enum InitStage {
        resolveStringRef=0, /*!< resolve references given by a string path to the corrosponding pointer */
        preInit, /*!< Make some early initialization. TODO This should be split into detailed stages. */
        plotting, /*!< Build/initialize the plot structure */
        unknownStage, /*!< Init all the rest. TODO This should be split into detailed stages. */
        LASTINITSTAGE
      };

      /**
       * \brief constructor
       */
      Element(const std::string &name);

      /**
       * \brief destructor
       */
      ~Element() override = default;

#ifndef SWIG
      [[noreturn]]
#endif
      void throwError(const std::string &msg) const {
        throw MBSimError(this, msg);
      }

      /**
       * \brief sets the used dynamics system solver to the element
       * \param pointer to the dynamic system solver of which the element is part of
       */
      virtual void setDynamicSystemSolver(DynamicSystemSolver *sys) { ds = sys; }

      /**
       * \brief plots time dependent data
       * \param simulation time
       * \param simulation time step size for derivative calculation
       */
      virtual void plot();

      /**
       * \brief plots time dependent data at special events
       * \param simulation time
       * \param simulation time step size for derivative calculation
       */
      virtual void plotAtSpecialEvent() { }

      /**
       * \return element name
       */
      const std::string& getName() const { return name; }

      /**
       * \param element name
       */
      void setName(const std::string &str) { name = str; }

      // internal function do not use
      void setPath(const std::string &str) { path=str; }

      /**
       * \return dynamic system solver
       */
      DynamicSystemSolver* getDynamicSystemSolver() { return ds; }

      /**
       * \brief plots time series header
       * \param invocing parent class
       */
      virtual void init(InitStage stage, const InitConfigSet &config=InitConfigSet());

      /**
       * \brief creates the plotGroup for H5-output
       */
      virtual void createPlotGroup();

      /**
       * \return associated plot group
       */
      H5::GroupBase *getPlotGroup() { return plotGroup; }
      virtual H5::GroupBase *getFramesPlotGroup() { return nullptr; }
      virtual H5::GroupBase *getContoursPlotGroup() { return nullptr; }
      virtual H5::GroupBase *getGroupsPlotGroup() { return nullptr; }
      virtual H5::GroupBase *getObjectsPlotGroup() { return nullptr; }
      virtual H5::GroupBase *getLinksPlotGroup() { return nullptr; }
      virtual H5::GroupBase *getConstraintsPlotGroup() { return nullptr; }
      virtual H5::GroupBase *getObserversPlotGroup() { return nullptr; }

      /** Get the state of the plot feature pf.
       * Returns false if the plot feature pf is not set till now. */
      bool getPlotFeature(const PlotFeatureEnum &pf) {
        auto it=plotFeature.find(std::ref(pf));
        if(it==plotFeature.end())
          return false;
        return it->second;
      }

      /**
       * \brief Set a plot feature
       *
       * Set the plot feature pf of this object.
       */
      virtual void setPlotFeature(const PlotFeatureEnum &pf, bool value);

      /**
       * \brief Set a plot feature for the children of this object
       *
       * Set the plot feature pf of all children for set plot features.
       */
      void setPlotFeatureForChildren(const PlotFeatureEnum &pf, bool value);

      /**
       * \brief Set a plot feature for this object and the children of this object.
       *
       * This is a convenience function. It simply calls setPlotFeature and setPlotFeatureForChildren.
       */
      void setPlotFeatureRecursive(const PlotFeatureEnum &pf, bool value) { setPlotFeature(pf,value); setPlotFeatureForChildren(pf,value); }

      /**
       * \brief Set a plot attribute: static data attached as key/value pairs to the plot datasets/groups.
       */
      template<class T>
      void setPlotAttribute(const std::string &name, const T &value) {
        plotAttribute[name] = value;
      }
      void setPlotAttribute(const std::string &name) {
        plotAttribute[name] = std::monostate();
      }

      virtual void initializeUsingXML(xercesc::DOMElement *element);

      /**
       * \brief Get the object of type T represented by the path path.
       * Do not set any argurment other than path!
       */
      template<class T> T* getByPath(const std::string &path, bool initialCaller=true) const;

      /**
       * \brief Return the path of this object.
       * If relativeTo is not NULL return a relative path to relativeTo.
       * Do not set any argurment other than relTo and sep!
       */
      std::string getPath(const Element *relTo=nullptr, std::string sep="/") const;

      /**
       * \brief Get the Element named name in the container named container.
       */
      virtual Element* getChildByContainerAndName(const std::string &container, const std::string &name) const {
        throwError("This element has no containers with childs.");
      }

      virtual std::shared_ptr<OpenMBV::Group> getOpenMBVGrp() { return std::shared_ptr<OpenMBV::Group>(); }
      virtual std::shared_ptr<OpenMBV::Group> getFramesOpenMBVGrp() { return std::shared_ptr<OpenMBV::Group>(); }
      virtual std::shared_ptr<OpenMBV::Group> getContoursOpenMBVGrp() { return std::shared_ptr<OpenMBV::Group>(); }
      virtual std::shared_ptr<OpenMBV::Group> getGroupsOpenMBVGrp() { return std::shared_ptr<OpenMBV::Group>(); }
      virtual std::shared_ptr<OpenMBV::Group> getObjectsOpenMBVGrp() { return std::shared_ptr<OpenMBV::Group>(); }
      virtual std::shared_ptr<OpenMBV::Group> getLinksOpenMBVGrp() { return std::shared_ptr<OpenMBV::Group>(); }
      virtual std::shared_ptr<OpenMBV::Group> getConstraintsOpenMBVGrp() { return std::shared_ptr<OpenMBV::Group>(); }
      virtual std::shared_ptr<OpenMBV::Group> getObserversOpenMBVGrp() { return std::shared_ptr<OpenMBV::Group>(); }

      virtual Element* getParent() {return parent;}
      virtual const Element* getParent() const {return parent;}
      virtual void setParent(Element* parent_) {parent = parent_;}

      /**
       * \brief checks dependency on other elements.
       * \return a vector of elements the calling element depends on.
       */
      std::vector<Element*> getDependencies() const { return dependency; }

      void addDependency(Element* ele) { if(ele) dependency.push_back(ele); }

      /**
       * \brief computes the length of the pathes in the graph that represents
       * the dependencies between all elements.
       * \return the length of the longest path in the graph.
       */
      int computeLevel();

      virtual void updatePositions(Frame *frame) { }
      virtual void updateVelocities(Frame *frame) { }
      virtual void updateAccelerations(Frame *frame) { }
      virtual void updateJacobians(Frame *frame, int j=0) { }
      virtual void updateGyroscopicAccelerations(Frame *frame) { }

      virtual void resetUpToDate() { }

      const double& getTime() const;
      double getStepSize() const;

#ifndef SWIG
      const MBXMLUtils::DOMEvalException& getDOMEvalError() const { return domEvalError; };
#endif

    protected:
      Element *parent { nullptr };

      /**
       * \brief name of element
       */
      std::string name;

      /**
       * \brief The path of this object.
       *  Is set during the init stage reorganizeHierarchy. Before this the path is calculated
       *  dynamically using getPath() after this stage getPath just returns this value.
       */
      std::string path;

#ifndef SWIG
      //! Special XML helper variable.
      MBXMLUtils::DOMEvalException domEvalError;
#endif

      /**
       * \brief dynamic system
       */
      DynamicSystemSolver *ds { nullptr };

      /**
       * \brief time series
       */
      H5::VectorSerie<double> *plotVectorSerie { nullptr };

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
      H5::GroupBase *plotGroup { nullptr };

      void updatePlotFeatures();

      /**
       * \brief vector containing all dependencies.
       */
      std::vector<Element*> dependency;

      /**
       * \brief plot feature
       */
      PlotFeatureMap plotFeature, plotFeatureForChildren;

      std::map<std::string, std::variant<
        std::monostate,
        int,
        double,
        std::string,
        std::vector<int>,
        std::vector<double>,
        std::vector<std::vector<double>>
      >> plotAttribute;

    public: // addToPlot / plot is public to allow helper classes/functions to handle plots in a element
      void addToPlot(const std::string &name);
      void addToPlot(const std::string &name, int size);
      void addToPlot(const std::string &name, const std::vector<std::string> &iname);

      template<class AT> void plot(const AT &x) {
	plotVector.push_back(x);
      }
      template<class Type, class AT> void plot(const fmatvec::Vector<Type,AT> &x) {
	for(int i=0; i<x.size(); i++)
	  plotVector.push_back(x(i));
      }
    private:
      Element* getByPathElement(const std::string &path, bool initialCaller=true) const;
  };

  template<class T>
  T* Element::getByPath(const std::string &path, bool initialCaller) const {
    Element *e = getByPathElement(path, initialCaller);
    auto *t=dynamic_cast<T*>(e);
    if(t)
      return t;
    else
       throwError(std::string("Cannot cast this element to type ")+boost::core::demangle(typeid(T).name())+".");
  }

}

#endif
