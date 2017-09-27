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
#include <string>
#include <unordered_map>
#include <hdf5serie/vectorserie.h>
#include <mbxmlutilshelper/dom.h>

#include "mbsim_event.h"

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

#ifndef SWIG
  const MBXMLUtils::NamespaceURI MBSIM("http://www.mbsim-env.de/MBSim");
#endif

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
      static std::size_t plotRecursive, openMBV, debug, separateFilePerGroup, energy;

      /** \brief Plot feature status */
      enum PlotFeatureStatus {
        unset, /*!< use the default value for the feature */
        enabled, /*!< set the feature enabled */
        disabled /*!< set the feature disabled */
      };

      /** \brief The stages of the initialization
       *
       * see also DynamicSystemSolver::init()
       */
      enum InitStage {
        resolveXMLPath=0, /*!< resolve the path (given by the XML ref* attributes) to the corrosponding pointer */
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
      virtual ~Element() { }

      /**
       * \return std::string representation
       */
      virtual std::string getType() const { return "Element"; }

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
      virtual void init(InitStage stage);

      /**
       * \brief creates the plotGroup for H5-output
       */
      virtual void createPlotGroup();

      /**
       * \return associated plot group
       */
      H5::GroupBase *getPlotGroup() { return plotGroup; }

      PlotFeatureStatus getPlotFeature(std::size_t pf) { return plotFeature[pf]; }

      /**
       * \brief Set a plot feature
       *
       * Set the plot feature pf of this object to enabled, disabled or unset.
       * If unset, this object uses the value of the plot feature pf of its parent object.
       */
      virtual void setPlotFeature(const std::string &pf, PlotFeatureStatus value);

      /**
       * \brief Set a plot feature for the children of this object
       *
       * Set the plot feature pf of all children which plot feature is unset to enabled, disabled or unset.
       */
      void setPlotFeatureForChildren(const std::string &pf, PlotFeatureStatus value);

      /**
       * \brief Set a plot feature for this object and the children of this object.
       *
       * This is a convenience function. It simply calls setPlotFeature and setPlotFeatureForChildren.
       */
      void setPlotFeatureRecursive(const std::string &pf, PlotFeatureStatus value) { setPlotFeature(pf,value); setPlotFeatureForChildren(pf,value); }

      PlotFeatureStatus initializePlotFeatureStatusUsingXML(xercesc::DOMElement *e);
      virtual void initializeUsingXML(xercesc::DOMElement *element);

      const std::vector<MBXMLUtils::EmbedDOMLocator>& getLocationStack() const { return locationStack; }

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
      std::string getPath(const Element *relTo=NULL, std::string sep="/") const;

      /**
       * \brief Get the Element named name in the container named container.
       */
      virtual Element* getChildByContainerAndName(const std::string &container, const std::string &name) const {
        THROW_MBSIMERROR("This element has no containers with childs.");
      }

      // some convenience function for XML
      static double getDouble(xercesc::DOMElement *e);
      static int getInt(xercesc::DOMElement *e);
      static bool getBool(xercesc::DOMElement *e);
      static fmatvec::Vec3 getVec3(xercesc::DOMElement *e);
      static fmatvec::Vec getVec(xercesc::DOMElement *e, int rows=0);
      static fmatvec::Mat3xV getMat3xV(xercesc::DOMElement *e, int cols=0);
      static fmatvec::Mat getMat(xercesc::DOMElement *e, int rows=0, int cols=0);
      static fmatvec::SqrMat3 getSqrMat3(xercesc::DOMElement *e);
      static fmatvec::SqrMat getSqrMat(xercesc::DOMElement *e, int size=0);
      static fmatvec::SymMat3 getSymMat3(xercesc::DOMElement *e);
      static fmatvec::SymMat getSymMat(xercesc::DOMElement *e, int size=0);

      virtual std::shared_ptr<OpenMBV::Group> getOpenMBVGrp() {return std::shared_ptr<OpenMBV::Group>();}

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

    protected:
      Element *parent;

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

      std::vector<MBXMLUtils::EmbedDOMLocator> locationStack;

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
      H5::GroupBase *plotGroup;

      void updatePlotFeatures();

      /**
       * \brief vector containing all dependencies.
       */
      std::vector<Element*> dependency;

      /**
       * \brief plot feature
       */
      std::unordered_map<std::size_t,PlotFeatureStatus> plotFeature, plotFeatureForChildren;
  };

  template<class T>
  T* Element::getByPath(const std::string &path, bool initialCaller) const {
    try {
      if(path.substr(0, 1) == "/") { // if absolute path ...
        if(parent) // .. and a parent exists ...
          return parent->getByPath<T>(path, false); // ... than call getByPath of the parent (walk to the top)
        else // .. and no parent exits ...
          return getByPath<T>(path.substr(1), false); // ... we are at top and call getByPath again with the leading "/" removed (call relative to top)
      }
      else if (path.substr(0, 3) == "../") { // if relative path to parent ...
        if(!parent)
          THROW_MBSIMERROR("Reference to parent '"+path+"' requested but already at the root node.");
        return parent->getByPath<T>(path.substr(3), false); // ... call getByPath of the parent with the leading "../" removed
      }
      else { // if relative path to a child ...
        // extract the first path and all other paths (rest)
        size_t idx=path.find('/');
        std::string first=path.substr(0, idx);
        std::string rest;
        if(idx!=std::string::npos)
          rest=path.substr(idx+1);
        // get the object of the first child path by calling the virtual function getChildByContainerAndName
        size_t pos0=first.find('[');
        if(pos0==std::string::npos)
          THROW_MBSIMERROR("Syntax error in subreference '"+first+"': no [ found.");
        std::string container=first.substr(0, pos0);
        if(first[first.size()-1]!=']')
          THROW_MBSIMERROR("Syntax error in subreference '"+first+"': not ending with ].");
        std::string name=first.substr(pos0+1, first.size()-pos0-2);
        Element *e=getChildByContainerAndName(container, name);
        // if their are other child paths call getByPath of e for this
        if(!rest.empty())
          return e->getByPath<T>(rest, false);
        // this is the last relative path -> check type and return
        T *t=dynamic_cast<T*>(e);
        if(t)
          return t;
        else
          THROW_MBSIMERROR("Cannot cast this element to type "+container+".");
      }
    }
    catch(MBSimError &ex) {
      if(initialCaller)
        THROW_MBSIMERROR("Evaluation of refernece '"+path+"' failed: Message from "+
          ex.getPath()+": "+ex.getErrorMessage());
      else
        throw ex;
    }
  }

}

#endif
