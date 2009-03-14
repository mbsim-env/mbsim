/* Copyright (C) 2004-2006  Martin Förg, Roland Zander, Felix Kahr
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *   rzander@users.berlios.de
 *
 */

#ifndef _ELEMENT_H_
#define _ELEMENT_H_

#include "fmatvec.h"
#include<string>
#include<vector>
#include<hdf5serie/vectorserie.h>

#ifdef NO_ISO_14882
#include<fstream.h>
#else 
#include<fstream>
#endif

using namespace std;
using namespace fmatvec;

namespace H5 {
  class Group;
}

/*! \brief Namespace MBSim */
namespace MBSim {

  class ObjectInterface;

  enum PlotFeatureStatus {
    enabled, disabled, unset
  };

  enum PlotFeature {
    plotRecursive=0, separateFilePerSubsystem, state, stateDerivative, rightHandSide, globalPosition, contact, amvis, LASTPLOTFEATURE
  };

  class MultiBodySystem;
  /*! 
   * \brief THE basic class of MBSim
   *
   * Used for definitions of general element parameters and general interactions
   */
  class Element {
    private:
      PlotFeatureStatus plotFeature[LASTPLOTFEATURE], plotFeatureForChildren[LASTPLOTFEATURE];

    protected:
      /* Short name of Element */
      string name;
      /* Short fullname of Element */
      string fullName;
      /* Vector for Data Interface Base References */
      vector<string> DIBRefs;

      MultiBodySystem *mbs;

      H5::VectorSerie<double> *plotVectorSerie;
      std::vector<double> plotVector;
      std::vector<std::string> plotColumns;
      H5::Group *plotGroup;

    public:
	  /*! Constructor */	
      Element(const string &name);
      /*! Destructor */
      virtual ~Element();
	  /*! Get element short \return name */
      const string& getName() const { return name; }
      /*! Set element name \param str */
      void setName(const string &str) {name = str;}
      /*! Get element \return fullname */
      const string& getFullName() const {return fullName;}
      /*! Set element name \param str */
      virtual void setFullName(const string &str) {fullName = str;}

      virtual void load(const string &path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
 
      void addDataInterfaceBaseRef(const string& DIBRef_);
      virtual void initDataInterfaceBase(MultiBodySystem *parentmbs) {};

      virtual string getType() const {return "Element";}

      static int getNumberOfElements(ifstream &inputfile);

      MultiBodySystem* getMultiBodySystem() {return mbs;}
      virtual void setMultiBodySystem(MultiBodySystem *sys) {mbs = sys;}

      virtual void plot(double t, double dt = 1, bool top=true);
      void initPlot(ObjectInterface* parent, bool createDefault, bool top=true); // Element has no access to a parent, so give parent as parameter
      void createDefaultPlot();
      virtual void closePlot();
      H5::Group *getPlotGroup() { return plotGroup; }

      /*! Set a plot feature.
       * Set the plot feature pf of this object to enabled, disabled or unset.
       * If unset, this object uses the value of the plot feature pf of its parent object.
       */
      void setPlotFeature(PlotFeature pf, PlotFeatureStatus value) { plotFeature[pf]=value; }

      /*! Set a plot feature for the children of this object.
       * Set the plot feature pf of all children which plot feature is unset to enabled, disabled or unset.
       */
      void setPlotFeatureForChildren(PlotFeature pf, PlotFeatureStatus value) { plotFeatureForChildren[pf]=value; }

      /*! Set a plot feature for this object and the children of this object.
       * This is a convenience function. It simply calls setPlotFeature and setPlotFeatureForChildren.
       */
      void setPlotFeatureRecursive(PlotFeature pf, PlotFeatureStatus value) { plotFeature[pf]=value; plotFeatureForChildren[pf]=value; }
      PlotFeatureStatus getPlotFeature(PlotFeature pf) { return plotFeature[pf]; }
      PlotFeatureStatus getPlotFeatureForChildren(PlotFeature pf) { return plotFeatureForChildren[pf]; }
  };

}

#endif
