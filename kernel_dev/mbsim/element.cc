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

#include <config.h>
#include <mbsim/element.h>
#include <mbsim/interfaces.h>

namespace MBSim {

  Element::Element(const string &name_) : name(name_) {
    for(int i=0; i<LASTPLOTFEATURE; i++) {
      plotFeature[(PlotFeature)i]=unset;
      plotFeatureForChildren[(PlotFeature)i]=unset;
    }
  }

  Element::~Element() {
  }

  void Element::load(const string &path, ifstream& inputfile) {
    string dummy;
    getline(inputfile,dummy); // # Type
    getline(inputfile,dummy); // Type
    getline(inputfile,dummy); // newline
    getline(inputfile,dummy); // # Name
    getline(inputfile,dummy); // Name
    name = dummy;
    getline(inputfile,dummy); // newline
    getline(inputfile,dummy); // # Full name
    getline(inputfile,dummy); // full name
    fullName = dummy;
    getline(inputfile,dummy); // newline
  }

  void Element::save(const string &path, ofstream& outputfile) {
    outputfile << "# Type:" << endl;
    outputfile << getType() << endl<<endl;
    outputfile << "# Name:" << endl;
    outputfile << name << endl<<endl;
    outputfile << "# Full name:" << endl;
    outputfile << getFullName() << endl<<endl;
  }

  void Element::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_front(t);

      if(plotColumns.size()>1) {
        assert(plotColumns.size()==plotVector.size());
        plotVectorSerie->append(plotVector);
        plotVector.clear();
      }
    }
  }

  void Element::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(plotVectorSerie) delete plotVectorSerie;
      delete plotGroup;
    }
  }

  int Element::getNumberOfElements(ifstream &inputfile) {
    string dummy;
    int num=0;
    int n = inputfile.tellg();
    do {
      num++;
      getline(inputfile,dummy);
    } while(!dummy.empty());
    num--;
    inputfile.seekg(n);
    return num;
  }

  void Element::addDataInterfaceBaseRef(const string& DIBRef_){
    DIBRefs.push_back(DIBRef_);
  }

  void Element::initPlot(ObjectInterface* parent) {
    updatePlotFeatures(parent);

    if(getPlotFeature(plotRecursive)==enabled) {
      plotGroup=new H5::Group(parent->getPlotGroup()->createGroup(name));
      H5::SimpleAttribute<string>::setData(*plotGroup, "Description", "Object of class: "+getType());

      plotColumns.push_front("Time");
      if(plotColumns.size()>1) {
        plotVectorSerie=new H5::VectorSerie<double>;
        // copy plotColumns to a std::vector
        vector<string> dummy; copy(plotColumns.begin(), plotColumns.end(), insert_iterator<vector<string> >(dummy, dummy.begin()));
        plotVectorSerie->create(*plotGroup,"data",dummy);
        plotVectorSerie->setDescription("Default dataset for class: "+getType());
      }
    }
  }

  void Element::updatePlotFeatures(ObjectInterface* parent) {
    for(int i=0; i<LASTPLOTFEATURE; i++) {
      if(getPlotFeature((PlotFeature)i)==unset) setPlotFeature((PlotFeature)i, parent->getPlotFeatureForChildren((PlotFeature)i));
      if(getPlotFeatureForChildren((PlotFeature)i)==unset) setPlotFeatureForChildren((PlotFeature)i, parent->getPlotFeatureForChildren((PlotFeature)i));
    }
  }

}

