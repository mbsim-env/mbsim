/* Copyright (C) 2004-2006  Martin Förg, Roland Zander
 
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
#include <config.h>
#include "element.h"

namespace MBSim {

  string Element::dirName = "./";


  Element::Element(const string &name_) : name(name_), plotNr(1), plotLevel(1), plotPrec(6) {
  }

  Element::~Element() {
    plotfile.close();
    parafile.close();
  }

  void Element::plot(double t, double dt) {
    if(plotLevel) {
      plotfile << endl;
      plotfile <<showpos<<setw(16)<< t;
      plotfile.precision(plotPrec);
    }
  }

  void Element::setPlotPrecision(int prec) {
   plotPrec = prec;
   if(plotfile.is_open()) plotfile.precision(plotPrec);
  }

  void Element::initPlotFiles() {
    // generate plotfile for time history
    if(plotLevel) {
      plotfile.open((dirName+getFullName()+".plt").c_str(), ios::out);
      plotfile <<"# " << plotNr++ << ": t" << endl;
    }
    parafile.open((dirName+getFullName()+".para").c_str(), ios::out);
  } 

  void Element::closePlotFiles() {
    if(plotLevel) {
      plotfile.close();
    }
    parafile.close();
  } 

  void Element::plotParameters() {
      parafile << "# Type:" << endl;
      parafile << getType() << endl;
      parafile << "# Name:" << endl;
      parafile << name << endl;
      parafile << "# Full name:" << endl;
      parafile << getFullName() << endl;
  }

  void Element::load(ifstream& inputfile) {
    //ifstream inputfile(model.c_str(), ios::in);
    char dummy[10000];
    inputfile.getline(dummy,10000); // # Type
    inputfile.getline(dummy,10000); // Type
    inputfile.getline(dummy,10000); // # Name
    inputfile.getline(dummy,10000); // Name
    name = dummy;
    cout << "Element name = " << name << endl;
    inputfile.getline(dummy,10000); // # Full name
    inputfile.getline(dummy,10000); // full name
 }

  void Element::addDataInterfaceBaseRef(const string& DIBRef_){
    DIBRefs.push_back(DIBRef_);
  }

  int Element::getNumberOfElements(ifstream &inputfile) {
    char dummy[10000];
    int num=0;
    int n = inputfile.tellg();
    do {
      num++;
      inputfile.getline(dummy,10000); // Object
    } while(dummy[0]!='#');
    num--;
    inputfile.seekg(n);
    return num;
  }
}
