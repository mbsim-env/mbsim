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
  } 

  void Element::closePlotFiles() {
    if(plotLevel) {
      plotfile.close();
    }
  } 

  void Element::save(const string &path, ofstream& outputfile) {
      outputfile << "# Type:" << endl;
      outputfile << getType() << endl<<endl;
      outputfile << "# Name:" << endl;
      outputfile << name << endl<<endl;
      outputfile << "# Full name:" << endl;
      outputfile << getFullName() << endl<<endl;
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

  void Element::addDataInterfaceBaseRef(const string& DIBRef_){
    DIBRefs.push_back(DIBRef_);
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
}
