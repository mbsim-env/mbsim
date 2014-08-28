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
 * Contact: thorsten.schindler@mytum.de
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "fmi_xml_export.h"
#include "fmi_utils.h"
#include "system.h"

using namespace std;
using namespace fmi;
int main (int argc, char* argv[]) {
  if(argc<2) {
    cout<<"Usage : fmixmlexport [<path/to/MBS.mbsim.flat.xml>]"<<endl;
    cout<<"If no input file then exports the file based on compile-time system implemented."<<endl;
  }
  FmiXmlExport *e;
  if(argc<2) {
    //creates dynamicsystem from intern model
    System* dss = new System("system");
//     dss->setInformationOutput(false);
//    dss->setPlotFeatureRecursive(MBSim::plotRecursive,MBSim::disabled);
    dss->initialize();
    e = new FmiXmlExport(dss);
  } else {
    e = new FmiXmlExport();
    //creates dynamicsystem from xml file provided
    initDssWithXml(argv[1],&(e->dss));
    e->dss->initialize();
  }
  e->buildDssFile();
  e->saveFile();
  delete(e);
  cout<<"Export successful."<<endl;
  return 0;
}
