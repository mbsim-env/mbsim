/* Copyright (C) 2004-2006  Martin Förg
 
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
 *
 */
#include <config.h>
#include "port.h"
#include "object.h"

namespace MBSim {

  Port::Port(const string &name) : Element(name), WrOP(3), WvP(3), WomegaP(3), AWP(3)  {

    AWP(0,0) = 1;
    AWP(1,1) = 1;
    AWP(2,2) = 1;
    plotLevel= 0;
  }

  void Port::init() {
    setFullName(parent->getName()+"."+name);
  }

  void Port::plot(double t, double dt) {				// HR 03.01.07
    Element::plot(t,dt);
    if (plotLevel > 0) {
      for(int i=0; i<3; i++)
	plotfile<<" "<< WrOP(i);
      for(int i=0; i<3; i++)
	plotfile<<" "<< WvP(i);
    }
  }

  void Port::initPlotFiles() {					// HR 03.01.07
    Element::initPlotFiles(); 
    if(plotLevel > 0) {
      for(int i=0; i<3; i++)
	plotfile <<"# "<< plotNr++ <<": WrOP("<<i<<")" << endl;
      for(int i=0; i<3; i++)
	plotfile <<"# "<< plotNr++ <<": WvP("<<i<<")" << endl;
    }
  }

  void Port::plotParameters() {					// HR 03.01.07
    if (plotLevel) {
      parafile << "Port :" <<name<<endl;
      parafile << "from object:" <<getObject()->getName() <<endl;
    }
  }

}
