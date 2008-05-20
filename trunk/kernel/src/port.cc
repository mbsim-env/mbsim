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
#ifdef HAVE_AMVIS
#include "crigidbody.h"
#include "data_interface_base.h"
#include "rotarymatrices.h"
using namespace AMVis;
#endif

namespace MBSim {

  Port::Port(const string &name) : Element(name), WrOP(3), WvP(3), WomegaP(3), AWP(3) {
#ifdef HAVE_AMVIS
bodyAMVisUserFunctionColor= NULL;
bodyAMVis = NULL;
AMVisInstance=0;
#endif
    AWP(0,0) = 1;
    AWP(1,1) = 1;
    AWP(2,2) = 1;
    plotLevel= 0;
  }

  void Port::init() {
    setFullName(parent->getName()+"."+name);
  }

#ifdef HAVE_AMVIS
  void Port::setAMVisBody(CRigidBody *AMVisBody, DataInterfaceBase *funcColor, int instance){
    bodyAMVis = AMVisBody;
    bodyAMVisUserFunctionColor = funcColor;
    AMVisInstance=instance;
    if (!plotLevel) plotLevel=1;
  }
#endif

  void Port::plot(double t, double dt) {				// HR 03.01.07
    Element::plot(t,dt);
    if (plotLevel > 0) {
      for(int i=0; i<3; i++)
	plotfile<<" "<< WrOP(i);
      for(int i=0; i<3; i++)
	plotfile<<" "<< WvP(i);
    }
#ifdef HAVE_AMVIS
    if (bodyAMVis) {
      Vec AlpBetGam;
      AlpBetGam = AIK2Cardan(AWP);
      if (bodyAMVisUserFunctionColor) {
	double color = (*bodyAMVisUserFunctionColor)(t)(0);
	if (color>1)   color =1;
	if (color<0) color =0;
	bodyAMVis->setColor(color);
      }
      bodyAMVis->setTime(t);
      bodyAMVis->setTranslation(WrOP(0),WrOP(1),WrOP(2));
      bodyAMVis->setRotation(AlpBetGam(0),AlpBetGam(1),AlpBetGam(2));
      bodyAMVis->appendDataset(AMVisInstance);
    }
#endif
  }

  void Port::initPlotFiles() {					// HR 03.01.07
#ifdef HAVE_AMVIS
    if(bodyAMVis && AMVisInstance==0)
      bodyAMVis->writeBodyFile();  
#endif

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
