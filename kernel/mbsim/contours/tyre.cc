/* Copyright (C) 2004-2022 MBSim Development Team
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

#include <config.h>
#include "tyre.h"
#include "mbsim/frames/frame.h"
#include <openmbvcppinterface/indexedfaceset.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Tyre)

  void Tyre::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
	int nEta = 51;
	int nXi;
	vector<vector<double>> vp;
	if(w > 0) {
	  nXi = 48;
	  vp.resize(nEta*nXi);
	  for (int i=0; i<nEta; i++) {
	    double eta = 2*M_PI*i/50.;
	    for (int j=0; j<2; j++) {
	      double xi = rRim*(1-j) + (rUnloaded-0.05*w)*j;
	      vp[i*nXi+j].push_back(xi*cos(eta));
	      vp[i*nXi+j].push_back(w/2);
	      vp[i*nXi+j].push_back(xi*sin(eta));
	    }
	    for (int j=0; j<21; j++) {
	      double xi = -M_PI/2 + M_PI*j/20.;
	      vp[i*nXi+2+j].push_back((rUnloaded+0.05*w*(sin(xi)-1))*cos(eta));
	      vp[i*nXi+2+j].push_back(w*(0.45+0.05*cos(xi)));
	      vp[i*nXi+2+j].push_back((rUnloaded+0.05*w*(sin(xi)-1))*sin(eta));
	    }
	    for (int j=0; j<2; j++) {
	      double xi = -0.9*w/2 + 0.9*w*j;
	      vp[i*nXi+23+j].push_back(rUnloaded*cos(eta));
	      vp[i*nXi+23+j].push_back(-xi);
	      vp[i*nXi+23+j].push_back(rUnloaded*sin(eta));
	    }
	    for (int j=0; j<21; j++) {
	      double xi = M_PI/2 - M_PI/4*j/20.;
	      vp[i*nXi+25+j].push_back((rUnloaded+0.05*w*(sin(xi)-1))*cos(eta));
	      vp[i*nXi+25+j].push_back(-w*(0.45+0.05*cos(xi)));
	      vp[i*nXi+25+j].push_back((rUnloaded+0.05*w*(sin(xi)-1))*sin(eta));
	    }
	    for (int j=0; j<2; j++) {
	      double xi = (rUnloaded-0.05*w)*(1-j)+rRim*j;
	      vp[i*nXi+46+j].push_back(xi*cos(eta));
	      vp[i*nXi+46+j].push_back(-w/2);
	      vp[i*nXi+46+j].push_back(xi*sin(eta));
	    }
	  }
	}
	else {
	  nXi = 51;
	  vector<double> ombvXiNodes(nXi);
	  vp.resize(nEta*nXi);
	  double rCrown = rUnloaded-rRim;
	  for (int i=0; i<nEta; i++) {
	    double eta = 2*M_PI*i/50.;
	    for (int j=0; j<nXi; j++) {
	      double xi = -M_PI/2 + M_PI*j/50.;
	      vp[i*nXi+j].push_back((rRim+rCrown*cos(xi))*cos(eta));
	      vp[i*nXi+j].push_back(-rCrown*sin(xi));
	      vp[i*nXi+j].push_back((rRim+rCrown*cos(xi))*sin(eta));
	    }
	  }
	}
	vector<int> indices(5*(nEta-1)*(nXi-1));
	int k=0;
	for(int i=0; i<nEta-1; i++) {
	  for(int j=0; j<nXi-1; j++) {
	    indices[k+2] = i*nXi+j;
	    indices[k+1] = i*nXi+j+1;
	    indices[k+3] = (i+1)*nXi+j;
	    indices[k] = (i+1)*nXi+j+1;
	    indices[k+4] = -1;
	    k+=5;
	  }
	}
	static_pointer_cast<OpenMBV::IndexedFaceSet>(openMBVRigidBody)->setVertexPositions(vp);
	static_pointer_cast<OpenMBV::IndexedFaceSet>(openMBVRigidBody)->setIndices(indices);
      }
    }
    RigidContour::init(stage, config);
  }

  void Tyre::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"unloadedRadius");
    setUnloadedRadius(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"rimRadius");
    setRimRadius(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"width");
    if(e) setWidth(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      auto ombv = shared_ptr<OpenMBVSpatialContour>(new OpenMBVSpatialContour);
      ombv->initializeUsingXML(e);
      openMBVRigidBody=ombv->createOpenMBV();
    }
  }

}
