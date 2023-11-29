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
    if(stage==preInit) {
      if(rRim>0) {
	msg(Deprecated) << "(Tyre::init): rim radius is deprecated. Use ellipse parameters to define tyre contour." << endl;
	ab(0) = r-rRim;
	ab(1) = ab(0);
      }
      if(fabs(ab(0)-ab(1))>1e-8)
	throwError("(Tyre::init): currently only circle contours are supported.");
      if(w<0) w = 2*ab(1);
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
	int nEta = 51;
	int nXi;
	vector<vector<double>> vp;
	if(ab(1)>0) {
	  nXi = 51;
	  vector<double> ombvXiNodes(nXi);
	  vp.resize(nEta*nXi);
	  double b = ab(1);
	  double al = (w/2<b)?asin(w/2/b):M_PI/2;
	  for (int i=0; i<nEta; i++) {
	    double eta = 2*M_PI*i/50.;
	    for (int j=0; j<nXi; j++) {
	      double xi = -al + 2*al*j/50.;
	      vp[i*nXi+j].push_back((r-b*(1-cos(xi)))*cos(eta));
	      vp[i*nXi+j].push_back(-b*sin(xi));
	      vp[i*nXi+j].push_back((r-b*(1-cos(xi)))*sin(eta));
	    }
	  }
	}
	else {
	  nXi = 48;
	  vp.resize(nEta*nXi);
	  for (int i=0; i<nEta; i++) {
	    double eta = 2*M_PI*i/50.;
	    for (int j=0; j<2; j++) {
	      double xi = 0.7*r*(1-j) + (r-0.1*w)*j;
	      vp[i*nXi+j].push_back(xi*cos(eta));
	      vp[i*nXi+j].push_back(w/2);
	      vp[i*nXi+j].push_back(xi*sin(eta));
	    }
	    for (int j=0; j<21; j++) {
	      double xi = M_PI/2*j/20.;
	      vp[i*nXi+2+j].push_back((r+0.1*w*(sin(xi)-1))*cos(eta));
	      vp[i*nXi+2+j].push_back(w*(0.4+0.1*cos(xi)));
	      vp[i*nXi+2+j].push_back((r+0.1*w*(sin(xi)-1))*sin(eta));
	    }
	    for (int j=0; j<2; j++) {
	      double xi = -0.7*w/2 + 0.7*w*j;
	      vp[i*nXi+23+j].push_back(r*cos(eta));
	      vp[i*nXi+23+j].push_back(-xi);
	      vp[i*nXi+23+j].push_back(r*sin(eta));
	    }
	    for (int j=0; j<21; j++) {
	      double xi = M_PI/2 - M_PI/2*j/20.;
	      vp[i*nXi+25+j].push_back((r+0.1*w*(sin(xi)-1))*cos(eta));
	      vp[i*nXi+25+j].push_back(-w*(0.4+0.1*cos(xi)));
	      vp[i*nXi+25+j].push_back((r+0.1*w*(sin(xi)-1))*sin(eta));
	    }
	    for (int j=0; j<2; j++) {
	      double xi = (r-0.1*w)*(1-j)+0.7*r*j;
	      vp[i*nXi+46+j].push_back(xi*cos(eta));
	      vp[i*nXi+46+j].push_back(-w/2);
	      vp[i*nXi+46+j].push_back(xi*sin(eta));
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"radius");
    if(e)
      setRadius(E(e)->getText<double>());
    else {
      e=E(element)->getFirstElementChildNamed(MBSIM%"unloadedRadius");
      Deprecated::message(cout, "Feature unloadedRadius is deprecated. Use feature radius instead.", e);
      setRadius(E(e)->getText<double>());
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"rimRadius");
    if(e) {
      Deprecated::message(cout, "Feature rimRadius is deprecated. Use feature ellipseParameters to define tyre contour.", e);
      rRim = E(e)->getText<double>();
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"width");
    if(e) setWidth(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"ellipseParameters");
    if(e) setEllipseParameters(E(e)->getText<Vec2>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      auto ombv = shared_ptr<OpenMBVSpatialContour>(new OpenMBVSpatialContour);
      ombv->initializeUsingXML(e);
      openMBVRigidBody=ombv->createOpenMBV();
    }
  }

}
