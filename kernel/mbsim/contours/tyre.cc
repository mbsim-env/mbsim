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
	vector<double> ombvEtaNodes(51);
	vector<double> ombvXiNodes(51);
	for(unsigned int i=0; i<ombvEtaNodes.size(); i++)
	  ombvEtaNodes[i] = 2*M_PI*i/50.;
	for(unsigned int i=0; i<ombvXiNodes.size(); i++)
	  ombvXiNodes[i] = -M_PI/2 + M_PI*i/50.;
	vector<vector<double>> vp(ombvEtaNodes.size()*ombvXiNodes.size());
	int n = ombvXiNodes.size();
	for (unsigned int i=0; i<ombvEtaNodes.size(); i++) {
	  double eta = ombvEtaNodes[i];
	  for (unsigned int j=0; j<ombvXiNodes.size(); j++) {
	    double xi = ombvXiNodes[j];
	    vp[i*n+j].push_back((rRim+(rUnloaded-rRim)*cos(xi))*cos(eta));
	    vp[i*n+j].push_back(-(rUnloaded-rRim)*sin(xi));
	    vp[i*n+j].push_back((rRim+(rUnloaded-rRim)*cos(xi))*sin(eta));
	  }
	}
	vector<int> indices(5*(ombvEtaNodes.size()-1)*(ombvXiNodes.size()-1));
	int k=0;
	for(unsigned int i=0; i<ombvEtaNodes.size()-1; i++) {
	  for(unsigned int j=0; j<ombvXiNodes.size()-1; j++) {
	    indices[k+2] = i*ombvXiNodes.size()+j;
	    indices[k+1] = i*ombvXiNodes.size()+j+1;
	    indices[k+3] = (i+1)*ombvXiNodes.size()+j;
	    indices[k] = (i+1)*ombvXiNodes.size()+j+1;
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      auto ombv = shared_ptr<OpenMBVSpatialContour>(new OpenMBVSpatialContour);
      ombv->initializeUsingXML(e);
      openMBVRigidBody=ombv->createOpenMBV();
    }
  }

}
