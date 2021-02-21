/* Copyright (C) 2004-2021 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#include <config.h>
#include "mbsimControl/lti_system_exporter.h"
#include "mbsimControl/namespace.h"
#include "mbsimControl/extern_signal_source.h"
#include "mbsimControl/extern_signal_sink.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/eps.h"
#include "mbsim/objects/object.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, LTISystemExporter)

  void LTISystemExporter::execute() {
    vector<ExternSignalSource*> source;
    vector<ExternSignalSink*> sink;
    vector<Link*> links = system->getLinks();
    int nsource=0, nsink=0;
    for(size_t i=0; i<links.size(); i++) {
      if(dynamic_cast<ExternSignalSource*>(links[i])) {
	source.push_back(static_cast<ExternSignalSource*>(links[i]));
	nsource += source[source.size()-1]->getSignalSize();
      }
      if(dynamic_cast<ExternSignalSink*>(links[i])) {
	sink.push_back(static_cast<ExternSignalSink*>(links[i]));
	nsink += sink[sink.size()-1]->getSignalSize();
      }
    }
    if(not(zEq.size()))
      zEq <<= system->evalz0();
    else if(zEq.size()!=system->getzSize()+system->getisSize())
      throwError(string("(LTISystemExporter::computeEigenvalues): size of z0 does not match, must be ") + to_string(system->getzSize()));

    SqrMat A(system->getzSize(),NONINIT);
    Mat B(system->getzSize(),nsource,NONINIT);
    Mat D(nsink,nsource,NONINIT);
    Mat C(nsink,system->getzSize(),NONINIT);

    system->setTime(t);
    system->setState(zEq);
    system->resetUpToDate();
    system->computeInitialCondition();
    zEq = system->getState();
    Vec zd0 = system->evalzd();
    Vec sigsi0(nsink), sigsi1(nsink);
    int j=0;
    for(size_t i=0; i<sink.size(); i++) {
      sigsi0.set(RangeV(j,j+sink[i]->getSignalSize()-1), sink[i]->evalSignal());
      j+=sink[i]->getSignalSize();
    }
    for (int i=0; i<system->getzSize(); i++) {
      double ztmp = system->getState()(i);
      system->getState()(i) += epsroot;
      system->resetUpToDate();
      Vec zd1 = system->evalzd();
      A.set(i, (zd1 - zd0) / epsroot);
      system->getState()(i) = ztmp;
    }

    int l=0;
    for(size_t i=0; i<source.size(); i++) {
      Vec sigso(source[i]->getSignalSize());
      for(int k=0; k<source[i]->getSignalSize(); k++) {
	sigso(k) = 1;
	source[i]->setSignal(sigso);
	system->resetUpToDate();
	Vec zd1 = system->evalzd();
	B.set(l, zd1 - zd0);
	j=0;
	for(size_t l=0; l<sink.size(); l++) {
	  sigsi1.set(RangeV(j,j+sink[l]->getSignalSize()-1), sink[l]->evalSignal());
	  j+=sink[l]->getSignalSize();
	}
	D.set(l, sigsi1 - sigsi0);
	l++;
	sigso(k) = 0;
	source[i]->setSignal(sigso);
      }
    }
    for(int i=0; i<system->getzSize(); i++) {
      double ztmp = system->getState()(i);
      system->getState()(i) += epsroot;
      system->resetUpToDate();
      j=0;
      for(size_t k=0; k<sink.size(); k++) {
	sigsi1.set(RangeV(j,j+sink[k]->getSignalSize()-1), sink[k]->evalSignal());
	j+=sink[k]->getSignalSize();
      }
      C.set(i, (sigsi1 - sigsi0) / epsroot);
      system->getState()(i) = ztmp;
    }

    ofstream os("inputtable.asc");
    if(os.is_open()) {
      for(size_t i=0; i<source.size(); i++) {
	for(int j=0; j<source[i]->getSignalSize(); j++)
	  os << source[i]->getPath() << " u " << j << endl;
      }
      os.close();
    }
    os.open("outputtable.asc");
    if(os.is_open()) {
      for(size_t i=0; i<sink.size(); i++) {
	for(int j=0; j<sink[i]->getSignalSize(); j++)
	  os << sink[i]->getPath() << " y " << j << endl;
      }
      os.close();
    }
    os.open("lti_system.mat");
    if(os.is_open()) {
      os << "# name: " << "A" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << A.rows() << endl;
      os << "# columns: " << A.cols() << endl;
      for(int i=0; i<A.rows(); i++) {
        for(int j=0; j<A.cols(); j++)
          os << setw(28) << A.e(i,j) << " ";
        os << endl;
      }
      os << endl;
      os << "# name: " << "B" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << B.rows() << endl;
      os << "# columns: " << B.cols() << endl;
      for(int i=0; i<B.rows(); i++) {
	for(int j=0; j<B.cols(); j++)
	  os << setw(28) << B.e(i,j) << " ";
	os << endl;
      }
      os << endl;
      os << "# name: " << "C" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << C.rows() << endl;
      os << "# columns: " << C.cols() << endl;
      for(int i=0; i<C.rows(); i++) {
        for(int j=0; j<C.cols(); j++)
          os << setw(28) << C.e(i,j) << " ";
        os << endl;
      }
      os << endl;
      os << "# name: " << "D" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << D.rows() << endl;
      os << "# columns: " << D.cols() << endl;
      for(int i=0; i<D.rows(); i++) {
        for(int j=0; j<D.cols(); j++)
          os << setw(28) << D.e(i,j) << " ";
        os << endl;
      }
      os << endl;
      os.close();
    }
  }

  void LTISystemExporter::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"time");
    if(e) setTime(E(e)->getText<double>());
  }

}
