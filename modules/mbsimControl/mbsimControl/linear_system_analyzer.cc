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
#include "mbsimControl/linear_system_analyzer.h"
#include "mbsimControl/namespace.h"
#include "mbsimControl/extern_signal_source.h"
#include "mbsimControl/extern_signal_sink.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/eps.h"
#include "fmatvec/linear_algebra_complex.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  const Vector<Ref,double> fromComplex(const Vector<Ref,complex<double>> &x) {
    Vector<Ref,double> y(x.size(),NONINIT);
    for(int i=0; i<x.size(); i++)
      y(i) = x(i).real();
    return y;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, LinearSystemAnalyzer)

  void LinearSystemAnalyzer::execute() {
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

    if(not(z0.size()))
      zEq <<= system->evalz0();
    else {
      if(z0.size()!=system->getzSize()+system->getisSize())
	throwError(string("(LinearSystemAnalyzer::execute): size of z0 does not match, must be ") + to_string(system->getzSize()));
      zEq <<= z0(RangeV(0,system->getzSize()-1));
      system->setcuris(z0(RangeV(system->getzSize(),z0.size()-1)));
    }

    if(not(u0.size()))
      u0.resize(nsource);
    else if(u0.size()!=nsource)
      throwError(string("(LinearSystemAnalyzer::execute): size of u0 does not match, must be ") + to_string(nsource));

    if(fex.size()) {
      if(not Amp)
	throwError(string("(LinearSystemAnalyzer::execute): excitation amplitude function must be defined"));
      else if(Amp->getRetSize().first!=nsource)
	throwError(string("(LinearSystemAnalyzer::execute): size of excitation amplitude function does not match, must be ") + to_string(nsource));
      Amp->setParent(system);
      Amp->setName("ExcitationAmplitudeFunction");
      Amp->init(Element::preInit, InitConfigSet());
    }

    SqrMat A(system->getzSize(),NONINIT);
    Mat B(system->getzSize(),nsource,NONINIT);
    Mat D(nsink,nsource,NONINIT);
    Mat C(nsink,system->getzSize(),NONINIT);

    system->setTime(t0);
    system->setState(zEq);
    int l=0;
    for(size_t i=0; i<source.size(); i++) {
      Vec sigso(source[i]->getSignalSize());
      for(int k=0; k<source[i]->getSignalSize(); k++) {
	sigso(k) = u0(l);
	source[i]->setSignal(sigso);
	l++;
      }
    }
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
    for(int i=0; i<system->getzSize(); i++) {
      double ztmp = system->getState()(i);
      system->getState()(i) += epsroot;
      system->resetUpToDate();
      Vec zd1 = system->evalzd();
      A.set(i, (zd1 - zd0) / epsroot);
      system->getState()(i) = ztmp;
    }

    l=0;
    for(size_t i=0; i<source.size(); i++) {
      Vec sigso(source[i]->getSignalSize());
      for(int k=0; k<source[i]->getSignalSize(); k++) {
	sigso(k) = u0(l) + epsroot;
	source[i]->setSignal(sigso);
	system->resetUpToDate();
	Vec zd1 = system->evalzd();
	B.set(l, (zd1 - zd0) / epsroot);
	j=0;
	for(size_t m=0; m<sink.size(); m++) {
	  sigsi1.set(RangeV(j,j+sink[m]->getSignalSize()-1), sink[m]->evalSignal());
	  j+=sink[m]->getSignalSize();
	}
	D.set(l, (sigsi1 - sigsi0) / epsroot);
	sigso(k) = u0(l);
	source[i]->setSignal(sigso);
	l++;
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

    SquareMatrix<Ref,complex<double>> V;
    Vector<Ref,complex<double>> w;
    eigvec(A,V,w);
    vector<pair<double,int>> fna;
    for(int i=0; i<w.size()-1; i++) {
      if((abs(imag(w(i)))>=2*M_PI*fmin) and (abs(imag(w(i)))<=2*M_PI*fmax) and (w(i+1)==conj(w(i)))) {
        fna.push_back(pair<double,int>(imag(w(i))/2/M_PI,i));
        i++;
      }
    }
    sort(fna.begin(), fna.end());

    Matrix<General,Ref,Ref,complex<double>> Zhna(Matrix<General,Ref,Ref,complex<double>>(system->getzSize(),fna.size(),NONINIT));
    Matrix<General,Ref,Ref,complex<double>> Yhna(Matrix<General,Ref,Ref,complex<double>>(nsink,fna.size(),NONINIT));
    for(size_t i=0; i<fna.size(); i++) {
      Vector<Ref,complex<double>> zh = V.col(fna[i].second);
      Vector<Ref,complex<double>> yh = C*zh;
      Zhna.set(i,zh);
      Yhna.set(i,yh);
    }

    vector<Matrix<General,Ref,Ref,complex<double>>> Zhex(nsource,Matrix<General,Ref,Ref,complex<double>>(system->getzSize(),fex.size(),NONINIT));
    vector<Matrix<General,Ref,Ref,complex<double>>> Yhex(nsource,Matrix<General,Ref,Ref,complex<double>>(nsink,fex.size(),NONINIT));
    Vector<Ref,complex<double>> u(nsource);
    for(int i=0; i<fex.size(); i++) {
      SquareMatrix<Ref,complex<double>> H = SquareMatrix<Ref,complex<double>>(A.size(),Eye(),complex<double>(0,2*M_PI*fex(i))) - A;
      for(int j=0; j<nsource; j++) {
	u(j).real((*Amp)(fex(i))(j));
	Vector<Ref,complex<double>> zh = slvLU(H, B*u);
	Vector<Ref,complex<double>> yh = C*zh + D*u;
	Zhex[j].set(i,zh);
	Yhex[j].set(i,yh);
	u(j).real(0);
      }
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

    os.open("initial_output.mat");
    if(os.is_open()) {
      os << "# name: " << "z0" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << zEq.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i<zEq.size(); i++)
        os << setw(28) << zEq.e(i) << endl;
      os << endl;
      os << "# name: " << "y0" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << sigsi0.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i<sigsi0.size(); i++)
        os << setw(28) << sigsi0.e(i) << endl;
      os.close();
    }

    os.open("eigenanalysis.mat");
    if(os.is_open()) {
      os << "# name: " << "lambda" << endl;
      os << "# type: " << "complex matrix" << endl;
      os << "# rows: " << w.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i<w.size(); i++)
        os << setw(28) << w.e(i) << endl;
      os << endl;
      os << "# name: " << "V" << endl;
      os << "# type: " << "complex matrix" << endl;
      os << "# rows: " << V.rows() << endl;
      os << "# columns: " << V.cols() << endl;
      for(int i=0; i<V.rows(); i++) {
        for(int j=0; j<V.cols(); j++)
          os << setw(28) << V.e(i,j) << " ";
        os << endl;
      }
      os.close();
    }

    os.open("modal_analysis.mat");
    if(os.is_open()) {
      os << "# name: " << "lambda" << endl;
      os << "# type: " << "complex matrix" << endl;
      os << "# rows: " << fna.size() << endl;
      os << "# columns: " << 1 << endl;
      for(size_t i=0; i<fna.size(); i++)
        os << setw(28) << w(fna[i].second) << endl;
      os << endl;
      os << "# name: " << "Zh" << endl;
      os << "# type: " << "complex matrix" << endl;
      os << "# rows: " << Zhna.rows() << endl;
      os << "# columns: " << Zhna.cols() << endl;
      for(int k=0; k<Zhna.rows(); k++) {
	for(int i=0; i<Zhna.cols(); i++)
	  os << setw(28) << Zhna(k,i) << " ";
	os << endl;
      }
      os << endl;
      os << "# name: " << "Yh" << endl;
      os << "# type: " << "complex matrix" << endl;
      os << "# rows: " << Yhna.rows() << endl;
      os << "# columns: " << Yhna.cols() << endl;
      for(int k=0; k<Yhna.rows(); k++) {
	for(int i=0; i<Yhna.cols(); i++)
	  os << setw(28) << Yhna(k,i) << " ";
	os << endl;
      }
      os.close();
    }

    os.open("frequency_response_analysis.mat");
    if(os.is_open()) {
      os << "# name: " << "f" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << fex.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i<fex.size(); i++)
	os << setw(28) << fex.e(i) << endl;
      os << endl;

      os << "# name: " << "Zh" << endl;
      os << "# type: " << "cell" << endl;
      os << "# rows: " << nsource << endl;
      os << "# columns: " << 1 << endl;
      os << endl;
      for(int i=0; i<nsource; i++) {
	os << "# name: " << "<cell-element>" << endl;
	os << "# type: " << "complex matrix" << endl;
	os << "# rows: " << Zhex[i].rows() << endl;
	os << "# columns: " << Zhex[i].cols() << endl;
	for(int j=0; j<Zhex[i].rows(); j++) {
	  for(int k=0; k<Zhex[i].cols(); k++)
	    os << setw(28) << Zhex[i].e(j,k) << " ";
	  os << endl;
	}
	os << endl;
      }
      os << "# name: " << "Yh" << endl;
      os << "# type: " << "cell" << endl;
      os << "# rows: " << nsource << endl;
      os << "# columns: " << 1 << endl;
      os << endl;
      for(int i=0; i<nsource; i++) {
	os << "# name: " << "<cell-element>" << endl;
	os << "# type: " << "complex matrix" << endl;
	os << "# rows: " << Yhex[i].rows() << endl;
	os << "# columns: " << Yhex[i].cols() << endl;
	for(int j=0; j<Yhex[i].rows(); j++) {
	  for(int k=0; k<Yhex[i].cols(); k++)
	    os << setw(28) << Yhex[i].e(j,k) << " ";
	  os << endl;
	}
	os << endl;
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
      os << "# name: " << "z0" << endl;
      os << "# type: " << "matrix" << endl;
      os << "# rows: " << zEq.size() << endl;
      os << "# columns: " << 1 << endl;
      for(int i=0; i<zEq.size(); i++)
        os << setw(28) << zEq.e(i) << endl;
      os.close();
    }

    t0 = 0;

    if(msv) {
      if(not(modeScale.size()))
	modeScale.resize(fna.size(),INIT,1);
      else if(modeScale.size()!=(int)fna.size())
	throwError(string("(LinearSystemAnalyzer::execute): size of mode scale does not match, must be ") + to_string(fna.size()));
      for(size_t i=0; i<fna.size(); i++) {
	if(modeScale(i)>0) {
	  complex<double> iom(0,2*M_PI*fna[i].first);
	  double T = double(loops)/fna[i].first;
	  for(double t=t0; t<t0+T+dtPlot; t+=dtPlot) {
	    system->setTime(t);
	    system->setState(zEq + fromComplex(Zhna.col(i)*(modeScale(i)*exp(iom*t))));
	    system->resetUpToDate();
	    system->plot();
	    system->checkExitRequest();
	    if(system->getIntegratorExitRequest())
	      break;
	  }
	  if(system->getIntegratorExitRequest())
	    break;
	  t0 += T+dtPlot;
	}
      }
    }

    if(frv) {
      if(nsource and (inum<0 or inum>=nsource))
	throwError(string("(LinearSystemAnalyzer::execute): input number does not match, must be greater than zero and smaller than ") + to_string(nsource));
      l=0;
      for(size_t j=0; j<source.size(); j++) {
	Vec sigso(source[j]->getSignalSize());
	for(int k=0; k<source[j]->getSignalSize(); k++) {
	  if(l==inum) {
	    for(int i=0; i<fex.size(); i++) {
	      if(fex(i)>=fexmin and fex(i)<=fexmax and (*Amp)(fex(i))(l)>0) {
		complex<double> iOm(0,2*M_PI*fex(i));
		double T = double(loops)/fex(i);
		for(double t=t0; t<t0+T+dtPlot; t+=dtPlot) {
		  system->setTime(t);
		  sigso(k) = (*Amp)(fex(i))(l)*cos(iOm.imag()*t);
		  source[j]->setSignal(sigso);
		  system->setState(zEq + fromComplex(Zhex[l].col(i)*exp(iOm*t)));
		  system->resetUpToDate();
		  system->plot();
		  system->checkExitRequest();
		  if(system->getIntegratorExitRequest())
		    break;
		}
		if(system->getIntegratorExitRequest())
		  break;
		sigso(k) = 0;
		source[j]->setSignal(sigso);
		t0 += T+dtPlot;
	      }
	    }
	  }
	  l++;
	}
      }
    }
  }

  void LinearSystemAnalyzer::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"initialTime");
    if(e) setInitialTime(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"initialState");
    if(e) setInitialState(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"initialInput");
    if(e) setInitialInput(E(e)->getText<fmatvec::Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"minimumNaturalFrequency");
    if(e) setMinimumNaturalFrequency(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"maximumNaturalFrequency");
    if(e) setMaximumNaturalFrequency(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"excitationFrequencies");
    if(e) setExcitationFrequencies(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"excitationAmplitudeFunction");
    if(e) setExcitationAmplitudeFunction(ObjectFactory::createAndInit<MBSim::Function<VecV(double)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"visualizeNaturalModeShapes");
    if(e) {
      msv = true;
      DOMElement *ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"naturalModeScale");
      if(ee) modeScale <<= E(ee)->getText<VecV>();
    }
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"visualizeFrequencyResponse");
    if(e) {
      frv = true;
      DOMElement *ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"minimumExcitationFrequency");
      if(ee) fexmin = E(ee)->getText<double>();
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"maximumExcitationFrequency");
      if(ee) fexmax = E(ee)->getText<double>();
      ee=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"inputNumber");
      if(ee) inum = E(ee)->getText<Index>()-1;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"plotStepSize");
    if(e) setPlotStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"loops");
    if(e) setLoops(E(e)->getText<double>());
  }

}
