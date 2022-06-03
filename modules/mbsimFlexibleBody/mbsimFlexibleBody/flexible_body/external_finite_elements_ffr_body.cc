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
#include "external_finite_elements_ffr_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, ExternalFiniteElementsFfrBody)

  void ExternalFiniteElementsFfrBody::setNodeNumbers(const VecVI &nodeNum) {
    for(int i=0; i<nodeNum.size(); i++)
      nodeMap[nodeNum(i)] = i;
  }

  void ExternalFiniteElementsFfrBody::importData() {
    if(inodes.size()==0 and nmodes.size()==0)
      throwError("(ExternalFiniteElementsFfrBody::init): error in component mode synthesis. At least one interface node number or normal mode number must be given.");

    if(nodes.cols()==4) {
      for(int i=0; i<nodes.rows(); i++)
	nodalPos[nodes(i,0)] = nodes.row(i)(RangeV(1,3)).T();
    }
    else if(nodes.cols()==3) {
      for(int i=0; i<nodes.rows(); i++)
	nodalPos[i+1] = nodes.row(i).T();
    }
    else
      throwError("(FiniteElementsFfrBody::init): number of columns in nodes does not match, must be 3 or 4");

    if(nodeMap.empty()) {
      for(int i=0; i<nodes.rows(); i++)
	nodeMap[i+1] = i;
    }

    int nen = net + ner;
    int nN = nodeMap.size();
    int ng = nN*nen;
    int nr = 0;
    if(bnodes.size() != dof.size())
      throwError("(ExternalFiniteElementsFfrBody::init): number of boundary nodes (" + to_string(bnodes.size()) + ") must equal number of degrees of freedom (" + to_string(dof.size()) + ")");
    for(size_t i=0; i<bnodes.size(); i++) {
      for(int j=0; j<bnodes[i].size(); j++) {
	bc[bnodes[i](j)].resize(nen);
	for(int k=0; k<dof[i].size(); k++) {
	  if(dof[i](k)<0 or dof[i](k)>nen-1)
	    throwError("(ExternalFiniteElementsFfrBody::init): degrees of freedom of boundary node number (" + to_string(i) + ") must be within range [0,3]");
	  bc[bnodes[i](j)](dof[i](k)) = 1;
	}
      }
    }
    for(const auto & i : bc)
      for(int j=0; j<i.second.size(); j++)
	nr += i.second(j);
    int n = ng-nr;

    SymMatV M0(ng);
    Ke0.resize(ng);

    KrKP.resize(nN);
    for(const auto & i : nodeMap)
      KrKP[i.second] = nodalPos[i.first];

    for(int i=0; i<M.rows(); i++)
      M0.e(M(i,0),M(i,1)) = M(i,2);

    for(int i=0; i<K.rows(); i++)
      Ke0.e(K(i,0),K(i,1)) = K(i,2);

    // compute mass and lumped mass matrix
    vector<double> mi(nN);
    double ds = 0;
    for(int i=0; i<nN; i++) {
      ds += M0.e(i*nen,i*nen);
      m += M0.e(i*nen,i*nen);
      for(int j=i+1; j<nN; j++)
	m += 2*M0.e(i*nen,j*nen);
    }
    for(int i=0; i<nN; i++)
      mi[i] = M0.e(i*nen,i*nen)/ds*m;

    vector<int> c;
    for(const auto & i : bc) {
      for(int j=0; j<i.second.size(); j++)
	if(i.second(j)) c.push_back(nodeMap[i.first]*nen+j);
    }
    sort(c.begin(), c.end());

    size_t h=0;
    Indices IF;
    Indices IX;
    for(int i=0; i<ng; i++) {
      if(h<c.size() and i==c[h]) {
	h++;
	IX.add(i);
      }
      else
	IF.add(i);
    }

    M0 <<= M0(IF);
    Ke0 <<= Ke0(IF);

    c.clear();
    for(int i=0; i<inodes.size(); i++) {
      VecVI bci = bc[inodes(i)];
      for(int j=0; j<nen; j++) {
	if((not bci.size()) or (not bci(j)))
	  c.push_back(nodeMap[inodes(i)]*nen+j);
      }
    }
    sort(c.begin(), c.end());
    h=0;
    Indices IH, IN;
    for(int i=0; i<IF.size(); i++) {
      if(h<c.size() and IF[i]==c[h]) {
	IH.add(i);
	h++;
      }
      else
	IN.add(i);
    }
    MatV Vsd(n,IH.size()+nmodes.size(),NONINIT);
    if(IH.size()) {
      Indices IJ;
      for(int i=0; i<IH.size(); i++)
	IJ.add(i);
      MatV Vs(IF.size(),IH.size(),NONINIT);
      Vs.set(IN,IJ,-slvLL(Ke0(IN),Ke0(IN,IH)));
      Vs.set(IH,IJ,MatV(IH.size(),IH.size(),Eye()));
      Vsd.set(RangeV(0,n-1),RangeV(0,Vs.cols()-1),Vs);
    }

    if(nmodes.size()) {
      SqrMat V;
      Vec w;
      if(fixedBoundaryNormalModes) {
	eigvec(Ke0(IN),M0(IN),V,w);
	vector<int> imod;
	for(int i=0; i<w.size(); i++) {
	  if(w(i)>pow(2*M_PI*0.1,2))
	    imod.push_back(i);
	}
	if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	  throwError(string("(ExternalFiniteElementsFfrBody::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
	for(int i=0; i<nmodes.size(); i++) {
	  Vsd.set(IN,IH.size()+i,V.col(imod[nmodes(i)-1]));
	  Vsd.set(IH,IH.size()+i,Vec(IH.size()));
	}
      }
      else {
	eigvec(Ke0,M0,V,w);

	vector<int> imod;
	for(int i=0; i<w.size(); i++) {
	  if(w(i)>pow(2*M_PI*0.1,2))
	    imod.push_back(i);
	}
	if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	  throwError(string("(ExternalFiniteElementsFfrBody::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
	for(int i=0; i<nmodes.size(); i++)
	  Vsd.set(IH.size()+i,V.col(imod[nmodes(i)-1]));
      }
    }

    if(IH.size()) {
      SqrMat V;
      Vec w;
      eigvec(JTMJ(Ke0,Vsd),JTMJ(M0,Vsd),V,w);
      vector<int> imod;
      for(int i=0; i<w.size(); i++) {
	if(w(i)>pow(2*M_PI*0.1,2))
	  imod.push_back(i);
      }
      MatV Vr(w.size(),imod.size(),NONINIT);
      for(size_t i=0; i<imod.size(); i++)
	Vr.set(i,V.col(imod[i]));
      Vsd <<= Vsd*Vr;
    }

    Phi = vector<Mat3xV>(nN,Mat3xV(Vsd.cols(),NONINIT));
    MatV Vsdg(ng,Vsd.cols(),NONINIT);
    Indices IJ;
    for(int i=0; i<Vsd.cols(); i++)
      IJ.add(i);
    Vsdg.set(IF,IJ,Vsd);
    Vsdg.set(IX,IJ,Mat(IX.size(),IJ.size()));
    for(size_t i=0; i<Phi.size(); i++)
      Phi[i] = Vsdg(RangeV(nen*i,nen*i+net-1),RangeV(0,Vsd.cols()-1));
    Psi.resize(nN,Mat3xV(Vsd.cols(),NONINIT));
    sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(Vsd.cols()));

    Ke0 <<= JTMJ(Ke0,Vsd);

    Pdm.resize(Vsd.cols());
    rPdm.resize(3,Mat3xV(Vsd.cols()));
    PPdm.resize(3,vector<SqrMatV>(3,SqrMatV(Vsd.cols())));
    if(formalism==lumpedMass) {
      // compute integrals
      for(int i=0; i<nN; i++) {
        rdm += mi[i]*KrKP[i];
        rrdm += mi[i]*JTJ(KrKP[i].T());
        Pdm += mi[i]*Phi[i];
      }
      for(int k=0; k<3; k++) {
        for(int i=0; i<nN; i++)
          rPdm[k] += mi[i]*KrKP[i](k)*Phi[i];
        for(int l=0; l<3; l++) {
          for(int i=0; i<nN; i++)
            PPdm[k][l] += mi[i]*Phi[i].row(k).T()*Phi[i].row(l);
        }
      }
    }
    else if(formalism==consistentMass) {
      if((fPrPK and fPrPK->getArg1Size()) or (fAPK and fAPK->getArg1Size()))
        throwError("Translation and rotation is not allowed for consistent mass formalism. Use lumped mass formalism instead.");
      // compute reduced mass matrix
      PPdm[0][0] = JTMJ(M0,Vsd);
    }
    else
      throwError("Formalism unknown.");
  }

  void ExternalFiniteElementsFfrBody::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef)
      importData();
    else if(stage==plotting) {
      if(plotFeature[openMBV] and ombvBody) {
        std::shared_ptr<OpenMBV::FlexibleBody> flexbody = ombvBody->createOpenMBV();
        openMBVBody = flexbody;
        ombvColorRepresentation = static_cast<OpenMBVFlexibleBody::ColorRepresentation>(ombvBody->getColorRepresentation());
      }
    }
    GenericFlexibleFfrBody::init(stage, config);
  }

  void ExternalFiniteElementsFfrBody::initializeUsingXML(DOMElement *element) {
    GenericFlexibleFfrBody::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"numberOfNodalTranslationalDegreesOfFreedom");
    setNumberOfNodalTranslationalDegreesOfFreedom(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"numberOfNodalRotationalDegreesOfFreedom");
    setNumberOfNodalRotationalDegreesOfFreedom(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodes");
    setNodes(E(e)->getText<MatV>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodeNumbers");
    if(e) setNodeNumbers(MBXMLUtils::E(e)->getText<VecVI>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"massMatrix");
    setMassMatrix(E(e)->getText<MatVx3>());
    for(int i=0; i<M.rows(); i++) {
      M(i,0)--; M(i,1)--;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"stiffnessMatrix");
    setStiffnessMatrix(E(e)->getText<MatVx3>());
    for(int i=0; i<K.rows(); i++) {
      K(i,0)--; K(i,1)--;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"formalism");
    if(e) {
      string formalismStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(formalismStr=="consistentMass") formalism=consistentMass;
      else if(formalismStr=="lumpedMass") formalism=lumpedMass;
      else formalism=unknown;
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"proportionalDamping");
    if(e) setProportionalDamping(E(e)->getText<Vec>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"boundaryNodeNumbers");
    while(e && MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"boundaryNodeNumbers") {
      addBoundaryNodes(MBXMLUtils::E(e)->getText<VecVI>());
      e=e->getNextElementSibling();
      if(MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"degreesOfFreedom") {
	VecVI dof = MBXMLUtils::E(e)->getText<VecVI>();
	for(int i=0; i<dof.size(); i++)
	  dof(i)--;
	addDegreesOfFreedom(dof);
      }
      e=e->getNextElementSibling();
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"interfaceNodeNumbers");
    if(e) setInterfaceNodeNumbers(MBXMLUtils::E(e)->getText<VecVI>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"normalModeNumbers");
    if(e) setNormalModeNumbers(MBXMLUtils::E(e)->getText<VecVI>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"fixedBoundaryNormalModes");
    if(e) setFixedBoundaryNormalModes(MBXMLUtils::E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBV");
    if(e) {
      ombvBody = shared_ptr<OpenMBVFlexibleFfrBody>(new OpenMBVFlexibleFfrBody);
      ombvBody->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"plotNodeNumbers");
    if(e) setPlotNodeNumbers(E(e)->getText<VecVI>());
  }

}
