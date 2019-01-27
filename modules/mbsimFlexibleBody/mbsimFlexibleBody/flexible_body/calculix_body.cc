/* Copyright (C) 2004-2019 MBSim Development Team
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
#include "calculix_body.h"
#include "openmbvcppinterface/dynamicindexedfaceset.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, CalculixBody)

  void CalculixBody::readDisplacements() {
    VecV dispi(3*nn,NONINIT);
    double d;
    string str;
    for(size_t i=0; i<5; i++)
      getline(isRes,str);
    for(size_t i=0; i<nn; i++) {
      isRes >> d >> d;
      for(size_t k=0; k<3; k++)
        isRes >> dispi.e(3*i+k);
    }
    disp.push_back(dispi);
  }

  void CalculixBody::readStresses() {
    VecV stressi(6*nn,NONINIT);
    double d;
    string str;
    for(size_t i=0; i<7; i++)
      getline(isRes,str);
    for(size_t i=0; i<nn; i++) {
      isRes >> d >> d;
      for(size_t k=0; k<6; k++)
        isRes >> stressi.e(6*i+k);
    }
    stress.push_back(stressi);
  }

  void CalculixBody::readNodes() {
    string str;
    while(isRes) {
      getline(isRes,str);
      if(str.length()>6 and str.substr(4,2)=="2C")
        break;
    }
    stringstream s(str);
    s >> str >> nn;
    u0.resize(3*nn,NONINIT);
    double d;
    for(size_t i=0; i<nn; i++) {
      isRes >> d >> d;
      for(size_t k=0; k<3; k++)
        isRes >> u0.e(3*i+k);
    }
  }

  void CalculixBody::readElements() {
    string str;
    while(isRes) {
      getline(isRes,str);
      if(str.length()>6 and str.substr(4,2)=="3C")
        break;
    }
    stringstream s(str);
    s >> str >> ne;
    eles.resize(ne,20,NONINIT);
    double d;
    size_t type, nn;
    for(size_t i=0; i<ne; i++) {
      isRes >> str >> str >> type;
      if(type==4)
        nn = 20;
      else
        throwError("Unknown element type.");
      getline(isRes,str);
      for(size_t j=0; j<nn;) {
        isRes >> d;
        if(d>0) {
          eles.e(i,j) = d-1;
          j++;
        }
      }
      getline(isRes,str);
    }
  }

  void CalculixBody::readModes() {
    string str;
    size_t i, nn_;
    while(isRes) {
      getline(isRes,str);
      if(str.length()>6 and str.substr(2,4)=="100C") {
//        cout << str[57] << endl;
        stringstream s(str);
        s >> str >> str >> str >> nn_;
        if(nn != nn_) throwError("Number of nodes does not match.");
        isRes >> i >> str;
        if(str=="DISP")
          readDisplacements();
        else if(str=="STRESS")
          readStresses();
      }
    }
    nm = min(max(int(nm),0),int(disp.size()));
  }

  void CalculixBody::readDOF() {
    // read dof table
    double d;
    while(true) {
      isDOF >> d;
      if(isDOF.eof()) break;
      dof.push_back(make_pair(size_t(d)-1,int(d*10)-size_t(d)*10-1));
    }
  }

  void CalculixBody::readStiffMatrix() {
    // read stiffness matrix
    K.resize(dof.size());
    int i, j;
    double d;
    while(true) {
      isStiff >> i >> j >> d;
      if(isStiff.eof()) break;
      K.e(i-1,j-1) = d;
    }
  }

  void CalculixBody::readMassMatrix() {
    // read mass matrix
    M.resize(dof.size());
    int i, j;
    double d;
    while(true) {
      isMass >> i >> j >> d;
      if(isMass.eof()) break;
      M.e(i-1,j-1) = d;
    }
  }

  void CalculixBody::importData() {
    string jobname = resultFileName.substr(0,resultFileName.length()-4);
    cout << jobname << endl;
    isRes.open(jobname+".frd");
    isStiff.open(jobname+".sti");
    isMass.open(jobname+".mas");
    isDOF.open(jobname+".dof");
    if(not isRes.is_open()) throwError("Result file does not exist.");
    if(not isStiff.is_open()) throwError("Stiffness matrix file does not exist.");
    if(not isMass.is_open()) throwError("Mass matrix file does not exist.");
    if(not isDOF.is_open()) throwError("DOF file does not exist.");

    readNodes();
    readElements();
    readModes();
    readDOF();
    readMassMatrix();
    readStiffMatrix();

    KrKP = getCellArray1D<Vec3>(3,u0);

    // set modes and stresses
    MatV Phi_(3*nn,nm,NONINIT);
    MatV Sr(6*nn,nm,NONINIT);
    for(int j=0; j<Phi_.cols(); j++) {
      for(int i=0; i<Phi_.rows(); i++)
        Phi_.e(i,j) = disp[j].e(i);
      for(int i=0; i<Sr.rows(); i++)
        Sr.e(i,j) = stress[j].e(i);
    }
    Phi = getCellArray1D<Mat3xV>(3,Phi_);
    sigmahel = getCellArray1D<Matrix<General, Fixed<6>, Var, double> >(6,Sr);

    Pdm.resize(nm);
    rPdm.resize(3,Mat3xV(nm));
    PPdm.resize(3,vector<SqrMatV>(3,SqrMatV(nm)));
    if(formalism==lumpedMass) {
      // compute mass and lumped mass matrix
      VecV mij(M.size(),NONINIT);
      VecV m_(3);
      for(size_t r=0; r<3; r++) {
        double ds = 0;
        for(size_t i=0; i<dof.size(); i++) {
          if(dof[i].second==r) {
            ds += M.e(i,i);
            m_.e(r) += M.e(i,i);
            for(size_t j=i+1; j<dof.size(); j++) {
              if(dof[j].second==r)
                m_.e(r) += 2*M.e(i,j);
            }
          }
        }
        for(size_t i=0; i<dof.size(); i++) {
          if(dof[i].second==r)
            mij(i) = M.e(i,i)/ds*m_.e(r);
        }
      }
      m = m_.e(0);

      // compute integrals
      RangeV J = RangeV(0,nm-1);
      for(size_t i=0; i<nn; i++) {
        RangeV I = RangeV(3*i,3*i+2);
        Vec3 u0i = u0(I);
        double mi = mij.e(3*i);
        rdm += mi*u0i;
        rrdm += mi*JTJ(u0i.T());
        Pdm += mi*Phi_(I,J);
      }
      for(size_t k=0; k<3; k++) {
        for(size_t i=0; i<nn; i++)
          rPdm[k] += mij.e(3*i)*u0.e(3*i+k)*Phi_(RangeV(3*i,3*i+2),J);
        for(size_t l=0; l<3; l++) {
          for(size_t i=0; i<nn; i++)
            PPdm[k][l] += mij.e(3*i)*Phi_.row(3*i+k).T()*Phi_.row(3*i+l);
        }
      }
    }
    else if(formalism==consistentMass) {
      if((fPrPK and fPrPK->getArg1Size()) or (fAPK and fAPK->getArg1Size()))
        throwError("Translation and rotation is not allowed for consistent mass formalism. Use lumped mass formalism instead.");
      // compute reduced mass matrix
      PPdm[0][0] = JTMJ(M,Phi_);
    }
    else
      throwError("Formalism unknown.");

    // compute reduced stiffness matrix
    Ke0 = JTMJ(K,Phi_);

    // visualisation
//    vector<Index> nodes(nn);
    ombvIndices.resize(5*6*ne);
//    for(size_t i=0; i<nn; i++)
//      nodes[i] = i;
//    setOpenMBVNodes(nodes);
    int j = 0;
    for(int i=0; i<eles.rows(); i++) {
      ombvIndices[j++] = eles(i,3);
      ombvIndices[j++] = eles(i,2);
      ombvIndices[j++] = eles(i,1);
      ombvIndices[j++] = eles(i,0);
      ombvIndices[j++] = -1;
      ombvIndices[j++] = eles(i,4);
      ombvIndices[j++] = eles(i,5);
      ombvIndices[j++] = eles(i,6);
      ombvIndices[j++] = eles(i,7);
      ombvIndices[j++] = -1;
      ombvIndices[j++] = eles(i,1);
      ombvIndices[j++] = eles(i,2);
      ombvIndices[j++] = eles(i,6);
      ombvIndices[j++] = eles(i,5);
      ombvIndices[j++] = -1;
      ombvIndices[j++] = eles(i,2);
      ombvIndices[j++] = eles(i,3);
      ombvIndices[j++] = eles(i,7);
      ombvIndices[j++] = eles(i,6);
      ombvIndices[j++] = -1;
      ombvIndices[j++] = eles(i,4);
      ombvIndices[j++] = eles(i,7);
      ombvIndices[j++] = eles(i,3);
      ombvIndices[j++] = eles(i,0);
      ombvIndices[j++] = -1;
      ombvIndices[j++] = eles(i,0);
      ombvIndices[j++] = eles(i,1);
      ombvIndices[j++] = eles(i,5);
      ombvIndices[j++] = eles(i,4);
      ombvIndices[j++] = -1;
    }

    isRes.close();
    isStiff.close();
    isMass.close();
    isDOF.close();
  }

  void CalculixBody::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef)
      importData();
    else if(stage==plotting) {
      if(plotFeature[openMBV] and ombvBody) {
        std::shared_ptr<OpenMBV::DynamicIndexedFaceSet> faceset = ombvBody->createOpenMBV();
        faceset->setIndices(ombvIndices);
        openMBVBody = faceset;
        ombvColorRepresentation = static_cast<OpenMBVFlexibleBody::ColorRepresentation>(ombvBody->getColorRepresentation());
      }
    }
    GenericFlexibleFfrBody::init(stage, config);
  }

  void CalculixBody::initializeUsingXML(DOMElement *element) {
    GenericFlexibleFfrBody::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"resultFileName");
    string str = X()%E(e)->getFirstTextChild()->getData();
    setResultFileName(E(e)->convertPath(str.substr(1,str.length()-2)).string());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"formalism");
    if(e) {
      string formalismStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(formalismStr=="consistentMass") formalism=consistentMass;
      else if(formalismStr=="lumpedMass") formalism=lumpedMass;
      else formalism=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBV");
    if(e) {
      ombvBody = shared_ptr<OpenMBVCalculixBody>(new OpenMBVCalculixBody);
      ombvBody->initializeUsingXML(e);
    }
  }

}
