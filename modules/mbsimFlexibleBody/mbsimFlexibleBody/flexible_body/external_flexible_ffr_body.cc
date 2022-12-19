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
#include "external_flexible_ffr_body.h"
#include "hdf5serie/simpledataset.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, ExternalFlexibleFfrBody)

  list<string>::iterator findChild(list<string> &names, const string &name) {
    for(list<string>::iterator it=names.begin(); it!=names.end(); it++) {
      if(*it==name)
	return it;
    }
    return list<string>::iterator();
  }

  void ExternalFlexibleFfrBody::importData() {
    H5::File file(inputDataFile, H5::File::read);
    auto names = file.getChildObjectNames();
    auto it = findChild(names,"mass");
    if(it!=list<string>::iterator()) {
      m = file.openChildObject<H5::SimpleDataset<double>>("mass")->read();
      names.erase(it);
    }
    else
      throwError("(ExternalFlexibleFfrBody::init): keyword \"mass\" not found in input data file.");

    it = findChild(names,"position integral");
    if(it!=list<string>::iterator()) {
      rdm = Vec3(file.openChildObject<H5::SimpleDataset<vector<double>>>("position integral")->read());
      names.erase(it);
    }
    else
      throwError("(ExternalFlexibleFfrBody::init): keyword \"position integral\" not found in input data file.");

    it = findChild(names,"position position integral");
    if(it!=list<string>::iterator()) {
      rrdm = SymMat3(file.openChildObject<H5::SimpleDataset<vector<vector<double>>>>("position position integral")->read());
      names.erase(it);
    }
    else
      throwError("(ExternalFlexibleFfrBody::init): keyword \"position position integral\" not found in input data file.");

    it = findChild(names,"shape function integral");
    if(it!=list<string>::iterator()) {
      Pdm <<= Mat3xV(file.openChildObject<H5::SimpleDataset<vector<vector<double>>>>("shape function integral")->read());
      names.erase(it);
    }
    else
      throwError("(ExternalFlexibleFfrBody::init): keyword \"shape function integral\" not found in input data file.");

    it = findChild(names,"stiffness matrix");
    if(it!=list<string>::iterator()) {
      Ke0 <<= SymMatV(file.openChildObject<H5::SimpleDataset<vector<vector<double>>>>("stiffness matrix")->read());
      names.erase(it);
    }
    else
      throwError("(ExternalFlexibleFfrBody::init): keyword \"stiffness matrix\" not found in input data file.");

    it = findChild(names,"damping matrix");
    if(it!=list<string>::iterator()) {
      De0 <<= SymMatV(file.openChildObject<H5::SimpleDataset<vector<vector<double>>>>("damping matrix")->read());
      names.erase(it);
    }
    else
      De0.resize(Ke0.size());

    it = findChild(names,"position shape function integral");
    if(it!=list<string>::iterator()) {
      vector<vector<double>> rPdm_ = file.openChildObject<H5::SimpleDataset<vector<vector<double>>>>("position shape function integral")->read();
      rPdm.resize(3,Mat3xV(Pdm.cols(),NONINIT));
      for(int i=0; i<3; i++) {
	//mdata=file.openChildObject<H5::SimpleDataset<vector<vector<double>>>>("rPdm"+to_string(i));
	//rPdm[i] <<= Mat3xV(mdata->read());
	for(int j=0; j<Pdm.cols(); j++) {
	  rPdm[0](i,j) = rPdm_[0*3+i][j];
	  rPdm[1](i,j) = rPdm_[1*3+i][j];
	  rPdm[2](i,j) = rPdm_[2*3+i][j];
	}
      }
      names.erase(it);
    }
    else
      throwError("(ExternalFlexibleFfrBody::init): keyword \"position shape function integral\" not found in input data file.");

    it = findChild(names,"shape function shape function integral");
    if(it!=list<string>::iterator()) {
      vector<vector<double>> PPdm_ = file.openChildObject<H5::SimpleDataset<vector<vector<double>>>>("shape function shape function integral")->read();
      PPdm.resize(3,vector<SqrMatV>(3,SqrMatV(Pdm.cols(),NONINIT)));
      for(int i=0; i<Pdm.cols(); i++) {
	for(int j=0; j<Pdm.cols(); j++) {
	  PPdm[0][0](i,j) = PPdm_[0*Pdm.cols()+i][j];
	  PPdm[0][1](i,j) = PPdm_[1*Pdm.cols()+i][j];
	  PPdm[0][2](i,j) = PPdm_[2*Pdm.cols()+i][j];
	  PPdm[1][1](i,j) = PPdm_[3*Pdm.cols()+i][j];
	  PPdm[1][2](i,j) = PPdm_[4*Pdm.cols()+i][j];
	  PPdm[2][2](i,j) = PPdm_[5*Pdm.cols()+i][j];
	  PPdm[1][0](j,i) = PPdm[0][1](i,j);
	  PPdm[2][0](j,i) = PPdm[0][2](i,j);
	  PPdm[2][1](j,i) = PPdm[1][2](i,j);
	}
      }
      names.erase(it);
    }
    else
      throwError("(ExternalFlexibleFfrBody::init): keyword \"shape function shape function integral\" not found in input data file.");

    it = findChild(names,"nodal relative position");
    if(it!=list<string>::iterator()) {
      KrKP = getCellArray1D<fmatvec::Vec3>(3,VecV(file.openChildObject<H5::SimpleDataset<vector<double>>>("nodal relative position")->read()));
      names.erase(it);
    }

    it = findChild(names,"nodal shape matrix of translation");
    if(it!=list<string>::iterator()) {
      Phi = getCellArray1D<fmatvec::Mat3xV>(3,MatV(file.openChildObject<H5::SimpleDataset<vector<vector<double>>>>("nodal shape matrix of translation")->read()));
      names.erase(it);
    }

    it = findChild(names,"nodal shape matrix of rotation");
    if(it!=list<string>::iterator()) {
      Psi = getCellArray1D<fmatvec::Mat3xV>(3,MatV(file.openChildObject<H5::SimpleDataset<vector<vector<double>>>>("nodal shape matrix of rotation")->read()));
      names.erase(it);
    }

    it = findChild(names,"nodal stress matrix");
    if(it!=list<string>::iterator()) {
      sigmahel = getCellArray1D<Matrix<General,Fixed<6>,Var,double>>(6,MatV(file.openChildObject<H5::SimpleDataset<vector<vector<double>>>>("nodal stress matrix")->read()));
      names.erase(it);
    }

    it = findChild(names,"openmbv indices");
    if(it!=list<string>::iterator()) {
      ombvIndices = file.openChildObject<H5::SimpleDataset<vector<int>>>("openmbv indices")->read();
      names.erase(it);
    }

    it = findChild(names,"node numbers");
    if(it!=list<string>::iterator()) {
      nodeNumbers = file.openChildObject<H5::SimpleDataset<vector<int>>>("node numbers")->read();
      names.erase(it);
    }
  }

  void ExternalFlexibleFfrBody::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(inputDataFile.empty()) 
	throwError("(ExternalFlexibleFfrBody::init): Input data file must be defined.");
      importData();
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] and ombvBody) {
        std::shared_ptr<OpenMBV::FlexibleBody> flexbody = ombvBody->createOpenMBV();
        openMBVBody = flexbody;
        if(ombvBody->getVisualization()==OpenMBVExternalFlexibleFfrBody::faces)
	  static_pointer_cast<OpenMBV::DynamicIndexedFaceSet>(flexbody)->setIndices(ombvIndices);
	else if(ombvBody->getVisualization()==OpenMBVExternalFlexibleFfrBody::lines)
	  static_pointer_cast<OpenMBV::DynamicIndexedLineSet>(flexbody)->setIndices(ombvIndices);
        ombvColorRepresentation = static_cast<OpenMBVFlexibleBody::ColorRepresentation>(ombvBody->getColorRepresentation());
      }
    }
    GenericFlexibleFfrBody::init(stage, config);
  }

  void ExternalFlexibleFfrBody::initializeUsingXML(DOMElement *element) {
    GenericFlexibleFfrBody::initializeUsingXML(element);

    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"inputDataFileName");
    string str = X()%E(e)->getFirstTextChild()->getData();
    setInputDataFile(E(e)->convertPath(str.substr(1,str.length()-2)).string());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBV");
    if(e) {
      ombvBody = shared_ptr<OpenMBVExternalFlexibleFfrBody>(new OpenMBVExternalFlexibleFfrBody);
      ombvBody->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"openMBVNodeNumbers");
    if(e) setOpenMBVNodeNumbers(E(e)->getText<vector<int>>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"plotNodeNumbers");
    if(e) setPlotNodeNumbers(E(e)->getText<vector<int>>());
  }

}
