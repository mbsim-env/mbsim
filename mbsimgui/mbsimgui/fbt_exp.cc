/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2022 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "wizards.h"
#include "basic_widgets.h"
#include "hdf5serie/file.h"
#include "hdf5serie/simpledataset.h"

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  void FlexibleBodyTool::exp() {
    if(Pdm.cols()) {
      QString fileName = static_cast<FileWidget*>(static_cast<LastPage*>(page(PageLast))->inputFile->getWidget())->getFile(true);
      if(not(fileName.isEmpty())) {
	fileName = fileName.endsWith(".h5")?fileName:fileName+".h5";
	H5::File file(fileName.toStdString(), H5::File::write);
	auto sdata=file.createChildObject<H5::SimpleDataset<double>>("mass")();
	sdata->write(m);
	auto vdata=file.createChildObject<H5::SimpleDataset<vector<double>>>("position integral")(rdm.size());
	vdata->write((vector<double>)rdm);
	auto mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("position position integral")(rrdm.rows(),rrdm.cols());
	mdata->write((vector<vector<double>>)rrdm);
	mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("shape function integral")(Pdm.rows(),Pdm.cols());
	mdata->write((vector<vector<double>>)Pdm);
	mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("stiffness matrix")(Ke0.rows(),Ke0.cols());
	mdata->write((vector<vector<double>>)Ke0);
	if(De0.size()) {
	  mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("damping matrix")(De0.rows(),De0.cols());
	  mdata->write((vector<vector<double>>)De0);
	}
	vector<vector<double>> rPdm_(3*3,vector<double>(Pdm.cols()));
	for(int i=0; i<3; i++) {
	  for(int j=0; j<Pdm.cols(); j++) {
	    rPdm_[0*3+i][j] = rPdm[0](i,j);
	    rPdm_[1*3+i][j] = rPdm[1](i,j);
	    rPdm_[2*3+i][j] = rPdm[2](i,j);
	  }
	}
	mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("position shape function integral")(rPdm_.size(),rPdm_[0].size());
	mdata->write((vector<vector<double>>)rPdm_);
	vector<vector<double>> PPdm_(6*Pdm.cols(),vector<double>(Pdm.cols()));
	for(int i=0; i<Pdm.cols(); i++) {
	  for(int j=0; j<Pdm.cols(); j++) {
	    PPdm_[0*Pdm.cols()+i][j] = PPdm[0][0](i,j);
	    PPdm_[1*Pdm.cols()+i][j] = PPdm[0][1](i,j);
	    PPdm_[2*Pdm.cols()+i][j] = PPdm[0][2](i,j);
	    PPdm_[3*Pdm.cols()+i][j] = PPdm[1][1](i,j);
	    PPdm_[4*Pdm.cols()+i][j] = PPdm[1][2](i,j);
	    PPdm_[5*Pdm.cols()+i][j] = PPdm[2][2](i,j);
	  }
	}
	mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("shape function shape function integral")(PPdm_.size(),PPdm_[0].size());
	mdata->write((vector<vector<double>>)PPdm_);

	vector<double> r(3*nN);
	vector<vector<double>> Phi_(3*nN,vector<double>(Pdm.cols()));
	for(int i=0; i<nN; i++) {
	  for(int j=0; j<3; j++) {
	    r[i*3+j] = KrKP[i](j);
	    for(int k=0; k<Pdm.cols(); k++)
	      Phi_[i*3+j][k] = Phi[i](j,k);
	  }
	}

	vdata=file.createChildObject<H5::SimpleDataset<vector<double>>>("nodal relative position")(r.size());
	vdata->write(r);
	mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("nodal shape matrix of translation")(Phi_.size(),Phi_[0].size());
	mdata->write(Phi_);

	if(Psi.size()) {
	  vector<vector<double>> Psi_(3*nN,vector<double>(Pdm.cols()));
	  for(int i=0; i<nN; i++) {
	    for(int j=0; j<3; j++) {
	      for(int k=0; k<Pdm.cols(); k++)
		Psi_[i*3+j][k] = Psi[i](j,k);
	    }
	  }
	  mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("nodal shape matrix of rotation")(Psi_.size(),Psi_[0].size());
	  mdata->write(Psi_);
	}

	if(sigmahel.size()) {
	  vector<vector<double>> sigmahel_(6*nN,vector<double>(Pdm.cols()));
	  for(int i=0; i<nN; i++) {
	    for(int j=0; j<6; j++) {
	      for(int k=0; k<Pdm.cols(); k++) {
		sigmahel_[i*6+j][k] = sigmahel[i](j,k);
	      }
	    }
	  }
	  mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("nodal stress matrix")(sigmahel_.size(),sigmahel_[0].size());
	  mdata->write(sigmahel_);
	}

	if(indices.size()) {
	  auto vidata=file.createChildObject<H5::SimpleDataset<vector<int>>>("openmbv indices")(indices.size());
	  vidata->write(indices);
	}
      }
    }
  }

}
