/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef _IMPORT_H_
#define _IMPORT_H_

#include <fmatvec/fmatvec.h>

namespace MBSimGUI {

  class ImportFEMData {
    public:
      void readDisplacements();
      void readStresses();
      void readNodes();
      void readElements();
      void readModes();
      void readDOF();
      void readStiffMatrix();
      void readMassMatrix();
      void read(const std::string &jobname);
      void setNumberOfModes(int nm_) { nm = nm_; }
      int getNumberOfModes() const { return nm; }
      double getm() const { return m(0); }
      const fmatvec::Vec3& getrdm() const { return rdm; }
      const fmatvec::SymMat3& getrrdm() const { return rrdm; }
      const fmatvec::Mat3xV& getPdm() const { return Pdm; }
      const fmatvec::VecV& getu0() const { return u0; }
      const fmatvec::MatV& getPhi() const { return Phi; }
      const fmatvec::MatV& getSr() const { return Sr; }
      const fmatvec::SymMatV& getKe() const { return Ke; }
      const fmatvec::MatV& getrPdm() const { return rPdm; }
      const fmatvec::MatV& getPPdm() const { return PPdm; }
      fmatvec::Vector<fmatvec::Var,int> getNodes() const { return nodes; }
      fmatvec::Vector<fmatvec::Var,int> getIndices() const { return indices; }
    private:
      std::ifstream isRes, isStiff, isMass, isDOF;
      int nn{0}, ne{0}, nm{1000};
      fmatvec::VecV u0, mij, m;
      fmatvec::Matrix<fmatvec::General,fmatvec::Var,fmatvec::Var,int> eles;
      fmatvec::MatV Phi, Sr, rPdm, PPdm;
      std::vector<std::pair<int,int>> dof;
      fmatvec::SymMatV K, M, Ke;
      fmatvec::Vec3 rdm;
      fmatvec::SymMat3 rrdm;
      fmatvec::Mat3xV Pdm;
      fmatvec::Vector<fmatvec::Var,int> nodes, indices;
      std::vector<fmatvec::VecV> disp;
      std::vector<fmatvec::VecV> stress;
  };

}

#endif
