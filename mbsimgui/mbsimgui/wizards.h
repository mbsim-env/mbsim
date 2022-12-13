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

#ifndef _WIZARDS_H_
#define _WIZARDS_H_

#include <QWizard>
#include "fmatvec/fmatvec.h"

class QRadioButton; 

namespace MBSimGUI {

  class ExtWidget;
  class FiniteElementType;

  class FirstPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
      FirstPage(QWidget *parent);
      int nextId() const override;
      void setVisible(bool visible) override;
      bool isComplete() const override;
    private:
      QRadioButton *rb1, *rb2, *rb3, *rb4;
  };

  class LastPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
      LastPage(QWidget *parent);
      QString getFile() const;
      int nextId() const override;
      void setVisible(bool visible) override;
    private:
      ExtWidget *inputFile;
  };

  class ExternalFiniteElementsPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
      ExternalFiniteElementsPage(QWidget *parent);
      int nextId() const override;
      bool isComplete() const override;
    private:
      ExtWidget *nodes, *mass, *stiff;
  };

  class CalculixPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
      CalculixPage(QWidget *parent);
      int nextId() const override;
    private:
      ExtWidget *file;
  };

  class FlexibleBeamPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
     FlexibleBeamPage(QWidget *parent);
      int nextId() const override;
    private:
      ExtWidget *n, *l, *A, *I, *E, *rho, *ten, *beny, *benz, *tor;
  };

  class FiniteElementsPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
     FiniteElementsPage(QWidget *parent);
      int nextId() const override;
    private:
      ExtWidget *E, *rho, *nu, *nodes, *elements, *exs;
  };

  class ReductionMethodsPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
      ReductionMethodsPage(QWidget *parent);
      int nextId() const override;
    private:
      QRadioButton *rb1, *rb2;
  };

  class BoundaryConditionsPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
      BoundaryConditionsPage(QWidget *parent);
      int nextId() const override;
    private:
      ExtWidget *bc;
  };

  class ModeShapesPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
      ModeShapesPage(QWidget *parent);
      int nextId() const override;
    private:
      ExtWidget *V, *S;
  };

  class ComponentModeSynthesisPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
      ComponentModeSynthesisPage(QWidget *parent);
      int nextId() const override;
    private:
      ExtWidget *inodes, *nmodes, *fbnm;
  };

  class OpenMBVPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
      OpenMBVPage(QWidget *parent);
      int nextId() const override;
    private:
      ExtWidget *ombvIndices;
  };

  class DampingPage : public QWizardPage {
    friend class FlexibleBodyTool;
    public:
      DampingPage(QWidget *parent);
      int nextId() const override;
    private:
      ExtWidget *mDamp, *pDamp;
  };

  class FlexibleBodyTool : public QWizard {
    public:
      FlexibleBodyTool(QWidget *parent);
      enum {
	PageFirst,
	PageExtFE,
	PageCalculix,
	PageFlexibleBeam,
	PageFiniteElements,
	PageRedMeth,
	PageBC,
	PageCMS,
	PageModeShapes,
	PageOMBV,
	PageDamp,
       	PageLast,
      };
      void create();
      void save();
      void load();
      QString getInputDataFile() const;
    private:
      static fmatvec::MatV readMat(const std::string &file);
      fmatvec::SymSparseMat createSymSparseMat(const std::vector<std::map<int,double>> &Am);
      fmatvec::SparseMat createSparseMat(int n, const std::vector<std::map<int,double>> &Am);
      std::vector<std::map<int,double>> reduceMat(const std::vector<std::map<int,double>> &Am, const fmatvec::Indices &iF);
      fmatvec::MatV reduceMat(const std::vector<std::map<int,double>> &Am, const fmatvec::Indices &iN, const fmatvec::Indices &iH);
      void reduceMat(const fmatvec::SymSparseMat &Ms, const fmatvec::SymSparseMat &Ks, fmatvec::SymSparseMat &Mrs, fmatvec::SymSparseMat &Krs, int n, const std::vector<std::vector<int>> &activeDof, const std::vector<int> &dofMap);
      fmatvec::MatV reduceMat(const fmatvec::SymSparseMat &Ks, const fmatvec::Indices &iN, const fmatvec::Indices &iH, const std::vector<std::vector<int>> &activeDof, const std::vector<int> &dofMapN, const std::vector<int> &dofMapH);
      void extfe();
      void calculix();
      void beam();
      void fe();
      void cms();
      void msm();
      void ombv();
      void fma();
      void lma();
      void damp();
      void exp();
      double m;
      fmatvec::MatV M, K, U, S;
      fmatvec::VecV mDamp;
      fmatvec::Vec2 pDamp;
      fmatvec::SymMatV Ke0, De0;
      fmatvec::Vec3 rdm;
      fmatvec::SymMat3 rrdm;
      fmatvec::Mat3xV Pdm;
      std::vector<fmatvec::Mat3xV> rPdm;
      std::vector<std::vector<fmatvec::SqrMatV>> PPdm;
      std::vector<fmatvec::Vec3> KrKP;
      std::vector<fmatvec::Mat3xV> Phi, Psi;
      std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>> sigmahel;
      std::vector<int> nodeTable, nodeCount;
      std::vector<int> indices;
      std::vector<int> nodeNumbers;
      int nN, nM, ng, net, ner, nen;
      std::vector<std::map<int,double>> Km, Mm;
      std::vector<fmatvec::MatVI> ele;
      std::vector<FiniteElementType*> type;
      fmatvec::SymSparseMat PPdms[3], Ks;
      fmatvec::SparseMat PPdm2s[3];
      std::vector<fmatvec::SparseMat> Phis, Psis, sigs;
      std::vector<std::map<int,int>> links;
  };

}

#endif
