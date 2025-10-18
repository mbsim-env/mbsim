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
#include "mbxmlutils/eval.h"
#include <xercesc/util/XercesDefs.hpp>
#include <mbxmlutilshelper/dom.h>

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

class QRadioButton; 

namespace MBSimGUI {

  class ExtWidget;
  class FiniteElementType;

  class WizardPage : public QWizardPage {
    public:
      WizardPage(QWidget *parent=nullptr) : QWizardPage(parent) { }
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) { return element; }
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) { return nullptr; }
  };

  class FirstPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      FirstPage(QWidget *parent);
      int nextId() const override;
      void setVisible(bool visible) override;
      bool isComplete() const override;
    private:
      QRadioButton *rb[4];
  };

  class LastPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      LastPage(QWidget *parent);
      QString getFile() const;
      int nextId() const override;
      void setVisible(bool visible) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *inputFile;
  };

  class ExternalFiniteElementsPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      ExternalFiniteElementsPage(QWidget *parent);
      int nextId() const override;
      bool isComplete() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *nodes, *mass, *stiff;
  };

  class CalculixPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      CalculixPage(QWidget *parent);
      int nextId() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *file;
  };

  class FlexibleBeamPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      FlexibleBeamPage(QWidget *parent);
      int nextId() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *n, *l, *A, *I, *E, *rho, *ten, *beny, *benz, *tor;
  };

  class FiniteElementsPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
     FiniteElementsPage(QWidget *parent);
      int nextId() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *E, *rho, *nu, *nodes, *elements, *exs;
  };

  class ReductionMethodsPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      ReductionMethodsPage(QWidget *parent);
      int nextId() const override;
    private:
      QRadioButton *rb[2];
  };

  class BoundaryConditionsPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      BoundaryConditionsPage(QWidget *parent);
      int nextId() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *bc;
  };

  class ModeShapesPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      ModeShapesPage(QWidget *parent);
      int nextId() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *V, *S;
  };

  class ComponentModeSynthesisPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      ComponentModeSynthesisPage(QWidget *parent);
      int nextId() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *typeOfConstraint, *idata, *nmodes, *normalModes;
  };

  class RemoveRigidBodyModesPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      RemoveRigidBodyModesPage(QWidget *parent);
      int nextId() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
   private:
      ExtWidget *rrbm, *nrb, *ft;
  };

  class OpenMBVPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      OpenMBVPage(QWidget *parent);
      int nextId() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *ombvIndices;
  };

  class DampingPage : public WizardPage {
    friend class FlexibleBodyTool;
    public:
      DampingPage(QWidget *parent);
      int nextId() const override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *mDamp, *pDamp;
  };

  class Wizard : public QWizard {
    public:
      Wizard(QWidget *parent=nullptr) : QWizard(parent) { }

      template<class PageType>
      PageType* page(int i) const {
#ifdef NDEBUG
        return static_cast<PageType*>(QWizard::page(i));
#else
        auto *pageType = dynamic_cast<PageType*>(QWizard::page(i));
        assert(pageType);
        return pageType;
#endif
      }
    protected:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
  };

  class FlexibleBodyTool : public Wizard {
    public:
      enum TypeOfConstraint {
	distributing=0,
	kinematic
      };
      enum NormalModes {
	freeBoundaryNormalModes=0,
	fixedBoundaryNormalModes,
	constrainedBoundaryNormalModes
      };
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
	PageRRBM,
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
      std::vector<std::map<int,double>> reduceMat(const std::vector<std::map<int,double>> &Am, int n, const fmatvec::MatVI &activeDof, const std::vector<int> &dofMap, int val=1);
      fmatvec::MatV reduceMat(const std::vector<std::map<int,double>> &Am, int m, int n, const fmatvec::MatVI &activeDof, const std::vector<int> &dofMapN, const std::vector<int> &dofMapH);
      void reduceMat(const fmatvec::SymSparseMat &Ms, const fmatvec::SymSparseMat &Ks, fmatvec::SymSparseMat &Mrs, fmatvec::SymSparseMat &Krs, int n, const fmatvec::MatVI &activeDof, const std::vector<int> &dofMap, int val=1);
      fmatvec::MatV reduceMat(const fmatvec::SymSparseMat &Ks, int m, int n, const fmatvec::MatVI &activeDof, const std::vector<int> &dofMapN, const std::vector<int> &dofMapH);
      void extfe();
      void calculix();
      void beam();
      void fe();
      void cms();
      void msm();
      void ombv();
      void fma();
      void lma();
      void rrbm();
      void damp();
      void exp();
      void createSingleInterfaceNodes();
      fmatvec::MatV M, K, U, S;
      fmatvec::VecV mDamp;
      fmatvec::Vec2 pDamp;
      fmatvec::SymMatV Ke0, De0;
      double m{0};
      fmatvec::Vec3 rdm;
      fmatvec::SymMat3 rrdm;
      fmatvec::Mat3xV Pdm;
      std::vector<fmatvec::Mat3xV> rPdm;
      std::vector<std::vector<fmatvec::SqrMatV>> PPdm;
      std::vector<fmatvec::Vec3> r;
      std::vector<fmatvec::Mat3xV> Phi, Psi;
      std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>> sigmahel;
      fmatvec::SymSparseMat PPdms[3], Ks;
      fmatvec::SparseMat PPdm2s[3];
      std::vector<fmatvec::SparseMat> Phis, Psis, sigs;
      std::vector<std::map<int,double>> Mm, Km;
      std::vector<int> nodeTable, nodeCount, nodeNumbers, indices, singleNodeNumbers;
      int net, ner;
      std::vector<fmatvec::MatVI> ele;
      std::vector<FiniteElementType*> type;
      std::vector<std::map<int,int>> links;
      std::vector<fmatvec::Vec3> rif;
      std::vector<fmatvec::Mat3xV> Phiif, Psiif;
      std::vector<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>> sigmahelif;
      MBXMLUtils::NewParamLevel npl;
  };

}

#endif
