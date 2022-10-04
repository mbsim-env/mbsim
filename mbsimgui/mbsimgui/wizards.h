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
      ExtWidget *V;
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
      void stiff();
      void calculix();
      void cms();
      void msm();
      void ombv();
      void damp();
      void exp();
      double m{0};
      fmatvec::MatV r, M, K, Phi_, Sr;
      fmatvec::VecV mDamp;
      fmatvec::Vec2 pDamp;
      fmatvec::SymMatV M0, Ke0, De0;
      std::map<int,int> nodeMap;
      std::vector<int> indices;
      int nN, nE, nM;
      int net{3};
      int ner{0};
      int nen;
  };

}

#endif
