/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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

#ifndef _DIALOGS_H_
#define _DIALOGS_H_

#include <QDialog>
#include <QPushButton>
#include <QTreeWidgetItem>
#include <QCheckBox>
#include <QUrl>
#include <xercesc/util/XercesDefs.hpp>

class QTableWidget;
class QSpinBox;
class QComboBox;
class QwtPlot;
class QwtPlotCurve;
class QTextEdit;

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMDocument;
}

namespace MBSimGUI {

  class TreeItemData;
  class Element;
  class DataPlot;
  class ExtWidget;
  class XMLEditorWidget;

  class ElementItem : public QTreeWidgetItem {
    private:
      Element* element;
    public:
      ElementItem(Element *element_) : element(element_) { }
      Element* getElement() const { return element; }
  };

  class Dialog : public QDialog {
    public:
      Dialog(QWidget *parent=nullptr) : QDialog(parent) { }
    protected:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
  };

  class EvalDialog : public Dialog {
    public:
      EvalDialog(const std::vector<std::vector<QString>> &var_, int type_, QWidget *parent);
    private:
      void formatVariables();
      void updateWidget();
      std::vector<std::vector<QString>> var, varf;
      int type;
      QComboBox *format;
      QSpinBox *precision;
      QTableWidget *tab;
  };

  class BasicElementBrowser : public Dialog {
    public:
      BasicElementBrowser(Element* selection_, const QString &name, QWidget *parent);
      ~BasicElementBrowser() override = default;
      void setSelection(Element *selection_) { selection = selection_; }
      virtual Element* getSelection() const { return nullptr; }
    protected:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      virtual bool checkForElement(TreeItemData *element) { return false; }
      void selectionChanged(const QModelIndex &current);
      QPushButton *okButton;
      QTreeView *eleList;
      Element *selection;
      std::string oldID;
  };

  template <class T>
  class ElementBrowser : public BasicElementBrowser {
    public:
      ElementBrowser(Element* selection, QWidget *parent) : BasicElementBrowser(selection,T::getTypeStatic(),parent) { }
      Element* getSelection() const override { return dynamic_cast<T*>(selection); }
    protected:
      bool checkForElement(TreeItemData *element) override { return dynamic_cast<T*>(element); }
  };

  class SourceCodeDialog : public Dialog {
    public:
      SourceCodeDialog(const QString &text, bool readOnly, QWidget *parent);
      void highlightLine(int n);
    private:
      XMLEditorWidget *xmlEditor;
  };

  class StateTableDialog : public Dialog {
    public:
      StateTableDialog(QWidget *parent);
      void updateWidget();
    private:
      QTreeWidget *stateTable;
  };

  class LoadModelDialog : public Dialog {
    public:
      LoadModelDialog(const QString &title);
      QString getModelFileName() const;
      QString getParameterFileName() const;
      bool referenceModel() const;
      bool referenceParameter() const;
      bool getAbsoluteModelFilePath() const;
      bool getAbsoluteParameterFilePath() const;
    private:
      ExtWidget *modelFile, *parameterFile, *e;
      QButtonGroup *mOpt, *pOpt;
      void modelFileChanged(const QString &fileName);
  };

  class SaveModelDialog : public Dialog {
    public:
      SaveModelDialog(const QString &title, const QString &name, bool param);
      QString getModelFileName() const;
      QString getParameterFileName() const;
    private:
      ExtWidget *modelFile, *parameterFile;
      void modelFileChanged(const QString &fileName);
  };

  class LoadParameterDialog : public Dialog {
    public:
      LoadParameterDialog();
      QString getParameterFileName() const;
      bool referenceParameter() const;
      bool getAbsoluteFilePath() const;
    private:
      ExtWidget *parameterFile;;
      QButtonGroup *pOpt;
  };

  class SaveParameterDialog : public Dialog {
    public:
      SaveParameterDialog(const QString &name);
      QString getParameterFileName() const;
    private:
      ExtWidget *parameterFile;;
  };

  class InitialOutputWidget : public QWidget {
    public:
      InitialOutputWidget();
      void loadData();
    private:
      QTextEdit *text;
  };

  class EigenanalysisWidget : public QWidget {
    public:
      EigenanalysisWidget();
      void loadData();
    private:
      QTextEdit *text;
  };

  class ModalAnalysisWidget : public QWidget {
    public:
      ModalAnalysisWidget();
      void loadData();
    private:
      QTreeWidget *modeTable, *elementTable;
      QComboBox *choice;
      QwtPlot *plot;
      QwtPlotCurve *curve1, *curve2;
      QMap<QString,QVector<double>> num;
      QMap<QString,QVector<QVector<double>>> A, phi;
      QVector<QString> stateName, outputName;
      QVector<QString> stateLabel, outputLabel;
      QVector<int> stateLabelNumber, outputLabelNumber;
      void updateWidget();
  };

  class FrequencyResponseWidget : public QWidget {
    public:
      FrequencyResponseWidget();
      void loadData();
    private:
      QTreeWidget *inputTable, *table;
      QwtPlot *plot;
      QwtPlotCurve *curve1, *curve2;
      QVector<double> freq;
      QVector<QVector<double>> A;
      QVector<QVector<double>> phi;
      QVector<QString> stateName, inputName, outputName;
      QVector<QString> stateLabel, inputLabel, outputLabel;
      QVector<int> stateLabelNumber, inputLabelNumber, outputLabelNumber;
      void updateWidget();
  };

  class LinearSystemAnalysisDialog : public Dialog {
    public:
      LinearSystemAnalysisDialog(QWidget *parent);
      void updateWidget();
    private:
      ModalAnalysisWidget *mawidget;
      FrequencyResponseWidget *frwidget;
      InitialOutputWidget *iowidget;
      EigenanalysisWidget *eawidget;
  };

  class CreateFMUDialog : public Dialog {
    public:
      CreateFMUDialog(const QString &name);
      QString getFileName() const;
      bool cosim() const;
      bool nocompress() const;
    private:
      ExtWidget *file;
      QButtonGroup *opt;
      QCheckBox *checkbox;
  };

}

#endif
