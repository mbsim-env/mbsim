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
#include <mbxmlutilshelper/dom.h>

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

  class EvalDialog : public QDialog {
    public:
      EvalDialog(const std::vector<std::vector<QString>> &var_, int type_, QWidget *parent);
    private:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      void formatVariables();
      void updateWidget();
      std::vector<std::vector<QString>> var, varf;
      int type;
      QComboBox *format;
      QSpinBox *precision;
      QTableWidget *tab { nullptr };
  };

  class BasicElementBrowser : public QDialog {
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

  class SourceCodeDialog : public QDialog {
    public:
      SourceCodeDialog(const QString &text, bool readOnly, QWidget *parent);
      void highlightLine(int n);
      XMLEditorWidget *getEditor() { return xmlEditor; }
    private:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      XMLEditorWidget *xmlEditor;
  };

  class StateTableDialog : public QDialog {
    public:
      StateTableDialog(QWidget *parent);
      void updateWidget();
    private:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      QTreeWidget *stateTable;
  };

  class LoadModelDialog : public QDialog {
    public:
      LoadModelDialog(const QString &title);
      QString getModelFileName() const;
      QString getParameterFileName() const;
      bool referenceModel() const;
      bool referenceParameter() const;
      bool getAbsoluteModelFilePath() const;
      bool getAbsoluteParameterFilePath() const;
    private:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      void modelFileChanged(const QString &fileName);
      ExtWidget *modelFile, *parameterFile, *e;
      QButtonGroup *mOpt, *pOpt;
  };

  class SaveModelDialog : public QDialog {
    public:
      SaveModelDialog(const QString &title, const QString &name, bool param);
      QString getModelFileName() const;
      QString getParameterFileName() const;
    private:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      void modelFileChanged(const QString &fileName);
      ExtWidget *modelFile, *parameterFile;
  };

  class LoadParameterDialog : public QDialog {
    public:
      LoadParameterDialog();
      QString getParameterFileName() const;
      bool referenceParameter() const;
      bool getAbsoluteFilePath() const;
    private:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      ExtWidget *parameterFile;;
      QButtonGroup *pOpt;
  };

  class SaveParameterDialog : public QDialog {
    public:
      SaveParameterDialog(const QString &name);
      QString getParameterFileName() const;
    private:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
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

  class LinearSystemAnalysisDialog : public QDialog {
    public:
      LinearSystemAnalysisDialog(QWidget *parent);
      void updateWidget();
    private:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      ModalAnalysisWidget *mawidget;
      FrequencyResponseWidget *frwidget;
      InitialOutputWidget *iowidget;
      EigenanalysisWidget *eawidget;
  };

  class CreateFMUDialog : public QDialog {
    public:
      CreateFMUDialog(const QString &name);
      QString getFileName() const;
      bool cosim() const;
      bool nocompress() const;
    private:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      ExtWidget *file;
      QButtonGroup *opt;
      QCheckBox *checkbox;
  };

  class TextEditDialog : public QDialog {
    public:
      TextEditDialog(const QString &title="", const QString &text="", bool readOnly=true, QWidget *parent=nullptr);
      QString getText() const;
      void setText(const QString &text);
      void appendText(const QString &text);
      void gotoLine(int n);
    private:
      void reset();
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      QTextEdit *editor;
  };

  class PlotFeatureDialog : public QDialog {
    public:
      PlotFeatureDialog(const MBXMLUtils::FQN &specialType_, QWidget *parent);
      void setType(const QString &type_);
      void setValue(const QString &value_);
      void setNamespace(const QString &ns_);
      void setStatus(const QString &status_);
      QString getType() const;
      QString getValue() const;
      QString getNamespace() const;
      QString getStatus() const;
    private:
      void updateNamespace();
      void reset();
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      std::vector<MBXMLUtils::FQN> feature;
      MBXMLUtils::FQN specialType;
      ExtWidget *type, *value, *ns, *status;
  };

  class StateDialog : public QDialog {
    public:
      StateDialog(QWidget *parent);
      void setName(const QString &name_);
      void setValue(const QString &value_);
      QString getName() const;
      QString getValue() const;
    private:
      void reset();
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      ExtWidget *name, *value;
  };

  class TransitionDialog : public QDialog {
    public:
      TransitionDialog(Element *element, QWidget *parent);
      void setSource(const QString &src_);
      void setDestination(const QString &dest_);
      void setSignal(const QString &sig_);
      void setThreshold(const QString &th_);
      QString getSource() const;
      QString getDestination() const;
      QString getSignal() const;
      QString getThreshold() const;
      void setStringList(const std::vector<QString> &list);
    private:
      void reset();
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      ExtWidget *src, *dest, *sig, *th;
  };

  class LineEditDialog : public QDialog {
    public:
      LineEditDialog(const QString &title="", const QString &text="", QWidget *parent=nullptr);
      QString getText() const;
      void setText(const QString &text);
    private:
      void reset();
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      QLineEdit *editor;
  };

}

#endif
