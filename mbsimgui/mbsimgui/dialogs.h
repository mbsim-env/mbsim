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

#ifndef _DIALOGS_H_
#define _DIALOGS_H_

#include <QDialog>
#include <QPushButton>
#include <QTreeWidgetItem>
#include <QCheckBox>
#include <QUrl>

class QTableWidget;
class QSpinBox;
class QComboBox;
class QWebView;

namespace MBSimGUI {

  class TreeItemData;
  class Element;
  class DataPlot;
  class ImportWidget;

  class ElementItem : public QTreeWidgetItem {
    private:
      Element* element;
    public:
      ElementItem(Element *element_) : element(element_) { }
      Element* getElement() const {return element;}
  };

  class EvalDialog : public QDialog {
    Q_OBJECT
    public:
      //EvalDialog(VariableWidget *widget);
      EvalDialog(const std::vector<std::vector<QString> > &var_);
    private:
      std::vector<std::vector<double> > var;
//      VariableWidget *var;
      QComboBox *format;
      QSpinBox *precision;
      QTableWidget *tab;
    private slots:
      void updateWidget();
  };

  class BasicElementBrowser : public QDialog {
    Q_OBJECT

    public:
      BasicElementBrowser(TreeItemData* selection_, const QString &name, int treeIndex_=0);
      ~BasicElementBrowser() override = default;
      void setSelection(TreeItemData *selection_) { selection = selection_; }
      virtual Element* getSelection() const { return nullptr; }
    protected:
      QPushButton *okButton;
      QTreeView *eleList;
      TreeItemData *selection;
      int treeIndex;
      QString oldID;
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      virtual bool checkForElement(TreeItemData *element) { return false; }
    protected slots:
      void selectionChanged(const QModelIndex &current);
  };

  template <class T>
  class ElementBrowser : public BasicElementBrowser {
    public:
      ElementBrowser(Element* selection) : BasicElementBrowser(selection,T().getType(),T::getTreeIndex()) { }
      Element* getSelection() const override { return dynamic_cast<T*>(selection); }
    protected:
      bool checkForElement(TreeItemData *element) override { return dynamic_cast<T*>(element); }
  };

  class SaveDialog : public QDialog {
    public:
      SaveDialog(QWidget *parent=0);
      bool includeParameter() const { return parameter->checkState()==Qt::Checked; }
    private:
      QCheckBox *parameter;
  };

  class WebDialog : public QDialog {
    public:
      WebDialog(QWidget *parent=0);
      void load(const QUrl &url_);
    private:
      QWebView *webView;
      QUrl url;
  };

  class EigenanalysisDialog : public QDialog {
    Q_OBJECT
    public:
      EigenanalysisDialog(const QString &name, QWidget *parent=0);
    private:
      QTableWidget *table;
      DataPlot *plot;
    private slots:
      void selectRow(int);
      void selectMode(int row, int col);
  };

  class HarmonicResponseDialog : public QDialog {
    public:
      HarmonicResponseDialog(const QString &name, QWidget *parent=0);
    private:
      DataPlot *plot;
  };

  class SourceDialog : public QDialog {
    public:
      SourceDialog(Element *element, QWidget *parent=0);
  };

  class ImportDialog : public QDialog {
    Q_OBJECT
    public:
      ImportDialog(QWidget *parent=0);
      ImportWidget *getImportWidget() { return import; }
    private:
      ImportWidget *import;
    private slots:
      void checkFile();
  };

}

#endif
