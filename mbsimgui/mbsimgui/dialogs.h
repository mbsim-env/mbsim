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
#include <xercesc/util/XercesDefs.hpp>

class QTableWidget;
class QSpinBox;
class QComboBox;

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMDocument;
}

namespace MBSimGUI {

  class TreeItemData;
  class Element;
  class DataPlot;

  class ElementItem : public QTreeWidgetItem {
    private:
      Element* element;
    public:
      ElementItem(Element *element_) : element(element_) { }
      Element* getElement() const { return element; }
  };

  class EvalDialog : public QDialog {
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
      ElementBrowser(Element* selection, QWidget *parent) : BasicElementBrowser(selection,T().getType(),parent) { }
      Element* getSelection() const override { return dynamic_cast<T*>(selection); }
    protected:
      bool checkForElement(TreeItemData *element) override { return dynamic_cast<T*>(element); }
  };

  class EigenanalysisDialog : public QDialog {
    public:
      EigenanalysisDialog(QWidget *parent);
    private:
      void selectRow(int);
      void selectMode(int row, int col);
      QTableWidget *table;
      DataPlot *plot;
  };

  class HarmonicResponseDialog : public QDialog {
    public:
      HarmonicResponseDialog(QWidget *parent);
    private:
      DataPlot *plot;
  };

  class SourceDialog : public QDialog {
    public:
      SourceDialog(xercesc::DOMElement *ele, QWidget *parent);
  };

  class StateTableDialog : public QDialog {
    public:
      StateTableDialog(QWidget *parent);
  };

}

#endif
