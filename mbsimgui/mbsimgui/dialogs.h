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

  class Element;
  class Object;
  class Link;
  class Constraint;
  class RigidBody;
  class Frame;
  class Contour;
  class Signal;

  class ElementItem : public QTreeWidgetItem {
    private:
      Element* element;
    public:
      ElementItem(Element *element_) : element(element_) {}
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

  class ObjectBrowser : public QDialog {
    Q_OBJECT

    public:
      ObjectBrowser(Element* element_, Object* object, QWidget *parentObject_);
      ~ObjectBrowser() override = default;
      QTreeWidget* getObjectList() const {return objectList;}
      void updateWidget(Object *sel);
    protected:
      QPushButton *okButton;
      QTreeWidget *objectList;
      Object *selection;
      ElementItem *savedItem;
      Element* element;
      QString oldID;
      void mbs2ObjectTree(Element* ele, QTreeWidgetItem* parentItem);
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      protected slots:
        void checkForObject(QTreeWidgetItem* item_,int);
  };

  class LinkBrowser : public QDialog {
    Q_OBJECT

    public:
      LinkBrowser(Element* element_, Link* link, QWidget *parentLink_);
      ~LinkBrowser() override = default;
      QTreeWidget* getLinkList() const {return linkList;}
      void updateWidget(Link *sel);
    protected:
      QPushButton *okButton;
      QTreeWidget *linkList;
      Link *selection;
      ElementItem *savedItem;
      Element* element;
      QString oldID;
      void mbs2LinkTree(Element* ele, QTreeWidgetItem* parentItem);
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      protected slots:
        void checkForLink(QTreeWidgetItem* item_,int);
  };

  class ConstraintBrowser : public QDialog {
    Q_OBJECT

    public:
      ConstraintBrowser(Element* element_, Constraint* constraint, QWidget *parentConstraint_);
      ~ConstraintBrowser() override = default;
      QTreeWidget* getConstraintList() const {return constraintList;}
      void updateWidget(Constraint *sel);
    protected:
      QPushButton *okButton;
      QTreeWidget *constraintList;
      Constraint *selection;
      ElementItem *savedItem;
      Element* element;
      QString oldID;
      void mbs2ConstraintTree(Element* ele, QTreeWidgetItem* parentItem);
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      protected slots:
        void checkForConstraint(QTreeWidgetItem* item_,int);
  };

  class RigidBodyBrowser : public QDialog {
    Q_OBJECT

    public:
      RigidBodyBrowser(Element* element_, RigidBody* rigidBody, QWidget *parentObject_);
      ~RigidBodyBrowser() override = default;
      QTreeWidget* getRigidBodyList() const {return rigidBodyList;}
      void updateWidget(RigidBody *sel);
    protected:
      QPushButton *okButton;
      QTreeWidget *rigidBodyList;
      RigidBody *selection;
      ElementItem *savedItem;
      Element* element;
      QString oldID;
      void mbs2RigidBodyTree(Element* ele, QTreeWidgetItem* parentItem);
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      protected slots:
        void checkForRigidBody(QTreeWidgetItem* item_,int);
  };

  class FrameBrowser : public QDialog {
    Q_OBJECT

    public:
      FrameBrowser(Element* element_, Frame* frame, QWidget *parentObject_);
      ~FrameBrowser() override = default;
      QTreeWidget* getFrameList() const {return frameList;}
      void updateWidget(Frame *sel);
    protected:
      QPushButton *okButton;
      QTreeWidget *frameList;
      Frame *selection;
      ElementItem *savedItem;
      Element* element;
      QString oldID;
      void mbs2FrameTree(Element* ele, QTreeWidgetItem* parentItem);
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      protected slots:
        void checkForFrame(QTreeWidgetItem* item_,int);
  };

  class ContourBrowser : public QDialog {
    Q_OBJECT

    public:
      ContourBrowser(Element* element_, Contour* contour, QWidget *parentObject_);
      ~ContourBrowser() override = default;
      QTreeWidget* getContourList() const {return contourList;}
      void updateWidget(Contour *sel);
    protected:
      QPushButton *okButton;
      QTreeWidget *contourList;
      Contour *selection;
      ElementItem *savedItem;
      Element* element;
      QString oldID;
      void mbs2ContourTree(Element* ele, QTreeWidgetItem* parentItem);
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      protected slots:
        void checkForContour(QTreeWidgetItem* item_,int);
  };

  class SignalBrowser : public QDialog {
    Q_OBJECT

    public:
      SignalBrowser(Element* element_, Signal* signal, QWidget *parentSignal_);
      ~SignalBrowser() override = default;
      QTreeWidget* getSignalList() const {return signalList;}
      void updateWidget(Signal *sel);
    protected:
      QPushButton *okButton;
      QTreeWidget *signalList;
      Signal *selection;
      ElementItem *savedItem;
      Element* element;
      void mbs2SignalTree(Element* ele, QTreeWidgetItem* parentItem);
      protected slots:
        void checkForSignal(QTreeWidgetItem* item_,int);
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
      void setUrl(const QUrl &url_);
    private:
      QWebView *webView;
      QUrl url;
  };

}

#endif
