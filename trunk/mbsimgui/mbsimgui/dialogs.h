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
#include "string_widgets.h"

class Element;
class RigidBody;
class Frame;

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
    EvalDialog(StringWidget *var);
    void setValue(const std::string &str) {var->setValue(str);}
    std::string getValue() const {return var->getValue();}
    void setButtonDisabled(bool flag) {button->setDisabled(flag);}
  protected:
    StringWidget *var;
    QPushButton *button;
  signals:
    void clicked(bool);
};

class RigidBodyBrowser : public QDialog {
  Q_OBJECT

  public:
    RigidBodyBrowser(QTreeWidget* tree, RigidBody* selection, QWidget *obj);
    ~RigidBodyBrowser() {}
    QTreeWidget* getRigidBodyList() const {return rigidBodyList;}
    void update(RigidBody *rigidBody);
  protected:
    QPushButton *okButton;
    QTreeWidget *rigidBodyList;
    RigidBody *selection;
    ElementItem *savedItem;
    QTreeWidget* tree;
    void mbs2RigidBodyTree(Element* item, QTreeWidgetItem* parentItem);
  protected slots:
    void checkForRigidBody(QTreeWidgetItem* item_,int);
};

class FrameBrowser : public QDialog {
  Q_OBJECT

  public:
    FrameBrowser(QTreeWidget* tree, Frame* selection, QWidget *obj);
    ~FrameBrowser() {}
    QTreeWidget* getFrameList() const {return frameList;}
    void update(Frame *frame);
  protected:
    QPushButton *okButton;
    QTreeWidget *frameList;
    Frame *selection;
    ElementItem *savedItem;
    QTreeWidget* tree;
    void mbs2FrameTree(Element* item, QTreeWidgetItem* parentItem);
  protected slots:
    void checkForFrame(QTreeWidgetItem* item_,int);
};


#endif
