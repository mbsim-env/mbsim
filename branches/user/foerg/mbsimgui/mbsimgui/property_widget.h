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

#ifndef _PROPERTY_WIDGET_H_
#define _PROPERTY_WIDGET_H_

#include <QScrollArea>
#include <QTabWidget>
#include <QDialog>
#include <map>

class ExtWidget;
class TextWidget;
class QVBoxLayout;
class TiXmlElement;
class TiXmlNode;
class QDialogButtonBox;
class QAbstractButton;
class Element;
class Frame;
class FixedRelativeFrame;
class Group;
class Solver;

class PropertyDialog : public QDialog {
  Q_OBJECT

  public:
    PropertyDialog(QWidget * parent = 0, Qt::WindowFlags f = 0);
    ~PropertyDialog();
    void setParentObject(QObject *obj);
    void addToTab(const QString &name, QWidget* widget_);
    void addTab(const QString &name, int i=-1);
    QObject* getParentObject() { return parentObject; }
    void addStretch();
    void updateWidget();
    void resizeVariables();
    virtual void toWidget(Element *element) {}
    virtual void fromWidget(Element *element) {}
  protected:
    QObject* parentObject;
    std::map<QString,QVBoxLayout*> layout;
    std::vector<QWidget*> widget;
    QTabWidget *tabWidget;
    QDialogButtonBox *buttonBox;
  public slots:
    void clicked(QAbstractButton *button);
  signals:
    void apply(QWidget *editor);
    void ok(QWidget *editor);
    void cancel(QWidget *editor);
};

class ElementPropertyDialog : public PropertyDialog {

  public:
    ElementPropertyDialog(QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    TextWidget *textWidget;
};

class FramePropertyDialog : public ElementPropertyDialog {

  public:
    FramePropertyDialog(Frame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *visuWidget;
};

class FixedRelativeFramePropertyDialog : public FramePropertyDialog {

  public:
    FixedRelativeFramePropertyDialog(FixedRelativeFrame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *refFrameWidget, *positionWidget, *orientationWidget;
};

class GroupPropertyDialog : public ElementPropertyDialog {

  public:
    GroupPropertyDialog(Group *group, QWidget * parent = 0, Qt::WindowFlags f = 0, bool disabled=false);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *positionWidget, *orientationWidget, *frameOfReferenceWidget; 
};

class SolverPropertyDialog : public GroupPropertyDialog {
  protected:
    ExtWidget *environmentWidget, *solverParametersWidget, *inverseKineticsWidget;

  public:
    SolverPropertyDialog(Solver *solver, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
};

#endif
