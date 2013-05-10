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

#ifndef __MAINWINDOW_H_
#define __MAINWINDOW_H_

#include "parameter.h"
#include <QMainWindow>
#include <QTabWidget>
#include <mbxmlutilstinyxml/tinyxml.h>
#include "treemodel.h"
#include "treeitem.h"
#include "element_view.h"
#include "object.h"
#include "link.h"
#include "group.h"
#include "frame.h"
#include "contour.h"
#include "observer.h"

class QTreeView;
class QAction;
class QLineEdit;
class QTextBrowser;
class QProcess;
class QUrl;
class Process;
class ElementView;
class ParameterView;
class IntegratorView;
class QModelIndex;

namespace OpenMBVGUI {
  class MainWindow;
}

namespace MBXMLUtils {
  class OctaveEvaluator;
}

class MainWindow : public QMainWindow {

  Q_OBJECT

  private:
    ElementView *elementList;
    ParameterView *parameterList;
    IntegratorView *integratorView;
    QLineEdit *fileMBS, *fileIntegrator, *fileParameter;
    Process *mbsim;
    void loadProj(const QString &file);
    void loadMBS(const QString &file);
    void loadIntegrator(const QString &file);
    void loadParameterList(const QString &file);
    OpenMBVGUI::MainWindow *inlineOpenMBVMW;
    void initInlineOpenMBV();
    QString uniqueTempDir, absoluteMBSFilePath;
    QAction *actionSaveProj, *actionSaveMBS, *actionSimulate, *actionOpenMBV, *actionH5plotserie, *actionSaveIntegrator, *actionSaveParameterList, *actionSaveDataAs, *actionSaveMBSimH5DataAs, *actionSaveOpenMBVDataAs;

  public:
    MainWindow();
    ~MainWindow();
    static MBXMLUtils::OctaveEvaluator *octEval;
    void mbsimxml(int task);
    const QString& getUniqueTempDir() const {return uniqueTempDir;}
    template <class T> bool addFrame(const std::string &name, bool file=false);
    template <class T> bool addContour(const std::string &name, bool file=false);
    template <class T> bool addGroup(const std::string &name, bool file=false);
    template <class T> bool addObject(const std::string &name, bool file=false);
    template <class T> bool addLink(const std::string &name, bool file=false);
    template <class T> bool addObserver(const std::string &name, bool file=false);
  public slots:
    void elementListClicked();
    void parameterListClicked();
    void loadProj();
    void saveProjAs();
    void saveProj();
    void newMBS(bool ask=true);
    void loadMBS();
    void saveMBSAs();
    void saveMBS();
    void selectIntegrator();
    void selectDOPRI5Integrator();
    void selectRADAU5Integrator();
    void selectLSODEIntegrator();
    void selectLSODARIntegrator();
    void selectTimeSteppingIntegrator();
    void selectEulerExplicitIntegrator();
    void selectRKSuiteIntegrator();
    void loadIntegrator();
    void saveIntegratorAs();
    void saveIntegrator();
    void newParameterList();
    void loadParameterList();
    void saveParameterListAs();
    void saveParameterList(const QString &filename="");
    void saveDataAs();
    void saveMBSimH5DataAs();
    void saveMBSimH5Data(const QString &file);
    void saveOpenMBVDataAs();
    void saveOpenMBVXMLData(const QString &file);
    void saveOpenMBVH5Data(const QString &file);
    void removeParameter();
    void addScalarParameter();
    void addVectorParameter();
    void addMatrixParameter();
    void simulate();
    void openmbv();
    void h5plotserie();
    void help();
    void about();
    void updateOctaveParameters(const ParameterList &list=ParameterList());
    void removeElement();
    void saveElementAs();
  protected slots:
    void selectElement(std::string);
    void changeWorkingDir();
    void selectionChanged();
    void openPropertyDialog();
    void simulationFinished(int exitCode);
  protected:
    void closeEvent ( QCloseEvent * event );
};

class Process : public QTabWidget {
  Q_OBJECT
  public:
    Process(QWidget *parent);
    QProcess *getProcess() { return process; }
    void clearOutputAndStart(const QString &program, const QStringList &arguments);
    QSize sizeHint() const;
    QSize minimumSizeHint() const;
  private:
    QProcess *process;
    QTextBrowser *out, *err;
    QString outText, errText;
    void convertToHtml(QString &text);
    void linkClicked(const QUrl &link, QTextBrowser *std);
  private slots:
    void output();
    void error();
    void outLinkClicked(const QUrl &link);
    void errLinkClicked(const QUrl &link);
};

template <class T>
bool MainWindow::addFrame(const std::string &name, bool file) {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = index.child(0,0);
  Element *parentElement = static_cast<Element*>(model->getItem(index)->getItemData());
  Frame *frame;
  if(file)
    frame = Frame::readXMLFile(name,parentElement);
  else
    frame = new T(name+toStr(model->getItem(containerIndex)->getID()-1),parentElement);
  if(!frame)
    return false;
  parentElement->addFrame(frame);
  model->createFrameItem(frame,containerIndex);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  return true;
}

template <class T>
bool MainWindow::addContour(const std::string &name, bool file) {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = index.child(1,0);
  Element *parentElement = static_cast<Element*>(model->getItem(index)->getItemData());
  Contour *contour;
  if(file)
    contour = Contour::readXMLFile(name,parentElement);
  else
    contour = new T(name+toStr(model->getItem(containerIndex)->getID()),parentElement);
  if(!contour)
    return false;
  parentElement->addContour(contour);
  model->createContourItem(contour,containerIndex);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  return true;
}

template <class T>
bool MainWindow::addGroup(const std::string &name, bool file) {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = index.child(2,0);
  Element *parentElement = static_cast<Element*>(model->getItem(index)->getItemData());
  Group *group;
  if(file) 
    group = Group::readXMLFile(name,parentElement);
  else
    group = new T(name+toStr(model->getItem(containerIndex)->getID()),parentElement);
  if(!group)
    return false;
  parentElement->addGroup(group);
  model->createGroupItem(group,containerIndex);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  return true;
}

template <class T>
bool MainWindow::addObject(const std::string &name, bool file) {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = index.child(3,0);
  Element *parentElement = static_cast<Element*>(model->getItem(index)->getItemData());
  Object *object;
  if(file) 
    object = Object::readXMLFile(name,parentElement);
  else
    object = new T(name+toStr(model->getItem(containerIndex)->getID()),parentElement);
  if(!object)
    return false;
  parentElement->addObject(object);
  model->createObjectItem(object,containerIndex);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  return true;
}

template <class T>
bool MainWindow::addLink(const std::string &name, bool file) {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = index.child(4,0);
  Element *parentElement = static_cast<Element*>(model->getItem(index)->getItemData());
  Link *link;
  if(file)
    link = Link::readXMLFile(name,parentElement);
  else
    link = new T(name+toStr(model->getItem(containerIndex)->getID()),parentElement);
  if(!link)
    return false;
  parentElement->addLink(link);
  model->createLinkItem(link,containerIndex);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  return true;
}

template <class T>
bool MainWindow::addObserver(const std::string &name, bool file) {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = index.child(5,0);
  Element *parentElement = static_cast<Element*>(model->getItem(index)->getItemData());
  Observer *observer;
  if(file)
    observer = Observer::readXMLFile(name,parentElement);
  else
    observer = new T(name+toStr(model->getItem(containerIndex)->getID()),parentElement);
  if(!observer)
    return false;
  parentElement->addObserver(observer);
  model->createObserverItem(observer,containerIndex);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  return true;
}

#endif
