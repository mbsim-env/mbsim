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

#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <QMainWindow>
#include <QStandardItemModel>
#include <QGridLayout>
#include <QComboBox>
#include <QTreeWidgetItem>

class QListWidget;
class QTableWidget;
class QTreeWidget;
class QStackedWidget;
class PropertyDialog;
class QAction;
class QMenu;
class QLabel;
class QLineEdit;

class ElementItem : public QObject, public QTreeWidgetItem {
  Q_OBJECT
  protected:
    QMenu *contextMenu;
    QTreeWidget *elementList;
  public:
    ElementItem(const QString &str, QTreeWidget* elementList);
    virtual ~ElementItem() {}
    QMenu* getContextMenu() {return contextMenu;}
    QString getName() const {return text(0);}
  public slots:
    virtual void add();
};

class FrameItem : public ElementItem {
  Q_OBJECT
  protected:
  public:
    FrameItem(const QString &str, QTreeWidget* elementList);
    virtual ~FrameItem() {}
  public slots:
    void add();
};

class GroupItem : public ElementItem {
  Q_OBJECT
  protected:
  public:
    GroupItem(const QString &str, QTreeWidget* elementList);
    virtual ~GroupItem() {}
  public slots:
    void add();
};

class ObjectItem : public ElementItem {
  Q_OBJECT
  protected:
  public:
    ObjectItem(const QString &str, QTreeWidget* elementList);
    virtual ~ObjectItem() {}
  public slots:
    void add();
};

class LinkItem : public ElementItem {
  Q_OBJECT
  protected:
  public:
    LinkItem(const QString &str, QTreeWidget* elementList);
    virtual ~LinkItem() {}
  public slots:
    void add();
};

class FileItem : public ElementItem {
  Q_OBJECT
  protected:
  public:
    FileItem(const QString &str, QTreeWidget* elementList);
    virtual ~FileItem() {}
  public slots:
    void add();
};

class MainWindow : public QMainWindow {

  Q_OBJECT

  private:
    QStandardItemModel *model;
    QWidget *centralWidget;
    QTreeWidget *elementList, *integratorList, *parameterList, *sourceList;
    PropertyDialog *properties;
    QStackedWidget *pagesWidget;
    QLineEdit *fileMBS, *fileIntegrator, *fileParameter;
    QAction *actionSaveMBSAs, *actionSaveMBS, *actionSimulate, *actionOpenMBV, *actionH5plotserie, *actionSaveIntegrator, *actionSaveParameter;
    void loadMBS(const QString &file);
    void loadIntegrator(const QString &file);
    void loadParameter(const QString &file);
  public:
    MainWindow();
  public slots:
    void elementListClicked();
    void sourceListClicked();
    void sourceListDoubleClicked();
    void integratorListClicked();
    void parameterListClicked();
    void newMBS();
    void loadMBS();
    void saveMBSAs();
    void saveMBS();
    void newDOPRI5Integrator();
    void newLSODEIntegrator();
    void loadIntegrator();
    void saveIntegratorAs();
    void saveIntegrator();
    void newDoubleParameter();
    void loadParameter();
    void saveParameterAs();
    void saveParameter();
    void preview();
    void simulate();
    void openmbv();
    void h5plotserie();
    void help();
    void about();
    void updateOctaveParameters();
};

#endif
