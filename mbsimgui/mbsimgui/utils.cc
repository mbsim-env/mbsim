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

#include <config.h>
#include <cstdio>
#include "utils.h"
#include <unordered_map>
#include <cmath>
#include <stdexcept>
#include <QPainter>
#include "group.h"
#include "contour.h"
#include "frame.h"
#include "frame.h"
#include "object.h"
#include "link_.h"
#include "constraint.h"
#include "observer.h"
#include "solver.h"
#include "parameter.h"

using namespace std;

namespace MBSimGUI {

  bool Utils::initialized=false;

  void Utils::initialize() {
    if(initialized) return;
    initialized=true;

  }

  const QIcon& Utils::QIconCached(const QString& filename) {
    static unordered_map<string, QIcon> myIconCache;
    pair<unordered_map<string, QIcon>::iterator, bool> ins=myIconCache.insert(pair<string, QIcon>(filename.toStdString(), QIcon()));
    if(ins.second)
      return ins.first->second=QIcon(filename);
    return ins.first->second;
  } 

  OverlayIconEngine::OverlayIconEngine(const string &baseFile, const string &overlayFile) {
    baseIcon    = Utils::QIconCached(QString::fromStdString(baseFile));
    overlayIcon = Utils::QIconCached(QString::fromStdString(overlayFile));
  }

  OverlayIconEngine::OverlayIconEngine(const QIcon &baseIcon_, const QIcon &overlayIcon_) {
    baseIcon    = baseIcon_;
    overlayIcon = overlayIcon_;
  }

  void OverlayIconEngine::paint(QPainter* painter, const QRect& rect, QIcon::Mode mode, QIcon::State state) {
    QPixmap basePixmap = baseIcon.pixmap(rect.size());
    painter->drawPixmap(rect, basePixmap, QRect(QPoint(0,0), rect.size()));

    QSize overlaySize = rect.size()*(14.0/16.0);
    QPixmap overlayPixmap = overlayIcon.pixmap(overlaySize);
    painter->drawPixmap(QRectF(QPoint(rect.x()+lround(1.0/16.0*rect.size().width()),
                                      rect.y()+lround(1.0/16.0*rect.size().height())),overlaySize),
                        overlayPixmap, QRectF(QPoint(0,0),overlaySize));
  }

  QIconEngine* OverlayIconEngine::clone() const {
    return new OverlayIconEngine(baseIcon, overlayIcon);
  }

  vector<vector<double>> mult(const vector<vector<double>> &A, const vector<vector<double>> &B) {
    vector<vector<double>> C(A.size());
    for(size_t i=0; i<A.size(); i++) {
      C[i].resize(B[0].size());
      for(size_t j=0; j<B[0].size(); j++) {
        for(size_t k=0; k<A[0].size(); k++)
          C[i][j] += A[i][k]*B[k][j];
      }
    }
    return C;
  }

  vector<vector<double>> BasicRotAKIx(double phi) {
    vector<vector<double>> AKI(3);
    for(int i=0; i<3; i++)
      AKI[i].resize(3);
    AKI[0][0]= 1.0;
    AKI[1][1]= cos(phi);
    AKI[2][2]= AKI[1][1];
    AKI[1][2]= sin(phi);
    AKI[2][1]=-AKI[1][2]; 
    return AKI;
  }

  vector<vector<double>> BasicRotAKIy(double phi) {
    vector<vector<double>> AKI(3);
    for(int i=0; i<3; i++)
      AKI[i].resize(3);
    AKI[1][1]= 1.0;
    AKI[0][0]= cos(phi);
    AKI[2][2]= AKI[0][0];
    AKI[0][2]=-sin(phi);
    AKI[2][0]=-AKI[0][2];
    return AKI; 
  }

  vector<vector<double>> BasicRotAKIz(double phi) {
    vector<vector<double>> AKI(3);
    for(int i=0; i<3; i++)
      AKI[i].resize(3);
    AKI[2][2]= 1.0;
    AKI[0][0]= cos(phi);
    AKI[1][1]= AKI[0][0];
    AKI[0][1]= sin(phi);
    AKI[1][0]= -AKI[0][1];
    return AKI; 
  }

  vector<vector<double>> BasicRotAIKx(double phi) {
    vector<vector<double>> AIK = BasicRotAKIx(-phi);
    return AIK;
  }

  vector<vector<double>> BasicRotAIKy(double phi) {
    vector<vector<double>> AIK = BasicRotAKIy(-phi);
    return AIK; 
  }

  vector<vector<double>> BasicRotAIKz(double phi) {
    vector<vector<double>> AIK = BasicRotAKIz(-phi);
    return AIK; 
  }

  vector<vector<double>> Cardan2AIK(const vector<vector<double>> &x) {
    vector<vector<double>> AIKx =  BasicRotAIKx(x[0][0]);
    vector<vector<double>> AIKy =  BasicRotAIKy(x[1][0]);
    vector<vector<double>> AIKz = BasicRotAIKz(x[2][0]);

    return mult(AIKx,mult(AIKy,AIKz));          //Wie im TM VI Skript
  }

  vector<vector<double>> AIK2Cardan(const vector<vector<double>> &AIK) {
    vector<vector<double>> AlphaBetaGamma(3);
    for(int i=0; i<3; i++)
      AlphaBetaGamma[i].resize(1);
    AlphaBetaGamma[1][0]= asin(AIK[0][2]);
    double nenner = cos(AlphaBetaGamma[1][0]);
    if (fabs(nenner)>1e-10) {
      AlphaBetaGamma[0][0] = atan2(-AIK[1][2],AIK[2][2]);
      AlphaBetaGamma[2][0] = atan2(-AIK[0][1],AIK[0][0]);
    } else {
      AlphaBetaGamma[0][0]=0;
      AlphaBetaGamma[2][0]=atan2(AIK[1][0],AIK[1][1]);
    }
    return AlphaBetaGamma;
  }

  string removeWhiteSpace(const string &str) {
    string ret = str;
    size_t found;
    found=ret.find_first_of(' ');
    while (found!=string::npos) {
      ret.erase(found,1);
      found=ret.find_first_of(' ',found);
    }
    return ret;
  }

  QString removeWhiteSpace(const QString &str) {
    QString ret = str;
    size_t found;
    found=ret.indexOf(" ");
    while (found!=string::npos) {
      ret.remove(found,1);
      found=ret.indexOf(" ",found);
    }
    return ret;
  }

  template<class Container>
  void createContextMenuFor(QMenu *self, TreeItemData *item, const QString &prefix) {
    QMenu* tempTopMenu=new QMenu(self); // create a temporary parent for everything
    const static QString menuPostfix("'s");
    const auto &funcs=ObjectFactory::getInstance().getAllTypesForContainer<Container>();
    // create a menu action for each class derived from Container
    const ObjectFactory::Funcs* funcForContainer=nullptr;
    for(const auto &func : funcs) {
      // build all submenus
      // search where to start in the class hierarchy.
      decltype(func->getTypePath().end()) typePathIt;
      for(typePathIt=--func->getTypePath().end(); typePathIt!=func->getTypePath().begin(); --typePathIt)
        if(typePathIt->second.get()==typeid(Container))
          break;
      // if the class is the container itself a special handling is applied (see container_is_constructable)
      if(next(typePathIt)==func->getTypePath().end()) {
        funcForContainer=func;
        continue;
      }
      QMenu *parent = tempTopMenu; // at start the tempTopMenu is the parent submenu
      // add submenu starting from ++typePathIt (the container itself is not a submenu
      for(++typePathIt; typePathIt!=func->getTypePath().end(); ++typePathIt) {
        // skip the last entry, this is the action not a submenu
        if(next(typePathIt)==func->getTypePath().end()) break;
        // search already existing menu or add new menu sorted alphabetically
        QMenu *menu;
        auto parentActions=parent->actions();
        bool createNew=true;
        QAction *createBefore=nullptr;
        for(auto actIt=parentActions.begin(); actIt!=parentActions.end(); ++actIt) {
          // if actIt is a menu and its text equals the to be created menu then
          // use the menu of actIt instead of creating a new one
          if((*actIt)->menu() && (*actIt)->text()==typePathIt->first+menuPostfix) {
            menu=(*actIt)->menu();
            createNew=false;
            break;
          }
          // if actIt is a menu and its text is larger then the to be created menu then
          // insert the to be created menu before actIt (we sort menus)
          if((*actIt)->menu() &&(*actIt)->text() > typePathIt->first+menuPostfix) {
            createBefore=*actIt;
            break;
          }
          // if actIt is not a menu then
          // insert the to be created menu before actIt (we add menus before actions)
          if((*actIt)->menu()==nullptr) {
            createBefore=*actIt;
            break;
          }
          // if none of the above "if" breaks the loop then
          // we need to insert the to be created menu at the end (createBefore==nullptr will insert back)
        }
        if(createNew) {
          menu = new QMenu(typePathIt->first+menuPostfix, parent);
          parent->insertMenu(createBefore, menu);
        }
        // use this menu as new parent
        parent = menu;
      }
      // add the action (for the last entry). This is the current func
      // the element lists are already sorted by func->getType() -> hence no sorting neede here
      auto action = new QAction(prefix+func->getType()+"'", parent);
      QObject::connect(action,&QAction::triggered,[=](){ mw->add<Container>(dynamic_cast<Container*>(func->ctor(nullptr, nullptr)), item); });
      parent->addAction(action);
    }
    // handle the action for the container if it exits (see container_is_constructable) -> add as first entry and add seperator
    if(funcForContainer) {
      auto action = new QAction(prefix+funcForContainer->getType()+"'", tempTopMenu);
      QObject::connect(action,&QAction::triggered,[=](){ mw->add<Container>(dynamic_cast<Container*>(funcForContainer->ctor(nullptr, nullptr)), item); });
      tempTopMenu->insertAction(tempTopMenu->actions().size()==0 ? nullptr : tempTopMenu->actions()[0], action);
      tempTopMenu->addSeparator();
    }
    // remove all submenus which have only one entry
    function<void(QMenu*)> removeEmptySubMenus=[&removeEmptySubMenus](QMenu *parentMenu) {
      for(auto &action : parentMenu->actions()) { // loop over all menus
        QMenu *menu=action->menu();
        if(menu==nullptr) continue; // really loop only over menus (not actions)
        if(menu->actions().size()==1 && menu->actions()[0]->menu()!=nullptr) { // menu contains only 1 menu, nothing else
          for(auto &childAction : menu->actions()[0]->menu()->actions())
            menu->addAction(childAction);
          menu->removeAction(menu->actions()[0]);
        }
        removeEmptySubMenus(menu);
      }
    };
    removeEmptySubMenus(tempTopMenu);
    // append all children from the temporary parent to self
    self->addActions(tempTopMenu->actions());
  }

  // explizit instantiation
  #define X(Type) \
    template void createContextMenuFor<Type>(QMenu *self, TreeItemData *item, const QString &prefix="");
  MBSIMGUI_TREE_CONTAINERS
  #undef X

}
