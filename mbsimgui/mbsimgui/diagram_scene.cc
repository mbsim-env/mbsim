/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2026 MBSim-Env

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
#include "diagram_scene.h"
#include "project.h"
#include "dynamic_system_solver.h"
#include "diagram_item.h"
#include "mainwindow.h"

using namespace std;

namespace MBSimGUI {

  extern MainWindow *mw;

  DiagramScene::DiagramScene(QWidget *parent) : QGraphicsScene(parent) {
    reset();
  }

  void DiagramScene::reset() {
    auto *dss = mw->getProject()->getDynamicSystemSolver();
    dss->createDiagramItem();
    dss->getDiagramItem()->setPos(2500,2500);
    addItem(dss->getDiagramItem());
    dss->createDiagramArrows();
  }

  void DiagramScene::savePos() {
  }

}
