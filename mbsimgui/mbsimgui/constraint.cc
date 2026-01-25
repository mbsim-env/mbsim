/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2020 Martin Förg

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
#include "constraint.h"
#include "utils.h"
#include "mainwindow.h"
#include "parameter.h"
#include "diagram_item.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {
  
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedAccelerationConstraint);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedConnectionConstraint);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedGearConstraint);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedPositionConstraint);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedVelocityConstraint);
  MBSIMGUI_REGOBJECTFACTORY(InverseKinematicsConstraint);
  MBSIMGUI_REGOBJECTFACTORY(JointConstraint);
  MBSIMGUI_REGOBJECTFACTORY(UnknownConstraint);

  Constraint::Constraint() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"constraint.svg").string()));
    parameterEmbedItem->setIcon(icon);
  }

  Constraint::~Constraint() {
    if(diagramItem) {
      diagramItem->removeDiagramArrows();
      delete diagramItem;
    }
  }

  void Constraint::createDiagramItem() {
    QPolygonF polygon;
    polygon << QPointF(-50, -50) << QPointF(50, -50) << QPointF(50, 50) << QPointF(-50, 50) << QPointF(-50, -50);
    diagramItem = new DiagramItem(polygon,parent->getDiagramItem());
    diagramItem->setBrush(Qt::white);
    auto *text = new QGraphicsSimpleTextItem(getName(),diagramItem);
    text->setPos(-4*text->text().length(),-50);
  }

  UnknownConstraint::UnknownConstraint() {
    icon = QIcon(new OverlayIconEngine((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"constraint.svg").string(),
                                       (MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"unknownelement.svg").string()));
  }

  xercesc::DOMElement* UnknownConstraint::processIDAndHref(xercesc::DOMElement *element) {
    auto ret = Constraint::processIDAndHref(element);
    processIDAndHrefOfUnknownElements(element);
    return ret;
  }

}

