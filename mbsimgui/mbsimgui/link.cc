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
#include "link_.h"
#include "utils.h"
#include "mainwindow.h"
#include "parameter.h"
#include "diagram_item.h"
#include "diagram_arrow.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {
  
  MBSIMGUI_REGOBJECTFACTORY(UnknownLink);

  Link::Link() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"link.svg").string()));
    parameterEmbedItem->setIcon(icon);
  }

  Link::~Link() {
    if(diagramItem) {
      diagramItem->removeDiagramArrows();
      delete diagramItem;
    }
  }

  void Link::createDiagramItem() {
    QPolygonF polygon;
    double size = 50;
    QPainterPath path;
            path.moveTo(50, 0);
            path.arcTo(0, -50, 50, 50, 0, 90);
            path.arcTo(-50, -50, 50, 50, 90, 90);
            path.arcTo(-50, 0, 50, 50, 180, 90);
            path.arcTo(0, 0, 50, 50, 270, 90);
            polygon = path.toFillPolygon();
    diagramItem = new DiagramItem(polygon,parent->getDiagramItem());
    diagramItem->setBrush(Qt::white);
    auto *text = new QGraphicsSimpleTextItem(getName(),diagramItem);
    text->setPos(-4*text->text().length(),-size);
  }

  UnknownLink::UnknownLink() {
    icon = QIcon(new OverlayIconEngine((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"link.svg").string(),
                                       (MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"unknownelement.svg").string()));
  }

  void FrameLink::createDiagramArrows() {
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"connect");
    if(e) {
       auto str = QString::fromStdString(MBXMLUtils::E(e)->getAttribute("ref1"));
       Element *ref = parent->getFrame(0);
       if(not str.isEmpty())
         ref = getByPath<Element>(str);
       auto *endItem = ref->getDiagramItem();
       if(endItem) {
         DiagramArrow *arrow = new DiagramArrow(endItem, diagramItem, diagramItem);
         arrow->updatePosition();
       }
       str = QString::fromStdString(MBXMLUtils::E(e)->getAttribute("ref2"));
       ref = getByPath<Element>(str);
       endItem = ref->getDiagramItem();
       if(endItem) {
         DiagramArrow *arrow = new DiagramArrow(endItem, diagramItem, diagramItem);
         arrow->updatePosition();
       }
    }
  }

  xercesc::DOMElement* UnknownLink::processIDAndHref(xercesc::DOMElement *element) {
    auto ret = Link::processIDAndHref(element);
    processIDAndHrefOfUnknownElements(element);
    return ret;
  }

}

