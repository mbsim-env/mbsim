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
#include "contour.h"
#include "utils.h"
#include "mainwindow.h"
#include "parameter.h"
#include "diagram_item.h"
#include "diagram_arrow.h"
#include <xercesc/dom/DOMDocument.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  MBSIMGUI_REGOBJECTFACTORY(BevelGear);
  MBSIMGUI_REGOBJECTFACTORY(Circle);
  MBSIMGUI_REGOBJECTFACTORY(Cylinder);
  MBSIMGUI_REGOBJECTFACTORY(Cuboid);
  MBSIMGUI_REGOBJECTFACTORY(CylindricalGear);
  MBSIMGUI_REGOBJECTFACTORY(Disk);
  MBSIMGUI_REGOBJECTFACTORY(FclBox);
  MBSIMGUI_REGOBJECTFACTORY(FclMesh);
  MBSIMGUI_REGOBJECTFACTORY(FclPlane);
  MBSIMGUI_REGOBJECTFACTORY(FclSphere);
  MBSIMGUI_REGOBJECTFACTORY(FlexiblePlanarFfrNurbsContour);
  MBSIMGUI_REGOBJECTFACTORY(FlexiblePlanarNurbsContour);
  MBSIMGUI_REGOBJECTFACTORY(FlexibleSpatialFfrNurbsContour);
  MBSIMGUI_REGOBJECTFACTORY(FlexibleSpatialNurbsContour);
  MBSIMGUI_REGOBJECTFACTORY(NodesContour);
  MBSIMGUI_REGOBJECTFACTORY(Line);
  MBSIMGUI_REGOBJECTFACTORY(LineSegment);
  MBSIMGUI_REGOBJECTFACTORY(PlanarContour);
  MBSIMGUI_REGOBJECTFACTORY(PlanarGear);
  MBSIMGUI_REGOBJECTFACTORY(PlanarNurbsContour);
  MBSIMGUI_REGOBJECTFACTORY(Plane);
  MBSIMGUI_REGOBJECTFACTORY(Point);
  MBSIMGUI_REGOBJECTFACTORY(Rack);
  MBSIMGUI_REGOBJECTFACTORY(Sphere);
  MBSIMGUI_REGOBJECTFACTORY(SpatialContour);
  MBSIMGUI_REGOBJECTFACTORY(SpatialNurbsContour);
  MBSIMGUI_REGOBJECTFACTORY(Tyre);
  MBSIMGUI_REGOBJECTFACTORY(Revolution);
  MBSIMGUI_REGOBJECTFACTORY(Extrusion);
  MBSIMGUI_REGOBJECTFACTORY(UnknownContour);

  Contour::Contour() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"contour.svg").string()));
    parameterEmbedItem->setIcon(icon);
  }

  Contour::~Contour() {
    if(diagramItem) {
      diagramItem->removeDiagramArrows();
      delete diagramItem;
    }
  }

  DOMElement* Contour::processIDAndHref(DOMElement *element) {
    element = Element::processIDAndHref(element);
    DOMElement *ELE=E(element)->getFirstElementChildNamed(NamespaceURI(getXMLType().first)%"enableOpenMBV");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());
    return element;
  }

  void Contour::createDiagramItem() {
    QPolygonF polygon;
    QPainterPath path;
    path.addEllipse(-25, -25, 50, 50);
    polygon = path.toFillPolygon();
    diagramItem = new DiagramItem(polygon,parent->getDiagramItem());
    diagramItem->setBrush(Qt::white);
    auto *text = new QGraphicsSimpleTextItem(getName(),diagramItem);
    text->setPos(-4*text->text().length(),0);
  }

  void RigidContour::createDiagramArrows() {
    DiagramItem *endItem = parent->getFrame(0)->getDiagramItem();
    auto *e=E(getXMLElement())->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(e) {
      auto refStr = QString::fromStdString(E(e)->getAttribute("ref"));
      auto *ref = getByPath<Element>(refStr);
      endItem = ref->getDiagramItem();
    }
    DiagramArrow *arrow = new DiagramArrow(diagramItem, endItem, diagramItem);
    arrow->updatePosition();
  }


  UnknownContour::UnknownContour() {
    icon = QIcon(new OverlayIconEngine((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"contour.svg").string(),
                                       (MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"unknownelement.svg").string()));
  }

}
