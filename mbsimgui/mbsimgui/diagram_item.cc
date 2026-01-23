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
#include "diagram_item.h"
#include "diagram_arrow.h"
#include <QGraphicsScene>

namespace MBSimGUI {

  DiagramItem::DiagramItem(const QPolygonF &polygon, QGraphicsItem *parent) : QGraphicsPolygonItem(polygon,parent) {
    setFlag(QGraphicsItem::ItemIsMovable, true);
  }

  void DiagramItem::removeDiagramArrows() {
    const auto arrows_ = arrows;
    for(DiagramArrow *arrow : arrows_) {
      arrow->getFirstDiagramItem()->removeDiagramArrow(arrow);
      arrow->getSecondDiagramItem()->removeDiagramArrow(arrow);
      scene()->removeItem(arrow);
      delete arrow;
    }
  }

  QVariant DiagramItem::itemChange(GraphicsItemChange change, const QVariant &value) {
    if(change == QGraphicsItem::ItemPositionChange) {
      for(DiagramArrow *arrow : qAsConst(arrows))
        arrow->updatePosition();
    }
    return value;
  }

}
