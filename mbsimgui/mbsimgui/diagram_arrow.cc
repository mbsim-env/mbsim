/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2026 Martin Förg

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

#define _USE_MATH_DEFINES
#include <config.h>
#include "diagram_arrow.h"
#include "diagram_item.h"
#include <QPainter>
#include <QPen>
#include <cmath>

using namespace std;

namespace MBSimGUI {

  DiagramArrow::DiagramArrow(DiagramItem *item1_, DiagramItem *item2_, QGraphicsItem *parent) : QGraphicsLineItem(parent), item1(item1_), item2(item2_)  {
    item1->addDiagramArrow(this);
    item2->addDiagramArrow(this);
    setZValue(1);
  }

  QPainterPath DiagramArrow::shape() const {
    QPainterPath path = QGraphicsLineItem::shape();
    path.addPolygon(head);
    return path;
  }

  void DiagramArrow::updatePosition() {
    setLine(QLineF(mapFromItem(item2, 0, 0), mapFromItem(item1, 0, 0)));
  }

  void DiagramArrow::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *) {

    painter->setPen(QPen(Qt::gray));
    painter->setBrush(Qt::gray);

    QLineF centerLine(mapFromItem(item2, 0, 0), mapFromItem(item1, 0, 0));
    QPointF p1 = item2->polygon().first() + mapFromItem(item2, 0, 0);
    QPointF point1;
    for(int i=1; i<item2->polygon().count(); i++) {
      QPointF p2 = item2->polygon().at(i) + mapFromItem(item2, 0, 0);
      QLineF polyLine = QLineF(p1, p2);
      QLineF::IntersectionType intersectionType = polyLine.intersects(centerLine, &point1);
      if (intersectionType == QLineF::BoundedIntersection)
        break;
      p1 = p2;
    }
    p1 = item1->polygon().first() + mapFromItem(item1, 0, 0);
    QPointF point2;
    for(int i=1; i<item1->polygon().count(); i++) {
      QPointF p2 = item1->polygon().at(i) + mapFromItem(item1, 0, 0);
      QLineF polyLine = QLineF(p1, p2);
      QLineF::IntersectionType intersectionType = polyLine.intersects(centerLine, &point2);
      if (intersectionType == QLineF::BoundedIntersection)
        break;
      p1 = p2;
    }

    setLine(QLineF(point1, point2));

    head.clear();
    head << line().p1();
    double angle = std::atan2(-line().dy(), line().dx());
    head << line().p1() + QPointF(sin(angle+M_PI/3)*10, cos(angle+M_PI/3)*10);
    head << line().p1() + QPointF(sin(angle+2./3*M_PI)*10, cos(angle+2./3*M_PI)*10);
    painter->drawLine(line());
    painter->drawPolygon(head);
  }

}
