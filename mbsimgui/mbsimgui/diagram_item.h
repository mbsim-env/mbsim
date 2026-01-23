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

#ifndef _DIAGRAM_ITEM__H_
#define _DIAGRAM_ITEM__H_

#include <QGraphicsPolygonItem>

namespace MBSimGUI {

  class DiagramArrow;

  class DiagramItem : public QGraphicsPolygonItem {
    public:
      DiagramItem(const QPolygonF &polygon, QGraphicsItem *parent=nullptr);

      void setItem(QGraphicsItem *item_) { item = item_; }
      QGraphicsItem* getItem() { return item; }
      void addDiagramArrow(DiagramArrow *arrow) {arrows.append(arrow); }
      void removeDiagramArrow(DiagramArrow *arrow) { arrows.removeAll(arrow); }
      void removeDiagramArrows();

    protected:
      QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

    private:
      QGraphicsItem *item;
      QVector<DiagramArrow*> arrows;
  };

}

#endif
