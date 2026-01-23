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

#ifndef _DIAGRAM_ARROW__H_
#define _DIAGRAM_ARROW__H_

#include <QGraphicsLineItem>

namespace MBSimGUI {

  class DiagramItem;

  class DiagramArrow : public QGraphicsLineItem {
    public:
      DiagramArrow(DiagramItem *item1_, DiagramItem *item2_, QGraphicsItem *parent=nullptr);

      QPainterPath shape() const override;
      DiagramItem* getFirstDiagramItem() const { return item1; }
      DiagramItem* getSecondDiagramItem() const { return item2; }

      void updatePosition();

    protected:
      void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget=nullptr) override;

    private:
      DiagramItem *item1=nullptr;
      DiagramItem *item2=nullptr;
      QPolygonF head;
  };

}

#endif
