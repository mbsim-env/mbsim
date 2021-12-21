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

#ifndef _CUSTOM_WIDGETS_H_
#define _CUSTOM_WIDGETS_H_

#include <QComboBox>
#include <QSpinBox>

namespace MBSimGUI {

  class CustomComboBox :public QComboBox {
    public:
      CustomComboBox(QWidget* parent = nullptr) : QComboBox(parent) {
        setFocusPolicy(Qt::StrongFocus);
      }

      void wheelEvent(QWheelEvent *e) override {
        if(hasFocus())
          QComboBox::wheelEvent(e);
      }
  };

  class CustomSpinBox :public QSpinBox {
    public:
      CustomSpinBox(QWidget* parent = nullptr) : QSpinBox(parent) {
        setFocusPolicy(Qt::StrongFocus);
      }

      void wheelEvent(QWheelEvent *e) override {
        if(hasFocus())
          QSpinBox::wheelEvent(e);
      }
  };

}

#endif
