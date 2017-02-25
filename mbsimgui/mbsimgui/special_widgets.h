/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef _SPECIAL_WIDGETS_H_
#define _SPECIAL_WIDGETS_H_

#include "widget.h"
#include "custom_widgets.h"

namespace MBSimGUI {

  class ExtWidget;
  class ExtProperty;

  class OneDimVecArrayWidget : public Widget {
      std::vector<ExtWidget*> ele;
    public:
      OneDimVecArrayWidget(int size=3, int m=3);
      const std::vector<ExtWidget*>& getArray() const { return ele; }
      void resize_(int size, int m, int n);
      void resize_(int m, int n);
  };

  class OneDimMatArrayWidget : public Widget {
      std::vector<ExtWidget*> ele;
    public:
      OneDimMatArrayWidget(int size=3, int m=3, int n=1);
      const std::vector<ExtWidget*>& getArray() const { return ele; }
      void resize_(int size, int m, int n);
      void resize_(int m, int n);
  };

  class TwoDimMatArrayWidget: public Widget {
    protected:
      std::vector<std::vector<ExtWidget*> > ele;
    public:
      TwoDimMatArrayWidget(int size=3, int m=3, int n=1);
      const std::vector<std::vector<ExtWidget*> >& getArray() const { return ele; }
      void resize_(int size, int m, int n);
      void resize_(int m, int n);
 };

  class OneDimVecArrayWidgetFactory : public WidgetFactory {
    public:
      OneDimVecArrayWidgetFactory();
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
  };

  class OneDimMatArrayWidgetFactory : public WidgetFactory {
    public:
      OneDimMatArrayWidgetFactory();
      //OneDimMatArrayWidgetFactory(const std::vector<std::vector<QString> > &A);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      //std::vector<std::vector<QString> > A;
  };

  class TwoDimMatArrayWidgetFactory : public WidgetFactory {
    public:
      TwoDimMatArrayWidgetFactory();
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
  };

}

#endif
