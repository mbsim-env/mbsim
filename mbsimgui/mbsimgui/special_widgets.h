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

  class OneDimBasicMatArrayWidget : public Widget {
    public:
      virtual const std::vector<ExtWidget*>& getArray() const = 0;
      virtual void resize_(int size, int m=3, int n=1) = 0;
    protected:
  };

  class OneDimMatColsVarArrayWidget : public OneDimBasicMatArrayWidget {
      std::vector<ExtWidget*> ele;
    public:
      OneDimMatColsVarArrayWidget(int size=3, int m=3, int n=1);
      const std::vector<ExtWidget*>& getArray() const { return ele; }
      void resize_(int size, int m=3, int n=1);
  };

  class OneDimSqrMatSizeVarArrayWidget : public OneDimBasicMatArrayWidget {
      std::vector<ExtWidget*> ele;
    public:
      OneDimSqrMatSizeVarArrayWidget(int size=3, int n=1);
      const std::vector<ExtWidget*>& getArray() const { return ele; }
      void resize_(int size, int m=3, int n=1);
  };

  class OneDimVarSizeMatColsVarArrayWidget : public OneDimBasicMatArrayWidget {
    Q_OBJECT
    protected:
      OneDimMatColsVarArrayWidget *widget;
      CustomSpinBox *sizeCombo;
      int m;
    public:
      OneDimVarSizeMatColsVarArrayWidget(int size, int m, int n);
      const std::vector<ExtWidget*>& getArray() const { return widget->getArray(); }
      void resize_(int size, int m=3, int n=1);
    public slots:
      void currentIndexChanged(int);
    signals:
      void sizeChanged(int);
  };

  class OneDimVarSizeSqrMatSizeVarArrayWidget : public OneDimBasicMatArrayWidget {
    Q_OBJECT
    protected:
      OneDimSqrMatSizeVarArrayWidget *widget;
      CustomSpinBox *sizeCombo;
    public:
      OneDimVarSizeSqrMatSizeVarArrayWidget(int size, int n);
      const std::vector<ExtWidget*>& getArray() const { return widget->getArray(); }
      void resize_(int size, int m=3, int n=1);
    public slots:
      void currentIndexChanged(int);
    signals:
      void sizeChanged(int);
  };

  class TwoDimBasicMatArrayWidget : public Widget {
    public:
      virtual const std::vector<std::vector<ExtWidget*> >& getArray() const = 0;
      virtual void resize_(int size, int m=3, int n=1) = 0;
    protected:
  };

  class TwoDimSqrMatSizeVarArrayWidget: public TwoDimBasicMatArrayWidget {
    protected:
      std::vector<std::vector<ExtWidget*> > ele;
    public:
      TwoDimSqrMatSizeVarArrayWidget(int size=3, int n=1);
      const std::vector<std::vector<ExtWidget*> >& getArray() const { return ele; }
      void resize_(int size, int m=3, int n=1);
 };

 class TwoDimVarSizeSqrMatSizeVarArrayWidget : public TwoDimBasicMatArrayWidget {
    Q_OBJECT
    protected:
      TwoDimSqrMatSizeVarArrayWidget *widget;
      CustomSpinBox *sizeCombo;
    public:
      TwoDimVarSizeSqrMatSizeVarArrayWidget(int size, int n);
      const std::vector<std::vector<ExtWidget*> >& getArray() const { return widget->getArray(); }
      void resize_(int size, int m=3, int n=1);
    public slots:
      void currentIndexChanged(int);
    signals:
      void sizeChanged(int);
  };


}

#endif
