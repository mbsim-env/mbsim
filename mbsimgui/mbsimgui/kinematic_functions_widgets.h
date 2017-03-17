/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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

#ifndef _KINEMATIC_FUNCTION_WIDGETS_H_
#define _KINEMATIC_FUNCTION_WIDGETS_H_

#include "function_widget.h"

namespace MBSimGUI {

  class ExtWidget;

  class TranslationAlongXAxisWidget : public FunctionWidget {

    public:
      TranslationAlongXAxisWidget() { }
      int getArg1Size() const {return 1;}
      std::string getType() const { return "TranslationAlongXAxis"; }
  };

  class TranslationAlongYAxisWidget : public FunctionWidget {

    public:
      TranslationAlongYAxisWidget() { }
      int getArg1Size() const {return 1;}
      std::string getType() const { return "TranslationAlongYAxis"; }
  };

  class TranslationAlongZAxisWidget : public FunctionWidget {

    public:
      TranslationAlongZAxisWidget() { }
      int getArg1Size() const {return 1;}
      std::string getType() const { return "TranslationAlongZAxis"; }
  };

  class TranslationAlongAxesXYWidget : public FunctionWidget {

    public:
      TranslationAlongAxesXYWidget() { }
      int getArg1Size() const {return 2;}
  };

  class TranslationAlongAxesYZWidget : public FunctionWidget {

    public:
      TranslationAlongAxesYZWidget() { }
      int getArg1Size() const {return 2;}
  };

  class TranslationAlongAxesXZWidget : public FunctionWidget {

    public:
      TranslationAlongAxesXZWidget() { }
      int getArg1Size() const {return 2;}
  };

  class TranslationAlongAxesXYZWidget : public FunctionWidget {

    public:
      TranslationAlongAxesXYZWidget() { }
      int getArg1Size() const {return 3;}
  };

  class TranslationAlongFixedAxisWidget : public FunctionWidget {

    friend class TranslationAlongFixedAxis;

    public:
    TranslationAlongFixedAxisWidget();
    int getArg1Size() const {return 1;}
    protected:
    ExtWidget *a;
  };

  class LinearTranslationWidget : public FunctionWidget {

    friend class LinearTranslation;

    public:
    LinearTranslationWidget(int m=1, int n=1);
    int getArg1Size() const;
    void resize_(int m, int n);
    protected:
    ExtWidget *A, *b;
  };

  class RotationAboutXAxisWidget : public FunctionWidget {

    public:
      RotationAboutXAxisWidget() { }
      int getArg1Size() const {return 1;}
  };

  class RotationAboutYAxisWidget : public FunctionWidget {

    public:
      RotationAboutYAxisWidget() { }
      int getArg1Size() const {return 1;}
  };

  class RotationAboutZAxisWidget : public FunctionWidget {

    public:
      RotationAboutZAxisWidget() { }
      int getArg1Size() const {return 1;}
  };

  class RotationAboutAxesXYWidget : public FunctionWidget {

    public:
      RotationAboutAxesXYWidget() { }
      int getArg1Size() const {return 2;}
  };

  class RotationAboutAxesYZWidget : public FunctionWidget {

    public:
      RotationAboutAxesYZWidget() { }
      int getArg1Size() const {return 2;}
  };

  class RotationAboutAxesXZWidget : public FunctionWidget {

    public:
      RotationAboutAxesXZWidget() { }
      int getArg1Size() const {return 2;}
  };

  class RotationAboutAxesXYZWidget : public FunctionWidget {

    public:
      RotationAboutAxesXYZWidget() { }
      int getArg1Size() const {return 3;}
  };

  class RotationAboutAxesZYXWidget : public FunctionWidget {

    public:
      RotationAboutAxesZYXWidget() { }
      int getArg1Size() const {return 3;}
  };

  class RotationAboutAxesZXZWidget : public FunctionWidget {

    public:
      RotationAboutAxesZXZWidget() { }
      int getArg1Size() const {return 3;}
  };

  class RotationAboutFixedAxisWidget : public FunctionWidget {

    friend class RotationAboutFixedAxis;

    public:
    RotationAboutFixedAxisWidget();
    int getArg1Size() const {return 1;}
    protected:
    ExtWidget *a;
  };

}

#endif
