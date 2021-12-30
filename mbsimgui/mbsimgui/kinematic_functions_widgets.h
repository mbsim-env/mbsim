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

#ifndef _KINEMATIC_FUNCTION_WIDGETS_H_
#define _KINEMATIC_FUNCTION_WIDGETS_H_

#include "function_widget.h"

namespace MBSimGUI {

  class ExtWidget;

  class TranslationAlongXAxisWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TranslationAlongXAxisWidget, FunctionWidget, MBSIM%"TranslationAlongXAxis", "dummy");

    public:
      TranslationAlongXAxisWidget() = default;
      int getArg1Size() const override { return 1; }
  };

  class TranslationAlongYAxisWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TranslationAlongYAxisWidget, FunctionWidget, MBSIM%"TranslationAlongYAxis", "dummy");

    public:
      TranslationAlongYAxisWidget() = default;
      int getArg1Size() const override { return 1; }
  };

  class TranslationAlongZAxisWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TranslationAlongZAxisWidget, FunctionWidget, MBSIM%"TranslationAlongZAxis", "dummy");

    public:
      TranslationAlongZAxisWidget() = default;
      int getArg1Size() const override { return 1; }
  };

  class TranslationAlongAxesXYWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TranslationAlongAxesXYWidget, FunctionWidget, MBSIM%"TranslationAlongAxesXY", "dummy");

    public:
      TranslationAlongAxesXYWidget() = default;
      int getArg1Size() const override { return 2; }
  };

  class TranslationAlongAxesYZWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TranslationAlongAxesYZWidget, FunctionWidget, MBSIM%"TranslationAlongAxesYZ", "dummy");

    public:
      TranslationAlongAxesYZWidget() = default;
      int getArg1Size() const override { return 2; }
  };

  class TranslationAlongAxesXZWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TranslationAlongAxesXZWidget, FunctionWidget, MBSIM%"TranslationAlongAxesXZ", "dummy");

    public:
      TranslationAlongAxesXZWidget() = default;
      int getArg1Size() const override { return 2; }
  };

  class TranslationAlongAxesXYZWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TranslationAlongAxesXYZWidget, FunctionWidget, MBSIM%"TranslationAlongAxesXYZ", "dummy");

    public:
      TranslationAlongAxesXYZWidget() = default;
      int getArg1Size() const override { return 3; }
  };

  class TranslationAlongFixedAxisWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TranslationAlongFixedAxisWidget, FunctionWidget, MBSIM%"TranslationAlongFixedAxis", "dummy");

    public:
      TranslationAlongFixedAxisWidget();
      int getArg1Size() const override { return 1; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a;
  };

  class LinearTranslationWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(LinearTranslationWidget, FunctionWidget, MBSIM%"LinearTranslation", "dummy");

    public:
      LinearTranslationWidget(int m=1, int n=1);
      int getArg1Size() const override;
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *A, *b;
  };

  class RotationAboutXAxisWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RotationAboutXAxisWidget, FunctionWidget, MBSIM%"RotationAboutXAxis", "dummy");

    public:
      RotationAboutXAxisWidget() = default;
      int getArg1Size() const override { return 1; }
  };

  class RotationAboutYAxisWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RotationAboutYAxisWidget, FunctionWidget, MBSIM%"RotationAboutYAxis", "dummy");

    public:
      RotationAboutYAxisWidget() = default;
      int getArg1Size() const override { return 1; }
  };

  class RotationAboutZAxisWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RotationAboutZAxisWidget, FunctionWidget, MBSIM%"RotationAboutZAxis", "dummy");

    public:
      RotationAboutZAxisWidget() = default;
      int getArg1Size() const override { return 1; }
  };

  class RotationAboutAxesXYWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RotationAboutAxesXYWidget, FunctionWidget, MBSIM%"RotationAboutAxesXY", "dummy");

    public:
      RotationAboutAxesXYWidget() = default;
      int getArg1Size() const override { return 2; }
  };

  class RotationAboutAxesYZWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RotationAboutAxesYZWidget, FunctionWidget, MBSIM%"RotationAboutAxesYZ", "dummy");

    public:
      RotationAboutAxesYZWidget() = default;
      int getArg1Size() const override { return 2; }
  };

  class RotationAboutAxesXZWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RotationAboutAxesXZWidget, FunctionWidget, MBSIM%"RotationAboutAxesXZ", "dummy");

    public:
      RotationAboutAxesXZWidget() = default;
      int getArg1Size() const override { return 2; }
  };

  class RotationAboutAxesXYZWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RotationAboutAxesXYZWidget, FunctionWidget, MBSIM%"RotationAboutAxesXYZ", "dummy");

    public:
      RotationAboutAxesXYZWidget() = default;
      int getArg1Size() const override { return 3; }
  };

  class RotationAboutAxesZXZWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RotationAboutAxesZXZWidget, FunctionWidget, MBSIM%"RotationAboutAxesZXZ", "dummy");

    public:
      RotationAboutAxesZXZWidget() = default;
      int getArg1Size() const override { return 3; }
  };

  class RotationAboutAxesZYXWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RotationAboutAxesZYXWidget, FunctionWidget, MBSIM%"RotationAboutAxesZYX", "dummy");

    public:
      RotationAboutAxesZYXWidget() = default;
      int getArg1Size() const override { return 3; }
  };

  class RotationAboutFixedAxisWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RotationAboutFixedAxisWidget, FunctionWidget, MBSIM%"RotationAboutFixedAxis", "dummy");

    public:
      RotationAboutFixedAxisWidget();
      int getArg1Size() const override { return 1; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a;
  };

}

#endif
