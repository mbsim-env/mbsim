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
      TranslationAlongXAxisWidget() = default;
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongXAxis"; }
  };

  class TranslationAlongYAxisWidget : public FunctionWidget {

    public:
      TranslationAlongYAxisWidget() = default;
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongYAxis"; }
  };

  class TranslationAlongZAxisWidget : public FunctionWidget {

    public:
      TranslationAlongZAxisWidget() = default;
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongZAxis"; }
  };

  class TranslationAlongAxesXYWidget : public FunctionWidget {

    public:
      TranslationAlongAxesXYWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongAxesXY"; }
  };

  class TranslationAlongAxesYZWidget : public FunctionWidget {

    public:
      TranslationAlongAxesYZWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongAxesYZ"; }
  };

  class TranslationAlongAxesXZWidget : public FunctionWidget {

    public:
      TranslationAlongAxesXZWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongAxesXZ"; }
  };

  class TranslationAlongAxesXYZWidget : public FunctionWidget {

    public:
      TranslationAlongAxesXYZWidget() = default;
      int getArg1Size() const override { return 3; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongAxesXYZ"; }
  };

  class TranslationAlongFixedAxisWidget : public FunctionWidget {

    public:
      TranslationAlongFixedAxisWidget();
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongFixedAxis"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a;
  };

  class LinearTranslationWidget : public FunctionWidget {

    public:
      LinearTranslationWidget(int m=1, int n=1);
      int getArg1Size() const override;
      void resize_(int m, int n) override;
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearTranslation"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *A, *b;
  };

  class RotationAboutXAxisWidget : public FunctionWidget {

    public:
      RotationAboutXAxisWidget() = default;
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutXAxis"; }
  };

  class RotationAboutYAxisWidget : public FunctionWidget {

    public:
      RotationAboutYAxisWidget() = default;
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutYAxis"; }
  };

  class RotationAboutZAxisWidget : public FunctionWidget {

    public:
      RotationAboutZAxisWidget() = default;
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutZAxis"; }
  };

  class RotationAboutAxesXYWidget : public FunctionWidget {

    public:
      RotationAboutAxesXYWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesXY"; }
  };

  class RotationAboutAxesYZWidget : public FunctionWidget {

    public:
      RotationAboutAxesYZWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesYZ"; }
  };

  class RotationAboutAxesXZWidget : public FunctionWidget {

    public:
      RotationAboutAxesXZWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesXZ"; }
  };

  class RotationAboutAxesXYZWidget : public FunctionWidget {

    public:
      RotationAboutAxesXYZWidget() = default;
      int getArg1Size() const override { return 3; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesXYZ"; }
  };

  class RotationAboutAxesZXZWidget : public FunctionWidget {

    public:
      RotationAboutAxesZXZWidget() = default;
      int getArg1Size() const override { return 3; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesZXZ"; }
  };

  class RotationAboutAxesZYXWidget : public FunctionWidget {

    public:
      RotationAboutAxesZYXWidget() = default;
      int getArg1Size() const override { return 3; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesZYX"; }
  };

  class RotationAboutFixedAxisWidget : public FunctionWidget {

    public:
      RotationAboutFixedAxisWidget();
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutFixedAxis"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a;
  };

}

#endif
