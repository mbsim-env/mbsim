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
      QString getType() const override { return "Translation along x axis"; }
  };

  class TranslationAlongYAxisWidget : public FunctionWidget {

    public:
      TranslationAlongYAxisWidget() = default;
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongYAxis"; }
      QString getType() const override { return "Translation along y axis"; }
  };

  class TranslationAlongZAxisWidget : public FunctionWidget {

    public:
      TranslationAlongZAxisWidget() = default;
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongZAxis"; }
      QString getType() const override { return "Translation along z axis"; }
  };

  class TranslationAlongAxesXYWidget : public FunctionWidget {

    public:
      TranslationAlongAxesXYWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongAxesXY"; }
      QString getType() const override { return "Translation along axes x and y"; }
  };

  class TranslationAlongAxesYZWidget : public FunctionWidget {

    public:
      TranslationAlongAxesYZWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongAxesYZ"; }
      QString getType() const override { return "Translation along axes y and z"; }
  };

  class TranslationAlongAxesXZWidget : public FunctionWidget {

    public:
      TranslationAlongAxesXZWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongAxesXZ"; }
      QString getType() const override { return "Translation along axes x and z"; }
  };

  class TranslationAlongAxesXYZWidget : public FunctionWidget {

    public:
      TranslationAlongAxesXYZWidget() = default;
      int getArg1Size() const override { return 3; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongAxesXYZ"; }
      QString getType() const override { return "Translation along axes x, y and z"; }
  };

  class TranslationAlongFixedAxisWidget : public FunctionWidget {

    public:
      TranslationAlongFixedAxisWidget();
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TranslationAlongFixedAxis"; }
      QString getType() const override { return "Translation along fixed axis"; }
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
      QString getType() const override { return "Linear translation"; }
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
      QString getType() const override { return "Rotation about x axis"; }
  };

  class RotationAboutYAxisWidget : public FunctionWidget {

    public:
      RotationAboutYAxisWidget() = default;
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutYAxis"; }
      QString getType() const override { return "Rotation about y axis"; }
  };

  class RotationAboutZAxisWidget : public FunctionWidget {

    public:
      RotationAboutZAxisWidget() = default;
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutZAxis"; }
      QString getType() const override { return "Rotation about z axis"; }
  };

  class RotationAboutAxesXYWidget : public FunctionWidget {

    public:
      RotationAboutAxesXYWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesXY"; }
      QString getType() const override { return "Rotation about axes x and y"; }
  };

  class RotationAboutAxesYZWidget : public FunctionWidget {

    public:
      RotationAboutAxesYZWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesYZ"; }
      QString getType() const override { return "Rotation about axes y and z"; }
  };

  class RotationAboutAxesXZWidget : public FunctionWidget {

    public:
      RotationAboutAxesXZWidget() = default;
      int getArg1Size() const override { return 2; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesXZ"; }
      QString getType() const override { return "Rotation about axes x and z"; }
  };

  class RotationAboutAxesXYZWidget : public FunctionWidget {

    public:
      RotationAboutAxesXYZWidget() = default;
      int getArg1Size() const override { return 3; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesXYZ"; }
      QString getType() const override { return "Rotation about axes x, y and z"; }
  };

  class RotationAboutAxesZXZWidget : public FunctionWidget {

    public:
      RotationAboutAxesZXZWidget() = default;
      int getArg1Size() const override { return 3; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesZXZ"; }
      QString getType() const override { return "Rotation about axes z, x and z"; }
  };

  class RotationAboutAxesZYXWidget : public FunctionWidget {

    public:
      RotationAboutAxesZYXWidget() = default;
      int getArg1Size() const override { return 3; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutAxesZYX"; }
      QString getType() const override { return "Rotation about axes z, y and x"; }
  };

  class RotationAboutFixedAxisWidget : public FunctionWidget {

    public:
      RotationAboutFixedAxisWidget();
      int getArg1Size() const override { return 1; }
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RotationAboutFixedAxis"; }
      QString getType() const override { return "Rotation about fixed axis"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a;
  };

}

#endif
