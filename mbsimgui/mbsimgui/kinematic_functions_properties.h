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

#ifndef _KINEMATIC_FUNCTIONS_PROPERTIES_H_
#define _KINEMATIC_FUNCTIONS_PROPERTIES_H_

#include "function_property.h"
#include "extended_properties.h"

namespace MBSimGUI {

  class TranslationAlongXAxis: public Function {
    public:
      TranslationAlongXAxis(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new TranslationAlongXAxis(*this);}
      int getArg1Size() const {return 1;}
      inline std::string getType() const { return "TranslationAlongXAxis"; }
  };

  class TranslationAlongYAxis: public Function {
    public:
      TranslationAlongYAxis(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new TranslationAlongYAxis(*this);}
      int getArg1Size() const {return 1;}
      inline std::string getType() const { return "TranslationAlongYAxis"; }
  };

  class TranslationAlongZAxis: public Function {
    public:
      TranslationAlongZAxis(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new TranslationAlongZAxis(*this);}
      int getArg1Size() const {return 1;}
      inline std::string getType() const { return "TranslationAlongZAxis"; }
  };

  class TranslationAlongAxesXY: public Function {
    public:
      TranslationAlongAxesXY(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new TranslationAlongAxesXY(*this);}
      int getArg1Size() const {return 2;}
      inline std::string getType() const { return "TranslationAlongAxesXY"; }
  };

  class TranslationAlongAxesYZ: public Function {
    public:
      TranslationAlongAxesYZ(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new TranslationAlongAxesYZ(*this);}
      int getArg1Size() const {return 2;}
      inline std::string getType() const { return "TranslationAlongAxesYZ"; }
  };

  class TranslationAlongAxesXZ: public Function {
    public:
      TranslationAlongAxesXZ(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new TranslationAlongAxesXZ(*this);}
      int getArg1Size() const {return 2;}
      inline std::string getType() const { return "TranslationAlongAxesXZ"; }
  };

  class TranslationAlongAxesXYZ: public Function {
    public:
      TranslationAlongAxesXYZ(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new TranslationAlongAxesXYZ(*this);}
      int getArg1Size() const {return 3;}
      inline std::string getType() const { return "TranslationAlongAxesXYZ"; }
  };

  class TranslationAlongFixedAxis : public Function {
    public:
      TranslationAlongFixedAxis(const std::string &name="");
      virtual PropertyInterface* clone() const {return new TranslationAlongFixedAxis(*this);}
      int getArg1Size() const {return 1;}
      inline std::string getType() const { return "TranslationAlongFixedAxis"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty a;
  };

  class LinearTranslation : public Function {
    public:
      LinearTranslation(const std::string &name="", int m=1, int n=1);
      virtual PropertyInterface* clone() const {return new LinearTranslation(*this);}
      int getArg1Size() const;
      inline std::string getType() const { return "LinearTranslation"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty A, b;
  };

  class RotationAboutXAxis : public Function {
    public:
      RotationAboutXAxis(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new RotationAboutXAxis(*this);}
      int getArg1Size() const {return 1;}
      inline std::string getType() const { return "RotationAboutXAxis"; }
  };

  class RotationAboutYAxis : public Function {
    public:
      RotationAboutYAxis(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new RotationAboutYAxis(*this);}
      int getArg1Size() const {return 1;}
      inline std::string getType() const { return "RotationAboutYAxis"; }
  };

  class RotationAboutZAxis : public Function {
    public:
      RotationAboutZAxis(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new RotationAboutZAxis(*this);}
      int getArg1Size() const {return 1;}
      inline std::string getType() const { return "RotationAboutZAxis"; }
  };

  class RotationAboutAxesXY : public Function {
    public:
      RotationAboutAxesXY(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new RotationAboutAxesXY(*this);}
      int getArg1Size() const {return 2;}
      inline std::string getType() const { return "RotationAboutAxesXY"; }
  };

  class RotationAboutAxesYZ : public Function {
    public:
      RotationAboutAxesYZ(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new RotationAboutAxesYZ(*this);}
      int getArg1Size() const {return 2;}
      inline std::string getType() const { return "RotationAboutAxesYZ"; }
  };

  class RotationAboutAxesXZ : public Function {
    public:
      RotationAboutAxesXZ(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new RotationAboutAxesXZ(*this);}
      int getArg1Size() const {return 2;}
      inline std::string getType() const { return "RotationAboutAxesXZ"; }
  };

  class RotationAboutAxesXYZ : public Function {
    public:
      RotationAboutAxesXYZ(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new RotationAboutAxesXYZ(*this);}
      int getArg1Size() const {return 3;}
      inline std::string getType() const { return "RotationAboutAxesXYZ"; }
  };

  class RotationAboutAxesZXZ : public Function {
    public:
      RotationAboutAxesZXZ(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new RotationAboutAxesZXZ(*this);}
      int getArg1Size() const {return 3;}
      inline std::string getType() const { return "RotationAboutAxesZXZ"; }
  };

  class RotationAboutAxesZYX : public Function {
    public:
      RotationAboutAxesZYX(const std::string &name="") : Function(name) { }
      virtual PropertyInterface* clone() const {return new RotationAboutAxesZYX(*this);}
      int getArg1Size() const {return 3;}
      inline std::string getType() const { return "RotationAboutAxesZYX"; }
  };

  class RotationAboutFixedAxis : public Function {
    public:
      RotationAboutFixedAxis(const std::string &name="");
      virtual PropertyInterface* clone() const {return new RotationAboutFixedAxis(*this);}
      int getArg1Size() const {return 1;}
      inline std::string getType() const { return "RotationAboutFixedAxis"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty a;
  };

}

#endif
