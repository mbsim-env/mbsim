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

#ifndef _FUNCTION_PROPERTIES_H_
#define _FUNCTION_PROPERTIES_H_

#include "function_property.h"
#include "extended_properties.h"

namespace MBSimGUI {

  class ConstantFunctionProperty : public FunctionProperty {
    public:
      ConstantFunctionProperty(int m=1);
      virtual Property* clone() const {return new ConstantFunctionProperty(*this);}
      inline std::string getType() const { return "ConstantFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty a0;
  };

  class LinearFunctionProperty : public FunctionProperty {
    public:
      LinearFunctionProperty(int m=1);
      virtual Property* clone() const {return new LinearFunctionProperty(*this);}
      inline std::string getType() const { return "LinearFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty a0, a1;
  };

  class QuadraticFunctionProperty : public FunctionProperty {
    public:
      QuadraticFunctionProperty(int m=1);
      virtual Property* clone() const {return new QuadraticFunctionProperty(*this);}
      inline std::string getType() const { return "QuadraticFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty a0, a1, a2;
  };

  class PolynomFunctionProperty : public FunctionProperty {
    public:
      PolynomFunctionProperty(int m=1);
      virtual Property* clone() const {return new PolynomFunctionProperty(*this);}
      inline std::string getType() const { return "PolynomFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty a;
  };

  class SinusoidalFunctionProperty : public FunctionProperty {
    public:
      SinusoidalFunctionProperty(int m=1);
      virtual Property* clone() const {return new SinusoidalFunctionProperty(*this);}
      inline std::string getType() const { return "SinusoidalFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty a, f, p, o;
  };

  //class StepFunctionProperty : public FunctionProperty {
  //}

  //class PositiveFunctionProperty : public FunctionProperty {
  //}

  class ModuloFunctionProperty : public FunctionProperty {
    public:
      ModuloFunctionProperty();
      virtual Property* clone() const {return new ModuloFunctionProperty(*this);}
      inline std::string getType() const { return "ModuloFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty denom;
  };

  class AbsoluteValueFunctionProperty : public FunctionProperty {
    public:
      AbsoluteValueFunctionProperty();
      virtual Property* clone() const {return new AbsoluteValueFunctionProperty(*this);}
      int getArg1Size() const;
      inline std::string getType() const { return "AbsoluteValueFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty function;
  };

  class PointSymmetricFunctionProperty : public FunctionProperty {
    public:
      PointSymmetricFunctionProperty();
      virtual Property* clone() const {return new PointSymmetricFunctionProperty(*this);}
      int getArg1Size() const;
      inline std::string getType() const { return "PointSymmetricFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty function;
  };
  
  class LineSymmetricFunctionProperty : public FunctionProperty {
    public:
      LineSymmetricFunctionProperty();
      virtual Property* clone() const {return new LineSymmetricFunctionProperty(*this);}
      int getArg1Size() const;
      inline std::string getType() const { return "LineSymmetricFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty function;
  };
  
  class ScaledFunctionProperty : public FunctionProperty {
    public:
      ScaledFunctionProperty();
      virtual Property* clone() const {return new ScaledFunctionProperty(*this);}
      int getArg1Size() const;
      inline std::string getType() const { return "ScaledFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty function, factor;
  };

  class SummationFunctionProperty : public FunctionProperty {

    public:
      SummationFunctionProperty();
      virtual Property* clone() const {return new SummationFunctionProperty(*this);}
      inline std::string getType() const { return "SummationFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty functions;
  };

  class VectorValuedFunctionProperty : public FunctionProperty {
    public:
      VectorValuedFunctionProperty(int m=0);
      virtual Property* clone() const {return new VectorValuedFunctionProperty(*this);}
      inline std::string getType() const { return "VectorValuedFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty functions;
  };

  class NestedFunctionProperty : public FunctionProperty {
    public:
      //NestedFunctionProperty(const std::string &ext, const std::vector<Property*> &property);
      NestedFunctionProperty(PropertyFactory *factoryo, PropertyFactory *factoryi);
      virtual Property* clone() const {return new NestedFunctionProperty(*this);}
      int getArg1Size() const;
      inline std::string getType() const { return "NestedFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      std::string ext;
      ExtProperty fo, fi;
  };

  class PiecewiseDefinedFunctionProperty : public FunctionProperty {
    public:
      PiecewiseDefinedFunctionProperty();
      virtual Property* clone() const {return new PiecewiseDefinedFunctionProperty(*this);}
      inline std::string getType() const { return "PiecewiseDefinedFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty functions;
      ExtProperty contDiff;
  };

  class SymbolicFunctionProperty : public FunctionProperty {
    public:
      SymbolicFunctionProperty(const std::string &ext, const std::vector<std::string> &var, int m);
      virtual Property* clone() const {return new SymbolicFunctionProperty(*this);}
      int getArg1Size() const;
      int getArg2Size() const;
      inline std::string getType() const { return "SymbolicFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      std::string ext;
      ExtProperty f;
      std::vector<ExtProperty> argname, argdim;
  };

  class TabularFunctionProperty : public FunctionProperty {
    public:
      TabularFunctionProperty();
      virtual Property* clone() const {return new TabularFunctionProperty(*this);}
      inline std::string getType() const { return "TabularFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ChoiceProperty2 choice;
  };

  class TwoDimensionalTabularFunctionProperty : public FunctionProperty {
    public:
      TwoDimensionalTabularFunctionProperty();
      virtual Property* clone() const {return new TwoDimensionalTabularFunctionProperty(*this);}
      inline std::string getType() const { return "TwoDimensionalTabularFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty x, y ,xy;
  };

  class PiecewisePolynomFunctionProperty : public FunctionProperty {
    public:
      PiecewisePolynomFunctionProperty();
      virtual Property* clone() const {return new PiecewisePolynomFunctionProperty(*this);}
      inline std::string getType() const { return "PiecewisePolynomFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ChoiceProperty2 choice;
      ExtProperty method;
  };

  class LinearSpringDamperForceProperty : public FunctionProperty {
    public:
      LinearSpringDamperForceProperty();
      virtual Property* clone() const {return new LinearSpringDamperForceProperty(*this);}
      inline std::string getType() const { return "LinearSpringDamperForce"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty c, d, l0;
  };

  class NonlinearSpringDamperForceProperty : public FunctionProperty {
    public:
      NonlinearSpringDamperForceProperty();
      virtual Property* clone() const {return new NonlinearSpringDamperForceProperty(*this);}
      inline std::string getType() const { return "NonlinearSpringDamperForce"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty g, gd;
  };

  class LinearRegularizedBilateralConstraintProperty: public FunctionProperty {
    public:
      LinearRegularizedBilateralConstraintProperty();
      virtual Property* clone() const {return new LinearRegularizedBilateralConstraintProperty(*this);}
      std::string getType() const { return "LinearRegularizedBilateralConstraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    private:
      ExtProperty c, d;
  };

  class LinearRegularizedUnilateralConstraintProperty: public FunctionProperty {
    public:
      LinearRegularizedUnilateralConstraintProperty(); 

      virtual Property* clone() const {return new LinearRegularizedUnilateralConstraintProperty(*this);}
      virtual std::string getType() const { return "LinearRegularizedUnilateralConstraint"; }

      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    private:
      ExtProperty c, d;
  };

  class LinearRegularizedCoulombFrictionProperty: public FunctionProperty {
    public:
      LinearRegularizedCoulombFrictionProperty(); 

      virtual Property* clone() const {return new LinearRegularizedCoulombFrictionProperty(*this);}
      virtual std::string getType() const { return "LinearRegularizedCoulombFriction"; }

      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    private:
      ExtProperty gd, mu;
  };

}

#endif

