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

#ifndef _PARAMETER_PROPERTY_DIALOG_H_
#define _PARAMETER_PROPERTY_DIALOG_H_

#include "property_dialog.h"

class Parameter;
class ScalarParameter;
class VectorParameter;
class MatrixParameter;
class TextWidget;
class ExtWidget;

class ParameterPropertyDialog : public PropertyDialog {

  public:
    ParameterPropertyDialog(Parameter *parameter, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Parameter *parameter);
    virtual void fromWidget(Parameter *parameter);
    void toWidget() {toWidget(parameter);}
    void fromWidget() {fromWidget(parameter);}
  protected:
    Parameter *parameter;
    ExtWidget *name;
};

class ScalarParameterPropertyDialog : public ParameterPropertyDialog {

  public:
    ScalarParameterPropertyDialog(ScalarParameter *parameter, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Parameter *parameter);
    virtual void fromWidget(Parameter *parameter);
  protected:
    ExtWidget *value;
};

class VectorParameterPropertyDialog : public ParameterPropertyDialog {

  public:
    VectorParameterPropertyDialog(VectorParameter *parameter, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Parameter *parameter);
    virtual void fromWidget(Parameter *parameter);
  protected:
    ExtWidget *value;
};

class MatrixParameterPropertyDialog : public ParameterPropertyDialog {

  public:
    MatrixParameterPropertyDialog(MatrixParameter *parameter, QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Parameter *parameter);
    virtual void fromWidget(Parameter *parameter);
  protected:
    ExtWidget *value;
};

#endif
