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

#include <octave/oct.h>
#include <octave/octave.h>
#include <octave/parse.h>
#include <octave/toplev.h>
#include <iostream>

class TiXmlElement;

extern std::map<std::string, octave_value> currentParam;

enum ValueType {
  ArbitraryType,
  ScalarType,
  VectorType,
  MatrixType,
  StringType
};

struct Param {
  Param(std::string n, std::string eq, TiXmlElement *e) : name(n), equ(eq), ele(e) {}
  std::string name, equ;
  TiXmlElement *ele;
};

void octaveEvalRet(std::string str, TiXmlElement *e=NULL);
std::string octaveGetRet(ValueType expectedType=ArbitraryType);
void initializeOctave();
int fillParam(std::vector<Param> param);

std::string evalOctaveExpression(const std::string &str);

