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

#ifndef _UNITS_H_
#define _UNITS_H_

class Units {
  protected:
    std::vector<std::string> unit;
    int defaultUnit;
  public:
    Units(int defaultUnit_=0) : defaultUnit(defaultUnit_) { }
    const std::string& getUnit(int i) const {return unit[i];}
    const std::string& getUnit() const {return getUnit(defaultUnit);}
    int getNumberOfUnits() const {return unit.size();}
    int getDefaultUnit() const {return defaultUnit;}
    int find(const std::string &str) const {
      for(int i=0; i<unit.size(); i++)
        if(unit[i]==str)
          return i;
      return -1;
    }
};

class NoUnitUnits : public Units {
  public:
    NoUnitUnits(int defaultUnit=0) : Units(defaultUnit) {
      unit.push_back("");
      unit.push_back("-");
      unit.push_back("%");
    }
};

class TimeUnits : public Units {
  public:
    TimeUnits(int defaultUnit=2) : Units(defaultUnit) {
      unit.push_back("mus");
      unit.push_back("ms");
      unit.push_back("s");
      unit.push_back("sec"); 
      unit.push_back("min");
      unit.push_back("h");
      unit.push_back("d");
    }
};

class MassUnits : public Units {
  public:
    MassUnits(int defaultUnit=2) : Units(defaultUnit) {
      unit.push_back("mg");
      unit.push_back("g"); 
      unit.push_back("kg"); 
      unit.push_back("t"); 
    }
};

class LengthUnits : public Units {
  public:
    LengthUnits(int defaultUnit=4) : Units(defaultUnit) {
      unit.push_back("mum");
      unit.push_back("mm"); 
      unit.push_back("cm"); 
      unit.push_back("dm"); 
      unit.push_back("m"); 
      unit.push_back("km");
    }
};

class AccelerationUnits : public Units {
  public:
    AccelerationUnits(int defaultUnit=0) : Units(defaultUnit) {
      unit.push_back("m/s^2");
    }
};

class AngleUnits : public Units {
  public:
    AngleUnits(int defaultUnit=1) : Units(defaultUnit) {
      unit.push_back("rad"); 
      unit.push_back("degree");
    }
};

#endif

