/* Copyright (C) 2004-2006  Martin Förg, Roland Zander, Felix Kahr
 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

 *
 * Contact:
 *   mfoerg@users.berlios.de
 *   rzander@users.berlios.de
 *
 */

#ifndef _ELEMENT_H_
#define _ELEMENT_H_

#include "fmatvec.h"
#include <string>
#include <vector>

#ifdef NO_ISO_14882
#include<fstream.h>
#else 
#include<fstream>
#endif


using namespace std;
using namespace fmatvec;

namespace MBSim {

  class MultiBodySystem;
  /*! 
   *  \brief THE basic class of MBSim
   *
   * Used for definitions of element parameters and interactions like plotting ...
   * 
   * */
  class Element {

    protected:

      MultiBodySystem *mbs;

      /** short name of Element
      */
      string name;
      /** full name of Element
      */
      string fullName;
      /** Name of the output directory
      */
      static string dirName;
      /** file used for output of time dependent data, specified using Element::plotLevel
      */
      ofstream plotfile;

      /** counter for enumeration of output data in Element::plotfile
      */
      int plotNr;
      /**
       * specify Plot Level:\n
       * 0: plot only time\n
       * 1: plot position\n
       * 2: ...
       */ 
      int plotLevel;

      /** file used for output of element parameters, e.g. mass ...
      */
      ofstream parafile;

      /** Vector for Data Interface Base References */
      vector<string> DIBRefs;

    public:

      Element(const string &name);
      virtual ~Element();

      //virtual void plot(double t);
      /*! first definition of plot routine to Element::plotfile\n plotting time, do not overload without explicit call to partent class, or previous outputs will be lost
      */
      virtual void plot(double t, double dt = 1);
      /*! predefinition for output of system parameters in Element:parafile
      */
      virtual void plotParameters();

      virtual void initPlotFiles();
      virtual void closePlotFiles();

      /*!
       * set Element::plotLevel and therewith specifiy outputs in plot-files
       */
      void setPlotLevel(int level) {plotLevel = level;}
      /*!
       * get Element::plotLevel
       */
      int getPlotLevel() {return plotLevel;}


      const string& getName() const { return name; }
      const string& getFullName() const { return fullName; }
      virtual void setFullName(const string &str) {fullName = str;}
      virtual void setName(const string &str) {name = str;}

      MultiBodySystem* getMbs() {return mbs;}
      virtual void setMbs(MultiBodySystem* mbs_) {mbs=mbs_;}

      void addDataInterfaceBaseRef(const string& DIBRef_);
      virtual void initDataInterfaceBase(MultiBodySystem *parentmbs) {};
  };

}

#endif
