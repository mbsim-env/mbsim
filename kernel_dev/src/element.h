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
#include<string>
#include<vector>

#ifdef NO_ISO_14882
#include<fstream.h>
#else 
#include<fstream>
#endif

using namespace std;
using namespace fmatvec;

/*! \brief Namespace MBSim */
namespace MBSim {

  class MultiBodySystem;
  /*! 
   * \brief THE basic class of MBSim
   *
   * Used for definitions of general element parameters and general interactions (e.g. plotting)
   */
  class Element {

    protected:
      /* Short name of Element */
      string name;
      /* Full name of Element */
      string fullName;
      /* Name of the output directory */
      static string dirName;
      /* File used for output of time dependent data, specified using Element::plotLevel */
      ofstream plotfile;
      /* Counter for enumeration of output data in Element::plotfile */
      int plotNr;
      /*
       * Specify Plot Level:\n
       * 0: plot only time\n
       * 1: plot position\n
       * 2: ...
       */ 
      int plotLevel;
      /* Output-precision of ostream */
      int plotPrec;
      /* File used for output of element parameters, e.g. mass ... */
      ofstream parafile;
      /* Vector for Data Interface Base References */
      vector<string> DIBRefs;

    public:
	  /*! Constructor */	
      Element(const string &name);
      /*! Destructor */
      virtual ~Element();
      /*! 
       * First definition of plot routine to Element::plotfile\n plotting time
       * Do not overload without explicit call to parent class; otherwise previous outputs will be lost
       */
      virtual void plot(double t, double dt = 1);
      /*! Predefinition for output of system parameters in Element:parafile */
      virtual void plotParameters();
	  /*! Initialises plotfiles */
      virtual void initPlotFiles();
      /*! Closes plotfiles */
      virtual void closePlotFiles();
      /*! Set Element::plotLevel \param level and therewith specifiy outputs in plot-files */
      void setPlotLevel(int level) {plotLevel = level;}
      /*! Set Element::plotPrec \param prec for output precision */
      void setPlotPrecision(int prec);
      /*! Get \return Element::plotLevel */
      int getPlotLevel() {return plotLevel;}
	  /*! Get element short \return name */
      const string& getName() const { return name; }
      /*! Get element \return fullname */
      const string& getFullName() const { return fullName; }
      /*! Set element fullname \param str */
      virtual void setFullName(const string &str) {fullName = str;}
      /*! Set element name \param str */
      virtual void setName(const string &str) {name = str;}
	  /*! Get element multibody system \return mbs */
      //MultiBodySystem* getMbs() {return mbs;}
      /*! Set element multibody system \param mbs */
      //virtual void setMbs(MultiBodySystem* mbs_) {mbs=mbs_;}
	  
      void addDataInterfaceBaseRef(const string& DIBRef_);
      virtual void initDataInterfaceBase(MultiBodySystem *parentmbs) {};
  };

}

#endif
