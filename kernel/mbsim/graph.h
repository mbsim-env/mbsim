/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */


#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "mbsim/dynamic_system.h"

namespace H5 {
  class Group;
}

namespace MBSim {

  /**
   * \brief class for tree-structured mechanical systems with recursive and flat memory mechanism
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   */
  class Graph : public DynamicSystem {
    public:
      /**
       * \brief constructor
       * \param name of tree
       */
      Graph(const std::string &name);

      /**
       * \brief destructor
       */
      ~Graph() override = default;

      void updateT() override;
      void updateM() override;
      void updateLLM() override;
      void updateh(int k=0) override;
      void updatedq() override;
      void updatedu() override;
      void updateqd() override;
      void updateud() override;
      void updatezd() override;
      void sethSize(int h, int j=0) override {(this->*sethSize_[j])(h);}
      void calcqSize() override;
      void calcuSize(int j=0) override {(this->*calcuSize_[j])();}
      void setqInd(int qInd) override;
      void setuInd(int uInd, int j=0) override {(this->*setuInd_[j])(uInd);}
      void sethInd(int hInd, int j=0) override {(this->*sethInd_[j])(hInd);}

#ifndef SWIG
      void (Graph::*calcuSize_[2])(); 
      void (Graph::*sethSize_[2])(int h); 
      void (Graph::*setuInd_[2])(int uInd); 
      void (Graph::*sethInd_[2])(int hInd); 
#endif
      void calcuSize0();
      void calcuSize1();
      void sethSize0(int h);
      void sethSize1(int h);
      void setuInd0(int uInd);
      void setuInd1(int uInd);
      void sethInd0(int hInd);
      void sethInd1(int hInd);

      /**
       * \brief add new object to graph at level
       * \param level
       * \param object
       */
      void addObject(int level, Object* object); 

    protected:
      /**
       * \brief none
       */
      std::vector< std::vector<Object*>> obj;
  };

}

#endif
