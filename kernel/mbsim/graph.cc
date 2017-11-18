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

#include <config.h>
#include "mbsim/graph.h"
#include "mbsim/objects/object.h"
#include "mbsim/frames/frame.h"

using namespace fmatvec; 
using namespace std;

namespace MBSim {

  Graph::Graph(const string &projectName) : DynamicSystem(projectName) {
    calcuSize_[0] = &Graph::calcuSize0;
    calcuSize_[1] = &Graph::calcuSize1;
    sethSize_[0] = &Graph::sethSize0;
    sethSize_[1] = &Graph::sethSize1;
    setuInd_[0] = &Graph::setuInd0;
    setuInd_[1] = &Graph::setuInd1;
    sethInd_[0] = &Graph::sethInd0;
    sethInd_[1] = &Graph::sethInd1;
  }

  Graph::~Graph() = default;

  void Graph::updatedu() {
    du = slvLLFac(evalLLM(), evalh()*getStepSize()+evalrdt());
  }

  void Graph::updatezd() {
    for(auto & i : object)
      (*i).updateqd();
    ud = slvLLFac(evalLLM(), evalh()+evalr());
  }

  void Graph::sethSize0(int hSize_) {
    hSize[0] = hSize_;
    int buf = 0;
    for(auto & i : obj) 
      for(unsigned int j=0; j<i.size(); j++)  {
      buf += i[j]->getuSize(0);
      i[j]->sethSize(buf,0);
      //(*i)->sethInd(0,0);
    }
  } 

  void Graph::sethSize1(int hSize_) {
    DynamicSystem::sethSize(hSize_,1);
  } 

  void Graph::sethInd0(int hInd_) {
    hInd[0] = hInd_;
    for(auto & i : object) {
      i->sethInd(hInd_,0);
    }
  } 

  void Graph::sethInd1(int hInd_) {
    DynamicSystem::sethInd(hInd_,1);
  } 

  void Graph::calcqSize() {
    qSize = 0;
    for(auto & i : obj) 
      for(unsigned int j=0; j<i.size(); j++) {
	i[j]->calcqSize();
	//obj[i][j]->setqInd(qSize);
	qSize += i[j]->getqSize();
      }
  }

  void Graph::setqInd(int qInd_) {
    qInd = qInd_;
    for(auto & i : obj) 
      for(unsigned int j=0; j<i.size(); j++) {
	i[j]->setqInd(qInd_);
	qInd_ += i[j]->getqSize();
      }
  }

  void Graph::calcuSize0() {
    uSize[0] = 0;
    for(auto & i : obj) 
      for(unsigned int j=0; j<i.size(); j++) {
	i[j]->calcuSize(0);
	//obj[i][j]->setuInd(uSize[0],0);
	uSize[0] += i[j]->getuSize(0);
      }
  }

  void Graph::setuInd0(int uInd_) {
    uInd[0] = uInd_;
    for(auto & i : obj) 
      for(unsigned int j=0; j<i.size(); j++) {
	i[j]->setuInd(uInd_,0);
	uInd_ += i[j]->getuSize(0);
      }
  }

  void Graph::setuInd1(int uInd) {
    DynamicSystem::setuInd(uInd,1);
  }

  void Graph::calcuSize1() {
    DynamicSystem::calcuSize(1);
  }

  void Graph::updateLLM() {
    LLM = facLL(evalM());
  }

  void Graph::addObject(int level, Object* object) {
    DynamicSystem::addObject(object);

    for(int i=obj.size(); i<=level; i++) {
      vector<Object*> vec;
      obj.push_back(vec);
    }

    obj[level].push_back(object);
  }

  void Graph::printGraph() {
    if(msgAct(Debug)) {
      msg(Debug) << "Content of object graph "<< name << ":" << endl;
      for(unsigned int i=0; i<obj.size(); i++) {
        msg(Debug) << "  Objects in level "<< i << ":"<< endl;
        for(auto & j : obj[i])
          msg(Debug) << "    "<< j->getPath()<<" (uSize=" <<j->getuSize()<<", hSize="<< j->gethSize()<<
            ", uInd=" << j->getuInd()<<", hInd=" << j->gethInd()<< ")"<<endl;
      }
    }
  }

}

