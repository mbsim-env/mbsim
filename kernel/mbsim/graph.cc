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

  Graph::~Graph() {}

  void Graph::updatedu() {
    ud[0] = slvLLFac(evalLLM(), evalh()*getStepSize()+evalrdt());
  }

  void Graph::updateud(int j) {
    ud[j] =  slvLLFac(evalLLM(j), evalh(j)+evalr(j));
  }

  void Graph::updatezd() {
    for(vector<Object*>::iterator i = object.begin(); i!= object.end(); ++i)
      (**i).updateqd();
    ud[0] = slvLLFac(evalLLM(), evalh()+evalr());
  }

  void Graph::sethSize0(int hSize_) {
    hSize[0] = hSize_;
    int buf = 0;
    for(unsigned int i=0; i<obj.size(); i++) 
      for(unsigned int j=0; j<obj[i].size(); j++)  {
      buf += obj[i][j]->getuSize(0);
      obj[i][j]->sethSize(buf,0);
      //(*i)->sethInd(0,0);
    }
  } 

  void Graph::sethSize1(int hSize_) {
    DynamicSystem::sethSize(hSize_,1);
  } 

  void Graph::sethInd0(int hInd_) {
    hInd[0] = hInd_;
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->sethInd(hInd_,0);
    }
  } 

  void Graph::sethInd1(int hInd_) {
    DynamicSystem::sethInd(hInd_,1);
  } 

  void Graph::calcqSize() {
    qSize = 0;
    for(unsigned int i=0; i<obj.size(); i++) 
      for(unsigned int j=0; j<obj[i].size(); j++) {
	obj[i][j]->calcqSize();
	//obj[i][j]->setqInd(qSize);
	qSize += obj[i][j]->getqSize();
      }
  }

  void Graph::setqInd(int qInd_) {
    qInd = qInd_;
    for(unsigned int i=0; i<obj.size(); i++) 
      for(unsigned int j=0; j<obj[i].size(); j++) {
	obj[i][j]->setqInd(qInd_);
	qInd_ += obj[i][j]->getqSize();
      }
  }

  void Graph::calcuSize0() {
    uSize[0] = 0;
    for(unsigned int i=0; i<obj.size(); i++) 
      for(unsigned int j=0; j<obj[i].size(); j++) {
	obj[i][j]->calcuSize(0);
	//obj[i][j]->setuInd(uSize[0],0);
	uSize[0] += obj[i][j]->getuSize(0);
      }
  }

  void Graph::setuInd0(int uInd_) {
    uInd[0] = uInd_;
    for(unsigned int i=0; i<obj.size(); i++) 
      for(unsigned int j=0; j<obj[i].size(); j++) {
	obj[i][j]->setuInd(uInd_,0);
	uInd_ += obj[i][j]->getuSize(0);
      }
  }

  void Graph::setuInd1(int uInd) {
    DynamicSystem::setuInd(uInd,1);
  }

  void Graph::calcuSize1() {
    DynamicSystem::calcuSize(1);
  }

  void Graph::updateLLM(int i) {
    LLM[i] = facLL(evalM(i));
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
    msg(Info) << "Content of object graph "<< name << ":" << endl;
    for(unsigned int i=0; i<obj.size(); i++) {
      msg(Info) << "  Objects in level "<< i << ":"<< endl;
      for(unsigned int j=0; j<obj[i].size(); j++)
	msg(Info) << "    "<< obj[i][j]->getPath()<<" (uSize=" <<obj[i][j]->getuSize()<<", hSize="<< obj[i][j]->gethSize()<<
                                           ", uInd=" << obj[i][j]->getuInd()<<", hInd=" << obj[i][j]->gethInd()<< ")"<<endl;
    }
  }

}

