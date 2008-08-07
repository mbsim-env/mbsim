/* Copyright (C) 2006  Mathias Bachmayer
 
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
 *
 * Contact:
 *   mbachmayer@gmx.de
 *
 */ 
#ifndef MECTOSIGNAL_H_
#define MECTOSIGNAL_H_

#include "coordinate_system.h"
#include "data_interface_base.h"
#include "spsys.h"
#include "body.h"

using namespace fmatvec;
using namespace MBSim;

/*! Comment
 *
 * */  
class PosInterface : public DataInterfaceBase {
protected:
  CoordinateSystem* port;
  Mat JT;
public:
  PosInterface(CoordinateSystem *port_, const Mat& JT_) {port=port_; JT=JT_;}
  ~PosInterface() {}
  Vec operator()(double t); 
};  

/*! Comment
 *
 * */  
class RotVelInterface : public DataInterfaceBase {
protected:
  CoordinateSystem* port;
  Mat JR;
public:
  RotVelInterface(CoordinateSystem *port_, const Mat& JR_) {port=port_; JR=JR_;}
  ~RotVelInterface() {}
  Vec operator()(double t); 
};  

/*! Comment
 *
 * */
class AngularInterface : public DataInterfaceBase {
protected:
  Body* body;
  int index;
public:
  AngularInterface(Body *body_);
  ~AngularInterface() {}
  Vec operator()(double t); 
};  

/*! Comment
 *
 * */
class QStateInterface : public DataInterfaceBase {
protected:
  Object* object;
  int index;
public:
  QStateInterface(Object *object_,int index_) {object=object_; index=index_;}
  ~QStateInterface() {}
  Vec operator()(double t) {
    return Vec(1,INIT,(object->getq())(index));
  } 
};

#endif
