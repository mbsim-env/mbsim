/*=============================================================================
        File: nurbsGL.cpp
     Purpose:       
    Revision: $Id: nurbsGL.cpp,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (28 September 1997)
 Modified by: 

 Copyright notice:
          Copyright (C) 1996-1998 Philippe Lavoie
 
          This library is free software; you can redistribute it and/or
          modify it under the terms of the GNU Library General Public
          License as published by the Free Software Foundation; either
          version 2 of the License, or (at your option) any later version.
 
          This library is distributed in the hope that it will be useful,
          but WITHOUT ANY WARRANTY; without even the implied warranty of
          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
          Library General Public License for more details.
 
          You should have received a copy of the GNU Library General Public
          License along with this library; if not, write to the Free
          Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
=============================================================================*/
#include <matrixRT.h>
#include <nurbsGL.h>
#include <string.h>
#include <stdio.h>
#include <iostream.h> 

#ifdef WITH_OPENGL

/*!
 */
namespace PLib {

Color objectColorDefault(255,255,255) ;
Color selectColorDefault(0,255,0) ;
Color currentColorDefault(0,255,255) ;

Color axisXColorDefault(255,0,0) ;
Color axisYColorDefault(0,255,0) ;
Color axisZColorDefault(0,0,255) ;

Color cpointColorDefault(255,0,0) ;
Color cpoint0ColorDefault(255,255,255) ;
Color cpointActiveColorDefault(255,0,255) ;
Color cPolygonColorDefault(255,255,0)  ;

Color knotColorDefault(255,255,0) ;
Color knotActiveColorDefault(0,255,255) ;

HPoint3Df dummyCPoint ;

int pSizeDefault = 4 ;

ObjectGLState objectStateDefault = objectState;


const int NURBS_FLAGS_AFFECT_ALL = 1 ;
const int NURBS_FLAGS_AFFECT_ACTIVE_ONLY = 2 ;
const int NURBS_FLAGS_AFFECT_SELECTED_ONLY = 4 ;
const int NURBS_FLAGS_VIEW_BBOX = 8 ; // view the bbox even if object is in hidden.

const int NURBSLIST_DELETE_AT_RESET = 1;
const int NURBSLIST_KEEP_AT_RESET = 0 ;

const int NURBS_DISPLAY_NORMAL = 0 ;
const int NURBS_DISPLAY_ISOCURVES  = 1 ;
const int NURBS_DISPLAY_SHADED = 2 ;
const int NURBS_DISPLAY_HIDDEN_LINES = 3 ; 
const int NURBS_DISPLAY_TESSELATION = 4 ;

float tessel_tolerance ;

int NurbsDisplayMode = 0 ;


Point3Df XAxis_3D(1,0,0) ;
Point3Df YAxis_3D(0,1,0) ;
Point3Df ZAxis_3D(0,0,1) ;



void initColorsGL(){
  objectColorDefault = Color(0,0,0) ;
  selectColorDefault = Color(0,255,0) ;
  currentColorDefault = Color(0,255,255) ;
  
  axisXColorDefault = Color(255,0,0) ;
  axisYColorDefault = Color(0,255,0) ;
  axisZColorDefault = Color(0,0,255) ;
  
  cpointColorDefault = Color(255,0,0) ;
  cpoint0ColorDefault = Color(255,255,255) ;
  cPolygonColorDefault = Color(255,255,0)  ;
  
  objectStateDefault = objectState;  
}

/*!
  \brief The basic constructor

  It sets default values for the state and the colors. The
  default values are defined by the default global variables:
  objectColorDefault, selectColorDefault, currentColorDefault
  and objectStateDefault.

  \author Philippe Lavoie
  \date 20 September 1997
*/
ObjectGL::ObjectGL(){
  prev_ = 0 ;
  next_ = 0 ;
  category = badType ; 
  callListId = 0 ;
  state = objectStateDefault ;
  objectColor = objectColorDefault ;
  selectColor = selectColorDefault ;
  currentColor = currentColorDefault ;
  tx = ty = tz = 0 ;
  rx = ry = rz = 0 ;
  sx = sy = sz = 1 ;
  materialColor = new float[4] ;
  active = selected = 0 ;
  name_ = new char[strlen("unknown")+1] ; 
  strcpy(name_,"unknown") ; 
}

/*!
  \brief Sets the name of the object

  Sets the name of the object. The content of n is copied
  to the name. 
  \param n  the name of the object

  \author Philippe Lavoie
  \date 20 September 1997
*/
void ObjectGL::setName(const char* n){
  int len = strlen(n) ;
  if(name_)
    delete []name_ ; 
  name_ = new char[len+1] ; 
  strcpy(name_,n) ; 
}

/*!
  \brief Returns the name of the type of the class

  \return the name of the type of the class, 0 if the type is unknown

  \author Philippe Lavoie
  \date 20 September 1997
*/
char* ObjectGL::typeName() const {
  switch(type){
  case badObject: return "bad object" ;  break ;
  case curveObject: return "NURBS curve" ; break ; 
  case surfaceObject: return "NURBS surface" ; break ; 
  case pointObject: return "point in 3D" ;  break ; 
  case cpointObject: return "control point" ;  break ; 
  case cpointPolygonObject: return "control point polygon" ; break ; 
  case bboxObject: return "bounding box" ; break ; 
  case vectorObject: return "vector" ; break ; 
  case listObject: return "list of objects" ; break ; 
  case hSurfObject: return "HNURBS surface" ; break ; 
  case hcpointObject: return "control point" ; break ; 
  case pointListObject: return "list of points" ; break ; 
  case spointObject: return "surface point";  break ; 
  default:
    return 0;
  };
  return 0 ;
}


/*!
  \brief Performs the local transformation

  Performs the local transformation in this order:
  scaling, translation, rotation in x, rotation in y
  and rotation in z.

  \author Philippe Lavoie
  \date 20 September 1997
*/
void ObjectGL::glTransform() const { 
  glScalef(sx,sy,sz) ; 
  glTranslatef(tx,ty,tz) ; 
  glRotatef(rx,1,0,0) ; 
  glRotatef(ry,0,1,0) ; 
  glRotatef(rz,0,0,1) ; 
}

/*!
  \brief The destructor 

  \author Philippe Lavoie
  \date 30 September 1997
*/
ObjectGL::~ObjectGL(){
  /*
  if(next_){
    next_->prev_ = prev_ ;
  }
  if(prev_){
    prev_->next_ = next_ ;
  }
  */
  next_ = 0 ;
  prev_ = 0 ;
  if(materialColor)
    delete materialColor ;
  if(name_)
    delete []name_ ; 
}

/*!
  \brief The constructor

  By default the reset mode is set to delete all the elements
  from the list (\code NURBSLIST_DELETE_AT_RESET \endcode). If you 
  don't want this behavior, call
  \code setResetMode(NURBS_KEEP_AT_RESET) \endcode

  \author Philippe Lavoie
  \date 30 September 1997
*/
ObjectListGL::ObjectListGL():ObjectGL(){
  first_ = 0 ;
  last_ = 0 ;
  current_ = 0 ;
  n = 0 ;
  resetMode = NURBSLIST_DELETE_AT_RESET ;
}

/*!
  \brief The destructor 

  \author Philippe Lavoie
  \date 30 September 1997
*/
ObjectListGL::~ObjectListGL(){
  reset() ;
}

/*!
  \brief The destructor 

  \author Philippe Lavoie
  \date 3 June 1998
*/
ObjectRefListGL::~ObjectRefListGL(){
  reset() ;
}

/*!
  \brief Deletes all the node of the list

  Deletes all the node of the list. Depending on the reset
  mode, it will also delete the elements.

  \author Philippe Lavoie
  \date 30 September 1997
*/
void ObjectListGL::reset(){
  if(resetMode==NURBSLIST_DELETE_AT_RESET){
    ObjectGL *c ;
    c = first_ ;
    while(c){
      current_ = c ;
      c = current_->next() ;
      delete current_ ;
    }
  }
  first_ = current_ = last_ = 0 ;
  n = 0 ;
}

/*!
  \brief Transforms the elements stored in the list

  Transforms the elements stored in the list \e by a certain 
  value if the elements are in a proper state. The behavior 
  variable specifies which objects should be affected by this 
  function.

  \param x  a translation in the $x$ direction
  \param y  a translation in the $y$ direction
  \param z  a translation in the $z$ direction
  \param a  a rotation around the $x$-axis
  \param b  a rotation around the $y$-axis
  \param c  a rotation around the $z$-axis
  \param sx  a scaling in the direction of the $x$-axis
  \param sy  a scaling in the direction of the $x$-axis
  \param sz  a scaling in the direction of the $x$-axis
  \param behavior  specifies which objects are affected by the function

  \author Philippe Lavoie
  \date 30 September 1997
*/
void ObjectListGL::transformBy(GLfloat x, GLfloat y, GLfloat z, GLfloat a, GLfloat b, GLfloat c,GLfloat sx, GLfloat sy, GLfloat sz, int behavior){
  ObjectGL *t ;
  t = first_ ;
  while(t){
    if(!(behavior & NURBS_FLAGS_AFFECT_ALL)){
      if(behavior & NURBS_FLAGS_AFFECT_ACTIVE_ONLY){
	if(!t->isActive()){
	  t = t->next() ;
	  continue ;
	}
      }
      if(behavior & NURBS_FLAGS_AFFECT_SELECTED_ONLY){
	if(!t->isSelected()){
	  t = t->next() ;
	  continue ;
	}
      }
    }
    t->tx += x ;
    t->ty += y ;
    t->tz += z ;
    t->rx += a ;
    t->ry += b ;
    t->rz += c ;
    t->sx += sx ;
    t->sy += sy ;
    t->sz += sz ;
    t = t->next() ;
   }
}

/*!
  \brief transforms the elements stored in the list

  Transforms the elements stored in the list \e to a certain 
  value if the elements are in a proper state. The behavior 
  variable specifies which objects should be affected by this 
  function.

  \param x  a translation in the $x$ direction
  \param y  a translation in the $y$ direction
  \param z  a translation in the $z$ direction
  \param a  a rotation around the $x$-axis
  \param b  a rotation around the $y$-axis
  \param c  a rotation around the $z$-axis
  \param sx  a scaling in the direction of the $x$-axis
  \param sy  a scaling in the direction of the $x$-axis
  \param sz  a scaling in the direction of the $x$-axis
  \param behavior  specifies which objects ared affected by the function

  \author Philippe Lavoie
  \date 30 September 1997
*/
void ObjectListGL::transformTo(GLfloat x, GLfloat y, GLfloat z, GLfloat a, GLfloat b, GLfloat c,GLfloat sx, GLfloat sy, GLfloat sz, int behavior){
  ObjectGL *t ;
  t = first_ ;
  while(t){
    if(!(behavior & NURBS_FLAGS_AFFECT_ALL)){
      if(behavior & NURBS_FLAGS_AFFECT_ACTIVE_ONLY){
	if(!t->isActive()){
	  t = t->next() ;
	  continue ;
	}
      }
      if(behavior & NURBS_FLAGS_AFFECT_SELECTED_ONLY){
	if(!t->isSelected()){
	  t = t->next() ;
	  continue ;
	}
      }
    }
    t->tx = x ;
    t->ty = y ;
    t->tz = z ;
    t->rx = a ;
    t->ry = b ;
    t->rz = c ;
    t->sx = sx ;
    t->sy = sy ;
    t->sz = sz ;
    t = t->next() ;
   }
}

/*!
  \brief Displays a control point

  \author Philippe Lavoie
  \date 30 September 1997
*/
void CPointGL::glObject() const {
  glPointSize(psize) ;
  glBegin(GL_POINTS) ;
  glColor() ;
  glVertex4fv(cpoint.data) ;
  glEnd() ;
}

/*!
  \brief Displays a list of points

  \author Philippe Lavoie
  \date 29 Mars 1998
*/
void PointListGL::glObject() const {
  BasicNode<Point3Df> *node ;
  node = static_cast<BasicNode<Point3Df>*>(list.first()) ;

  glPointSize(psize) ;
  glBegin(GL_POINTS) ;  
  //for(int i=0;i<list.size();++i){
  for(int i=0;i<minimum(20000,list.size());++i){
    glColor() ;
    glVertex3f(node->data->x(),node->data->y(),node->data->z()) ;
    node = node->next ; 
  }
  glEnd() ;
}


/*!
  \brief Constructor from a list of points

  \param l  list of points

  \author Philippe Lavoie
  \date 30 Mars 1998
*/
PointListGL::PointListGL(const BasicList<Point3Df> &l): ObjectGL(){
  category = pointType ;
  type = pointListObject ;
  psize = pSizeDefault ;
  list = l ;
}


/*!
  \brief Copy constructor
  \param pl  list to copy

  \author Philippe Lavoie
  \date 30 Mars 1998
*/
PointListGL::PointListGL(const PointListGL &pl): ObjectGL(){
  category = pointType ;
  type = pointListObject ;
  psize = pl.psize ;
  list = pl.list ;
}


/*!
  \brief Reads a list of points.
  \param fin  input file stream

  \author Philippe Lavoie
  \date 29 Mars 1998
*/
int PointListGL::read(ifstream &fin) {
  list.reset() ;

  char *type ;
  type = new char [3] ;

  if(!fin.read(type,sizeof(char)*3)) { delete []type ; return 0 ;}
  int r1 = strncmp(type,"pt3",3) ;
  
  if(r1!=0){
    delete []type ;
    return 0 ;
  }
  setPsize(2) ;

  while(!fin.eof()){
    int mark = fin.tellg() ; // mark position
    if(!fin.read(type,sizeof(char)*3)) { delete []type ; return 1 ; }
    fin.seekg(mark) ; // restore position
    char* ext ;
    ext = strstr(type,"#") ;
    if(ext) { // this line is a comment
      while( *ext != '\n'){
	fin.read(ext,sizeof(char)) ;
      }
    }
    else { 
      ext = strstr(type,"X") ;
      if(ext){ // end of list of points
	while(*type != 'X')
	  fin.read(type,sizeof(char)) ;
	fin.read(type,sizeof(char));
	delete []type ;
	return 1 ;
      }
      // it should be 3 points as in    x  y  z
      double x, y, z ;
      if(fin >> x >> y >> z){
	Point3Df p;
	//float scale = 10.0 ;
	//p = Point3Df(x/scale,y/scale,z/scale) ;
	p = Point3Df(x,y,z) ;
	/*
	if(list.size()==0)
	  offset = p ;
	p -= offset  ;
	*/
	list.add(p) ;
      }
      else{
	delete []type ;
	return 0 ;
      }
    }
  }
  return 1 ;

}

/*!
  \brief Writes a list of points.

  \param fin  output file stream

  \author Philippe Lavoie
  \date 29 Mars 1998
*/
int PointListGL::write(ofstream &fout) const {
  fout << "pt3" ;
  BasicNode<Point3Df> *node ;
  node = (BasicNode<Point3Df>*)list.first() ;

  for(int i=0;i<list.size();++i){
    fout << *(node->data) << endl ;
    node = node->next ;
  }
  fout << "X\n" ;
  if(!fout)
    return 0 ;
  return 1 ;
}


/*!
  \brief Displays a control point.

  \author Philippe Lavoie
  \date 3 November 1997
*/
void HCPointGL::glObject() const {
  glPointSize(psize) ;
  glBegin(GL_POINTS) ;
  glColor() ;
  glVertex4fv(s->ctrlPnts()(i0,j0).data) ;
  glEnd() ;
}

/*!
  \brief Displays a control point

  Displays a control point on the surface of a NURBS curve or a
  NURBS surface.

  \author Philippe Lavoie
  \date 12 May 1998
*/
void SPointCurveGL::glObject() const {
  glPointSize(psize) ;
  glBegin(GL_POINTS) ;
  glColor() ;
  glVertex4fv(cpoint.data) ;
  glEnd() ;
}

/*!
  \brief Displays a control point

  Displays a control point on the surface of a NURBS curve or a
  NURBS surface.

  \author Philippe Lavoie
  \date 12 May 1998
*/
void SPointSurfaceGL::glObject() const {
  glPointSize(psize) ;
  glBegin(GL_POINTS) ;
  glColor() ;
  glVertex4fv(cpoint.data) ;
  glEnd() ;
}

/*!
  \brief Displays a control point

  Displays a control point on the surface of a NURBS curve or a
  NURBS surface.

  \author Philippe Lavoie
  \date 12 May 1998
*/
void SPointHSurfaceGL::glObject() const {
  glPointSize(psize) ;
  glBegin(GL_POINTS) ;
  glColor() ;
  glVertex4fv(cpoint.data) ;
  glEnd() ;
}

/*!
  \brief Modifies the offset point.

  \author Philippe Lavoie
  \date 3 November 1997
*/
void HCPointGL::modify(const HPoint3Df& v) {
  if(canModify){
    offset += v ; 
    s->updateSurface(i0,j0) ;
  }
}

/*!
  \brief Displays a point.

  \author Philippe Lavoie
  \date 30 September 1997
*/
void PointGL::glObject() const {
  glPointSize(psize) ;
  glBegin(GL_POINTS) ;
  glColor() ;
  glVertex3fv(p.data) ;
  glEnd() ;
}

/*!
  \brief Displays a knot

  \author Philippe Lavoie
  \date 30 September 1997
*/
void KnotGL::glObject() const {
  glPointSize(psize) ;
  glBegin(GL_POINTS) ;
  glColor() ;
  glVertex3fv(p.data) ;
  glEnd() ;
}

/*!
  \brief Calls glObject for all the elements from the list

  Displays all the elements from the list


  \author Philippe Lavoie
  \date 30 September 1997
*/
void ObjectListGL::glObject() const {
  ObjectGL *t ;
  t = first_ ;
  while(t){
    if(t->getState())
      t->glObject() ;
    t = t->next() ;
  }
}

/*!
  \brief Displays all the elements from the list

  \author Philippe Lavoie
  \date 30 September 1997
*/
void ObjectListGL::display() const {
  ObjectGL *t ;
  t = first_ ;
  while(t){
    if(t->getState()){
      glPushMatrix() ;
      t->glTransform() ;
      t->glObject() ;
      glPopMatrix() ;
    }
    t = t->next() ;
  }
}

/*!
  \brief Displays all the elements from the list

  \author Philippe Lavoie
  \date 30 September 1997
*/
void ObjectListGL::displayName() const {
  ObjectGL *t ;
  int name = 1 ;
  t = first_ ;
  while(t){
    if(t->getState()){
      glPushMatrix() ;
      t->glTransform() ;
      glLoadName(name) ;
      t->glObject() ;
      glPopMatrix() ;
      ++name ;
    }
    t = t->next() ;
  }
}

/*!
  \brief Calls glObject for all the Nurbs Object in the list.

  \author Philippe Lavoie
  \date 30 September 1997
*/
void NurbsListGL::glObject() const {
  ObjectGL *t ;
  t = first_ ;
  while(t){
    t->glObject() ;
    t = t->next() ;
  }
}

/*!
  \brief Displays all the NURBS object from the list.

  \author Philippe Lavoie
  \date 30 September 1997
*/
void NurbsListGL::display() const {
  ObjectGL *t ;
  t = first_ ;
  while(t){
    glPushMatrix() ;
    t->glTransform() ;
    t->glObject() ;
    glPopMatrix() ;
    t = t->next() ;
  }
}

/*!
  \brief Resets the display flags for the elements

  Resets the display flags for the elements in the list. It
  will only resets the flags according to the behavior value.

  \param o  display flag
  \param cp display the control points
  \param p  display the control polygon
  \param b  display the bounding box
  \param k  display the knots
  \param behavior  specifies which object are affected by the function

  \author Philippe Lavoie
  \date 30 September 1997
*/
void NurbsListGL::resetDisplayFlags(int o, int cp, int p, int b, int k, int behavior) {
  ObjectGL *pnt ;
  pnt = (ObjectGL*)first_ ;

  NurbsGL *t ;
  while(pnt){
    if(pnt->category == nurbsType)
      t = (NurbsGL*)pnt ;
    else{
      pnt = pnt->next() ;
      continue ;
    }
      
    if(!(behavior & NURBS_FLAGS_AFFECT_ALL)){
      if(behavior & NURBS_FLAGS_AFFECT_ACTIVE_ONLY){
	if(!t->isActive()){
	  //t = (NurbsGL*)t->next() ;
	  pnt = pnt->next() ;
	  continue ;
	}
      }
      if(behavior & NURBS_FLAGS_AFFECT_SELECTED_ONLY){
	if(!t->isSelected()){
	  //t = (NurbsGL*)t->next() ;
	  pnt = pnt->next() ;
	continue ;
	}
      }
    }
    if(o){
      if(cp)
	t->viewCPoints() ;
      else
	t->hideCPoints() ;
      if(p)
	t->viewCpolygon() ;
      else
	t->hideCpolygon() ;
      if(b)
	t->viewBBox() ;
      else
	t->hideBBox() ;
      if(k)
	t->viewKnots() ;
      else
	t->hideKnots() ;
      t->viewObject() ;
    }
    else{
      if(behavior & NURBS_FLAGS_VIEW_BBOX){
	if(b)
	  t->viewBBox() ;
	else
	  t->hideBBox() ;
      }
      t->hideObject() ;
    }
    
    pnt = pnt->next() ; // (NurbsGL*)t->next() ;
  }
}

/*!
  \brief Deactivates all the objects of the list.

  \author Philippe Lavoie
  \date 30 September 1997
*/
void ObjectListGL::deactivate() {
  ObjectGL *t ;
  t = first_ ;
  while(t){
    t->deactivate() ;
    t = t->next() ;
  }
}

/*!
  \brief Activate all the objects of the list

  \author Philippe Lavoie
  \date 30 September 1997
*/
void ObjectListGL::activate(){
  ObjectGL *t ;
  t = first_ ;
  while(t){
    t->activate() ;
    t = t->next() ;
  }
}

/*!
  \brief Select all the objects of the list

  \author Philippe Lavoie
  \date 30 September 1997
*/
void ObjectListGL::select(){
  ObjectGL *t ;
  t = first_ ;
  while(t){
    t->select() ;
    t = t->next() ;
  }
}


/*!
  \brief Deselect all the objects of the list

  \author Philippe Lavoie
  \date 2 October 1997
*/
void ObjectListGL::deselect(){
  ObjectGL *t ;
  t = first_ ;
  while(t){
    t->deselect() ;
    t = t->next() ;
  }
}

/*!
  \brief Moves the current pointer to the \a n th element
  
  Moves the current pointer to the \a n th element. This must
  be in a valid range otherwise 0 is returned.

  \param a  the \a a th element 

  \return A pointer to the current element or 0 if a was out of range.

  \author Philippe Lavoie
  \date 30 October 1997
*/
ObjectGL* ObjectListGL::goTo(int a){
  if(a>=n)
    return 0 ;
  current_ = first_ ;
  while(current_ && a>0){
    current_ = current_->next() ;
    --a ;
  }
  return current_ ;
}

/*!
  \brief moves the current pointer to the \a n th active element

  Moves the current pointer to the \a n th active element. This 
  must be in a valid range otherwise 0 is returned.

  \param a  the \a a th active element 

  \return A pointer to the current element or 0 if a was out of range.

  \author Philippe Lavoie
  \date 29 January 1998
*/
ObjectGL* ObjectListGL::goToActive(int a){
  if(a>=n)
    return 0 ;
  current_ = first_ ;
  while(current_ ){
    if(current_->getState())
      --a ;
    if(a>=0)
      current_ = current_->next() ;
    else
      break ;
  }
  return current_ ;
}

/*!
  \brief move the current pointer to the next element

  Moves the current pointer to the next element. If there are
  no next element, it goes to the first element.

  \return A pointer to the current element

  \author Philippe Lavoie
  \date 2 October 1997
*/
ObjectGL* ObjectListGL::goToNext(){
  if(current_->next()){
    current_ = current_->next() ;
  }
  else
    current_ = first_ ;
  return current_ ;
}

/*!
  \brief move the current pointer to the previous one

  Moves the current pointer to the previous element. If there are
  no previous element, it goes to the last element.

  \return A pointer to the current element

  \author Philippe Lavoie
  \date 30 September 1997
*/
ObjectGL* ObjectListGL::goToPrevious(){
  if(current_->previous()){
    current_ = current_->previous() ;
  }
  else
    current_ = last_ ;
  return current_ ;
}

/*!
  \brief move the current pointer to the next active element

  Moves the current pointer to the next element. If there are
  no next element, it goes to the first element.

  \return A pointer to the current element

  \author Philippe Lavoie
  \date 29 January 1998
*/
ObjectGL* ObjectListGL::goToNextActive(){
  ObjectGL *na ;

  na = 0 ;
  if(current_)
    na = current_->next() ;
  if(!na)
    na = first_ ;

  if(na){
    while(!na->getState()){
      if(!na->next())
	na = first_ ;
      else
	na = na->next() ;
      if(na == current_)
	break ;
    }
  }

  current_ = na ;

  return current_ ;
}

/*!
  \brief move the current pointer to the previous activeElement

  Moves the current pointer to the previous element. If there are
  no previous element, it goes to the last element.

  \return A pointer to the current element

  \author Philippe Lavoie
  \date 29 January 1998
*/
ObjectGL* ObjectListGL::goToPreviousActive(){
  ObjectGL *na ;

  na = 0 ;
  if(current_)
    na = current_->previous() ;
  if(!na)
    na = last_ ;

  if(na){
    while(!na->getState()){
      if(!na->previous())
	na = last_ ;
      else
	na = na->previous() ;
      if(na == current_)
	break ;
    }
  }
  current_ = na ;

  return current_ ;
}

/*!
  \brief move the current pointer to the next element

  Moves the current pointer to the next element. If there are
  no next element, it goes to the first element.

  \return A pointer to the current element

  \author Philippe Lavoie
  \date 2 October 1997
*/
ObjectGL* ObjectListGL::jumpToNext(){
  for(int i=0;i<jumpSize-1;++i)
    goToNext() ;
  return goToNext() ;
}

/*!
  \brief move the current pointer to the next element

  Moves the current pointer to the next element. If there are
  no next element, it goes to the first element.

  \return A pointer to the current element

  \author Philippe Lavoie
  \date 2 October 1997
*/
ObjectGL* ObjectListGL::jumpToPrevious(){
  for(int i=0;i<jumpSize-1;++i)
    goToPrevious() ;
  return goToPrevious() ;
}

/*!
  \brief adds an element to the list

  Adds an element to the list.

  \param obj  the element to add

  \author Philippe Lavoie
  \date 30 September 1997
*/
void ObjectListGL::add(ObjectGL* obj){
  if(obj){
    if(!first_){
      first_ = obj ;
    }
    else{
      last_->next() = obj ;
      obj->previous() = last_ ;
    }
    last_ = obj ;
    obj->next() = 0 ; 
    current_ = obj ;
    ++n ;
  }
}

/*!
  \brief finds an element and remove it from the list

  Finds an element and delete it from the list. The element
  will \e not be deleted. This is up to the calling function.

  \param obj  the element to search

  \return a pointer to obj if it was found in the list, 0 otherwise

  \author Philippe Lavoie
  \date 30 September 1997
*/
ObjectGL* ObjectListGL::remove(ObjectGL* obj){
  ObjectGL* t ;

  if(!obj)
    return 0 ;

  if(current_ == obj){
    t = obj ;
    current_ = 0 ;
    if(t->previous()){
      t->previous()->next() = t->next() ;
      current_ = t->previous() ;
    }
    if(t->next()){
      t->next()->previous() = t->previous() ;
      current_ = t->next() ;
    }
    --n ;
    if(first_==t)
      first_ = t->next() ;
    if(last_==t)
      last_ = t->previous() ;
    return t;
  }

  t = first_ ;
  while(t){
    if(t==obj){
      if(t->previous())
	t->previous()->next() = t->next() ;
      if(t->next())
	t->next()->previous() = t->previous() ;
      --n ;
      if(first_ ==t)
	first_ = t->next() ;
      if(last_ ==t)
	last_ = t->previous() ;
      return t;
    }
    else
      t = t->next() ;
  }
  return 0 ;
}

/*!
  \brief finds an element and remove it from the list

  Finds an element and delete it from the list. The element
  will \e not be deleted. This is up to the calling function.
  Since this is a reference list, we check if the element is
  referencing obj. If it is,  we remove it from the list. If 
  not, we check if obj is one of the element from the list
  and if so we remove it.

  \param obj  the element to remove

  \return the element from the list or 0 if obj wasn't found.

  \author Philippe Lavoie
  \date 30 September 1997
*/
ObjectGL* ObjectRefListGL::remove(ObjectGL* obj){
  ObjectGL* t ;
  ObjectRefGL* t2 ;

  if(((ObjectRefGL*)current_)->ptr == obj){
    t2 = (ObjectRefGL*)current_ ;
    current_ = 0 ;
    if(t2->previous()){
      t2->previous()->next() = t2->next() ;
      current_ = t2->previous() ;
    }
    if(t2->next()){
      t2->next()->previous() = t2->previous() ;
      current_ = t2->next() ;
    }
    --n ;
    if(first_ ==t2)
      first_ = t2->next() ;
    if(last_ ==t2)
      last_ = t2->previous() ;
    return t2;
  }

  t2 = (ObjectRefGL*)first_ ;
  while(t2){
    if(t2->ptr==obj){
      if(t2->previous())
	t2->previous()->next() = t2->next() ;
      if(t2->next())
	t2->next()->previous() = t2->previous() ;
      --n ;
      if(first_ ==t2)
	first_  = t2->next() ;
      if(last_ ==t2)
	last_  = t2->previous() ;
      return t2;
    }
    else
      t2 = (ObjectRefGL*)t2->next() ;
  }

  if(current_ == obj){
    t = obj ;
    current_ = 0 ;
    if(t->previous()){
      t->previous()->next() = t->next() ;
      current_ = t->previous() ;
    }
    if(t->next()){
      t->next()->previous() = t->previous() ;
      current_ = t->next() ;
    }
    --n ;
    if(first_ ==t)
      first_  = t->next() ;
    if(last_ ==t)
      last_  = t->previous() ;
    return t;
  }

  t = first_ ;
  while(t){
    if(t==obj){
      if(t->previous())
	t->previous()->next() = t->next() ;
      if(t->next())
	t->next()->previous() = t->previous() ;
      --n ;
      if(first_ ==t)
	first_  = t->next() ;
      if(last_ ==t)
	last_  = t->previous() ;
      return t;
    }
    else
      t = t->next() ;
  }
  return 0 ;
}

/*!
  \brief reference all the elements from a list

  Reference all the elements from a list. If the addOnce variable
  is set, it will reference the elements of the list only once. 
  So if the reference list was already referencing one of the 
  elements, it will not be added again. If the addOnce variable
  is not set, the resulting list might reference an element
  more than once.

  \param list  the list to reference

  \author Philippe Lavoie
  \date 30 September 1997
*/
void ObjectRefListGL::refList(const ObjectListGL* list, int addOnce) {
  ObjectGL *t ;
  t = (ObjectGL*)list->first() ;
  while(t){
    if(addOnce){
      int addOk ;
      // check if t is already in the list
      ObjectRefGL *t2 ;
      t2 = (ObjectRefGL*)first_ ;
      addOk = 1 ;
      while(t2){
	if(t2->ptr == t){
	  addOk = 0 ;
	  break ;
	}
	t2 = (ObjectRefGL*)t2->next() ;
      }
      if(addOk)
	add(t) ;
    }
    else
      add(t) ;
    t = t->next() ;
  }
}


/*!
  \brief generates a default call list

  Generates a default call list. It generates a call list
  from the glObject() call using \verb.GL_COMPILE..

  \return The call list identification number for the object if the call
               was succesfull, 0 otherwise.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void ObjectGL::glNewList(){
  if(callListId)
    glDeleteLists(callListId,1) ;

  callListId = glGenLists(1) ;
  ::glNewList(callListId,GL_COMPILE) ;
  glObject() ;
  glEndList() ;
}

/*!
  \brief an object to represent a bounding box

  This will set the colors for the X,Y and Z axis to the value
  specified by the axisXColorDefault, axisYColorDefault and
  axisZColorDefault global variables.
  Use setColorXYZ() if you want to change this default.


  \author Philippe Lavoie
  \date 23 September 1997
*/
BoundingBoxGL::BoundingBoxGL():ObjectGL() {
  objectColor = cpoint0ColorDefault ;
  colorX = axisXColorDefault ;
  colorY = axisYColorDefault ;
  colorZ = axisZColorDefault ;
  type = bboxObject ;
}


/*!
  \brief generates a bounding box 

  This function generates a bounding box around the NURBS object.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void BoundingBoxGL::glObject() const {
  glBegin(GL_LINES);

  //glColor3f(colorX) ;
  glColor(colorX) ;
  glVertex3f(minP.x(),minP.y(),minP.z()) ;
  glVertex3f(maxP.x(),minP.y(),minP.z()) ;

  //glColor3f(colorY) ;
  glColor(colorY) ;
  glVertex3f(minP.x(),minP.y(),minP.z()) ;
  glVertex3f(minP.x(),maxP.y(),minP.z()) ;

  //glColor3f(colorZ) ;
  glColor(colorZ) ;
  glVertex3f(minP.x(),minP.y(),minP.z()) ;
  glVertex3f(minP.x(),minP.y(),maxP.z()) ;

  glColor() ;
  glVertex3f(minP.x(),minP.y(),maxP.z()) ;  glVertex3f(minP.x(),maxP.y(),maxP.z()) ;
  glVertex3f(minP.x(),maxP.y(),minP.z()) ;  glVertex3f(maxP.x(),maxP.y(),minP.z()) ;
  glVertex3f(minP.x(),maxP.y(),minP.z()) ;  glVertex3f(minP.x(),maxP.y(),maxP.z()) ;
  glVertex3f(minP.x(),minP.y(),maxP.z()) ;  glVertex3f(maxP.x(),minP.y(),maxP.z()) ;
  glVertex3f(minP.x(),maxP.y(),maxP.z()) ;  glVertex3f(maxP.x(),maxP.y(),maxP.z()) ;
  glVertex3f(maxP.x(),minP.y(),minP.z()) ;  glVertex3f(maxP.x(),minP.y(),maxP.z()) ;
  glVertex3f(maxP.x(),minP.y(),maxP.z()) ;  glVertex3f(maxP.x(),maxP.y(),maxP.z()) ;
  glVertex3f(maxP.x(),maxP.y(),minP.z()) ;  glVertex3f(maxP.x(),minP.y(),minP.z()) ;
  glVertex3f(maxP.x(),maxP.y(),maxP.z()) ;  glVertex3f(maxP.x(),maxP.y(),minP.z()) ;

  glEnd() ;
}

/*!
  \brief the default constructor

  Initialized the colors of the objects.

  \author Philippe Lavoie
  \date 23 September 1997
*/
NurbsGL::NurbsGL() : 
  ObjectGL(), editSP(1), nurbsRenderer(0), nUlines(4), nVlines(4)
{ 
  polygon = 0 ;
  nurbsState = objectState; 
  category = nurbsType ;
  editControlPoints(0) ;
  editFixPoints(0) ; 
  cpoints.objectColor = cpointColorDefault ;
  cpoints.currentColor = cpointActiveColorDefault ;
  knots.objectColor = knotColorDefault ;
  knots.currentColor = knotActiveColorDefault ;
  cpoints.hideObject() ;
  knots.hideObject() ;
  setULines(4) ;
  setVLines(4) ; 
  //resetAll(); 
}


/*!
  \brief displays the object if it's not hiding

  Displays the object if it's not hiding.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsGL::glObject() const {
  if(getState()){
    if(nurbsState){
      gluNurbs() ;
    }
    if(cpoints.getState())
      cpoints.glObject() ;
    if(polygon->getState())
      polygon->glObject() ;
    if(knots.getState())
      knots.glObject() ;
  }
  if(bbox.getState())
    bbox.glObject() ;
}

/*!
  \relates NurbsGL
  \brief reads a Nurb object from a file

  Reads a NURBS object from a file. The routine can 
  read a Nurbs curve or surface.

  \param filename  the name of the file to read
  \param o  a pointer to a Nurbs Object. If this pointer
	    as any value upon entry, the old entry is
	    deleted first.

  \return 1 if the Nurbs object was read succesfully, 0 otherwise

  \warning Calling this function without a proper curve initialized might
           result in strange results.

  \author Philippe Lavoie
  \date 23 September 1997
*/
NurbsGL* readNurbsObject(const char* filename) {
  NurbsGL *temp ;
  // guess the type of the curve first, if that doesn't work try all of them
  char* ext ; 
  //ext = strstr(filename,".n()ca") ;
  //if(ext){
  //  openByType = OPENCURVEARRAY ;      
  //}
  //else{
  ext = strstr(filename,".n()c") ;
  if(ext){
    temp = new NurbsCurveGL ;
    if(temp->read(filename)){
      temp->resetBoundingBox() ;
      return temp ;
    }
    else
      delete temp ;
  }
  else{
    ext = strstr(filename,".n()s") ;
    if(ext){
      temp = new NurbsSurfaceGL ;
      if(temp->read(filename)){
	return temp ;
      }
      else
	delete temp ;
    }
  }
  //}
   
  // try all the types 1 by 1
  temp = new NurbsCurveGL ;
  if(temp->read(filename)){
    return temp ;
  }
  else
    delete temp ;
  
  temp = new NurbsSurfaceGL ;
  if(temp->read(filename)){
    return temp ;
  }
  else
    delete temp ;

  return 0 ;
}

/*!
  \brief resets the minP and maxP values of bbox

  Resets the minP and maxP values for the bouding box.

  \warning Calling this function without a proper curve initialized might
               result in strange results.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsCurveGL::resetBoundingBox(){
  bbox.minP.x() = extremum(1,coordX) ;
  bbox.minP.y() = extremum(1,coordY) ;
  bbox.minP.z() = extremum(1,coordZ) ;
  bbox.maxP.x() = extremum(0,coordX) ;
  bbox.maxP.y() = extremum(0,coordY) ;
  bbox.maxP.z() = extremum(0,coordZ) ;
}

/*!
  \brief resets the minP and maxP values of bbox

  Resets the minP and maxP values for the bouding box.

  \warning Calling this function without a proper surface initialized might
               result in strange results.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsSurfaceGL::resetBoundingBox(){
  bbox.minP.x() = extremum(1,coordX) ;
  bbox.minP.y() = extremum(1,coordY) ;
  bbox.minP.z() = extremum(1,coordZ) ;
  bbox.maxP.x() = extremum(0,coordX) ;
  bbox.maxP.y() = extremum(0,coordY) ;
  bbox.maxP.z() = extremum(0,coordZ) ;
}

/*!
  \brief resets the minP and maxP values of bbox

  Resets the minP and maxP values for the bouding box.

  \warning Calling this function without a proper surface initialized might
               result in strange results.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void HNurbsSurfaceGL::resetBoundingBox(){
  bbox.minP.x() = extremum(1,coordX) ;
  bbox.minP.y() = extremum(1,coordY) ;
  bbox.minP.z() = extremum(1,coordZ) ;
  bbox.maxP.x() = extremum(0,coordX) ;
  bbox.maxP.y() = extremum(0,coordY) ;
  bbox.maxP.z() = extremum(0,coordZ) ;
}


/*!
  \brief creates a nurbs curve for OpenGL

  This function calls between a gluBeginCurve/gluEndCurve
  the proper functions to generate a NURBS curve.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsCurveGL::gluNurbs() const{
  if(nurbsRenderer){
    GLenum t = GL_MAP1_VERTEX_4 ;
    
    gluBeginCurve(nurbsRenderer) ;
    glColor() ;
    gluNurbsCurve(nurbsRenderer, U.n(), U.memory(), 4, P.memory()->data, deg_+1, t) ;
    gluEndCurve(nurbsRenderer) ;
    
    /*
      // This code tests the tesselation code for a Nurbs Curve
    BasicList<Point3Df> list ;
    list = tesselate(0.01) ;
    BasicNode<Point3Df>* node = (BasicNode<Point3Df>*)list.first ;
    glBegin(GL_LINE_STRIP) ;
    glColor() ;
    while(node){
      glVertex3fv(node->data->data) ;
      node = node->next ;
    }    
    glEnd() ;
    */
  }
}

/*!
  \brief creates a nurbs curve for OpenGL

  This function calls between a gluBeginCurve/gluEndCurve
  the proper functions to generate a NURBS curve.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void SimpleNurbsCurveGL::gluNurbs() const{
  if(nurbsRenderer){
    GLenum t = GL_MAP1_VERTEX_4 ;
    
    gluBeginCurve(nurbsRenderer) ;
    glColor() ;
    gluNurbsCurve(nurbsRenderer, U.n(), U.memory(), 4, P.memory()->data, deg_+1, t) ;
    gluEndCurve(nurbsRenderer) ;
    
  }
}

/*!
  \brief apply the local transformation to the curve.

  Apply the local transformation to the curve. This is necessary
  if you want to get the proper position for the control points
  before doing anymore processing on them.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void PointListGL::applyTransform(){
  rx *= M_PI/180.0 ;
  ry *= M_PI/180.0 ;
  rz *= M_PI/180.0 ;

  MatrixRT<float> Tx(rx,ry,rz,tx,ty,tz) ;
  MatrixRT<float> Sx ; 
  Sx.scale(sx,sy,sz) ;

  MatrixRT<float> TM(Tx*Sx) ;

  BasicNode<Point3Df> *node ;
  node = (BasicNode<Point3Df>*)list.first() ;
  
  while(node){
    *node->data = TM * (*node->data) ;
    node = node->next ;
  }
  
  tx = ty = tz = rx = ry = rz = 0 ;
  sx = sy = sz = 1 ;
}

/*!
  \brief apply the local transformation to the curve.

  Apply the local transformation to the curve. This is necessary
  if you want to get the proper position for the control points
  before doing anymore processing on them.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsCurveGL::applyTransform(){
  rx *= M_PI/180.0 ;
  ry *= M_PI/180.0 ;
  rz *= M_PI/180.0 ;

  MatrixRTf T(rx,ry,rz,tx,ty,tz) ;
  MatrixRTf Sx ; 
  Sx.scale(sx,sy,sz) ;

  transform(T*Sx) ;
  
  tx = ty = tz = rx = ry = rz = 0 ;
  sx = sy = sz = 1 ;
  resetAll() ;
}

/*!
  \brief apply the local transformation to the surface.

  Apply the local transformation to the surface. This is necessary
  if you want to get the proper position for the control points
  before doing anymore processing on them.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsSurfaceGL::applyTransform(){
  rx *= M_PI/180.0 ;
  ry *= M_PI/180.0 ;
  rz *= M_PI/180.0 ;

  MatrixRTf T(rx,ry,rz,tx,ty,tz) ;
  MatrixRTf Sx ; 
  Sx.scale(sx,sy,sz) ;

  transform(T*Sx) ;
  
  tx = ty = tz = rx = ry = rz = 0 ;
  sx = sy = sz = 1 ;
  resetAll() ;
}

/*!
  \brief draws the polygon joining the control points

  This function calls between a glBegin/glEnd
  the proper functions to represent the polygon joining
  all the control points of the NURBS curve.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsCpolygonGL::glObject() const{

  glBegin(GL_LINE_STRIP);
  glColor() ;
  for(int i=0;i<curve.ctrlPnts().n();++i)
    glVertex4fv(curve.ctrlPnts()[i].data) ;
  glEnd() ;
}


/*!
  \brief draws the polygon joining the control points

  This function calls between a glBegin/glEnd
  the proper functions to represent the polygon joining
  all the control points of the NURBS surface.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsSpolygonGL::glObject() const{
  int i,j ;
  for(i=0;i<surface.ctrlPnts().rows();++i){
    glBegin(GL_LINE_STRIP);
    glColor() ;
    for(j=0;j<surface.ctrlPnts().cols();++j)
      glVertex4fv(surface.ctrlPnts()(i,j).data) ;
    glEnd() ;
  }
  
  for(j=0;j<surface.ctrlPnts().cols();++j){
    glBegin(GL_LINE_STRIP);
    glColor() ;
    for(i=0;i<surface.ctrlPnts().rows();++i)
      glVertex4fv(surface.ctrlPnts()(i,j).data);
    glEnd() ;
  }    
}

/*!
  \brief reset the control point information

  Reset the control points information stored in cpoints.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsCurveGL::resetCPoints() {
  CPointGL *tempCP ;

  cpoints.reset() ;
  if(P.n()>10)
    cpoints.setJumpSize(int(P.n()/10.0)) ;
  else
    cpoints.setJumpSize(int(float(P.n())/2.001 + 1)) ;

  for(int i=0;i<P.n();++i){
    if(editControlPoints())
      tempCP = new CPointGL(P[i],i) ;
    else
      tempCP = new SPointCurveGL(i,this,editFix) ;
    tempCP->setObjectColor(cpointColorDefault) ;
    tempCP->setCurrentColor(cpointActiveColorDefault) ;
    if(i==0)
      tempCP->setObjectColor(cpoint0ColorDefault) ;
    cpoints.add(tempCP) ;
  }
  
  if(editSurfacePoints()){  
    // we need to setup the start-end data for the surface points
    SPointCurveGL *a,*b ;
    a = (SPointCurveGL*)cpoints.first() ;
    while(a){
      b = a ; 
      int r ;
      r = 0 ;
      while(b->previous() && r<deg_+1){
	b = (SPointCurveGL*)b->previous() ; 
	++r ; 
      }
      a->setStartEnd(b,deg_+r+1) ; 
      a = (SPointCurveGL*) a->next() ;
    }
  }

}

/*!
  \brief Reset the knots information

  Reset the knot information stored in knots.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsCurveGL::resetKnots() {
  KnotGL *tempCP ;

  knots.reset() ;
  if(P.n()>10)
    knots.setJumpSize(int(P.n()/10.0)) ;
  else
    knots.setJumpSize(2) ;

  for(int i=0;i<U.n();++i){
    Point3Df p = pointAt(U[i]) ;
    // we must represent multiple knots by only 1 point
    // but with a bigger radius (increase radius by 1)
    tempCP = new KnotGL(p,i,-1) ;
    tempCP->setObjectColor(knots.objectColor) ;
    tempCP->setCurrentColor(knots.currentColor) ;
    if(i==0)
      tempCP->setObjectColor(cpoint0ColorDefault) ;
    int psize = 4 ;
    int j=i ;
    while(j<U.n()-1){
      if(U[j+1]<=U[j]){
	++psize ; ++j ;
      }
      else
	j = U.n() ;
    }
    tempCP->setPsize(psize) ;
    knots.add(tempCP) ;
  }
}

/*!
  \brief Reset the control point information

  Reset the control point information stored in cpoints.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsSurfaceGL::resetCPoints() {
  CPointGL *tempCP ;

  cpoints.reset() ;
  cpoints.setJumpSize(P.cols()) ;

  for(int i=0;i<P.rows();++i)
    for(int j=0;j<P.cols();++j){
      if(editControlPoints())
	tempCP = new CPointGL(P(i,j),i,j) ;
      else
	tempCP = new SPointSurfaceGL(i,j,this,&cpoints,editFix) ; 
      tempCP->setObjectColor(cpointColorDefault) ;
      tempCP->setCurrentColor(cpointActiveColorDefault) ;
      if(j==0 && (i!=P.rows()-1))
	tempCP->setObjectColor(cpoint0ColorDefault) ;
      cpoints.add(tempCP) ;
    }

  if(editSurfacePoints()){  
    // we need to setup the start-end data for the surface points
    SPointSurfaceGL *a,*b ;
    a = (SPointSurfaceGL*)cpoints.first() ;
    while(a){
      b = a ; 
      int r,c ;
      r = 0 ;
      c = 0 ;
      cpoints.current() = (ObjectGL*)a ;
      while(cpoints.jumpToPrevious() && r<degU){
	if(((SPointSurfaceGL*)cpoints.current())->row() < a->row() )
	  b = (SPointSurfaceGL*) cpoints.current() ;
	else
	  break ;
	++r ;
      }
	
      while(b->previous() && c<degV){
	if(((SPointSurfaceGL*)b->previous())->col() < a->col() )
	  b = (SPointSurfaceGL*)b->previous() ; 
	else
	  break ;
	++c ; 
      }
      a->setStartEnd(b,degU+r+1,degV+c+1) ; 
      a = (SPointSurfaceGL*) a->next() ;
    }
    cpoints.current() = (ObjectGL*)cpoints.first() ;
  }
}

/*!
  \brief Reset the control point information

  Reset the control point information stored in cpoints

  \author Philippe Lavoie
  \date 23 September 1997
*/
void HNurbsSurfaceGL::resetCPoints() {
  ObjectGL *tempCP ;
  if(activePatch){
    cpoints.reset() ;
    cpoints.setJumpSize(activePatch->ctrlPnts().cols()) ;
    
    for(int i=0;i<activePatch->ctrlPnts().rows();++i)
      for(int j=0;j<activePatch->ctrlPnts().cols();++j){
	int mod ;
	mod = 1 ;
	if(i==0 || j==0 
	   || i==activePatch->ctrlPnts().rows()-1 
	   || j==activePatch->ctrlPnts().cols()-1)
	  mod = 0 ;
	mod = 1 ; // if we don't use multi-patches it should be safe
	if(editControlPoints())
	  tempCP = new HCPointGL(activePatch->offset(i,j),i,j,activePatch,mod) ;
	else
	  tempCP = new SPointHSurfaceGL(i,j,activePatch,&cpoints,editFix) ; 
	tempCP->setObjectColor(cpointColorDefault) ;
	tempCP->setCurrentColor(cpointActiveColorDefault) ;
	if(j==0 && (i!=activePatch->ctrlPnts().rows()-1))
	  tempCP->setObjectColor(cpoint0ColorDefault) ;
	cpoints.add(tempCP) ;
      }

    if(editSurfacePoints()){  
      // we need to setup the start-end data for the surface points
      SPointSurfaceGL *a,*b ;
      a = (SPointSurfaceGL*)cpoints.first() ;
      while(a){
	b = a ; 
	int r,c ;
	r = 0 ;
	c = 0 ;
	cpoints.current() = (ObjectGL*)a ;
	while(cpoints.jumpToPrevious() && r<degU){
	  if(((SPointSurfaceGL*)cpoints.current())->row() < a->row() )
	    b = (SPointSurfaceGL*) cpoints.current() ;
	  else
	    break ;
	  ++r ;
	}
	
	while(b->previous() && c<degV){
	  if(((SPointSurfaceGL*)b->previous())->col() < a->col() )
	    b = (SPointSurfaceGL*)b->previous() ; 
	  else
	    break ;
	  ++c ; 
	}
	a->setStartEnd(b,degU+r+1,degV+c+1) ; 
	a = (SPointSurfaceGL*) a->next() ;
      }
      cpoints.current() = (ObjectGL*)cpoints.first() ;
    }
  }
}

/*!
  \brief reset the knots information

  Reset the knots information stored in knots.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsSurfaceGL::resetKnots() {
  KnotGL *tempCP ;

  knots.reset() ;
  knots.setJumpSize(P.cols()) ;

  Vector<int> sU,sV ;
  sU.resize(U.n()) ;
  sV.resize(V.n()) ;

  int i,j ;
  
  sU[0] = 0 ;
  for(i=0;i<U.n()-1;++i){
    if(U[i+1]>=U[i]){
      sU[i+1] = sU[i]+1 ;
      sU[i] = 0 ;
    }
    else
      sU[i+1] = 1 ;
  }

  sV[0] = 0 ;
  for(i=0;i<V.n()-1;++i){
    if(V[i+1]>=V[i]){
      sV[i+1] = sV[i]+1 ;
      sV[i] = 0 ;
    }
    else
      sV[i+1] = 1 ;
  }

  for(i=0;i<U.n();++i)
    for(j=0;j<V.n();++j){
      //if(sU[i] && sV[i]){
	Point3Df p = pointAt(U[i],V[j]) ;
	// we must represent multiple knots by only 1 point
	// but with a bigger radius (increase radius by 1)
	tempCP = new KnotGL(p,i,j) ;
	tempCP->setObjectColor(knotColorDefault) ;
	tempCP->setCurrentColor(knotActiveColorDefault) ;
	if(j==0 && (i!=P.rows()-1))
	  tempCP->setObjectColor(cpoint0ColorDefault) ;
	tempCP->setPsize(3+maximum(sU[i],sV[j])) ;
	knots.add(tempCP) ;
	//}
    }
}

/*!
  \brief draws a point at the location \a C(u)

  This function calls between a glBegin/glEnd
  the proper functions to represent the point which is
  at \a C(u) on the curve.
  
  \param u  the parametric value
  \param v  a dummy variable so the call is the same as with a NURBS surface.
  \param psize  the size of the control points
  \param colorP  the color of the control points

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsCurveGL::point(float &u,float &v,int pSize,const Color& colorP, int cp_flag) const {
  if(u<U[0])
    u = U[0] ;
  if(u>U[U.n()-1])
    u = U[U.n()-1] ;
  glPointSize(pSize) ;
  glBegin(GL_POINTS);
  float color[4] ;
  color[0] = color[1] = color[2] =  0.0 ;
  color[3] = 1.0 ;
  glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color) ;
  color[0] = float(colorP.r)/255.0 ;
  color[1] = float(colorP.g)/255.0 ;
  color[2] = float(colorP.b)/255.0 ;
  glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,color) ;
  glVertex4fv(hpointAt(u).data) ;
  glEnd() ;
  v *= 1.0 ; // to shut up the warning messages
  
  if(cp_flag){ // if this is set, only display CP which affect the point
    
  }
}

/*!
  \brief creates a nurbs surface for OpenGL

  This function calls between a gluBeginSurface/gluEndSurface
  the proper functions to generate a NURBS surface.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsSurfaceGL::gluNurbs() const{
  int i ;
  if(nurbsRenderer){
    switch(NurbsDisplayMode){
    case NURBS_DISPLAY_ISOCURVES:
      {
      // create iso curves
      SimpleNurbsCurveGL t ;
      t.setObjectColor(objectColor,selectColor,currentColor) ;
      t.setNurbsRenderer(nurbsRenderer) ;
      t.hideBBox() ;
      t.hideCPoints() ;
      t.hideCpolygon() ;
      t.viewNurbs() ;
      if(isActive()) t.activate() ;
      if(isSelected()) t.select() ;

      int i ;
      for(i=0;i<nUlines;++i){
	float u = U[0] + float(i)/float(nUlines-1) * (U[U.n()-1]-U[0]) ; 
	isoCurveU(u,t) ;
	t.glObject() ;
      }

      for(i=0;i<nVlines;++i){
	float v = V[0] + float(i)/float(nVlines-1) * (V[V.n()-1]-V[0]) ; 
	isoCurveV(v,t) ;
	t.glObject() ;
      }
    }
      break ;
    case NURBS_DISPLAY_SHADED:
      {
	GLenum t = GL_MAP2_VERTEX_4 ; 
	if(image)
	  glShadeModel(GL_FLAT) ;
	else
	  glShadeModel(GL_SMOOTH) ;
	gluNurbsProperty(nurbsRenderer,GLU_DISPLAY_MODE,GLU_FILL) ;

	
    //GLfloat lmodel_ambient[] = { 0.3, 0.3, 0.3, 1.0 };

    //glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

    //glEnable(GL_LIGHTING);


    //glDepthFunc(GL_LESS);
    //glEnable(GL_DEPTH_TEST);
    //glEnable(GL_AUTO_NORMAL);
    
    //colorToMaterial(Color(255,100,255),materialColor) ;
    //glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,materialColor) ;
	//colorToMaterial(Color(0,0,0),materialColor) ;
	//glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,materialColor) ;
    //glColor() ;


    gluBeginSurface(nurbsRenderer) ;
    if(image){
      static GLfloat texpts[2][2][2] = {{{0,0},{0,1}},{{1,0},{1,1}}} ;
      
      glMap2f(GL_MAP2_TEXTURE_COORD_2,0,1,2,2,0,1,4,2,&texpts[0][0][0]) ;
      glEnable(GL_MAP2_TEXTURE_COORD_2) ;
      glMapGrid2f(20,0,1,20,0,1) ;
      glEvalMesh2(GL_FILL,0,20,0,20) ;
      glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE, GL_DECAL) ;
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT) ;
      glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT) ;
      glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST) ;
      glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST) ;
      glTexImage2D(GL_TEXTURE_2D,0,3,imgW,imgH,0,GL_RGB,GL_UNSIGNED_BYTE,image) ;
      glShadeModel(GL_FLAT) ;
      glEnable(GL_TEXTURE_2D) ;
    }
    GLfloat mat_ambient[] = { 0.8, 0.8, 0.8, 1.0 };
    GLfloat mat_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
    GLfloat mat_specular[] = { 0.8, 0.8, 0.8, 1.0 };
    GLfloat mat_shininess[] = { 50.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);

    gluNurbsSurface(nurbsRenderer, U.n(), U.memory(), V.n(), V.memory(),4,4*P.rows(), P[0]->data, degU+1, degV+1, t) ;
    gluEndSurface(nurbsRenderer) ;
    glDisable(GL_TEXTURE_2D) ;
      }
      break ;
    case NURBS_DISPLAY_HIDDEN_LINES:
      {
	GLenum t = GL_MAP2_VERTEX_4 ; 
	gluNurbsProperty(nurbsRenderer,GLU_DISPLAY_MODE,GLU_FILL) ;

	glDisable(GL_LIGHTING);
	glColor3f(.7f, .7f, .7f);
	
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_ALWAYS, 0, 0); /* clear stencil for this object */

	gluBeginSurface(nurbsRenderer) ;
	//glColor() ;
	gluNurbsSurface(nurbsRenderer, U.n(), U.memory(), V.n(), V.memory(),4,4*P.rows(), P[0]->data, degU+1, degV+1, t) ;
	gluEndSurface(nurbsRenderer) ;

	glEnable(GL_LIGHTING);

	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE); /* turn off color */
	glDisable(GL_DEPTH_TEST); /* turn off depth */
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glStencilFunc(GL_ALWAYS, 1, 1);
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

	gluBeginSurface(nurbsRenderer) ;
	//glColor() ;
	gluNurbsSurface(nurbsRenderer, U.n(), U.memory(), V.n(), V.memory(),4,4*P.rows(), P[0]->data, degU+1, degV+1, t) ;
	gluEndSurface(nurbsRenderer) ;

	glStencilFunc(GL_EQUAL, 1 , 1);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

	gluBeginSurface(nurbsRenderer) ;
	//glColor() ;
	gluNurbsSurface(nurbsRenderer, U.n(), U.memory(), V.n(), V.memory(),4,4*P.rows(), P[0]->data, degU+1, degV+1, t) ;
	gluEndSurface(nurbsRenderer) ;


	/* clean up state */
	glDisable(GL_STENCIL_TEST);
	glDepthFunc(GL_LESS);
      }
      break ;
    case NURBS_DISPLAY_TESSELATION:
      {
	/*
	BasicList<Point3Df> pts ;
	BasicList<int> connect ;
	if(tessel_tolerance<0.001) 
	  tessel_tolerance = 0.001 ;
	tesselate(tessel_tolerance,pts,connect) ;

	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE) ;
	glBegin(GL_TRIANGLES) ;
	glColor() ;
	connect.goToFirst() ;
	pts.goToFirst() ;
	for(i=0;i<connect.size();++i){
	  if(*(connect[i]->data) >= 0)
	    glVertex3fv(pts[*(connect[i]->data)]->data->data) ;
	}
	glEnd() ;
	*/
	Color col ;
	if(isActive()) col = currentColor ;
        else{
	  if(isSelected()) col = selectColor ; 
	  else col = objectColor ; 
	}
	NurbsSubSurfaceGL s(*this,col) ;
	s.drawSubdivisionGL(0.01) ;
      }
      break ;
    case NURBS_DISPLAY_NORMAL:
    default:
      {
      GLenum t = GL_MAP2_VERTEX_4 ; 
      gluNurbsProperty(nurbsRenderer,GLU_DISPLAY_MODE,GLU_OUTLINE_POLYGON) ;
      gluBeginSurface(nurbsRenderer) ;
      glColor() ;
      gluNurbsSurface(nurbsRenderer, U.n(), U.memory(), V.n(), V.memory(),4,4*P.rows(), P[0]->data, degU+1, degV+1, t) ;
      
      for(std::list<NurbsCurve_2Df*>::const_iterator it = trimmedCurves.begin() ;
	  it != trimmedCurves.end(); ++it){
	NurbsCurve_2Df *tc = *it ; 
	gluBeginTrim(nurbsRenderer) ;
	gluNurbsCurve(nurbsRenderer,tc->knot().n(),tc->knot().memory(),3,tc->ctrlPnts().memory()->data, 3, GLU_MAP1_TRIM_3 ) ;
	gluEndTrim(nurbsRenderer) ; 
      }

      gluEndSurface(nurbsRenderer) ;
      }      
    }
  }
}

/*
void splitAtActiveInU(NurbsCurveGL &t, NurbsCurveGL &ta, float u, HNurbsSurface *active, int lod){
  if(lod >= 0 && lod < active->level)
    return ;
  if(u<active->uS || u >active->uE){
    t.glObject() ;
    return ;
  }
  NurbsCurve c ;
  c = t ;
  ta = c ;
  if(active->vS<=c.Knot[0] && active->vE>=c.Knot[c.Knot.n()-1]){
    ta.glObject() ;
    return ;
  }
  if(active->vS>c.Knot[0] && active->vE < c.Knot[c.Knot.n()-1]){
    // active curve is in the middle
    c.splitAt(active->vS,t,ta) ;
    t.glObject() ;
    c = ta ;
    c.splitAt(active->vE,ta,t) ;
    t.glObject() ;
    ta.glObject() ;
  }
  else{
    if(active->vS>c.Knot[0]) {
      c.splitAt(active->vS,t,ta) ;
      t.glObject() ;
      ta.glObject() ;
    }
    else{
      c.splitAt(active->vE,ta,t) ;
      t.glObject() ;
      ta.glObject() ;
    }
  }
}

void splitAtActiveInV(NurbsCurveGL &t, NurbsCurveGL &ta, float v, HNurbsSurface *active, int lod){
  if(lod >= 0 && lod < active->level)
    return ;
  if(v<active->vS || v >active->vE){
    t.glObject() ;
    return ;
  }
  NurbsCurve c ;
  c = t ;
  ta = c ;
  if(active->uS<=c.Knot[0] && active->uE>=c.Knot[c.Knot.n()-1]){
    ta.glObject() ;
    return ;
  }
  if(active->uS>c.Knot[0] && active->uE < c.Knot[c.Knot.n()-1]){
    // active curve is in the middle
    c.splitAt(active->uS,t,ta) ;
    t.glObject() ;
    c = ta ;
    c.splitAt(active->uE,ta,t) ;
    t.glObject() ;
    ta.glObject() ;
  }
  else{
    if(active->uS>c.Knot[0]) {
      c.splitAt(active->uS,t,ta) ;
      t.glObject() ;
      ta.glObject() ;
    }
    else{
      c.splitAt(active->uE,ta,t) ;
      t.glObject() ;
      ta.glObject() ;
    }
  }
}
*/

void isoCurves(const HNurbsSurfaceSPf *hs, SimpleNurbsCurveGL &t, int lod, int nu, int nv){
  if(lod<0)
    lod = hs->maxLevel() ; 

  if(lod>=0 && lod<hs->level()){
    return ;
  }

  if(lod == hs->level()){
    int i ;
    for(i=0;i<nu+hs->level();++i){
      float u = hs->knotU()[0] + float(i)/float(nu+hs->level()-1) * (hs->knotU()[hs->knotU().n()-1] - hs->knotU()[0]) ; 
      hs->isoCurveU(u,t,lod) ; 
      t.glObject() ; 
    }
    for(i=0;i<nv+hs->level();++i){
      float v = hs->knotV()[0] + float(i)/float(nv+hs->level()-1) * (hs->knotV()[hs->knotV().n()-1] - hs->knotV()[0]) ; 
      hs->isoCurveV(v,t,lod) ; 
      t.glObject() ; 
    }
    return ; 
  }
 
  HNurbsSurfaceSPf *child ; 
  child = (HNurbsSurfaceSPf*) hs->nextLevel() ;
  if(child){
    isoCurves(child,t,lod,nu,nv) ;
  }
  return ; 
}

/*!
  \brief creates a HNURBS surface for OpenGL

  This draws a HNURBS. Presently only isocurves are drawn
  to represent the surface.

  \author Philippe Lavoie
  \date 23 September 1997
*/
void HNurbsSurfaceGL::gluNurbs() const{
  if(nurbsRenderer){

    HNurbsSurfaceGL* dispPatch ;
    dispPatch = activePatch ;
    int level = levelOfDetail() ;
    if(level<0) 
      level = maxLevel() ;
    while(dispPatch->level() < level)
      dispPatch = (HNurbsSurfaceGL*)dispPatch->nextLevel() ;
    
    switch(NurbsDisplayMode){
    case NURBS_DISPLAY_ISOCURVES:
      {
      // create iso curves
      SimpleNurbsCurveGL t ;
      t.setObjectColor(objectColor,selectColor,currentColor) ;
      t.setNurbsRenderer(nurbsRenderer) ;
      t.hideBBox() ;
      t.hideCPoints() ;
      t.hideCpolygon() ;
      t.viewNurbs() ;
      if(isActive()) t.activate() ;
      if(isSelected()) t.select() ;

      isoCurves(dispPatch,t,level,nUlines,nVlines) ; 

      if(dispPatch->level() != activePatch->level()){
	t.setObjectColor(Color(255,255,0),Color(255,255,0),Color(255,255,0));
	isoCurves(activePatch,t,activePatch->level(),nUlines,nVlines) ; 
      }

    }
      break ;
    case NURBS_DISPLAY_SHADED:
    case NURBS_DISPLAY_HIDDEN_LINES:
    case NURBS_DISPLAY_TESSELATION:
    case NURBS_DISPLAY_NORMAL:
    default:
      {
      GLenum t = GL_MAP2_VERTEX_4 ; 
      gluNurbsProperty(nurbsRenderer,GLU_DISPLAY_MODE,GLU_OUTLINE_POLYGON) ;
      gluBeginSurface(nurbsRenderer) ;
      glColor() ;
      gluNurbsSurface(nurbsRenderer, dispPatch->U.n(), dispPatch->U.memory(), dispPatch->V.n(), dispPatch->V.memory(),4,4*dispPatch->P.rows(), dispPatch->P[0]->data, dispPatch->degU+1, dispPatch->degV+1, t) ;
      gluEndSurface(nurbsRenderer) ;
      }      
    }
  }
}

/*!
  \brief draws a point at the location \a C(u)

  This function calls between a glBegin/glEnd
  the proper functions to represent the point which is
  at \a S(u,v) on the surface.

  \param u  the \a U parametric value
  \param v  the \a V parametric value
  \param psize  the size of the control points
  \param colorP  the color of the control points

  \author Philippe Lavoie
  \date 23 September 1997
*/
void NurbsSurfaceGL::point(float &u, float &v, int pSize,const Color& colorP, int cp_flag) const {
  if(u<U[0])
    u = U[0] ;
  if(u>U[U.n()-1])
    u = U[U.n()-1] ;
  if(v<V[0])
    v = V[0] ;
  if(v>V[V.n()-1])
    v = V[V.n()-1] ;
  glPointSize(pSize) ;
  glBegin(GL_POINTS);
  float color[4] ;
  color[0] = color[1] = color[2] =  0.0 ;
  color[3] = 1.0 ;
  glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color) ;
  color[0] = float(colorP.r)/255.0 ;
  color[1] = float(colorP.g)/255.0 ;
  color[2] = float(colorP.b)/255.0 ;
  glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,color) ;
  glVertex4fv(hpointAt(u,v).data) ;
  glEnd() ;

  SimpleNurbsCurveGL t ;
  t.setObjectColor(colorP,colorP,colorP) ;
  t.setNurbsRenderer(nurbsRenderer) ;
  t.hideBBox() ;
  t.hideCPoints() ;
  t.hideCpolygon() ;
  t.viewNurbs() ;
  isoCurveU(u,t) ;
  t.glObject() ;
  isoCurveV(v,t) ;
  t.glObject() ;
}


/*!
  \brief draws a point at the location \a C(u)

  This function calls between a glBegin/glEnd
  the proper functions to represent the point which is
  at \a S(u,v) on the hierarchical surface.

  \param u  the \a U parametric value
  \param v  the \a V parametric value
  \param psize  the size of the control points
  \param colorP  the color of the control points

  \author Philippe Lavoie
  \date 23 September 1997
*/
void HNurbsSurfaceGL::point(float &u, float &v, int pSize,const Color& colorP, int cp_flag) const {
  if(u<U[0])
    u = U[0] ;
  if(u>U[U.n()-1])
    u = U[U.n()-1] ;
  if(v<V[0])
    v = V[0] ;
  if(v>V[V.n()-1])
    v = V[V.n()-1] ;
  glPointSize(pSize) ;
  glBegin(GL_POINTS);
  float color[4] ;
  color[0] = color[1] = color[2] =  0.0 ;
  color[3] = 1.0 ;
  glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color) ;
  color[0] = float(colorP.r)/255.0 ;
  color[1] = float(colorP.g)/255.0 ;
  color[2] = float(colorP.b)/255.0 ;
  glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,color) ;
  glVertex4fv(hpointAt(u,v,levelOfDetail()).data) ;
  glEnd() ;


  SimpleNurbsCurveGL t ;
  t.setObjectColor(colorP,colorP,colorP) ;
  t.setNurbsRenderer(nurbsRenderer) ;
  t.hideBBox() ;
  t.hideCPoints() ;
  t.hideCpolygon() ;
  t.viewNurbs() ;
  isoCurveU(u,t,levelOfDetail()) ;
  t.glObject() ;
  isoCurveV(v,t,levelOfDetail()) ;
  t.glObject() ;  

  if(cp_flag){ // display only the CP affected by the point
    int spanU, spanV ;
    activePatch->findSpan(u,v,spanU,spanV) ;
    int i,j,n ;
    
    ObjectGL *pnt ;
    pnt = (ObjectGL*)cpoints.first() ;
    n = 0 ;
    while(pnt){
      i = n / activePatch->ctrlPnts().cols() ;
      j = n % activePatch->ctrlPnts().cols() ;
      if(i<spanU-activePatch->degreeU() || i>spanU)
	pnt->hideObject() ;
      else{
	if(j<spanV-activePatch->degreeV() || j>spanV)
	  pnt->hideObject() ;
	else
	  pnt->viewObject() ;
      }
      ++n ;
      pnt = pnt->next() ;      
    }
  }
}


/*!
  \brief default constructor

  This sets the values of all the variables to their default
  values (according to their manual).

  \author Philippe Lavoie
  \date 13 October 1997
*/
Material::Material(){
  // POV-Ray options
  // finish options
  //double ambient = 0.2 ;
  //double diffuse = 0.6 ;
  //double roughness = 0 ;
}

/*!
  \brief reads a list of objects

  \param filename  the name of the file to read from 

  \return 1 on success, 0 otherwise

  \author Philippe Lavoie
  \date 13 October 1997
*/
int ObjectListGL::read(const char* filename){
  ifstream fin(filename) ;
  if(!fin)
    return 0 ;
  fin.clear() ;
  fin.seekg(0,ios::beg) ;

  char *type ;
  type = new char[3] ;

  ObjectGL* newObject ;

  while(!fin.eof()){
    // get the first 3 characters to determine the type of the object
    int mark = fin.tellg() ; // mark position
    if(!fin.read(type,sizeof(char)*3)){ delete []type ;  return 1 ;  }
    fin.seekg(mark) ; // restore position

    int ot=0 ; // unknown object
    char* ext ;

    ext = strstr(type,"nc3");
    if(ext) ot = 1 ;
    if(!ot){
      ext = strstr(type,"nc4");
      if(ext) ot = 1 ;
    }
    if(!ot){
      ext = strstr(type,"ns3");
      if(ext) ot = 2 ;
    }
    if(!ot){
      ext = strstr(type,"ns4");
      if(ext) ot = 2 ;
    }
    if(!ot){
      ext = strstr(type,"hns");
      if(ext) ot = 3 ;
    }
    if(!ot){
      ext = strstr(type,"pt3");
      if(ext) ot = 4 ;
    }
    
    
    if(!ot){  delete []type ;  return 1 ;     }
    
    newObject = 0 ;
    switch(ot){
    case 0: break ;
    case 1: // A NURBS curve
      newObject = new NurbsCurveGL ;
      if(!newObject->read(fin)) {delete []type ; return 0 ;}
      break ;
    case 2: // A NURBS surface
      newObject = new NurbsSurfaceGL ;
      if(!newObject->read(fin)) {delete []type ; return 0 ;}
      break ;
    case 3: // A HNURBS surface
      newObject = new HNurbsSurfaceGL ;
      if(!newObject->read(fin)) { delete []type ; return 0 ; }
      ((HNurbsSurfaceGL*)newObject)->updateLevels() ;
      ((HNurbsSurfaceGL*)newObject)->highestLevelOfDetail() ;      
      break ;
    case 4: // a list of points in 3D
      newObject = new PointListGL ;
      if(!newObject->read(fin)) { delete []type ; cerr << "B" ; return 0 ;}
      break ;
    }
    
    if(newObject)
      add(newObject) ;

    //if(fin.fail()){ delete []type ; return 0 ;  } 
  }
  delete []type ;
  return 1 ;
}

/*!
  \brief Writess a list of objects

  \param filename  the name of the file to read from 

  \return 1 on success, 0 otherwise

  \author Philippe Lavoie
  \date 13 October 1997
*/
int ObjectListGL::write(const char* filename) const {
  ofstream fout(filename) ;
  if(!fout)
    return 0 ;
  
  ObjectGL *t ;
  t = (ObjectGL*)first_ ;
  while(t){
    if(!t->write(fout))
      return 0 ;
    t = t->next() ;
  }
  return fout.good() ;
}

/*!
  \brief Write a list of object in the RIB format 

  \param filename  the name of the file to read from 

  \return 1 on success, 0 otherwise

  \author Philippe Lavoie
  \date 13 October 1997
*/
int ObjectListGL::writeRIB(const char* filename) const {
  ofstream fout(filename) ;
  if(!fout)
    return 0 ;
  
  ObjectGL *t ;
  t = (ObjectGL*)first_ ;
  while(t){
    if(!t->writeRIB(fout))
      return 0;
    t = t->next() ;
  }
  return fout.good() ;
}

/*!
  \brief Write a list of object in the POVRAY format

  \param filename  the name of the file to read from 

  \return 1 on success, 0 otherwise

  \author Philippe Lavoie
  \date 13 October 1997
*/
int ObjectListGL::writePOVRAY(const char* filename) const {
  ofstream fout(filename) ;
  if(!fout)
    return 0 ;
  
  ObjectGL *t ;
  t = (ObjectGL*)first_ ;
  while(t){
    if(!t->writePOVRAY(fout))
      return 0;
    t = t->next() ;
  }
  return fout.good() ;
}

/*!
  \brief Activates the next patch at the same level

  \author Philippe Lavoie
  \date 3 November 1997
*/
/*
void HNurbsSurfaceGL::selectNextPatch(){
  if(activePatch){
    if(activePatch->nextLevel_)
      activePatch = (HNurbsSurfaceGL*)activePatch->nextPatch ;
    else
      if(activePatch->base)
	activePatch = (HNurbsSurfaceGL*)activePatch->base->firstPatch ;
  }
}
*/
/*!
  \brief Activates the previous patch at the same level

  \author Philippe Lavoie
  \date 3 November 1997
*/
/*void HNurbsSurfaceGL::selectPrevPatch(){
  if(activePatch){
    if(activePatch->prevPatch)
      activePatch = (HNurbsSurfaceGL*)activePatch->prevPatch ;
    else
      if(activePatch->baseLevel()){
	activePatch = (HNurbsSurfaceGL*)activePatch->base->lastPatch ;
    }
  }
}
*/
/*!
  \brief Activates the patch at a higher level.

  \author Philippe Lavoie
  \date 3 November 1997
*/
void HNurbsSurfaceGL::selectHigherLevel() {
  if(activePatch){
    if(activePatch->level() < levelOfDetail())
      if(activePatch->nextLevel())
	activePatch = (HNurbsSurfaceGL*)activePatch->nextLevel() ;
  }
}

/*!
  \brief Activates the patch at a higher level.

  \author Philippe Lavoie
  \date 28 January 1998
*/
void HNurbsSurfaceGL::selectHighestLevel() {
  if(activePatch){
    while(activePatch->level() < levelOfDetail())
      if(activePatch->nextLevel())
	activePatch = (HNurbsSurfaceGL*)activePatch->nextLevel() ;
  }
}

/*!
  \brief Activates the patch at a higher level.

  \author Philippe Lavoie
  \date 3 November 1997
*/
void HNurbsSurfaceGL::selectLowerLevel() {
  if(activePatch){
    if(activePatch->baseLevel())
      activePatch = (HNurbsSurfaceGL*)activePatch->baseLevel() ;
  }
}

/*!
  \brief Activates the patch at a higher level.

  \author Philippe Lavoie
  \date 3 November 1997
*/
void HNurbsSurfaceGL::decreaseLevelOfDetail() {
  --lod ; 
  if(lod<0) lod=0 ; 
  if(activePatch)
    while(activePatch->level() > lod)
      activePatch = (HNurbsSurfaceGL*)activePatch->baseLevel() ;
}

/*!
  \brief Copies another ObjectGL

  Copies another ObjectGL. This will not copy the prev and
  next pointers.

  \param a  the object to copy

  \author Philippe Lavoie
  \date 6 November 1997
*/
ObjectGL& ObjectGL::operator=(const ObjectGL &a){
  objectColor = a.objectColor ;
  selectColor = a.selectColor ;
  currentColor = a.currentColor ;
  type = a.type ;
  tx = a.tx ;
  ty = a.ty ;
  tz = a.tz ;
  rx = a.rx ;
  ry = a.ry ;
  rz = a.rz ;
  sx = a.sx ;
  sy = a.sy ;
  sz = a.sz ;
  selected = a.selected ;
  active = a.active ;
  state = a.state ;
  
  return *this ;
}

/*!
  \brief Copies another Nurbs Curve GL

  \param a  the Nurbs curve to copy

  \author Philippe Lavoie
  \date 6 November 1997
*/
NurbsGL& NurbsGL::operator=(const NurbsGL &a){
  this->ObjectGL::operator=(a) ;
  
  nurbsRenderer = a.nurbsRenderer ;
  cpoints.objectColor = a.cpoints.objectColor ;
  cpoints.currentColor = a.cpoints.currentColor ;
  knots.objectColor = a.knots.objectColor ;
  knots.currentColor = a.knots.currentColor ;
  
  editSP = a.editSP ;

  editFix = a.editFix ; 

  nUlines = a.nUlines ;
  nVlines = a.nVlines ;

  return *this ;  
}

/*!
  \brief Copies another Nurbs Curve GL

  \param a  the Nurbs curve to copy

  \author Philippe Lavoie
  \date 6 November 1997
*/
NurbsCurveGL& NurbsCurveGL::operator=(const NurbsCurveGL &a){
  this->NurbsCurveSPf::operator=(a) ;
  this->NurbsGL::operator=(a) ;
  resetAll() ;
  return *this ;  
}

/*!
  \brief Copies another Nurbs Curve GL

  \param a  the Nurbs curve to copy

  \author Philippe Lavoie
  \date 6 November 1997
*/
NurbsCurveGL& NurbsCurveGL::operator=(const NurbsCurvef &a){
  this->NurbsCurveSPf::operator=(a) ;
  return *this ;
}

/*!
  \brief Copies another Nurbs Curve GL

  \param a  the Nurbs curve to copy

  \author Philippe Lavoie
  \date 6 November 1997
*/
NurbsSurfaceGL& NurbsSurfaceGL::operator=(const NurbsSurfaceGL &a){
  this->NurbsSurfaceSPf::operator=(a) ;
  this->NurbsGL::operator=(a) ;
  resetAll() ;
  return *this ;  
}

/*!
  \brief Copies another Nurbs Curve GL

  \param a  the Nurbs curve to copy

  \author Philippe Lavoie
  \date 6 November 1997
*/
NurbsSurfaceGL& NurbsSurfaceGL::operator=(const NurbsSurfacef &a){
  this->NurbsSurfaceSPf::operator=(a) ;
  return *this ;
}

/*!
  \brief Modifies a point on the curve

  \param u  the u parametric value
  \param v  the v parametric value
  \param dx  the delta value in the $x$-axis direction
  \param dy  the delta value in the $y$-axis direction
  \param dz  the delta value in the $z$-axis direction

  \author Philippe Lavoie
  \date 7 November 1997
*/
void NurbsCurveGL::modifyPoint(float u, float v, float dx, float dy, float dz){
  Point3Df delta(dx,dy,dz) ;
  if(u<U[0]) u = U[0] ;
  if(u>U[U.n()-1]) u = U[U.n()-1] ;
  movePoint(u,delta) ;
}

/*!
  \brief Modifies a point on the surface

  \param  u  the u parametric value
  \param v  the v parametric value
  \param dx  the delta value in the $x$-axis direction
  \param dy  the delta value in the $y$-axis direction
  \param dz  the delta value in the $z$-axis direction

  \author Philippe Lavoie
  \date 7 November 1997
*/
void NurbsSurfaceGL::modifyPoint(float u, float v, float dx, float dy, float dz){
  Point3Df delta(dx,dy,dz) ;
  boundTo(u,knotU()[0],knotU()[knotU().n()-1]) ;
  boundTo(v,knotV()[0],knotV()[knotV().n()-1]) ; 
  movePoint(u,v,delta) ;
}

/*!
  \brief Modifies a point on the surface

  \param  u  the u parametric value
  \param v  the v parametric value
  \param dx  the delta value in the $x$-axis direction
  \param dy  the delta value in the $y$-axis direction
  \param dz  the delta value in the $z$-axis direction

  \author Philippe Lavoie
  \date 7 November 1997
*/
void HNurbsSurfaceGL::modifyPoint(float u, float v, float dx, float dy, float dz){
  Point3Df delta(dx,dy,dz) ;
  boundTo(u,activePatch->knotU()[0],activePatch->knotU()[activePatch->knotU().n()-1]) ;
  boundTo(v,activePatch->knotV()[0],activePatch->knotV()[activePatch->knotV().n()-1]) ;
  activePatch->movePointOffset(u,v,delta) ;
  resetCPoints() ; 
}

/*!
  \brief apply the local transformation to the surface.

  Apply the local transformation to the surface. This is necessary
  if you want to get the proper position for the control points
  before doing anymore processing on them.

  \author Philippe Lavoie
  \date 7 November 1997
*/
void HNurbsSurfaceGL::applyTransform(){
  rx *= M_PI/180.0 ;
  ry *= M_PI/180.0 ;
  rz *= M_PI/180.0 ;

  MatrixRTf Tx(rx,ry,rz,tx,ty,tz) ;

  transform(Tx) ; // only need to transform the base class...
  scale(Point3Df(sx,sy,sz)) ; 
  // but scaling doesn't work then... sucks.
  
  tx = ty = tz = rx = ry = rz = 0 ;
  sx = sy = sz = 1 ;

  offset = P ; 
  ++updateN ;
  updateLevels(lod) ; 

  resetAll() ;
}

/*!
  \brief Default constructor

  \author Philippe Lavoie
  \date 7 November 1997
*/
HNurbsSurfaceGL::HNurbsSurfaceGL():HNurbsSurfaceSPf(),NurbsGL() {
  type = hSurfObject ; 
  polygon = new NurbsSpolygonGL(*this) ; 
  lod = maxLevel() ; 
  activePatch = this ; 
}

/*!
  \brief Constructor from a surface
  
  \param nS  a Nurbs Surface

  \author Philippe Lavoie
  \date 7 November 1997
*/
HNurbsSurfaceGL::HNurbsSurfaceGL(const NurbsSurfacef& nS):HNurbsSurfaceSPf(nS),NurbsGL() { 
  type = hSurfObject ; 
  polygon = new NurbsSpolygonGL(*this) ; 
  lod = maxLevel() ; 
  activePatch = this ; 
}

/*!
  \brief Copy constructor with patch information
  
  \param bS  the object to copy
  \param us  the start of the U parametric patch
  \param ue  the end of the U parametric patch
  \param vs  the start of the V parametric patch
  \param ue  the end of the V parametric patch

  \author Philippe Lavoie
  \date 7 November 1997
*/
HNurbsSurfaceGL::HNurbsSurfaceGL(const HNurbsSurfaceGL& bS):HNurbsSurfaceSPf(bS),NurbsGL() { 
  type = hSurfObject ;
  polygon = new NurbsSpolygonGL(*this) ; 
  lod = maxLevel() ; 
  activePatch = this ; 
}

/*!
  \brief Copy constructor
  
  \param bS  a pointer to the object to copy

  \author Philippe Lavoie
  \date 7 November 1997
*/
HNurbsSurfaceGL::HNurbsSurfaceGL(const HNurbsSurfaceGL* bS): HNurbsSurfaceSPf(*bS),NurbsGL() {  
  type = hSurfObject ; 
  polygon = new NurbsSpolygonGL(*this) ; 
  lod = maxLevel() ; 
  activePatch = this ; 
}

/*!
  \brief Adds a level to the Hierarchical surface.

  \author Philippe Lavoie
  \date 28 January 1998
*/
HNurbsSurfaceSPf* HNurbsSurfaceGL::addLevel() {  
  // go to the last level
  if(activePatch){
    activePatch = (HNurbsSurfaceGL*)activePatch->lastLevel() ;
    return activePatch->HNurbsSurfaceSPf::addLevel(1) ;
  }
  return 0 ;
}

/*!
  \brief Reset the control point information stored in cpoints

  \author Philippe Lavoie
  \date 29 January 1998
*/
void HNurbsSurfaceGL::resetPolygon() {
  int h ;
  h = 0 ; 
  if(polygon){
    h = polygon->getState() ;
    delete polygon ;
  }
  polygon = new NurbsSpolygonGL(*activePatch) ;
  if(h)
    viewCpolygon() ;
  else
    hideCpolygon() ;
}

/*!
  \brief Sets all object inside the list to view mode

  \author Philippe Lavoie
  \date 29 January 1998
*/
void ObjectListGL::viewAllObjects(){
  state = objectState ;
  ObjectGL* obj = first_ ;
  while(obj){
    obj->viewObject() ;
    obj = obj->next() ;
  }
}

/*!
  \brief Sets all object inside the list to hide mode

  \author Philippe Lavoie
  \date 29 January 1998
*/
void ObjectListGL::hideAllObjects(){
  state = hideState ;
  ObjectGL* obj = first_ ;
  while(obj){
    obj->hideObject() ;
    obj = obj->next() ;
  }
}

/*!
  \brief Modify a control point and his symmetrical point

  \param v  modify the points by this value

  \author Philippe Lavoie
  \date 29 January 1998
*/
void CPointGL::modifySym(const HPoint3Df &v) {
  modify(v) ;
  if(symPoint){
    HPoint3Df v2(v) ;
    v2.x() *= xCoord ;
    v2.y() *= yCoord ;
    v2.z() *= zCoord ;
    //v2.w() *= wCoord ;
    symPoint->modify(v2) ;
  }
}

/*!
  \brief Sets the symmetry for the control points

  \param true  1 if it should be in symmetrical mode

  \author Philippe Lavoie
  \date 29 January 1998
*/
void HNurbsSurfaceGL::setSym(int set, int uDir, float x, float y, float z, float w) {
  if(activePatch){
    int r = activePatch->ctrlPnts().rows() ;
    int c = activePatch->ctrlPnts().cols() ;
    if(true){
      if(uDir) { // do it so that (i,j) and (rows-i,j) are symmetrical
	ObjectGL *pnt = (ObjectGL*)cpoints.first() ;
	int i,j ;
	i = 0 ;
	j = 0 ;
	while(pnt){
	  ((CPointGL*)pnt)->setSym((CPointGL*)cpoints.goTo((r-1-i)*c+j),x,y,z,w) ;
	  ++j ;
	  if(j>=c){
	    j = 0 ;
	    ++i ;
	  }
	  pnt = pnt->next() ;
	}
      }
      else{  // (i,j) and (i,cols-j) are symmetrical
	ObjectGL *pnt = (ObjectGL*)cpoints.first() ;
	int i,j ;
	i = 0 ;
	j = 0 ;
	while(pnt){
	  ((CPointGL*)pnt)->setSym((CPointGL*)cpoints.goTo(i*c+(c-1-j)),x,y,z,w) ;
	  ++j ;
	  if(j>=c){
	    j = 0 ;
	    ++i ;
	  }
	  pnt = pnt->next() ;
	}
      }
    }
    else{
      ObjectGL *pnt = (ObjectGL*)cpoints.first() ;
      while(pnt){
	((CPointGL*)pnt)->setSym((CPointGL*)0,float(1),float(1),float(1),float(1)) ;
	pnt = pnt->next() ;
      }
    }
  }
}

/*!
  \brief Sets the symmetry for the control points

  \param true  1 if it should be in symmetrical mode

  \author Philippe Lavoie
  \date 2 July 1998
*/
void NurbsSurfaceGL::setSym(int set, int uDir, float x, float y, float z, float w) {
  if(set){
    const int r = ctrlPnts().rows() ;
    const int c = ctrlPnts().cols() ;
    if(uDir) { // do it so that (i,j) and (rows-i,j) are symmetrical
      ObjectGL *pnt = (ObjectGL*)cpoints.first() ;
      int i,j ;
      i = 0 ;
      j = 0 ;
      while(pnt){
	((CPointGL*)pnt)->setSym((CPointGL*)cpoints.goTo((r-1-i)*c+j),x,y,z,w) ;
	++j ;
	if(j>=c){
	  j = 0 ;
	  ++i ;
	}
	pnt = pnt->next() ;
      }
    }
    else{  // (i,j) and (i,cols-j) are symmetrical
      ObjectGL *pnt = (ObjectGL*)cpoints.first() ;
      int i,j ;
      i = 0 ;
      j = 0 ;
      while(pnt){
	((CPointGL*)pnt)->setSym((CPointGL*)cpoints.goTo(i*c+(c-1-j)),x,y,z,w) ;
	++j ;
	if(j>=c){
	  j = 0 ;
	  ++i ;
	}
	pnt = pnt->next() ;
      }
    }
  }
  else{
    ObjectGL *pnt = (ObjectGL*)cpoints.first() ;
    while(pnt){
      ((CPointGL*)pnt)->setSym((CPointGL*)0,float(1),float(1),float(1),float(1)) ;
      pnt = pnt->next() ;
    }
  }
}

/*!
  \brief attach an image to a surface

  Attach an image to a surface. The image must contain the
  red, green and  blue component in succesive GLubyte
  of data. The image data must be sent row wise.
  
  OpenGL only uses images which are \a 2^n pixels in width
  or height. If the image is not of the proper size, it
  will be padded to be acceptable by OpenGL. If you want
  to center the image, you have to provide an already
  centered image.
  
  The image data is reversed so that it shows upside up
  when mapped to a NURBS surface. The img data is copied
  into a new vector so it is safe to delete it once it
  has been passed to this routine.

  \param img  a pointer to the image data
  \param w  the width of the image
  \param h  the height of the image

  \author Philippe Lavoie
  \date 5 February 1998
*/
void NurbsSurfaceGL::setImage(GLubyte *img, GLint w, GLint h) { 
  if(image) { delete []image ; } 
  int hs = 2 ;
  int ws = 2 ;
  while(ws<w)
    ws *= 2 ;
  while(hs<h)
    hs *= 2 ;
  
  image = new GLubyte[hs*ws*3] ;

  imgW = ws ; 
  imgH = hs ;

  if(w<ws || h<hs){// must patch the data
    for(int i=0;i<hs;++i)
      for(int j=0;j<ws;++j){
	if(i<h && j<w){
	  image[(hs-1-i)*ws*3+j*3] =  img[i*w*3+j*3] ;
	  image[(hs-1-i)*ws*3+j*3+1] =  img[i*w*3+j*3+1] ;
	  image[(hs-1-i)*ws*3+j*3+2] =  img[i*w*3+j*3+2];
	}
	else{
	  image[(hs-1-i)*ws*3+j*3] = 0 ;
	  image[(hs-1-i)*ws*3+j*3+1] = 0  ;
	  image[(hs-1-i)*ws*3+j*3+2] = 0 ;
	}
      }
    
  }
  else{ // just reverse the data
    for(int i=0;i<hs;++i)
      for(int j=0;j<ws;++j){
	image[(h-1-i)*w*3+j*3] =  img[i*w*3+j*3] ;
	image[(h-1-i)*w*3+j*3+1] =  img[i*w*3+j*3+1];
	image[(h-1-i)*w*3+j*3+2] = img[i*w*3+j*3+2] ;
      }
  }

}

/*!
  \brief The constructor for a curve point object

  \param i  the index of the control point
  \param c  the pointer to the NURBS curve

  \author Philippe Lavoie
  \date 20 September 1997
*/
SPointCurveGL::SPointCurveGL(int i, NurbsCurveSPf *c,int fix): 
  SPointGL(spoint,i,-1,fix), curve(c)
{ 
  if(!c){
#ifdef USE_EXCEPTION
    throw NurbsInputError() ; 
#else
    Error error("SPointCurveGL constructor with a curve point") ;
    error << "You must provide a *valid* nurbs curve pointer." ;
    error.fatal() ; 
#endif
  }
  type = spointObject; 
  psize=pSizeDefault ; 
  i0 = i ; 
  cpoint = curve->surfP(i) ; 
  if(!curve->okMax()) { // it wasn't setup properly
    curve->updateMaxU() ; 
  }
  start = 0 ;
  rows = 0 ; 
} 

/*!
  \brief The constructor for a surface point object

  \param i  the row of the control point
  \param j  the column of the control point
  \param c  the pointer to the NURBS surface

  \author Philippe Lavoie
  \date 20 September 1997
*/
SPointSurfaceGL::SPointSurfaceGL(int i, int j, NurbsSurfaceSPf *s, ObjectListGL *sp,int fix): 
  SPointGL(spoint,i,j,fix), surface(s), spoints(sp)
{ 
  if(!s){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error error("SPointSurfaceGL constructor with a surface point") ;
    error << "You must provide a *valid* nurbs surface pointer." ;
    error.fatal() ; 
#endif
  }
  if(!spoints){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error error("SPointSurfaceGL constructor with a surface point") ;
    error << "You must provide a *valid* point list pointer." ;
    error.fatal() ; 
#endif
  }
  type = spointObject; 
  psize=pSizeDefault ; 
  i0=i ; 
  j0=j ; 
  if(!surface->okMax()) // it wasn't setup properly
    surface->updateMaxUV() ; 

  cpoint = surface->surfP(i,j) ; 
} 

/*!
  \brief The constructor for a surface point object

  \param i  the row of the control point
  \param j  the column of the control point
  \param c  the pointer to the NURBS surface

  \author Philippe Lavoie
  \date 20 September 1997
*/
SPointHSurfaceGL::SPointHSurfaceGL(int i, int j, HNurbsSurfaceSPf *s, ObjectListGL *sp,int fix): 
  SPointGL(spoint,i,j,fix), surface(s), spoints(sp)
{ 
  if(!s){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error error("SPointSurfaceGL constructor with a surface point") ;
    error << "You must provide a *valid* nurbs surface pointer." ;
    error.fatal() ; 
#endif
  }
  if(!spoints){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error error("SPointSurfaceGL constructor with a surface point") ;
    error << "You must provide a *valid* point list pointer." ;
    error.fatal() ; 
#endif
  }
  type = spointObject; 
  psize=pSizeDefault ; 
  i0=i ; 
  j0=j ; 
  if(!surface->okMax()) // it wasn't setup properly
    surface->updateMaxUV() ; 

  if(i>surface->ctrlPnts().rows())
    cerr << " BIG ERROR\n" ;
  if(j>surface->ctrlPnts().cols())
    cerr << " HUGE ERROR\n" ;

  cpoint = surface->surfP(i,j,surface->level()) ; 
} 

/*!
  \brief Updates the other control points 

  \author Philippe Lavoie
  \date 12 May 1998
*/
void SPointCurveGL::updateOthers(){
  SPointCurveGL *a ;
  a = start ;
  int n = 0 ;
  if(curve)
    while(a && n<rows){
      if(a!=this){
	a->spoint = a->curve->surfP(a->i0) ; 
      }
      a = (SPointCurveGL*) a->next() ;      
      ++n ; 
    }
}

/*!
  \brief Updates the other control points 

  \author Philippe Lavoie
  \date 12 May 1998
*/
void SPointSurfaceGL::updateOthers(){
  SPointSurfaceGL *a ;
  a = start ;
  int n = 0 ;
  int c = 0 ;
  if(surface){
    ObjectGL* saveCurrent = spoints->current() ;
    ObjectGL* rowStart ;
    spoints->current() = a ;
    rowStart = a ; 
    int mc = a->col() ;
    while(a && n<rows){
      if(a!=this){
	a->spoint = a->surface->surfP(a->i0,a->j0) ;
      }
      if(a->next()){
	if(((SPointSurfaceGL*)a->next())->col() > mc)
	  a = (SPointSurfaceGL*) a->next() ; 
	else
	  c = cols ;
      }
      ++c ;       
      if(c>=cols){
	++n ; 
	if(n==rows)
	  break ; 
	spoints->current() = (ObjectGL*)rowStart ; 
	a = (SPointSurfaceGL*) spoints->jumpToNext() ;
	rowStart = a ; 
	if(a->row() == 0)
	  break ; 
	c = 0 ;
      }
    }
    spoints->current() = saveCurrent ;
  }
}

/*!
  \brief Updates the other control points 

  \author Philippe Lavoie
  \date 12 May 1998
*/
void SPointHSurfaceGL::updateOthers(){
  SPointHSurfaceGL *a ;
  a = start ;
  int n = 0 ;
  int c = 0 ;
  if(surface){
    ObjectGL* saveCurrent = spoints->current() ;
    ObjectGL* rowStart ;
    spoints->current() = a ;
    rowStart = a ; 
    int mc = a->col() ;
    while(a && n<rows){
      if(a!=this){
	a->spoint = a->surface->surfP(a->i0,a->j0,a->surface->level()) ;
      }
      if(a->next()){
	if(((SPointSurfaceGL*)a->next())->col() > mc)
	  a = (SPointHSurfaceGL*) a->next() ; 
	else
	  c = cols ;
      }
      ++c ;       
      if(c>=cols){
	++n ; 
	if(n==rows)
	  break ; 
	spoints->current() = (ObjectGL*)rowStart ; 
	a = (SPointHSurfaceGL*) spoints->jumpToNext() ;
	rowStart = a ; 
	if(a->row() == 0)
	  break ; 
	c = 0 ;
      }
    }
    spoints->current() = saveCurrent ;
  }
}

/*!
  \brief Modifies the surface point

  \param v  modifies the point by this value

  \author Philippe Lavoie
  \date 12 May 1998
*/
void SPointCurveGL::modify(const HPoint3Df& v)  
{ 
  if(curve){
    if(editFix)
      curve->modOnlySurfCPby(i0,v) ; 
    else{
      curve->modSurfCPby(i0,v) ; 
      updateOthers(); 
    }
    spoint += v ; 
  }
}

/*!
  \brief Modifies the surface point
  \param v  modifies the point by this value

  \author Philippe Lavoie
  \date 12 May 1998
*/
void SPointSurfaceGL::modify(const HPoint3Df& v)  
{ 
  if(surface) {
    if(editFix)
      surface->modOnlySurfCPby(i0,j0,v) ; 
    else{
      surface->modSurfCPby(i0,j0,v) ; 
      updateOthers();
    }
    spoint += v ; 
  } 
}

/*!
  \brief Modifies the surface point
  \param v  modifies the point by this value
  \return 
  \warning 

  \author Philippe Lavoie
  \date 12 May 1998
*/
void SPointHSurfaceGL::modify(const HPoint3Df& v)  
{ 
  if(surface) {
    if(editFix)
      surface->modOnlySurfCPby(i0,j0,v) ; 
    else{
      surface->modSurfCPby(i0,j0,v) ;
      updateOthers();
    }
    spoint = surface->surfP(i0,j0,surface->level()) ; 
  } 
}

/*!
  \brief Reads the information from a stream

  \param fin  the input stream

  \return 1 on sucess, 0 on failure

  \author Philippe Lavoie
  \date 19 June 1998
*/
int NurbsCurveGL::read(ifstream &fin) { 
  static int ncurves = 0 ;
  if(NurbsCurvef::read(fin)){ 
    if(!ObjectGL::read(fin)){
      char *maxName = new char[25] ; 
      sprintf(maxName,"curve_%d",ncurves++) ; 
      setName(maxName) ; 
      delete []maxName ; 
    }
    return 1 ;
  }
  return 0 ; 
}

/*!
  \brief Reads the information from a stream

  \param fin  the input stream

  \return 1 on sucess, 0 on failure

  \author Philippe Lavoie
  \date 19 June 1998
*/
int NurbsSurfaceGL::read(ifstream &fin) { 
  static int nsurfaces = 0 ;
  if(NurbsSurfacef::read(fin)){ 
    if(!ObjectGL::read(fin)){
      char *maxName = new char[25] ; 
      sprintf(maxName,"surface_%d",nsurfaces++) ; 
      setName(maxName) ; 
      delete []maxName ; 
    }
    return 1 ;
  }
  return 0 ; 
}

/*!
  \brief Reads the information from a stream

  \param fin  the input stream

  \return 1 on sucess, 0 on failure

  \author Philippe Lavoie
  \date 19 June 1998
*/
int HNurbsSurfaceGL::read(ifstream &fin) { 
  static int nhsurfaces = 0 ;
  if(HNurbsSurfacef::read(fin)){ 
    if(!ObjectGL::read(fin)){
      char *maxName = new char[25] ; 
      sprintf(maxName,"hsurface_%d",nhsurfaces++) ; 
      setName(maxName) ; 
      delete []maxName ; 
    }
    return 1 ;
  }
  return 0 ; 
}

/*!
  \brief Writes a NurbsCurveGL to an output stream.

  \param fout  the output stream

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 19 June 1998
*/
int NurbsCurveGL::write(ofstream &fout) const {
  if(!NurbsCurvef::write(fout))
    return 0 ; 
  if(!ObjectGL::write(fout))
    return 0 ; 
  return 1;
}

/*!
  \brief Writes a NurbsCurveGL to an output stream.

  \param fout  the output stream

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 19 June 1998
*/
int HNurbsSurfaceGL::write(ofstream &fout) const {
  if(!HNurbsSurfacef::write(fout))
    return 0 ; 
  if(!ObjectGL::write(fout))
    return 0 ; 
  return 1;
}

/*!
  \brief Writes a NurbsCurveGL to an output stream.

  \param fout  the output stream

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 19 June 1998
*/
int NurbsSurfaceGL::write(ofstream &fout) const {
  if(!NurbsSurfacef::write(fout))
    return 0 ; 
  if(!ObjectGL::write(fout))
    return 0 ; 
  return 1;
}


/*!
  \brief Reads the information from a stream

  \param fin  the input stream

  \return 1 on sucess, 0 on failure

  \author Philippe Lavoie
  \date 19 June 1998
*/
int ObjectGL::read(ifstream &fin) { 
  // We might have some name and global transform information 
  // attached to this object
  int mark = fin.tellg() ; // marker position
  char *type = new char[5] ; 
  if(!fin.read(type,sizeof(char)*5)){ delete []type ; return 0 ; }
  
  char *maxName ;
  maxName = new char[256] ; // more then enough
  char *pat ;
  pat = strstr(type,"object") ;
  if(pat) {
    // setup the name and global transform information
    fin >> maxName ; 
    setName(maxName) ;
    fin >> tx >> ty >> tz >> rx >> ry >> rz >> sx >> sy >> sz ; 
  }
  else{
    // restore to the marker position
    fin.seekg(mark) ; 
    delete []type ; 
    delete []maxName ; 
    return 0 ; 
  }
  delete []type ; 
  delete []maxName ; 
  return 1 ; 
}

/*!
  \brief Writes a ObjectGL to a stream

  \param fout  the output stream

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 19 June 1998
*/
int ObjectGL::write(ofstream &fout) const {
  fout << "object" << name_ << tx << ty << tz << rx << ry << rz << sx << sy << sz ;
  if(!fout)
    return 0 ; 
  return 1 ; 
}

/*!
  \brief Reads the information from a stream

  \param filename  the input file
  \return 1 on sucess, 0 on failure

  \author Philippe Lavoie
  \date 19 June 1998
*/
int ObjectGL::read(const char* filename) {
  ifstream fin(filename) ; 
  if(!fin) 
    return 0 ; 
  return read(fin) ; 
}

/*!
  \brief Writes a ObjectGL to a file

  \param filename  the filename to write to.

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 19 June 1998
*/
int ObjectGL::write(const char* filename) const {
  ofstream fout(filename) ; 
  if(!fout) 
    return 0 ; 
  return write(fout) ; 
}


template <class T>
class RenderMeshGL : public RenderMesh<T> {
public:
  RenderMeshGL(const Color& c): color(c) {;}
  virtual ~RenderMeshGL() { }
  virtual void drawHeader() ;
  virtual void drawTriangle( const SurfSample<T> &v0, const SurfSample<T> &v1, const SurfSample<T> & v2 );
  virtual void drawFooter() ;
  virtual void screenProject(const HPoint_nD<T,3> &worldPt, Point_nD<T,3> &screenPt ) ; 
protected:
  Color color ; 
};

template <class T>
void RenderMeshGL<T>::screenProject(const HPoint_nD<T,3> &worldPt, Point_nD<T,3> &screenPt )
{
  screenPt = project(worldPt) ;
}

template <class T>
  void RenderMeshGL<T>::drawHeader()
{
  //glBegin(GL_TRIANGLES);
  glBegin(GL_LINE_LOOP);
  glColor(color);
}
 
template <class T>
void RenderMeshGL<T>::drawFooter(){
  glEnd();
}


template <class T>
  void RenderMeshGL<T>::drawTriangle(const SurfSample<T> &v0, const SurfSample<T> &v1, const SurfSample<T> & v2 )
{
  glVertex3f(v0.point.x(),v0.point.y(),v0.point.z());
  glVertex3f(v1.point.x(),v1.point.y(),v1.point.z());
  glVertex3f(v2.point.x(),v2.point.y(),v2.point.z());
}


void NurbsSubSurfaceGL::drawSubdivisionGL(float tolerance)
{
  if(render)
    delete render ;
  render = new RenderMeshGL<float>(color) ;
  drawSubdivision(tolerance) ;
}



#ifdef NO_IMPLICIT_TEMPLATES

template class std::list<NurbsCurve_2Df*> ;
template class std::list<NurbsCurve_2Df*>::iterator ;

template class RenderMeshGL<float> ;

template class std::allocator<PLib::NurbsCurve<float, 2> *>;

template void  std::_List_base<PLib::NurbsCurve<float, 2> *, std::allocator<PLib::NurbsCurve<float, 2> *> >::clear(void);

template void std::list<NurbsCurve_2Df*>::clear(void);

template void std::list<PLib::NurbsCurve<float, 2> *, std::allocator<PLib::NurbsCurve<float, 2> *> >::_M_insert_dispatch<std::_List_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float, 2> *const &, PLib::NurbsCurve<float, 2> *const *> >(std::_List_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float, 2> *&, PLib::NurbsCurve<float, 2> **>, std::_List_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float,
2> *const &, PLib::NurbsCurve<float, 2> *const *>, std::_List_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float, 2> *const &, PLib::NurbsCurve<float, 2> *const *>, __false_type);

/*
template void list<PLib::NurbsCurve<float, 2> *, __default_alloc_template<0, 0> >::insert<__list_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float, 2> *const &, PLib::NurbsCurve<float, 2> *const *> >(__list_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float, 2> *&, PLib::NurbsCurve<float, 2> **>, __list_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float, 2> *const &, PLib::NurbsCurve<float, 2> *const *>, __list_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float, 2> *const &, PLib::NurbsCurve<float, 2> *const *>) ;
template void list<PLib::NurbsCurve<float, 2> *, __default_alloc_template<1, 0> >::insert<__list_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float, 2> *const &, PLib::NurbsCurve<float, 2> *const *> >(__list_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float, 2> *&, PLib::NurbsCurve<float, 2> **>, __list_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float, 2> *const &, PLib::NurbsCurve<float, 2> *const *>, __list_iterator<PLib::NurbsCurve<float, 2> *, PLib::NurbsCurve<float, 2> *const &, PLib::NurbsCurve<float, 2> *const *>) ;


#if defined(USING_GNU_SOLARIS) 
  template void* __default_alloc_template<true, 0>::refill(unsigned int) ;
  template void* __default_alloc_template<true, 0>::free_list ; 
  template char* __default_alloc_template<true, 0>::chunk_alloc(unsigned int, int &);
  template void* __default_alloc_template<true, 0>::heap_size ;
  template void* __default_alloc_template<true, 0>::end_free ;
  template void* __default_alloc_template<true, 0>::start_free ;
#endif 

#ifdef USING_LINUX
  template void* __default_alloc_template<false, 0>::free_list ;
  template void* __default_alloc_template<false, 0>::refill (unsigned int) ;
  template char* __default_alloc_template<false, 0>::chunk_alloc(unsigned int, int &);
  template void* __default_alloc_template<false, 0>::end_free;
  template void* __default_alloc_template<false, 0>::heap_size;
  template void* __default_alloc_template<false, 0>::start_free ;
#endif

*/

#endif

} // end namespace

#endif


