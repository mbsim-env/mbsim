/*=============================================================================
        File: list.h
     Purpose:      
    Revision: $Id: list.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (3 Oct, 1996)
 Modified by: 

 Copyright notice:
          Copyright (C) 1996-1997 Philippe Lavoie
 
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

#ifndef _Matrix_list_h_
#define _Matrix_list_h_

#include "specialType.h"

/**  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **
        class BasicNode --- A basic node for a linked list 
   A basic node for a linked list 
   author Philippe Lavoie (27 October 1997)
  Modified by:
 **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **/
template<class T> 
struct BasicNode{
public:
  BasicNode() { prev = next = 0 ; data = 0 ;}
  BasicNode(T *a) : data(a) { prev = next = 0; }
  ~BasicNode() { if(data) delete data ; }
  
  T* data ;
  BasicNode<T> *prev, *next ;
};

/**  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **
        class BasicList --- A basic linked list class
   A basic linked list class.
   author Philippe Lavoie (27 October 1997)
  Modified by:
 **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **/
template<class T> 
class BasicList: public BasicNode<T>
{
public:
  BasicList() ;
  BasicList(BasicList<T>& a) ;
  ~BasicList() { reset() ; }


  BasicNode<T>* first() { return first_ ; }
  BasicNode<T>* last() { return last_ ; }

  BasicNode<T> *current ;

  void reset() ;
  void add(BasicNode<T>* obj)  ;
  void add(const T& data) ;
  void addElements(BasicList<T>& list) ;
  BasicNode<T>* remove(BasicNode<T>* obj) ;
  void erase(BasicNode<T>* obj) ;

  BasicList<T>& operator=(const BasicList<T>& a) ;
  
  BasicNode<T>* goToFirst() {  return (current = first_) ; }
  BasicNode<T>* goToNext() { if(current) return (current = current->next) ; return 0 ;}
  BasicNode<T>* goToPrevious() { if(current) return (current = current->prev) ; return 0 ;}

  int size() const { return n ; }
  

  BasicNode<T>* operator[](int i) ;

  // Reset Mode
  enum ListResetMode { delete_at_reset, keep_at_reset } ;

  int resetMode() const { return reset_mode ; }
  void setResetMode(ListResetMode a ) { reset_mode = a ; }

protected:
  BasicNode<T> *first_,*last_ ;
  int n ;
  int nc ;
  ListResetMode reset_mode ;
};


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       Member: BasicList --- basic list constructor
   Basic list constructor.
        Input: 
       Output:
 Restrictions: 
   author Philippe Lavoie (28 October 1997)
  Modified by:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
template <class T>
BasicList<T>::BasicList():  BasicNode<T>() {
  first_ = last_ = 0 ;
  current = first_ ;
  reset_mode = delete_at_reset ;
  n = 0 ;
  nc = 0 ;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       Member: BasicList --- copy constructor
   The copy constructor.
        Input: a --> the list to copy
       Output:
 Restrictions: 
   author Philippe Lavoie (28 October 1997)
  Modified by:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
template <class T>
BasicList<T>::BasicList(BasicList<T>& a): BasicNode<T>() {
  first_ = last_ = 0 ;
  current = first_ ;
  *this = a ; 
  nc = 0 ;
  n = 0 ;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       Member: add --- adds an object to the list
   Adds an object to the list
        Input: obj --> the object to add to the list
       Output:
 Restrictions: 
   author Philippe Lavoie (28 October 1997)
  Modified by:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
template <class T>
void BasicList<T>::add(BasicNode<T>* obj){
  if(obj){
    if(!first_){
      first_ = obj ;
    }
    else{
      last_->next = obj ;
      obj->prev = last_ ;
    }
    last_ = obj ;
    obj->next = 0 ;
    ++n ;
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       Member: add --- adds an object to the list
   Adds an object to the list
        Input: obj --> the object to add to the list
       Output:
 Restrictions: 
   author Philippe Lavoie (28 October 1997)
  Modified by:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
template <class T>
void BasicList<T>::add(const T& data){
  T *p = new T(data) ;
  BasicNode<T> *node = new BasicNode<T>(p) ;
  add(node) ;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       Member: remove --- finds an element and remove it from the list
   Finds an element and delete it from the list. The element
               will {\em not} be deleted. This is up to the calling function.
        Input: obj --> the element to search
       Output: a pointer to obj if it was found in the list, 0 otherwise
 Restrictions: 
   author Philippe Lavoie (28 October 1997)
  Modified by:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
template <class T>
BasicNode<T>* BasicList<T>::remove(BasicNode<T>* obj){
  BasicNode<T>* t ;
  
  if(!obj)
    return 0 ;

  if(current == obj){
    t = obj ;
    current = 0 ;
    if(t->prev){
      t->prev->next = t->next ;
      current = t->prev ;
    }
    if(t->next){
      t->next->prev = t->prev ;
      current = t->next ;
    }
    --n ;
    --nc ;
    if(first_==t)
      first_ = t->next ;
    if(last_==t)
      last_ = t->prev ;
    return t;
  }

  t = first_ ;
  while(t){
    if(t==obj){
      if(t->prev)
	t->prev->next = t->next ;
      if(t->next)
	t->next->prev = t->prev ;
      --n ;
      if(first_==t)
	first_ = t->next ;
      if(last_==t)
	last_ = t->prev ;
      return t;
    }
    else
      t = t->next ;
  }
  return 0 ;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       Member: reset --- deletes all the node of the list
   Deletes all the nodes of the list. If the reset mode is
               set to delete the elements, all the elements will be 
	       deleted. Otherwise, the elements are removed but not
	       deleted.
        Input: 
       Output: 
 Restrictions: 
   author Philippe Lavoie (28 October 1997)
  Modified by:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
template <class T>
void BasicList<T>::reset(){
  if(reset_mode==delete_at_reset){
    BasicNode<T> *c ;
    c = first_ ;
    while(c){
      current = c ;
      c = current->next ;
      delete current ;
    }
  }
  else{
    BasicNode<T> *c ;
    c = first_ ;
    while(c){
      current = c ;
      c = current->next ;
      current->next = current->prev = 0  ;
    }
  }
  first_ = current = last_ = 0 ;
  n = 0 ;
  nc = 0 ;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       Member: operator= --- copies a list
   Copies a list
        Input: a --> the list to copy
       Output: a reference to itself
 Restrictions: 
   author Philippe Lavoie (28 October 1997)
  Modified by:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
template <class T>
BasicList<T>& BasicList<T>::operator=(const BasicList<T> &a){
  BasicNode<T> *t,*t2 ;
  T* c; 

  reset() ;

  t = a.first_ ;
  while(t){
    c = new T(*t->data) ;
    t2 = new BasicNode<T>(c) ;
    add(t2) ;

    if(a.current == t){
      current = t2 ;
      nc = a.nc ;
    }

    t = t->next ;
  }
  
  if(!current){
    current = first_ ;
    nc = 0 ;
  }

  reset_mode = a.reset_mode ;

  return *this ;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       Member: addElements --- adds the elements from a list
   Copies a list
        Input: a --> the list to copy
       Output: a reference to itself
 Restrictions: 
   author Philippe Lavoie (28 October 1997)
  Modified by:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
template <class T>
void BasicList<T>::addElements(BasicList<T> &list){
  BasicNode<T> *t,*t2 ;
  T* c; 

  t = list.first_ ;
  while(t){
    c = new T(*t->data) ;
    t2 = new BasicNode<T>(c) ;
    add(t2) ;
    t = t->next ;
  }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       Member: erase --- removes and deletes the element from the list
   Removes and deletes the element from the list. If the object
               is not in the list, nothing happens.
        Input: obj <-> the object to delete
       Output: 
 Restrictions: 
   author Philippe Lavoie (28 October 1997)
  Modified by:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
template <class T>
void BasicList<T>::erase(BasicNode<T> *obj){
  BasicNode<T> *o ;
  o = remove(obj) ;
  if(o)
    delete o ;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       Member: operator[] --- returns the nth element from the list
   Returns the nth element from the list or 0 if the value
               is out of bound.
        Input: i <-- the element to return
       Output: A pointer to the nth element or 0 if n is out of bound
 Restrictions: 
   author Philippe Lavoie (28 October 1997)
  Modified by:
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
template <class T>
BasicNode<T>* BasicList<T>::operator[](int i){
  if(i==nc)
    return current ;
  if(i<0 || i>=n) return 0 ;
  if(i<nc)
    while(nc!=i){
      goToPrevious() ;
      --nc ;
    }
  else
    while(nc!=i){
      goToNext() ;
      ++nc ;
    }
  return current ;
}

#endif
