/*=============================================================================
        File: image.cpp
     Purpose:
    Revision: $Id: image.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by:    Philippe Lavoie          (3 Oct, 1996)
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

#include "image.h"
#include <stdio.h>

/*!
 */
namespace PLib {

/*!
  \brief draws a line from point 1 to point 2

  \latexonly
   Draws a line without aliasing from point $(i_1,j_1)$ to 
   $(i_2,j_2)$ with the value {\tt color}.
  \endlatexonly
  \htmlonly
   Draws a line without aliasing from point \a (i1,j1) to 
   \a (i2,j2) with the value \a color.  
  \endhtmlonly

  \param i1  the row of point 1
  \param j1  the column of point 1
  \param i2  the row of point 2
  \param j2  the column of point 2
  \param color  the line is drawn with this value

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void MatrixImage<T>::drawLine(int i1, int j1, int i2, int j2, T color){
  int i,j ;
  double mx,b ;
  if(i1<0 || j1<0 || i1>rows() || j1>=cols()  ){
#ifdef USE_EXCEPTION
    throw OutOfBound2D(i1,j1,0,rows()-1,0,cols()-1) ;
#else
    Error error("MatrixImage<T>::drawLine") ;
    error << "Error in drawing line\n Invalid index ("<< i1 << ", " << j1 << ") to ( " << i2 << ", " << j2 << ") \n" ;
    error.warning() ;
#endif
    return ;
  }
  if(i2 <0 || j2<0 || i2>rows() || j2>=cols() ){
#ifdef USE_EXCEPTION
    throw OutOfBound2D(i2,j2,0,rows()-1,0,cols()-1) ;
#else
    Error error("MatrixImage<T>::drawLine") ;
    error << "Error in drawing line\n Invalid index ("<< i1 << ", " << j1 << ") to ( " << i2 << ", " << j2 << ") \n" ;
    error.warning() ;
#endif
    return ;
  }

  // check if line is vertical
  if(j1==j2){
    for(i=minimum(i1,i2);i<=maximum(i1,i2);i++)
     operator()(i,j1) = color ;
    return ;
  }
  mx = (double)(i1-i2)/(double)(j1-j2) ;
  b = (double)i1 - mx*j1 ;
  if(absolute(i1-i2)>absolute(j1-j2)){ // draw vertically
    if(i1>i2){
      for(i=i1;i>=i2;i--){
	j = int(((double)i-b)/mx) ;
	operator()(i,j) = color ;
      }
    }
    else{
      for(i=i1;i<=i2;i++){
	j = (int)((i-b)/mx) ;
	operator()(i,j) = color ;
      }
    }
  }
  else{
    if(j1>j2){
      for(j=j1;j>=j2;j--){
	i = (int)(mx*j+b) ;
	operator()(i,j) = color ;
      }
    }
    else{
      for(j=j1;j<=j2;j++){
	i = (int)(mx*j+b) ;
	operator()(i,j) = color ;
      }
    }
  }

}

/*!
  \brief draws a point of radius \a r

  Draws a point of radius \a r at location \a (i,j) with the 
  value \a color.

  \param i  the row of the center point to draw
  \param j  the column of the center point to draw
  \param r  the radius of the point
  \param color  the value to draw the point with

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void MatrixImage<T>::drawPoint(int i, int j, double r , T color){
  for(int y=i-int(ceil(r)) ; y<i+int(ceil(r)) ; y++)
    for(int x = j-int(ceil(r)) ; x<j+int(ceil(r)) ; x++){
      if(y>=0 && y<rows() && x>=0 && x<cols()){
	if( ((y-i)*(y-i)+(x-j)*(x-j))<= r*r)
	  operator()(y,x) = color ;
      }
    }
}


/*!
  \brief copies an image to a Matrix

  copies the image to a matrix of the same type.

  \param a  the matrix to store the image to

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void MatrixImage<T>::store(Matrix<T>& a){
  if(a.rows() != rows() || a.cols() != cols()) {
    a.resize(rows(),cols()) ;
  }
  T *aptr, *bptr ;
  int size,i ;
  aptr = &a(0,0)-1 ;
  bptr = m-1 ;
  size = cols()*rows() ;
  for(i=0;i<size;i++)
    *(++aptr) = *(++bptr) ;  
}

#ifdef WITH_IMAGE_MAGICK

/*!
  \brief default constructor

  \warning The IM_ImageT class must be of type \a Color or type 
               \a unsigned \a char.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
IM_ImageT<T>::IM_ImageT(): MatrixImage<T>(){
  autoSave = 0 ;
  file_name = 0 ;
  image = 0 ;
  GetImageInfo(&image_info) ;
}

/*!
  \brief constructor with a filename and a save flag

  The image in \a filename will be read. If the save flag is 
  set to 1, then when the destructor is called the image is 
  saved to \a filename.

  If en error occurs, it is not reported. The use of the read 
  and write member functions is recommended instead of this 
  constructor.

  \param filename  the file to read an image from
  \param save  a flag indicating if autosave should occur, defaults to 0

  \warning The IM_ImageT class must be of type \a Color or type 
               unsigned char. The image stored in \a filename must 
	       be readable from the Image Magick library.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
IM_ImageT<T>::IM_ImageT(const char* filename, int save): MatrixImage<T>(){
  autoSave = save ;
  file_name = new char[1024] ; // 1024 characters for the name of a file, should be enough
  (void)strcpy(file_name,filename) ;
  GetImageInfo(&image_info) ;
  read(filename) ;
}

/*!
  \brief constructor specifying the size of the image

  \param r  the number of rows() for the image
  \param c  the number of columns for the image

  \warning The IM_ImageT class must be of type \a Color or type 
           unsigned char.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
IM_ImageT<T>::IM_ImageT(const int r, const int c): MatrixImage<T>(r,c){
  autoSave = 0 ;
  file_name = 0 ;
  image = 0 ;
  GetImageInfo(&image_info) ;
}

/*!
  \brief reads from a file an image

  Reads an image file. The filename should be in the format 
  ``imagename.type'' or ``type:imagename''.

  \param filename  the name of the file to read

  \warning The IM_ImageT class must be of type \a Color or type 
           unsigned char. The image stored in \a filename must 
	   be readable from the Image Magick library.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
int IM_ImageT<T>::read(const char* filename) {
#ifdef USE_EXCEPTION
  throw MatrixErr() ;
#else
  Error error("IM_ImageT<T>::read") ;
  error << "An image of this type is NOT supported!\n" ;
  error.fatal() ;
#endif
  return 0;
}

/*!
  \brief writes to a file an image

  Writes to a file an image. The filename should be in the 
  format "imagename.type" or "type:imagename".

  \param filename  the file to write to. 

  \warning The \verb.IM_ImageT. class must be of type \a Color or type 
           unsigned char. \a filename must specify a valid type 
	   for the Image Magick library.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
int IM_ImageT<T>::write(const char* filename) {
#ifdef USE_EXCEPTION
  throw MatrixErr() ;
#else
  Error error("IM_ImageT<T>::write") ;
  error << "An image of this type is NOT supported!\n" ;
  error.fatal() ;
#endif
  return 0 ;
}

/*!
  \brief sets the matrix internal data from the image data

  The matrix data is set to the one from the image data.
  This step is necessary since the ImageMagick data format 
  can't be made compatible with the matrix data format. The data 
  between the two internal formats must be copied each time 
  because of this.

  \param filename  the file to write to. 

  \warning The image data must be initialized properly.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void IM_ImageT<T>::setMatrix(){
#ifdef USE_EXCEPTION
  throw MatrixErr() ;
#else
  Error error("IM_ImageT<T>::setImage") ;
  error << "An image of this type is NOT supported!\n" ;
  error.fatal() ;
#endif
}


/*!
  \brief sets the image data from the matrix data.

  Sets the image data from the matrix data. This step is necessary
  since the ImageMagick data format can't be made compatible with
  the matrix data format. The data between the two internal 
  formats must be copied each time because of this.

  \param filename  the file to write to. 

  \warning image must be initialized.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void IM_ImageT<T>::setImage(){
#ifdef USE_EXCEPTION
  throw MatrixErr() ;
#else
  Error error("IM_ImageT<T>::setImage") ;
  error << "An image of this type is NOT supported!\n" ;
  error.fatal() ;
#endif
}

/*!
  \brief destructor

  If the autosave flag has been set, it saves the image to a file.

  \warning

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
IM_ImageT<T>::~IM_ImageT(){
  if(autoSave && file_name)
    write(file_name) ;
  if(image)
    DestroyImage(image) ;

  DestroyImageInfo(&image_info) ;
  if(file_name){
    delete []file_name ;
  }
}

#endif


} // end namespace
