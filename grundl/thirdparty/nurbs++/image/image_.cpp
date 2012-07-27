/*=============================================================================
        File: image.cpp
     Purpose:
    Revision: $Id: image_.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (18 February 1999)
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

#include "image.cpp"

namespace PLib {

#if defined(WITH_IMAGE_MAGICK)

  void IM_ImageT<Color>::setMatrix(){
#ifdef COLUMN_ORDER
    for(int i=0;i<rows();++i)
      for(int j=0;j<cols();++j){
	vm[j][i].r = image->pixels[i*cols()+j].red ;
	vm[j][i].g = image->pixels[i*cols()+j].green ;
	vm[j][i].b = image->pixels[i*cols()+j].blue ;
      }
#else
    int size=rows()*cols() ;
    for(int i=0;i<size;++i){
      m[i].r = image->pixels[i].red ;
      m[i].g = image->pixels[i].green ;
      m[i].b = image->pixels[i].blue;
    }
#endif
  }
  
  void IM_ImageT<unsigned char>::setMatrix(){
#ifdef COLUMN_ORDER
    for(int i=0;i<rows();++i)
      for(int j=0;j<cols();++j)
	vm[j][i] = image->pixels[i*cols()+j].red ;
#else
    int size=rows()*cols() ;
    for(int i=0;i<size;++i){
      m[i] = image->pixels[i].red ;
    }
#endif
  }

 
  void IM_ImageT<Color>::setImage(){
    if(image->rows != (unsigned int)rows() || image->columns != (unsigned int)cols()){
      image->rows = rows() ;
      image->columns = cols() ;
      image->packets = rows()*cols() ;
      if(image->pixels)
	delete []image->pixels ;
      image->pixels = (RunlengthPacket*)malloc(sizeof(RunlengthPacket)*image->packets) ;
    }
    
#ifdef COLUMN_ORDER
    for(int i=0;i<rows();++i)
      for(int j=0;j<cols();++j){
	image->pixels[i*cols()+j].red = vm[j][i].r ;
	image->pixels[i*cols()+j].green = vm[j][i].g ; 
	image->pixels[i*cols()+j].blue =  vm[j][i].b ; 
	image->pixels[i*cols()+j].index =  0;
	image->pixels[i*cols()+j].length = 0 ;
      }
#else
    int size=rows()*cols() ;
    for(int i=0;i<size;++i){
      image->pixels[i].red = m[i].r ;
      image->pixels[i].green = m[i].g ; 
      image->pixels[i].blue =  m[i].b ; 
      image->pixels[i].index =  0;
      image->pixels[i].length = 0 ;
    }
#endif
  }
  
  void IM_ImageT<unsigned char>::setImage(){
    if(image->rows != (unsigned int)rows() || image->columns != (unsigned int)cols()){
      image->rows = rows() ;
      image->columns = cols() ;
      image->packets = rows()*cols() ;
      if(image->pixels)
	delete []image->pixels ;
      image->pixels = (RunlengthPacket*)malloc(sizeof(RunlengthPacket)*image->packets) ;
    }
    
#ifdef COLUMN_ORDER
    for(int i=0;i<rows();++i)
      for(int j=0;j<cols();++j){
	image->pixels[i*cols()+j].red = vm[j][i] ;
	image->pixels[i*cols()+j].green = vm[j][i] ; 
	image->pixels[i*cols()+j].blue =  vm[j][i] ; 
	image->pixels[i*cols()+j].index =  vm[j][i] ;
	image->pixels[i*cols()+j].length = 0 ;
      }
#else
    int size=rows()*cols() ;
    for(int i=0;i<size;++i){
      image->pixels[i].red = m[i] ;
      image->pixels[i].green = m[i] ;
      image->pixels[i].blue = m[i] ;
      image->pixels[i].index = m[i] ; 
      image->pixels[i].length = 0 ;
    }
#endif
  }
  
  
  
  int IM_ImageT<Color>::read(const char* filename) {
    (void)strcpy(image_info.filename,filename) ;
    if(image)
      DestroyImage(image) ;
    image=ReadImage(&image_info) ;
    if(!image)
      return 0 ;
    UncondenseImage(image) ;
    resize(image->rows,image->columns) ;
    
    setMatrix() ;
    
    return 1 ;
  }
  
  int IM_ImageT<unsigned char>::read(const char* filename) {
    (void)strcpy(image_info.filename,filename) ;
    image_info.monochrome = 1 ;
    if(image)
      DestroyImage(image) ;
    image=ReadImage(&image_info) ;
    if(!image)
      return 0 ;
    UncondenseImage(image) ;
    resize(image->rows,image->columns) ;
    
    setMatrix() ;
    
    return 1 ;
  }
  
  int IM_ImageT<Color>::write(const char* filename) {
    
    if(image)
      DestroyImage(image) ;
    
    image=AllocateImage(&image_info) ;
    
    if(!image)
      return 0 ;
    
    (void)strcpy(image->filename,filename) ;
    
    image->rows = rows() ;
    image->columns = cols() ;
    image->packets=rows()*cols() ;
    image->pixels = (RunlengthPacket*)malloc(sizeof(RunlengthPacket)*image->packets) ;
    
    setImage() ;
    
    return WriteImage(&image_info,image) ;
  }
  
  int IM_ImageT<unsigned char>::write(const char* filename) {
    if(image)
      DestroyImage(image) ;
    
    image=AllocateImage(&image_info) ;
    
    if(!image)
      return 0 ;
    
    (void)strcpy(image->filename,filename) ;
    
    image->rows = rows() ;
    image->columns = cols() ;
    image->packets=rows()*cols() ;
    image->pixels = (RunlengthPacket*)malloc(sizeof(RunlengthPacket)*image->packets) ;
    
    setImage() ;
    
    image_info.monochrome = 1 ;  
    return WriteImage(&image_info,image) ;
  }


#ifdef NO_IMPLICIT_TEMPLATES
  template class IM_ImageT<unsigned char> ;
  template class IM_ImageT<Color> ;
#endif


#endif


#ifdef NO_IMPLICIT_TEMPLATES
  template class MatrixImage<int> ;
  template class MatrixImage<float> ;
  template class MatrixImage<double> ;
  template class MatrixImage<char> ;
  template class MatrixImage<unsigned char> ;
  template class MatrixImage<Color> ;
#endif
  
}
