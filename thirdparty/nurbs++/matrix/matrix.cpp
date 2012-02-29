/*=============================================================================
        File: matrix.cpp
     Purpose:
    Revision: $Id: matrix.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by:    Philippe Lavoie          (3 Oct, 1996)
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

#ifndef MATRIX_SOURCES_
#define MATRIX_SOURCES_

#include "matrix_global.h"
#include <fstream>
#include <string.h>

#include "matrix.h"


/*!
 */
namespace PLib {

/*!
  \brief assignment operator

  \param a  the matrix to copy

  \warning the matrix a must have compatible dimensions

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Matrix<T>& Matrix<T>::operator=(const Matrix<T> &a){
  int i;
  
  if ( this == &a )
    return *this;
  
  if ( a.rows() != rows() || a.cols() != cols() ){
    resize(a.rows(),a.cols()) ;
  }
  
  int sze = rows()*cols() ;
  T *ptr, *aptr ;
  ptr = m-1 ;
  aptr = a.m-1 ;
  
  for (i = sze; i > 0; --i)
    *(++ptr) = *(++aptr) ;
  
  by_columns = a.by_columns;
  
  return *this;
}

/*!
  \brief sets the submatrix \a (s_r,s_c) to \a a .

  \latexonly
  The matrix can be  viewed as
	       \[ \left[\begin{array}{ccc}
	         sub_{0,0} & \vdots & sub_{0,m} \\
	         \cdots & \cdots & \cdots \\
	         sub_{n,0} & \vdots & sub_{n,m} \end{array} \right] \]
	       where each sub matrix is of the size of $a$. This function 
	       replaces $sub_{s_r,s_c}$ with $a$.
  \endlatexonly	      

  \param sr  the row of the submatrix
  \param sc  the column of the submatrix
  \param a  the submatrix to copy from

  \warning Since the size of \a a defines the size of the submatrices, 
           this size must be such that a submatrix located at \a (s_r,s_c)
	   exists.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void Matrix<T>::submatrix(int sr, int sc, Matrix<T> &a)
{
  int rwz,coz,i,j;
  
  if ( rows() % a.rows() != 0 || cols() % a.cols() != 0 || rows() < a.rows() || cols() < a.cols() )
    {
#ifdef USE_EXCEPTION
      throw WrongSize2D(rows(),cols(),a.rows(),a.cols()) ;
#else
      Error error("Matrix<T>::submatrix");
      error << "Matrix and submatrix incommensurate" ;
      error.fatal() ;
#endif
    }
  
  if ( sr >= rows()/a.rows() || sr < 0 || sc >= cols()/a.cols() || sc < 0 )
    {
#ifdef USE_EXCEPTION
      throw OutOfBound2D(sr,sc,0,rows()/a.rows()-1,0,cols()/a.cols()-1) ;
#else
      Error error("Matrix<T>::submatrix");
      error << "Submatrix location out of bounds.\nrowblock " << sr << ", " << rows()/a.rows() << " colblock " << sc << ", " << a.cols() << endl ;
      error.fatal() ;
#endif
    }
  rwz = sr*a.rows();
  coz = sc*a.cols();
  
#ifdef COLUMN_ORDER
  for ( i = a.rows()-1; i >= 0; --i )
    for(j=a.cols()-1;j>=0;--j)
      elem(i+rwz,j+coz) = a(i,j) ;
#else
  T *ptr, *aptr ;
  aptr = a.m - 1;
  for ( i = a.rows()-1; i >= 0; --i )
    {
      ptr = &m[(i+rwz)*cols()+coz]-1 ;
      for ( j = a.cols(); j > 0; --j)
	*(++ptr) = *(++aptr) ;
    }  
#endif
}

/*!
  \brief copies a matrix into this matrix starting at index \a (rw,cl)

  \param rw  the row to insert the matrix at
  \param cl  the column to insert the matrix at
  \param a  the matrix to insert

  \warning The matrix \a a must fit inside the matrix starting from 
           \a (rw,cl).

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T> 
void Matrix<T>::as(int rw, int cl, Matrix<T>& a)
{
  // Assign matrix a to this matrix at (i,j)
  int i, j;
  
  if ( (rw + a.rows()) > rows() || ( cl + a.cols()) > cols()) {
#ifdef USE_EXCEPTION
    throw MatrixErr();
#else
    Error error("Matrix<T>::as") ;
    error << "Matrix A will not fit in this Matrix at " << rw << ", " << cl << endl ;
    error.fatal() ;
#endif
  }

#ifdef COLUMN_ORDER
  for(i=0;i<a.rows();++i)
    for(j=0;j<a.cols();++j)
      elem(i+rw,j+cl) = a(i,j) ;
#else
  T *pptr,*aptr ;
  aptr = a.m-1 ;
  for ( i = 0; i<a.rows(); ++i) {
    pptr = &m[(i+rw)*cols()+cl]-1 ;
    for ( j = 0; j < a.cols(); ++j)
      *(++pptr) = *(++aptr);
  }
#endif
}

/*!
  \brief returns the matrix of size \a (nr,nc) starting at \a (rw,cl).

  \latexonly
  This generates a matrix of size $(n_r,n_c)$ with its first 
  point located at $rw,cl$.
  \endlatexonly

  \param rw  the index of the row 
  \param cl  the index of the column
  \param nr  the number of rows() of the matrix to generate
  \param nc  the number of coluns of the matrix to generate

  \return the matrix of size \a (nr,nc) starting at index \a (rw,cl).

  \warning The matrix to return must fit inside the original matrix.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T> 
Matrix<T> Matrix<T>::get(int rw, int cl, int nr, int nc) const
{
  Matrix<T> getmat(nr,nc) ;
  if ( (rw+nr) > rows() || (cl+nc) > cols()) {
#ifdef USE_EXCEPTION
    throw MatrixErr();
#else
    Error error("Matrix<T>::get") ;
    error << "Matrix of size "<< nr << ", " << nc << " not available at " << rw << ", " << cl << endl ;
    error.fatal() ;
#endif
  }
  
  int i, j;

#ifdef COLUMN_ORDER
  for(i=0;i<nr;++i)
    for(j=0;j<nc;++j)
      getmat(i,j) = elem(i+rw,j+cl) ;
#else
  T *pptr,*aptr ;
  aptr = getmat.m-1;
  for (i = 0; i < nr; ++i) {
    pptr = &m[(i+rw)*cols()+cl]-1 ;
    for ( j = 0; j < nc; ++j)
      *(++aptr) = *(++pptr) ;
  }
#endif
  return getmat ;
}



/*!
  \brief Finds the first norm of the matrix

  \return the norm of the matrix

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T> 
double Matrix<T>::norm(void){
  int i,j ;
  double sum, maxsum;
  int init=0 ;
  T *pptr ;
  pptr = m-1 ;
  maxsum = 0 ; // Silence the warning message
  for(i=0;i<rows();++i){
    sum = 0 ;
    for ( j = 0; j < cols(); ++j) 
      sum += *(++pptr) ;
    if(init)
      maxsum = (maxsum>sum) ? maxsum : sum;
    else{
      maxsum = sum ;
      init = 1;
    }
  }
  return maxsum;
}


/*!
  \brief sets the diagonal of the matrix to \a a

  Sets the diagonal points of the matrix to \a a. The diagonal 
  points are \a (0,0),(1,1),(2,2),etc.

  \param a  the value to set the diagonal to

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void Matrix<T>::diag(const T a)
{
  int i, iend;
  
  iend = rows();
  if ( iend > cols() )
    iend = cols();
  
  for (i = iend-1; i >=0; --i)
    elem(i,i) = a;

}

/*!
  \brief returns the diagonal of the matrix

  Returns a vector with the component \a [i] being set to the 
  component \a (i,i) of the matrix.

  \return the vector representing the diagonal of the matrix.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Vector<T> Matrix<T>::getDiag(){
  int i, iend;
  Vector<T> vec(minimum(rows(),cols())) ;
  iend = minimum(rows(),cols());
  for (i = iend-1; i >=0; --i)
      vec[i] = elem(i,i);
  return vec ;
}

/*!
  \brief increase every elements by a double

  \param a  the value to increase the elements by
  \return a reference to itself

  \author Philippe Lavoie 
  \date 1 June 1998
*/
template <class T> 
Matrix<T>& Matrix<T>::operator+=(double a)
{
  T *p1 ;
  p1 = m-1 ;
  const int size = rows()*cols() ;
  for(int i=size; i>0; --i)
    *(++p1) += a ;  
  return *this ;
}

/*!
  \brief decrease every elements by a double

  \param a  the value to decrease the elements by
  \return a reference to itself

  \author Philippe Lavoie 
  \date 1 June 1998
*/
template <class T> 
Matrix<T>& Matrix<T>::operator-=(double a)
{
  T *p1 ;
  p1 = m-1 ;
  const int size = rows()*cols() ;
  for(int i=size; i>0; --i)
    *(++p1) -= a ;  
  return *this ;
}

/*!
  \brief multiply every elements by a double

  \param a  the value to mutiply the elements with
  \return a reference to itself

  \author Philippe Lavoie 
  \date 1 June 1998
*/
template <class T> 
Matrix<T>& Matrix<T>::operator*=(double a)
{
  T *p1 ;
  p1 = m-1 ;
  const int size = rows()*cols() ;
  for(int i=size; i>0; --i)
    *(++p1) *= a ;  
  return *this ;
}

/*!
  \brief divide every elements by a double

  \param a  the value to divide the elements with
  \return a reference to itself

  \author Philippe Lavoie 
  \date 1 June 1998
*/
template <class T> 
Matrix<T>& Matrix<T>::operator/=(double a)
{
  T *p1 ;
  p1 = m-1 ;
  const int size = rows()*cols() ;
  for(int i=size; i>0; --i)
    *(++p1) /= a ;  
  return *this ;
}

/*!
  \brief adds a matrix to itself

  \param a  the matrix to increment itself with
  \return a reference to itself
  \warning the matrix a must have a size compatible with the matrix

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T> 
Matrix<T>& Matrix<T>::operator+=(const Matrix<T> &a)
{
  if ( a.rows() != rows() || a.cols() != cols() )
    {
#ifdef USE_EXCEPTION
      throw WrongSize2D(rows(),cols(),a.rows(),a.cols());
#else
      Error error("Matrix<T>::operator+=") ;
      if ( rows() != a.rows() )
	error << "Matrices are of diferent size, a.rows() = " << rows() << " and b.rows() = " << a.rows() << endl ;
      if ( cols() != a.cols())
	error << "Matrices are of diferent size, a.cols() = " << cols() << " and b.cols() = " << a.cols() << endl ;
      error.fatal() ;
#endif
    }

  int i, sze ;
  T *aptr,*sptr ;
  aptr = a.m - 1 ;
  sptr = m - 1 ;
  sze = rows()*cols() ;
  for (i = sze; i > 0; --i){
      *(++sptr) += *(++aptr) ;
  }
  return *this ;
}

/*!
  \brief the addition operator

  \param a  the first matrix to add
  \param b  the second matrix to add
  \return a matrix equal to $a+b$

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Matrix<T> operator+(const Matrix<T> &a,const Matrix<T> &b)
{
  Matrix<T> sum(a) ;
  sum += b ;
  return sum;
}

/*!
  \brief self substraction

  This will substract the matrix a from the matrix. The result is
  thus matrix = matrix - a.

  \param a  the matrix to substract
  \return a reference to itself 

  \warning The matrix $a$ must be compatible with this matrix

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T> 
Matrix<T>& Matrix<T>::operator-=(const Matrix<T> &a)
{
  if ( a.rows() != rows() || a.cols() != cols() )
    {
#ifdef USE_EXCEPTION
      throw WrongSize2D(rows(),cols(),a.rows(),a.cols());
#else
      Error error("Matrix<T>::operator-=") ;
      if ( rows() != a.rows() )
	error << "Matrices are of diferent size, a.rows() = " << rows() << " and b.rows() = " << a.rows() << endl ;
      if ( cols() != a.cols())
	error << "Matrices are of diferent size, a.cols() = " << cols() << " and b.cols() = " << a.cols() << endl ;
      error.fatal() ;
#endif
    }

  int i, size;
  T *aptr,*sptr ;
  aptr = a.m - 1 ;
  sptr = m - 1 ;
  size = rows()*cols() ;
  for (i = size; i > 0; --i){
      *(++sptr) -= *(++aptr) ;
  }
  return *this ;
}



/*!
  \brief the substraction operator

  \param a  the matrix to substract from 
  \param b  the matrix to substract
  \return a matrix equal to \a a-b

  \warning the matrix must be compatible

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Matrix<T> operator-(const Matrix<T> &a,const Matrix<T> &b)
{
  Matrix<T> diff(a) ;
  diff -= b ;
  return diff;
}

/*!
  \brief the multiplication operator

  \param a  a matrix
  \param b  the matrix to multiply with
  \return A matrix equal to $a b$

  \warning The matrix must be compatible for the multiplication: 
           a.cols() == b.rows()

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Matrix<T> operator*(const Matrix<T> &a,const Matrix<T> &b)
{
  if ( a.cols() != b.rows() )
    {
#ifdef USE_EXCEPTION
      throw WrongSize2D(a.rows(),a.cols(),b.rows(),b.cols()) ;
#else
      Error error("Matrix<T> operator*(Matrix<T>&,Matrix<T>&)");
      error << "Matrix<T> a * Matrix<T> b incommensurate, a.cols() = " << a.cols() << ", b.rows() = " << b.rows() << endl ;
      error.fatal() ;
#endif
    }

  int i, j, k, row=a.rows(), col=b.cols(),size = a.cols();
  Matrix<T> prod(row,col);
  T zero = (T)0;
  
#ifdef COLUMN_ORDER
  for(i=row-1;i>=0;--i)
    for(j=size-1;j>=0;--j)
      if(a(i,j) != zero){
	for(k=col-1;k>=0; --k)
	  prod(i,k) += a(i,j)* b(j,k) ;
      }
#else
  T *pptr,*aptr,*bptr ;
  aptr = a.m ;
  for (i = 0; i < row; ++i)
    for (j = 0; j < size; ++j){
      if ( *aptr != zero )
	{
	  pptr = prod[i]-1;
	  bptr = b[j]-1;
	  for (k = col; k > 0; --k){
	    *(++pptr) += *aptr * *(++bptr);
	  }
	}
      ++aptr;
    }
#endif
  return prod;
  
}


/*!
  \brief multiplication of a double and a matrix

  \param d  the double value to scale the matrix with
  \param a  the matrix to scale with the double value $d$
  \return A matrix equal to \a d.A

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Matrix<T>  operator*(const double d, const Matrix<T> &a)
{
  int i, size=a.rows()*a.cols() ;
  Matrix<T> b(a.rows(),a.cols());
  
  T *bptr,*aptr ;
  bptr = b.m - 1 ;
  aptr = a.m - 1 ;
  for (i = size; i > 0; --i)
    *(++bptr) = (T)(d * (*(++aptr))) ;
  return b;
  
}

/*!
  \brief multiplies a matrix with a complex value

  \param d  a complex value
  \param a  a matrix

  \return returns a matrix equal to \a d.A

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Matrix<T>  operator*(const Complex &d, const Matrix<T> &a)
{
  int i, size=a.rows()*a.cols() ;
  Matrix<T> b(a.rows(),a.cols());
  
  T *bptr,*aptr ;
  bptr = b.m - 1 ;
  aptr = a.m - 1 ;
  for (i = size; i > 0; --i)
    *(++bptr) = (T)real(d) * *(++aptr) ;
  return b;
}

/*!
  \brief multiplies a matrix with a vector

  \param a  the matrix
  \param x  the vector
  \return returns a vector representing \a a \a x

  \warning The matrix and the vector must be of compatible sizes

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Vector<T> operator*(const Matrix<T> &a, const Vector<T> &x)
{
  if ( a.cols() != x.size() )
    {
#ifdef USE_EXCEPTION
      throw WrongSize2D(a.rows(),a.cols(),x.size(),1);
#else
      Error error("Matrix<T> operator*(Matrix<T>& a,Vector<T>& b)");
      error << "a * b incommensurate, a.cols() = " << a.cols() << ", b.rows() = " << x.size() << endl ;
      error.fatal() ;
#endif
    }
  
  int i, k, row = a.rows(), size = a.cols();
  Vector<T> prod(row);
  T zero = (T)0;
  
  T *pptr,*aptr,*xptr ;
  aptr = a.m - 1 ;
  pptr = &prod[0] ;
  for (i = row; i > 0; --i){
    xptr = x.memory()-1 ;
    for (k = size, *pptr = zero; k > 0 ; --k)
      *pptr += *(++aptr) * *(++xptr) ;
    ++pptr ;
  }
  
  return prod;
}


/*!
  \brief the equality operator

  Every elements are compared with each others. If one of them
  in matrix \a a is not equal to the one in matrix \a b, then
  the result is negative.

  \param a  the first matrix to compare 
  \param b  the second matrix to compare
  \return 1 if equal, 0 otherwise

  \warning The matrices must be of compatible sizes

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
int operator==(const Matrix<T> &a,const Matrix<T> &b)
{
  if ( a.rows() != b.rows() || a.cols() != b.cols() )
    {
#ifdef USE_EXCEPTION
      throw WrongSize2D(a.rows(),a.cols(),b.rows(),b.cols()) ;
#else
      Error error("operator==(const Matrix<T>&,const Matrix<T>&)");
      if ( b.rows() != a.rows() )
	error << "Matrices are of diferent size, a.rows() = " << a.rows() << " and b.rows() = " << b.rows() << endl ;
      if ( b.cols() != a.cols())
	error << "Matrices are of diferent size, a.cols() = " << a.cols() << " and b.cols() = " << b.cols() << endl ;
      error.fatal() ;
#endif
    }
  
  int i, j, row = a.rows(), col = a.cols();
  int l = 1;
  
  for (i = 0; i < row; ++i)
    for (j = 0; j < col; ++j)
      l = l && ( a.elem(i,j) == b.elem(i,j) );
  
  return l;
}

/*!
  \brief computes a b - b a 

  \param a  the a matrix
  \param b  the b matrix

  \warning The a and b matrix must be compatible for the comm operation.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Matrix<T> comm(const Matrix<T> &a,const Matrix<T> &b)
{
  Matrix<T> r = a * b - b * a;
  
  return r;
}

/*!
  \brief The sum of all diagonal elements

  \param a  the matrix to trace
  \return a value representing the sum of all diagonal elements

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
T Matrix<T>::trace() const
{
  int size = rows();
  T sum = (T)0;
  
  if ( size > cols() )
    size = cols();
  
  for (int d = 0; d < size; ++d)
    sum += elem(d,d) ;
  
  return sum;
}


/*!
  \brief computes the hermitian of the matrix

  This functions returns a matrix for which every elements \a (i,j)
  correspond to the element \a (j,i) of the original matrix.

  \param a  the a matrix 
  \return A matrix corresponding to the hermitian of $a$

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Matrix<T> Matrix<T>::herm() const
{
  int i, j, r = cols(), c = rows();
  Matrix<T> adj(r,c);
  
  for (i = 0; i < r; ++i)
    for (j = 0; j < c; ++j)
      adj.elem(i,j) = elem(j,i) ;

  return adj;

}

/*!
  \brief returns the matrix flopped

  The flop pixel (i,j) = (i,cols-j-1) 

  \return the flop of the matrix

  \author Philippe Lavoie 
  \date 2 May 1999
*/
template <class T>
Matrix<T> Matrix<T>::flop() const
{					
  Matrix<T> f(rows(),cols()) ;
  for(int i=rows()-1;i>=0;--i)
    for(int j=cols()-1;j>=0;--j)
      {
	f(i,j) = elem(i,cols()-j-1);
      }
  return f; 
}

/*!
  \brief returns the transpose of the matrix

  \param a  the matrix to transpose
  \return the transpose of the matrix

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Matrix<T> Matrix<T>::transpose() const
{					
  // same as hermitian for real Matrix<T>
  int i, j;
  const int& r = cols();
  const int& c = rows();
  Matrix<T> adj(r,c);
  
  for (i = r-1; i >=0; --i)
    for (j = c-1; j >=0; --j)
      adj.elem(i,j) = elem(j,i) ;
  
  
  return adj; 
}


/*!
  \brief read a matrix file
  Reads a matrix file. The format of a file is
{\tt rows() cols() data...}, where rows() and cols() are int and data is a vector of the matrix type.
  \param filename  the name of the file to read
  \return 1 if reading the file was successfull, 0 otherwise
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
int Matrix<T>::read(char* filename) {
  ifstream fin(filename) ;
  if(!fin) {
    resize(1,1) ;
    return 0 ;
  }
  int r,c ;
  char *type ;
  type = new char[6] ;
  if(!fin.read(type,sizeof(char)*6)) return 0 ;
  r = strncmp(type,"matrix",6) ;
  if(r) return 0 ;
  if(!fin.read((char*)&r,sizeof(int))) return 0 ;
  if(!fin.read((char*)&c,sizeof(int))) return 0 ;
  resize(r,c) ;
  if(!fin.read((char*)m,sizeof(T)*r*c)) return 0 ;

  delete []type ;
  return 1 ;
}

/*!
  \brief read a raw file containing a matrix of size $(r,c)$
  Reads a file containing raw data of a matrix of size $(r,c)$.
  \param filename  the name of the file to read
                      r  the number of rows() 
                      c  the number of columns
  \return 1 if reading the file was successfull, 0 otherwise
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
int Matrix<T>::read(char* filename,int r, int c) {
  ifstream fin(filename) ;
  if(!fin) {
    resize(1,1) ;
    return 0 ;
  }
  resize(r,c) ;
  if(!fin.read((char*)m,sizeof(T)*r*c)) return 0 ;

  return 1 ;
}


/*!
  \brief write a matrix into a Matrix file
  Writes a matrix file. The format of the file is
               {\tt rows() cols() data...}, where rows() and cols() are int and data 
	       is a vector of the matrix type.
  \param filename  the name of the file to write to
  \return
  \warning 1 if reading the file was successfull, 0 otherwise
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
int Matrix<T>::write(char* filename) {
  ofstream fout(filename) ;
  if(!fout)
    return 0 ;
  int r,c ;
  r = rows() ; c = cols() ;
  if(!fout.write((char*)&"matrix",sizeof(char)*6)) return 0 ;
  if(!fout.write((char*)&r,sizeof(int))) return 0 ;
  if(!fout.write((char*)&c,sizeof(int))) return 0 ;
  if(!fout.write((char*)m,sizeof(T)*r*c)) return 0 ;
  return 1;
}

/*!
  \brief write the raw data to a file
  Writes the raw data to a file. The size information is 
               {\em not} written to the file.
  \param filename  the name of the file to write to
  \return 0 if an error occurs, 1 otherwise
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
int Matrix<T>::writeRaw(char* filename) {
  ofstream fout(filename) ;
  if(!fout)
    return 0 ;
  if(!fout.write((char*)m,sizeof(T)*rows()*cols())) return 0 ;
  return 1;
}


template <class T>
void Matrix<T>::qSort(){
#ifdef USE_EXCEPTION
  throw MatrixErr();
#else
  Error error("Matrix<T>::qSort()");
  error << "qSort is not defined for that type.\nPlease defined it in your .cpp file!";
  error.fatal() ;
#endif
}


}


#endif
