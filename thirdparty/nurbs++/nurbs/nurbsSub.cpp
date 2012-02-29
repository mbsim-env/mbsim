/*=============================================================================
        File: nurbsSub.cpp
     Purpose:       
    Revision: $Id: nurbsSub.cpp,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (20 Januray, 1999)
 Modified by: 

 Copyright notice:
          Copyright (C) 1999 Philippe Lavoie
 
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
#include "nurbsSub.h"
#include <iostream>
#include <cstdio>
#include <stdio.h>

/*!
 */
namespace PLib {

  
  const int FALSE_ = 0 ;
  const int TRUE_ = 1 ; 

  struct BOOL{
    int value ;
    BOOL(): value(FALSE_) {;}
    BOOL(int a) { if(a>0) value=TRUE_; else value=FALSE_;}
    BOOL(const BOOL& b) { value = b.value ; }
    int operator!() { if(value>0) return FALSE_; return TRUE_ ;}
    operator int() const {return value;}
    int& operator=(int a) { value = a ;  return value ; }
  };

  /*!
    \brief Constructor from a NurbsSurface
    
    Constructor from a NurbsSurface
    
    \param s the NurbsSurface to construct from 
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    NurbsSubSurface<T>::NurbsSubSurface(const NurbsSurface<T,3>& s): rsurf(s) {
    surf = 0 ;
    render = 0 ; 
    //initSurf();
    
  }

#define MAXORDER 20	    /* Maximum order allowed (for local array sizes) */

  template <class T>
    struct NurbSurface {
      /* Number of Points in the U and V directions, respectivly */
      int numU, numV;
      /* Order of the surface in U and V (must be >= 2, < MAXORDER) */
      int orderU, orderV;
      /* Knot vectors, indexed as [0..numU+orderU-1] and [0..numV+orderV-1] */
      T * kvU, * kvV;
      /* Control points, indexed as points[0..numV-1][0..numU-1] */
      /* Note the w values are *premultiplied* with the x, y and z values */
      Matrix<HPoint_nD<T,3> >& points;
      
      /* These fields are added to support subdivision */
      BOOL strV0, strVn,   /* Edge straightness flags for subdivision */
      strU0, strUn;
      BOOL flatV, flatU;   /* Surface flatness flags for subdivision */
      SurfSample<T> c00, c0n,
      cn0, cnn;    /* Corner data structures for subdivision */
      RenderMesh<T> *render;
      
      static T epsilon ;
      
      NurbSurface(): points(tmpPoints) {   render = 0 ; }
      NurbSurface(Matrix<HPoint_nD<T,3> >& s, int du, int dv);
      
    protected:
      Matrix<HPoint_nD<T,3> > tmpPoints ;
      
    };

#define CHECK( n ) \
  { if (!(n)) { fprintf( stderr, "Ran out of memory\n" ); exit(-1); } }

#define DIVW( rpt, pt ) \
  { (pt)->x() = (rpt)->x() / (rpt)->w(); \
  (pt)->y() = (rpt)->y() / (rpt)->w(); \
  (pt)->z() = (rpt)->z() / (rpt)->w(); }
  
  
  /* Function prototypes */
  
  template <class T> void DrawSubdivision( NurbSurface<T> *, T tolerance );
  template <class T> void DrawEvaluation( NurbSurface<T> * );
  
  template <class T> int FindBreakPoint( T u, T * kv, int m, int k );
  template <class T> void AllocNurb( NurbSurface<T> *, T *, T * );
  template <class T> void CloneNurb( NurbSurface<T> *, NurbSurface<T> * );
  template <class T> void FreeNurb( NurbSurface<T> * );
  template <class T> void RefineSurface( NurbSurface<T> *, NurbSurface<T> *, BOOL );
  
  template <class T> void CalcPoint( T, T, NurbSurface<T> *, Point_nD<T,3> *, Point_nD<T,3> *, Point_nD<T,3> * );
  
  
  /*!
    \brief Draw the subdivision of the NURBS surface
    
    Draw the subdivision of the NURBS surface
    
    \param tolerance the accepted tolerance
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void NurbsSubSurface<T>::drawSubdivision(T tolerance)
    {
      initSurf();
      surf->render = render ; 
      DrawSubdivision(surf,tolerance);
    }
  
  /*!
    \brief perform the subdivision of the NURBS and write the result in a PS file.

    \param f the file name to write to
    \param tolerance the accepted tolerance
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void NurbsSubSurface<T>::drawSubdivisionPS(const char* f, T tolerance)
    {
      ofstream fout ;
      fout.open(f) ;
      if(fout)
	drawSubdivisionPS(fout,tolerance) ;
      fout.close();
    }

  /*!
    \brief perform the subdivision of the NURBS and write the result in a VRML file.

    
    \param f the file name to write to
    \param tolerance the accepted tolerance
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void NurbsSubSurface<T>::drawSubdivisionVRML(const char* f, T tolerance, const Color& col)
    {
      ofstream fout ;
      fout.open(f) ;
      if(fout)
	drawSubdivisionVRML(fout,tolerance,col) ;
      fout.close();
    }

  /*!
    \brief perform the subdivision of the NURBS and write the result in a VRML file.

    
    \param f the file name to write to
    \param tolerance the accepted tolerance
    
    \author Philippe Lavoie 
    \date 30 April 1999
  */
  template <class T>
    void NurbsSubSurface<T>::drawSubdivisionVRML97(const char* f, T tolerance, const Color& col)
    {
      ofstream fout ;
      fout.open(f) ;
      if(fout)
	drawSubdivisionVRML97(fout,tolerance,col) ;
      fout.close();
    }

  /*!
    \brief perform the subdivision of the NURBS and write the result in a PS file.

    
    \param os the ostream to write to
    \param tolerance the accepted tolerance
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void NurbsSubSurface<T>::drawSubdivisionPS(ostream &os, T tolerance)
    {
      if(render)
	delete render ;
      render = new RenderMeshPS<T>(os) ;
      drawSubdivision(tolerance) ;
    }

  /*!
    \brief perform the subdivision of the NURBS and write the result in a VRML file.

    \param os the ostream to write to
    \param tolerance the accepted tolerance
    
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void NurbsSubSurface<T>::drawSubdivisionVRML(ostream &os, T tolerance, const Color& col)
    {
      if(render) 
	delete render ;
      render = new RenderMeshVRML<T>(os,col) ;
      drawSubdivision(tolerance) ;
    }

  /*!
    \brief perform the subdivision of the NURBS and write the result in a VRML file.

    \param os the ostream to write to
    \param tolerance the accepted tolerance
    
    
    \author Philippe Lavoie 
    \date 30 April 1999
  */
  template <class T>
    void NurbsSubSurface<T>::drawSubdivisionVRML97(ostream &os, T tolerance, const Color& col)
    {
      if(render) 
	delete render ;
      render = new RenderMeshVRML97<T>(os,col) ;
      drawSubdivision(tolerance) ;
    }

  /*!
    \brief perform the subdivision of the NURBS and write the result in a VRML file.

    \param os the ostream to write to
    \param tolerance the accepted tolerance
    
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
  //void NurbsSubSurface<T>::drawSubdivisionPoints(vector<Point_nD<T,3> > &pnts, T tolerance)
  void NurbsSubSurface<T>::drawSubdivisionPoints(BasicArray<Point_nD<T,3> >&pnts, T tolerance)
    {
      if(render) 
	delete render ;
      render = new RenderMeshPoints<T>(pnts) ;
      drawSubdivision(tolerance) ;
    }

  /*!
    \brief initialise the subdivision surface

    
    \param tolerance the accepted tolerance
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void NurbsSubSurface<T>::initSurf()
    {
      if(surf)
	delete surf ;
      surf = new NurbSurface<T> ;
      surf->numU = rsurf.ctrlPnts().rows() ;
      surf->numV = rsurf.ctrlPnts().cols() ; 
      surf->orderU = rsurf.degreeU()+1 ;
      surf->orderV = rsurf.degreeV()+1 ;
      surf->kvU = new T[surf->numU + surf->orderU];
      surf->kvV = new T[surf->numV + surf->orderV];
      surf->points.resize(surf->numV,surf->numU);
      
      surf->flatV = FALSE_;
      surf->flatU = FALSE_;
      surf->strU0 = FALSE_;
      surf->strUn = FALSE_;
      surf->strV0 = FALSE_;
      surf->strVn = FALSE_;

      surf->render = render ; 
     
      int i;
      for(i=rsurf.knotU().n()-1;i>=0;--i){
	surf->kvU[i] = rsurf.knotU()[i] ;
      }
      for(i=rsurf.knotV().n()-1;i>=0;--i){
	surf->kvV[i] = rsurf.knotV()[i] ;
      }
      for(i=rsurf.ctrlPnts().rows()-1;i>=0;--i)
	for(int j=rsurf.ctrlPnts().cols()-1;j>=0;--j)
	  surf->points(j,i) = rsurf.ctrlPnts()(i,j) ; 
    }
  
  template <class T>
    NurbSurface<T>::NurbSurface(Matrix<HPoint_nD<T,3> >& s, int du, int dv): points(s), orderU(du+1), orderV(dv+1){
    numU = points.rows() ;
    numV = points.cols() ;
    kvU = new T[numU + orderU] ;
    kvV = new T[numV + orderV] ; 
    render = 0 ; 
  }
  
  /*!
    \brief Destructor

    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    NurbsSubSurface<T>::~NurbsSubSurface()
    {      
      if(surf){
	delete []surf->kvU ;
	delete []surf->kvV ;
      }
    }
  
  /*!
    \brief projects from world to screen coordinates

    A Post Script point is the projection of the point from the
    homogenous space to the 2D paper surface with the axis multiplied
    by 100 and with an offset of 200.

    There is no perspective projection performed.

    \param worldPt the point in world coordinate
    \param screenPt the point in the VRML coordinate
    
    \author Philippe Lavoie 
    \date 20 January 1999 */
  template <class T>
    void RenderMeshPS<T>::screenProject(const HPoint_nD<T,3> &worldPt, Point_nD<T,3> &screenPt )
    {
      screenPt.x() = worldPt.x() / worldPt.w() * 100 + 200;
      screenPt.y() = worldPt.y() / worldPt.w() * 100 + 200;
      screenPt.z() = worldPt.z() / worldPt.w() * 100 + 200;
    }

  /*!
    \brief projects from world to screen coordinates

    In the case of a VRML file, the world and screen coordinate are the same.
    Except that one is in homogenous space and the other in normal space.

    \param worldPt the point in world coordinate
    \param screenPt the point in the VRML coordinate
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void RenderMeshVRML<T>::screenProject(const HPoint_nD<T,3> &worldPt, Point_nD<T,3> &screenPt )
    {
      screenPt = project(worldPt) ;
    }


  /*!
    \brief projects from world to screen coordinates

    In the case of a VRML file, the world and screen coordinate are the same.
    Except that one is in homogenous space and the other in normal space.

    \param worldPt the point in world coordinate
    \param screenPt the point in the VRML coordinate
    
    \author Philippe Lavoie 
    \date 30 April 1999
  */
  template <class T>
    void RenderMeshVRML97<T>::screenProject(const HPoint_nD<T,3> &worldPt, Point_nD<T,3> &screenPt )
    {      
      screenPt = project(worldPt) ;
      if(init){
	p_min = p_max = screenPt ;
	init = 0 ; 
      }
      if(screenPt.x()<p_min.x()) p_min.x() = screenPt.x();
      if(screenPt.y()<p_min.y()) p_min.y() = screenPt.y();
      if(screenPt.z()<p_min.z()) p_min.z() = screenPt.z();
      if(screenPt.x()>p_max.x()) p_max.x() = screenPt.x();
      if(screenPt.y()>p_max.y()) p_max.y() = screenPt.y();
      if(screenPt.z()>p_max.z()) p_max.z() = screenPt.z();
    }


  /*!
    \brief projects from world to screen coordinates

    The world and screen coordinate are the same.  Except that one is
    in homogenous space and the other in normal space.

    \param worldPt the point in world coordinate
    \param screenPt the point in the normal space
    
    \author Philippe Lavoie 
    \date 20 January 1999 
  */
  template <class T>
    void RenderMeshPoints<T>::screenProject(const HPoint_nD<T,3> &worldPt, Point_nD<T,3> &screenPt )
    {
      screenPt = project(worldPt) ;
    }


  inline void LineTo( ostream& out, short x, short y )
    {
      out << (int)x << " " << (int)y << " lineto\n" ; 
    }
  
  inline void MoveTo( ostream& out, short x, short y )
    {
      out << (int)x << " " << (int)y << " moveto\n" ; 
    }
  
  /*!
    \brief write the header of a PS file

    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
  void RenderMeshPS<T>::drawHeader(){
    out << "%!PS-Adobe-2.1\n";
    out << "%%Title: code5_0.ps (20)\n" ;
    out << "%%Creator: color_grid_generator\n" ; 
    out << "%%BoundingBox: 0 0 500 500\n" ;
    out << "%%Pages: 0\n" ;
    out << "%%EndComments\n" ;
    out << "0 setlinewidth\n" ;
    out << "0 0 0 setrgbcolor\n" ;
  }

  /*!
    \brief write the footer of a PS file

    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void RenderMeshPS<T>::drawFooter(){
    out << "showpage\n%%EOF\n" ; 
  }


  /*!
    \brief Draw a triangle
    
    \param v0
    \param v1
    \param v2
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void RenderMeshPS<T>::drawTriangle( const SurfSample<T> &v0, const SurfSample<T> &v1, const SurfSample<T> & v2 )
    {
      out << "newpath\n" ;
      MoveTo( out, (short) (v0.point.x() * 100 + 200), (short) (v0.point.y() * 100 + 200) );
      LineTo( out, (short) (v1.point.x() * 100 + 200), (short) (v1.point.y() * 100 + 200) );
      LineTo( out, (short) (v2.point.x() * 100 + 200), (short) (v2.point.y() * 100 + 200) );
      LineTo( out, (short) (v0.point.x() * 100 + 200), (short) (v0.point.y() * 100 + 200) );
      out << "stroke\n" ;
    }

  /*!
    \brief Draw a line
    
    \param v0
    \param v1
    
    \author Philippe Lavoie 
    \date 18 may 1999    
   */
  template <class T>
    void RenderMeshPS<T>::drawLine( const SurfSample<T> &v0, const SurfSample<T> &v1)
    {
      out << "newpath\n" ; 
      MoveTo(out , (short) (v0.point.x() * 100 + 200), (short) (v0.point.y() * 100 + 200) );
      LineTo( out, (short) (v1.point.x() * 100 + 200), (short) (v1.point.y() * 100 + 200) );
      out << "stroke\n" ; 
    }


  /*!
    \brief write the header information for a VRML file

    \author Philippe Lavoie 
    \date 30 April 1999
  */
  template <class T>
    void RenderMeshVRML97<T>::drawHeader()
    {
      size = 0 ; 
      out << "#VRML V2.0 utf8\n" ;
      out << "#  Generated by Phil's NURBS library\n" ;
      out << "\nGroup {\n" ;
      out << "\n  children [\n" ; 
      out << "\tDEF T Transform {\n"; 
      out << "\t  children [\n" ;
      out << "\t\tShape {\n" ;
      out << "\t\t appearance Appearance {\n" ; 
      out << "\t\t\tmaterial Material{ diffuseColor " << float(color.r/255.0) << ' ' << float(color.g/255.0) << ' ' << float(color.b/255.0) << " }\n" ;
      out << "\t\t }\n" ; 
      out << "\t\t geometry IndexedFaceSet {\n" ;
      out << "\t\t\tsolid FALSE\n" ; 
      out << "\t\t\tcoord Coordinate {\n" ;
      out << "\t\t\t point [\n" ;
    }
 
  /*!
    \brief write the footer information for a VRML file

    Write the footer information for a VRML file
    
    \author Philippe Lavoie 
    \date 30 April 1999
  */
  template <class T>
    void RenderMeshVRML97<T>::drawFooter(){

    out << "\t\t\t ]\n" ; // point [
    out << "\t\t\t}\n" ; // coord
    
    out << "\t\t\t coordIndex [\n" ;
  
    for(int i=0;i<size;++i){
      out << "\t\t\t\t" << 3*i << ", " << 3*i+1 << ", " << 3*i+2 << ", -1,\n" ;
    }
    out << "\t\t\t ]\n" ;
    out << "\t\t\t}\n" ; // IndexedFaceSet
    out << "\t\t}\n" ;
    out << "\t ]\n" ; 
    out << "\t}\n" ;
    out << "  ]\n" ; 
    out << "}\n" ; 
    
    Point_nD<T,3> p_mid((p_max.x()+p_min.x())/T(2),
			(p_max.y()+p_min.y())/T(2),
			(p_max.z()+p_min.z())/T(2));
    
    T x_axis = p_max.x() - p_min.x() ;
    T y_axis = p_max.y() - p_min.y() ; 
    
    T axis = (x_axis< y_axis) ? y_axis : x_axis ;
    axis *= T(2) ;
    
    out << "Viewpoint {\n\t position " << p_mid.x() << ' ' << p_mid.y() << ' ' << p_max.z()+axis << "\n\t description \"top\"\n}\n" ; 
    
    
    out << "NavigationInfo { type \"EXAMINE\" }\n" ; 

  }

  /*!
    \brief write the header information for a VRML file

    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void RenderMeshVRML<T>::drawHeader()
    {
      size = 0 ; 
      out << "#VRML V1.0 ascii\n" ;
      out << "#  Generated by Phil's NURBS library\n" ;
      out << "\nSeparator {\n" << "\tMaterialBinding { value PER_VERTEX_INDEXED }\n"  ;
      out << "\tMaterial{ ambientColor 0.25 0.25 0.25\n\t\tdiffuseColor " << float(color.r/255.0) << ' ' << float(color.g/255.0) << ' ' << float(color.b/255.0) << " }\n" ;
      out << "\tCoordinate3 {\n" ;
      out << "\t\tpoint [\n" ;
    }
 
  /*!
    \brief write the footer information for a VRML file

    Write the footer information for a VRML file
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void RenderMeshVRML<T>::drawFooter(){

    out << "\t\t]\n" ; // point [
    out << "\t}\n" ; // cordinate3
    
    out << "\tIndexedFaceSet{\n" ;
    out << "\t\tcoordIndex[\n" ;
    
    for(int i=0;i<size;++i){
      out << "\t\t\t" << 3*i << ", " << 3*i+1 << ", " << 3*i+2 << ", -1,\n" ;
    }
    out << "\t\t]\n" ;
    out << "\t}\n" ; // IndexedFaceSet
    
    out << "}\n" ;
  }


  /*!
    \brief draws the triangle

    This function draws the triangle points to the ostream.
    
    \param v0 a corner point of the triangle
    \param v1 a corner point of the triangle
    \param v2 a corner point of the triangle
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void RenderMeshVRML<T>::drawTriangle(const SurfSample<T> &v0, const SurfSample<T> &v1, const SurfSample<T> & v2 )
    {
      ++size ;
      out << "\t\t\t" << v0.point.x() << " " << v0.point.y() << " " << v0.point.z() << ",\n" ;
      out << "\t\t\t" << v1.point.x() << " " << v1.point.y() << " " << v1.point.z() << ",\n" ;
      out << "\t\t\t" << v2.point.x() << " " << v2.point.y() << " " << v2.point.z() << ",\n" ;
      
    }
  
  /*!
    \brief draws the triangle

    This function draws the triangle points to the ostream.
    
    \param v0 a corner point of the triangle
    \param v1 a corner point of the triangle
    \param v2 a corner point of the triangle
    
    \author Philippe Lavoie 
    \date 30 April 1999
  */
  template <class T>
    void RenderMeshVRML97<T>::drawTriangle(const SurfSample<T> &v0, const SurfSample<T> &v1, const SurfSample<T> & v2 )
    {
      ++size ;
      out << "\t\t\t\t" << v0.point.x() << " " << v0.point.y() << " " << v0.point.z() << ",\n" ;
      out << "\t\t\t\t" << v1.point.x() << " " << v1.point.y() << " " << v1.point.z() << ",\n" ;
      out << "\t\t\t\t" << v2.point.x() << " " << v2.point.y() << " " << v2.point.z() << ",\n" ;
      
    }
  
 /*!
    \brief write the header information for a mesh file

    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void RenderMeshPoints<T>::drawHeader()
    {
      points.clear();
    }
 
  /*!
    \brief empty function

    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void RenderMeshPoints<T>::drawFooter(){
  }


  /*!
    \brief draws the triangle

    Adds the triangle points to the point vector.
    
    \param v0 a corner point of the triangle
    \param v1 a corner point of the triangle
    \param v2 a corner point of the triangle
    
    \author Philippe Lavoie 
    \date 20 January 1999
  */
  template <class T>
    void RenderMeshPoints<T>::drawTriangle(const SurfSample<T> &v0, const SurfSample<T> &v1, const SurfSample<T> & v2 )
    {
      // naive method
      points.push_back(v0.point);
      points.push_back(v1.point);
      points.push_back(v2.point);
    }
  
template <class T>
int
FindBreakPoint( T u, T * kv, int m, int k )
{
  int i;
  
  if (u == kv[m+1])	/* Special case for closed interval */
    return m;
  
  i = m + k;
  while ((u < kv[i]) && (i > 0))
    i--;
  return( i );
}

/*
 * Compute Bi,k(u), for i = 0..k.
 *  u		is the parameter of the spline to find the basis functions for
 *  brkPoint	is the start of the knot interval ("segment")
 *  kv		is the knot vector
 *  k		is the order of the curve
 *  bvals	is the array of returned basis values.
 *
 * (From Bartels, Beatty & Barsky, p.387)
 */

template <class T> void
BasisFunctions( T u, int brkPoint, T * kv, int k, T * bvals )
{
  int r, s, i;
  T omega;
  
  bvals[0] = 1.0;
  for (r = 2; r <= k; r++)
    {
      i = brkPoint - r + 1;
      bvals[r - 1] = 0.0;
      for (s = r-2; s >= 0; s--)
	{
	  i++;
	  if (i < 0)
	    omega = 0;
	  else
	    omega = (u - kv[i]) / (kv[i + r - 1] - kv[i]);
	  bvals[s + 1] = bvals[s + 1] + (1 - omega) * bvals[s];
	  bvals[s] = omega * bvals[s];
	}
    }
}

/*
 * Compute derivatives of the basis functions Bi,k(u)'
 */
template <class T> void
BasisDerivatives( T u, int brkPoint, T * kv, int k, T * dvals )
{
  register int s, i;
  T omega, knotScale;

  BasisFunctions( u, brkPoint, kv, k - 1, dvals );
  
  dvals[k-1] = 0.0;	    /* BasisFunctions misses this */
  
  knotScale = kv[brkPoint + 1] - kv[brkPoint];
  
  i = brkPoint - k + 1;
  for (s = k - 2; s >= 0; s--)
    {
      i++;
      omega = knotScale * ((T)(k-1)) / (kv[i+k-1] - kv[i]);
      dvals[s + 1] += -omega * dvals[s];
      dvals[s] *= omega;
    }
}

/*
 * Calculate a point p on NurbSurface<T> n at a specific u, v using the tensor product.
 * If utan and vtan are not nil, compute the u and v tangents as well.
 *
 * Note the valid parameter range for u and v is
 * (kvU[orderU] <= u < kvU[numU), (kvV[orderV] <= v < kvV[numV])
 */

template <class T>
void
CalcPoint(T u, T v, NurbSurface<T> * n,
	   Point_nD<T,3> * p, Point_nD<T,3> * utan, Point_nD<T,3> * vtan)
{
  int i, j, ri, rj;
  HPoint_nD<T,3> * cp;
  T tmp;
  T wsqrdiv;
  int ubrkPoint, ufirst;
  T bu[MAXORDER], buprime[MAXORDER];
  int vbrkPoint, vfirst;
  T bv[MAXORDER], bvprime[MAXORDER];
  HPoint_nD<T,3> r, rutan, rvtan;
  
  r.x() = 0.0;
  r.y() = 0.0;
  r.z() = 0.0;
  r.w() = 0.0;
  
  rutan = r;
  rvtan = r;
  
  /* Evaluate non-uniform basis functions (and derivatives) */
  
  ubrkPoint = FindBreakPoint( u, n->kvU, n->numU-1, n->orderU );
  ufirst = ubrkPoint - n->orderU + 1;
  BasisFunctions( u, ubrkPoint, n->kvU, n->orderU, bu );
  if (utan)
    BasisDerivatives( u, ubrkPoint, n->kvU, n->orderU, buprime );
  
  vbrkPoint = FindBreakPoint( v, n->kvV, n->numV-1, n->orderV );
  vfirst = vbrkPoint - n->orderV + 1;
  BasisFunctions( v, vbrkPoint, n->kvV, n->orderV, bv );
  if (vtan)
    BasisDerivatives( v, vbrkPoint, n->kvV, n->orderV, bvprime );
  
  /* Weight control points against the basis functions */
  
  for (i = 0; i < n->orderV; i++)
    for (j = 0; j < n->orderU; j++)
	{
	  ri = n->orderV - 1 - i;
	  rj = n->orderU - 1 - j;
	  
	  tmp = bu[rj] * bv[ri];
	  cp = &( n->points(i+vfirst,j+ufirst) );
	  r.x() += cp->x() * tmp;
	  r.y() += cp->y() * tmp;
	  r.z() += cp->z() * tmp;
	  r.w() += cp->w() * tmp;

	  if (utan)
	    {
	      tmp = buprime[rj] * bv[ri];
	      rutan.x() += cp->x() * tmp;
	      rutan.y() += cp->y() * tmp;
	      rutan.z() += cp->z() * tmp;
	      rutan.w() += cp->w() * tmp;
	    }
	  if (vtan)
	    {
	      tmp = bu[rj] * bvprime[ri];
	      rvtan.x() += cp->x() * tmp;
	      rvtan.y() += cp->y() * tmp;
	      rvtan.z() += cp->z() * tmp;
	      rvtan.w() += cp->w() * tmp;
	    }
	}
  
  /* Project tangents, using the quotient rule for differentiation */
  
  wsqrdiv = 1.0 / (r.w() * r.w());
  if (utan)
    {
      utan->x() = (r.w() * rutan.x() - rutan.w() * r.x()) * wsqrdiv;
      utan->y() = (r.w() * rutan.y() - rutan.w() * r.y()) * wsqrdiv;
      utan->z() = (r.w() * rutan.z() - rutan.w() * r.z()) * wsqrdiv;
    }
  if (vtan)
    {
      vtan->x() = (r.w() * rvtan.x() - rvtan.w() * r.x()) * wsqrdiv;
      vtan->y() = (r.w() * rvtan.y() - rvtan.w() * r.y()) * wsqrdiv;
      vtan->z() = (r.w() * rvtan.z() - rvtan.w() * r.z()) * wsqrdiv;
    }
  
  p->x() = r.x() / r.w();
  p->y() = r.y() / r.w();
  p->z() = r.z() / r.w();
}

/*
 * Draw a mesh of points by evaluating the surface at evenly spaced
 * points.
 */
template <class T>
void
DrawEvaluation( NurbSurface<T> * n )
{
  Point_nD<T,3> p, utan, vtan;
  register int i, j;
  register T u, v, d;
  SurfSample<T> ** pts ;
  
  int Granularity = 10;  /* Controls the number of steps in u and v */
  
  /* Allocate storage for the grid of points generated */
  
  CHECK( pts = new (SurfSample<T>*)[Granularity+1]);
  CHECK( pts[0] = new SurfSample<T>[(Granularity+1)*(Granularity+1)]);
  
  for (i = 1; i <= Granularity; i++)
    pts[i] = &(pts[0][(Granularity+1) * i]);
  
  /* Compute points on curve */
  
  for (i = 0; i <= Granularity; i++)
    {
      v = ((T) i / (T) Granularity)
	    * (n->kvV[n->numV] - n->kvV[n->orderV-1])
	+ n->kvV[n->orderV-1];
      
      for (j = 0; j <= Granularity; j++)
	{
	  u = ((T) j / (T) Granularity)
	    * (n->kvU[n->numU] - n->kvU[n->orderU-1])
	    + n->kvU[n->orderU-1];
	  
	  CalcPoint( u, v, n, &(pts[i][j].point), &utan, &vtan );
	  p = crossProduct(utan,vtan) ; //(void) V3Cross( &utan, &vtan, &p );
	  d = norm(p) ; // d = V3Length( &p );
	  if (d != 0.0)
	    {
	      p.x() /= d;
	      p.y() /= d;
	      p.z() /= d;
	    }
	  else
	    {
	      p.x() = 0;
	      p.y() = 0;
	      p.z() = 0;
	    }
	  pts[i][j].normLen = d;
	  pts[i][j].normal = p;
	  pts[i][j].u = u;
	  pts[i][j].v = v;
	}
    }
  
  /* Draw the grid */
  
  for (i = 0; i < Granularity; i++)
    for (j = 0; j < Granularity; j++)
      {
	n->render->drawTriangle( pts[i][j], pts[i+1][j+1], pts[i+1][j] );
	n->render->drawTriangle( pts[i][j], pts[i][j+1],   pts[i+1][j+1] );
      }
  
  delete [] pts[0];
  delete [] pts ;
}

/*
 * NurbRefine.c - Given a refined knot vector, add control points to a surface.
 *
 * John Peterson
 */


/*
 * Given the original knot vector ukv, and a new knotvector vkv, compute
 * the "alpha matrix" used to generate the corresponding new control points.
 * This routines allocates the alpha matrix if it isn't allocated already.
 *
 * This is from Bartels, Beatty & Barsky, p. 407
 */
template <class T> void
CalcAlpha( T * ukv, T * wkv, int m, int n, int k, T *** alpha )
{
  int i, j;
  int brkPoint, r, rm1, last, s;
  T omega;
  T aval[MAXORDER];
  
  if (! *alpha)	/* Must allocate alpha */
    {
      CHECK( *alpha = new (T*)[k+1]);
      for (i = 0; i <= k; i++)
	CHECK( (*alpha)[i] = new T[(m + n + 1)]);
    }
  
  for (j = 0; j <= m + n; j++)
    {
      brkPoint = FindBreakPoint( wkv[j], ukv, m, k );
      aval[0] = 1.0;
      for (r = 2; r <= k; r++)
	{
	  rm1 = r - 1;
	  last = minimum( rm1, brkPoint );
	  i = brkPoint - last;
	  if (last < rm1)
	    aval[last] = aval[last] * (wkv[j + r - 1] - ukv[i])
	      / (ukv[i + r - 1] - ukv[i]);
	  else
	    aval[last] = 0.0;
	  
	  for (s = last - 1; s >= 0; s-- )
	    {
	      i++;
	      omega = (wkv[j + r - 1] - ukv[i]) / (ukv[i + r - 1] - ukv[i]);
	      aval[s + 1] = aval[s+1] + (1 - omega) * aval[s];
	      aval[s] = omega * aval[s];
	    }
	}
      last = minimum( k - 1, brkPoint );
      for (i = 0; i <= k; i++)
	(*alpha)[i][j] = 0.0;
      for (s = 0; s <= last; s++)
	(*alpha)[last - s][j] = aval[s];
    }
}

/*
 * Apply the alpha matrix computed above to the rows (or columns)
 * of the surface.  If dirflag is TRUE_ do the U's (row), else do V's (col).
 */
template <class T>
void
RefineSurface( NurbSurface<T> * src, NurbSurface<T> * dest, BOOL dirflag )
{
  int i, j, out;
  HPoint_nD<T,3> * dp, * sp;
  int i1, brkPoint, maxj, maxout;
  register T tmp;
  T ** alpha = 0;
  
  // Compute the alpha matrix and indexing variables for the requested direction 
  
  if (dirflag)
    {
      CalcAlpha( src->kvU, dest->kvU, src->numU - 1, dest->numU - src->numU,
		 src->orderU, &alpha );
      maxj = dest->numU;
      maxout = src->numV;
    }
  else
    {
      CalcAlpha( src->kvV, dest->kvV, src->numV - 1, dest->numV - src->numV,
		 src->orderV, &alpha );
      maxj = dest->numV;
      maxout = dest->numU;
    }
  
  /* Apply the alpha matrix to the original control points, generating new ones */
  
  for (out = 0; out < maxout; out++)
    for (j = 0; j < maxj; j++)
      {
	if (dirflag)
	  {
	    dp = &(dest->points(out,j));
	    brkPoint = FindBreakPoint( dest->kvU[j], src->kvU,
				       src->numU-1, src->orderU );
	    i1 = maximum( brkPoint - src->orderU + 1, 0 );
	    sp = &(src->points(out,i1));
	  } else {
	    dp = &(dest->points(j,out));
	    brkPoint = FindBreakPoint( dest->kvV[j], src->kvV,
				       src->numV-1, src->orderV );
	    i1 = maximum( brkPoint - src->orderV + 1, 0 );
	    sp = &(src->points(i1,out));
	  }
	dp->x() = 0.0;
	dp->y() = 0.0;
	dp->z() = 0.0;
	dp->w() = 0.0;
	for (i = i1; i <= brkPoint; i++)
	  {
	    tmp = alpha[i - i1][j];
	    sp = (dirflag ? &(src->points(out,i)) : &(src->points(i,out)) );
	    dp->x() += tmp * sp->x();
	    dp->y() += tmp * sp->y();
	    dp->z() += tmp * sp->z();
	    dp->w() += tmp * sp->w();
	  }
      }
  
  /* Free up the alpha matrix */
  for (i = 0; i <= (dirflag ? src->orderU : src->orderV); i++)
    delete [] alpha[i] ;
  delete [] alpha ;
}

/*
 * NurbUtils.c - Code for Allocating, freeing, & copying NURB surfaces.
 *
 * John Peterson
 */


/*
 * Allocate memory for a NURB (assumes numU, numV, orderU
 * and orderV have been set).  If ukv or vkv are not NIL, they
 * are assumed to be pointers to valid knot vectors.
 */

template <class T>
void
AllocNurb( NurbSurface<T> * n, T * ukv, T * vkv )
{
  int i;
  
  if (! ukv)
    n->kvU = new T[n->numU + n->orderU] ;
  else
    n->kvU = ukv;
  if (! vkv)
    n->kvV = new T[n->numV + n->orderV];
  else
    n->kvV = vkv;
  
  n->points.resize(n->numV,n->numU) ;
}

/*
 * Release storage for a patch
 */

template <class T>
void
FreeNurb( NurbSurface<T> * n )
{
  int i;
  
  if (n->kvU) delete [] n->kvU ;
  n->kvU = 0;
  if (n->kvV) delete [] n->kvV ;
  n->kvV = 0;
  delete n ; 
  n = 0 ; 
  // Don't touch render, it might still be used.
}

/*
 * Clone a nurb (deep copy)
 */

template <class T>
void
CloneNurb( NurbSurface<T> * src, NurbSurface<T> * dst )
{
  int i, j;
  T * srcp, *dstp;
  
  //*dst = *src;	/* Copy fields that don't change */
  dst->numU = src->numU ;
  dst->numV = src->numV ;
  dst->orderU = src->orderU ;
  dst->orderV = src->orderV ;
  
  dst->strU0 = src->strU0 ; 
  dst->strUn = src->strUn ; 
  dst->strV0 = src->strV0 ; 
  dst->strVn = src->strVn ; 
  
  dst->kvU = 0;
  dst->kvV = 0;	/* So they get allocated */
  dst->points = 0;
  
  AllocNurb( dst, (T*)0, (T*)0 );
  
  /* Copy kv's */
  srcp = src->kvU;
  dstp = dst->kvU;
  for (i = 0; i < src->numU + src->orderU; i++)
    *dstp++ = *srcp++;
  
  srcp = src->kvV;
  dstp = dst->kvV;
  for (i = 0; i < src->numV + src->orderV; i++)
    *dstp++ = *srcp++;

  /* Copy control points */
  for (i = 0; i < src->numV; i++)
    for (j = 0; j < src->numU; j++)
      dst->points(i,j) = src->points(i,j);
}

/*
 * NurbSubdiv.c - Perform adaptive subdivision on a NURB surface.
 *
 * John Peterson
 *
 */

#define DIVPT( p, dn ) { ((p).x()) /= (dn); ((p).y()) /= (dn); ((p).z()) /= (dn); }

#define maxV(surf) ((surf)->numV-1)
#define maxU(surf) ((surf)->numU-1)

/*
 * Split a knot vector at the center, by adding multiplicity k knots near
 * the middle of the parameter range.  Tries to start with an existing knot,
 * but will add a new knot value if there's nothing in "the middle" (e.g.,
 * a Bezier curve).
 */
template <class T> int
SplitKV( T * srckv,
	 T ** destkv,
	 int * splitPt,    /* Where the knot interval is split */
	 int m, int k )
{
  int i, last;
  int middex, extra, same;	/* "middex" ==> "middle index" */
  T midVal;
  
  extra = 0;
  last = (m + k);
  
  middex = last / 2;
  midVal = srckv[middex];
  
  /* Search forward and backward to see if multiple knot is already there */
  
  i = middex+1;
  same = 1;
  while ((i < last) && (srckv[i] == midVal)) {
    i++;
    same++;
  }
  
  i = middex-1;
  while ((i > 0) && (srckv[i] == midVal)) {
    i--;
    middex--;	    /* middex is start of multiple knot */
    same++;
  }
  
  if (i <= 0)	    /* No knot in middle, must create it */
    {
      midVal = (srckv[0] + srckv[last]) / 2.0;
      middex = last / 2;
      while (srckv[middex + 1] < midVal)
	middex++;
      same = 0;
    }
  
  extra = k - same;
  *destkv = new T[m+k+extra+1];
  
  if (same < k)	    /* Must add knots */
    {
      for (i = 0; i <= middex; i++)
	(*destkv)[i] = srckv[i];
      
      for (i = middex+1; i <= middex+extra; i++)
	(*destkv)[i] = midVal;
      
      for (i = middex + k - same + 1; i <= m + k + extra; i++)
	(*destkv)[i] = srckv[i - extra];
    }
  else
    {
      for (i = 0; i <= m + k; i++)
	(*destkv)[i] = srckv[i];
    }
  
  *splitPt = (extra < k) ? middex - 1 : middex;
  return( extra );
}

/*
 * Given a line defined by firstPt and lastPt, project midPt onto
 * that line.  Used for fixing "cracks".
 */
template <class T> void
ProjectToLine( Point_nD<T,3> * firstPt, Point_nD<T,3> * lastPt, Point_nD<T,3> * midPt )
{
  Point_nD<T,3> base, v0, vm;
  T fraction, denom;
  
  base = *firstPt;
  
  v0 = *lastPt - base ; // (void) V3Sub( lastPt, &base, &v0 );
  vm = *midPt - base ; // (void) V3Sub( midPt, &base, &vm );
  
  denom = norm2(v0) ; // V3SquaredLength( &v0 );
  //fraction = (denom == 0.0) ? 0.0 : (V3Dot( &v0, &vm ) / denom);
  fraction = (denom == 0.0) ? 0.0 : (v0*vm ) / denom;
  
  midPt->x() = base.x() + fraction * v0.x();
  midPt->y() = base.y() + fraction * v0.y();
  midPt->z() = base.z() + fraction * v0.z();
}

/*
 * If a normal has collapsed to zero (normLen == 0.0) then try
 * and fix it by looking at its neighbors.  If all the neighbors
 * are sick, then re-compute them from the plane they form.
 * If that fails too, then we give up...
 */
template <class T> void
FixNormals( SurfSample<T> * s0, SurfSample<T> * s1, SurfSample<T> * s2 )
{
  BOOL goodnorm;
  int i, j, ok;
  T dist;
  SurfSample<T> * V[3];
  Point_nD<T,3> normal;
  
  V[0] = s0; V[1] = s1; V[2] = s2;
  
  /* Find a reasonable normalal */
  for (ok = 0, goodnorm = FALSE_;
       (ok < 3L) && !(goodnorm = (V[ok]->normLen > 0.0)); ok++);
  
  if (! goodnorm)	/* All provided normals are zilch, try and invent one */
    {
      normal.x() = 0.0; normal.y() = 0.0; normal.z() = 0.0;
      
      for (i = 0; i < 3L; i++)
	{
	  j = (i + 1) % 3L;
	  normal.x() += (V[i]->point.y() - V[j]->point.y()) * (V[i]->point.z() + V[j]->point.z());
	  normal.y() += (V[i]->point.z() - V[j]->point.z()) * (V[i]->point.x() + V[j]->point.x());
	  normal.z() += (V[i]->point.x() - V[j]->point.x()) * (V[i]->point.y() + V[j]->point.y());
	}
      //dist = V3Length( &norm );
      dist = norm(normal) ; 
      if (dist == 0.0)
	return;		/* This sucker's hopeless... */
      
      DIVPT( normal, dist );
      
      for (i = 0; i < 3; i++)
	{
	  V[i]->normal = normal;
	  V[i]->normLen = dist;
	}
    }
  else	    /* Replace a sick normal with a healthy one nearby */
    {
      for (i = 0; i < 3; i++)
	if ((i != ok) && (V[i]->normLen == 0.0))
	  V[i]->normal = V[ok]->normal;
    }
  return;
}

/*
 * Normalize the normal in a sample.  If it's degenerate,
 * flag it as such by setting the normLen to 0.0
 */
template <class T> void
AdjustNormal( SurfSample<T> * samp )
{
  // If it's not degenerate, do the normalization now */
  samp->normLen = norm(samp->normal) ; // V3Length( &(samp->normal) );

  if (samp->normLen < samp->epsilon )
    samp->normLen = 0.0;
  else
    DIVPT( (samp->normal), samp->normLen );
}

/*
 * Compute the normal of a corner point of a mesh.  The
 * base is the value of the point at the corner, indU and indV
 * are the mesh indices of that point (either 0 or numU|numV).
 */
template <class T> void
GetNormal( NurbSurface<T> * n, int indV, int indU )
{
  Point_nD<T,3> tmpL, tmpR;	/* "Left" and "Right" of the base point */
  SurfSample<T> * crnr;
  
  if ( (indU && indV) || ((! indU) && (!indV)) )
    {
      if (indU)
	crnr = &(n->cnn);
      else
	crnr = &(n->c00);
      DIVW( &(n->points(indV,(indU ? (indU-1) : 1))), &tmpL );
      DIVW( &(n->points((indV ? (indV-1) : 1),indU)), &tmpR );
    }
  else
    {
      if (indU)
	crnr = &(n->c0n);
      else
	crnr = &(n->cn0);
      DIVW( &(n->points(indV,(indU ? (indU-1) : 1))), &tmpR );
      DIVW( &(n->points((indV ? (indV-1) : 1),indU)), &tmpL );
    }
  
  tmpL -= crnr->point ; //(void) V3Sub( &tmpL, &(crnr->point), &tmpL );
  tmpR -= crnr->point ;//(void) V3Sub( &tmpR, &(crnr->point), &tmpR );
  crnr->normal = crossProduct(tmpL,tmpR); //(void) V3Cross( &tmpL, &tmpR, &(crnr->normal) );
  AdjustNormal( crnr );
}

/*
 * Build the new corners in the two new surfaces, computing both
 * point on the surface aint with the normal.	Prevent cracks that may occur.
 */
template <class T> void
MakeNewCorners( NurbSurface<T> * parent,
		NurbSurface<T> * kid0,
		NurbSurface<T> * kid1,
		BOOL dirflag )
{
  DIVW( &(kid0->points(maxV(kid0),maxU(kid0))), &(kid0->cnn.point) );
  GetNormal( kid0, maxV(kid0), maxU(kid0) );
  
  if (dirflag)
    {
      kid0->strUn = FALSE_;	/* Must re-test new edge straightness */
      
      DIVW( &(kid0->points(0,maxU(kid0))), &(kid0->c0n.point) );
      GetNormal( kid0, 0, maxU(kid0) );
      /*
       * Normals must be re-calculated for kid1 in case the surface
       * was split at a c1 (or c0!) discontinutiy
       */
      kid1->c00.point = kid0->c0n.point;
      GetNormal( kid1, 0, 0 );
      kid1->cn0.point = kid0->cnn.point;
      GetNormal( kid1, maxV(kid1), 0 );
      
      /*
       * Prevent cracks from forming by forcing the points on the seam to
       * lie aint any straight edges.  (Must do this BEFORE finding normals)
       */
      if (parent->strV0)
	ProjectToLine( &(parent->c00.point),
		       &(parent->c0n.point),
		       &(kid0->c0n.point) );
      if (parent->strVn)
	ProjectToLine( &(parent->cn0.point),
		       &(parent->cnn.point),
		       &(kid0->cnn.point) );
      
      kid1->c00.point = kid0->c0n.point;
      kid1->cn0.point = kid0->cnn.point;
      kid1->strU0 = FALSE_;
    }
  else
    {
      kid0->strVn = FALSE_;
      
      DIVW( &(kid0->points(maxV(kid0),0)), &(kid0->cn0.point) );
      GetNormal( kid0, maxV(kid0), 0 );
      kid1->c00.point = kid0->cn0.point;
      GetNormal( kid1, 0, 0 );
      kid1->c0n.point = kid0->cnn.point;
      GetNormal( kid1, 0, maxU(kid1) );
      
      if (parent->strU0)
	ProjectToLine( &(parent->c00.point),
		       &(parent->cn0.point),
		       &(kid0->cn0.point) );
      if (parent->strUn)
	ProjectToLine( &(parent->c0n.point),
		       &(parent->cnn.point),
		       &(kid0->cnn.point) );
      
      kid1->c00.point = kid0->cn0.point;
      kid1->c0n.point = kid0->cnn.point;
      kid1->strV0 = FALSE_;
    }
}

/*
 * Split a surface into two halves.  First inserts multiplicity k knots
 * in the center of the parametric range.  After refinement, the two
 * resulting surfaces are copied into separate data structures.	 If the
 * parent surface had straight edges, the points of the children are
 * projected onto those edges.
 */
template <class T> void
SplitSurface( NurbSurface<T> * parent,
	      NurbSurface<T> * kid0, NurbSurface<T> * kid1,
	      BOOL dirflag )	    /* If TRUE_ subdivided in U, else in V */
{
  NurbSurface<T> *tmp;
  T * newkv;
  int i, j, splitPt;
  
  tmp = new NurbSurface<T> ;

  //
  // Add a multiplicty k knot to the knot vector in the direction
  // specified by dirflag, and refine the surface.  This creates two
  // adjacent surfaces with c0 discontinuity at the seam.
  //

  //tmp = *parent;	// Copy order, # of points, etc. 
  tmp->numU = parent->numU ;
  tmp->numV = parent->numV ;
  tmp->orderU = parent->orderU ;
  tmp->orderV = parent->orderV ;
  
  tmp->strU0 = parent->strU0 ; 
  tmp->strUn = parent->strUn ; 
  tmp->strV0 = parent->strV0 ; 
  tmp->strVn = parent->strVn ; 
  
  tmp->render = parent->render ; 
  
  if (dirflag)
    {
      tmp->numU = parent->numU + SplitKV( parent->kvU,
					 &newkv,
					 &splitPt,
					 maxU(parent),
					 parent->orderU );
      AllocNurb( tmp, newkv, (T*)0 );
      for (i = 0; i < tmp->numV + tmp->orderV; i++)
	tmp->kvV[i] = parent->kvV[i];
    }
  else
    {
      tmp->numV = parent->numV + SplitKV( parent->kvV,
					 &newkv,
					 &splitPt,
					 maxV(parent),
					 parent->orderV );
      AllocNurb( tmp, (T*)0, newkv );
      for (i = 0; i < tmp->numU + tmp->orderU; i++)
	tmp->kvU[i] = parent->kvU[i];
    }
  RefineSurface( parent, tmp, dirflag );
  
  //
  // Build the two child surfaces, and copy the data from the refined
  // version of the parent (tmp) into the two children
  //

  // First half 
  
  //    *kid0 = *parent;	// copy various edge flags and orders 
  kid0->orderU = parent->orderU ;
  kid0->orderV = parent->orderV ;
  
  kid0->strU0 = parent->strU0 ; 
  kid0->strUn = parent->strUn ; 
  kid0->strV0 = parent->strV0 ; 
  kid0->strVn = parent->strVn ; 
  
  kid0->c00 = parent->c00 ;
  kid0->c0n = parent->c0n ;
  kid0->cn0 = parent->cn0 ;
  kid0->cnn = parent->cnn ;
  
  kid0->render = parent->render ; 
  
  kid0->numU = dirflag ? splitPt+1 : parent->numU;
  kid0->numV = dirflag ? parent->numV : splitPt+1;
  kid0->kvU = kid0->kvV = 0;
  AllocNurb( kid0, (T*)0, (T*)0 );
  
  for (i = 0; i < kid0->numV; i++)	// Copy the point and kv data 
    for (j = 0; j < kid0->numU; j++)
      kid0->points(i,j) = tmp->points(i,j) ;
  for (i = 0; i < kid0->orderU + kid0->numU; i++)
    kid0->kvU[i] = tmp->kvU[i];
  for (i = 0; i < kid0->orderV + kid0->numV; i++)
    kid0->kvV[i] = tmp->kvV[i];
  
  // Second half 
  
  splitPt++;
  //*kid1 = *parent;
  kid1->orderU = parent->orderU ;
  kid1->orderV = parent->orderV ;
  
  kid1->strU0 = parent->strU0 ; 
  kid1->strUn = parent->strUn ; 
  kid1->strV0 = parent->strV0 ; 
  kid1->strVn = parent->strVn ; 

  kid1->c00 = parent->c00 ;
  kid1->c0n = parent->c0n ;
  kid1->cn0 = parent->cn0 ;
  kid1->cnn = parent->cnn ;
  
  kid1->render = parent->render ; 
  
  kid1->numU = dirflag ? tmp->numU - splitPt : parent->numU;
  kid1->numV = dirflag ? parent->numV : tmp->numV - splitPt;
  kid1->kvU = kid1->kvV = 0;
  AllocNurb( kid1, (T*)0, (T*)0 );
  
  for (i = 0; i < kid1->numV; i++)	// Copy the point and kv data 
    for (j = 0; j < kid1->numU; j++)
      kid1->points(i,j)
	= tmp->points(dirflag ? i: (i + splitPt) ,dirflag ? (j + splitPt) : j);
  for (i = 0; i < kid1->orderU + kid1->numU; i++)
    kid1->kvU[i] = tmp->kvU[dirflag ? (i + splitPt) : i];
  for (i = 0; i < kid1->orderV + kid1->numV; i++)
    kid1->kvV[i] = tmp->kvV[dirflag ? i : (i + splitPt)];
    
  // Construct new corners on the boundry between the two kids 
  MakeNewCorners( parent, kid0, kid1, dirflag );
  
  FreeNurb( tmp );	    // Get rid of refined parent 
    
}

/*
 * Test if a particular row or column of control points in a mesh
 * is "straight" with respect to a particular tolerance.  Returns TRUE_
 * if it is.
 */

#define GETPT( i )  (( dirflag ? (n->points(crvInd,i)) : (n->points(i,crvInd)) ))

#define EPSILON n->epsilon

template <class T> BOOL
IsCurveStraight( NurbSurface<T> * n,
		 T tolerance,
		 int crvInd,
		 BOOL dirflag )  /* If TRUE_, test in U direction, else test in V */
{
  Point_nD<T,3> p, vec, prod;
  Point_nD<T,3> cp, e0;
  int i, last;
  T linelen, dist;
  
  /* Special case: lines are automatically straight. */
  if ((dirflag ? n->numU : n->numV) == 2)
    return( TRUE_ );
  
  last = (dirflag ? n->numU : n->numV) - 1;
  n->render->screenProject( GETPT( 0 ), e0 );
  
  /* Form an initial line to test the other points against (skiping degen lines) */
  
  linelen = 0.0;
  for (i = last; (i > 0) && (linelen < EPSILON); i--)
    {
      n->render->screenProject( GETPT( i ), cp );
      vec = cp - e0 ; 
      linelen = norm(vec) ;
    }
  
  DIVPT( vec, linelen );

  if (linelen > EPSILON)	/* If no non-degenerate lines found, it's all degen */
    for (i = 1; i <= last; i++)
      {
	/* The cross product of the vector defining the
	 * initial line with the vector of the current point
	 * gives the distance to the line. */
	n->render->screenProject( GETPT( i ), cp );
	p = cp - e0 ; 
	
	prod = crossProduct(p,vec) ; 
	dist = norm(prod) ; 
	
	if (dist > tolerance)
	  return( FALSE_ );
      }
  
  return( TRUE_ );
}

/*
 * Check to see if a surface is flat.  Tests are only performed on edges and
 * directions that aren't already straight.  If an edge is flagged as straight
 * (from the parent surface) it is assumed it will stay that way.
 */
template <class T> BOOL
TestFlat( NurbSurface<T> * n, T tolerance )
{
  int i;
  BOOL straight;
  Point_nD<T,3> cp00, cp0n, cpn0, cpnn, planeEqn;
  T dist,d ;
  
  /* Check edge straightness */
  
  if (! n->strU0)
    n->strU0 = IsCurveStraight( n, tolerance, 0, FALSE_ );
  if (! n->strUn)
    n->strUn = IsCurveStraight( n, tolerance, maxU(n), FALSE_ );
  if (! n->strV0)
    n->strV0 = IsCurveStraight( n, tolerance, 0, TRUE_ );
  if (! n->strVn)
    n->strVn = IsCurveStraight( n, tolerance, maxV(n), TRUE_ );
  
  /* Test to make sure control points are straight in U and V */
  
  straight = TRUE_;
  if ( (! n->flatU) && (n->strV0) && (n->strVn) )
    for (i = 1;
	 (i < maxV(n)) && (straight = IsCurveStraight( n, tolerance, i, TRUE_ ));
	 i++);
  
  if (straight && n->strV0 && n->strVn)
    n->flatU = TRUE_;
  
  straight = TRUE_;
  if ( (! n->flatV) && (n->strU0) && (n->strUn) )
    for (i = 1;
	 (i < maxU(n)) && (straight = IsCurveStraight( n, tolerance, i, FALSE_ ));
	 i++);
  
  if (straight && n->strU0 && n->strUn)
    n->flatV = TRUE_;
  
  if ( (! n->flatV) || (! n->flatU) )
    return( FALSE_ );
  
  // The surface can pass the above tests but still be twisted. 
  
  n->render->screenProject( (n->points(0,0)),	    cp00 );
  n->render->screenProject( (n->points(0,maxU(n))),	    cp0n );
  n->render->screenProject( (n->points(maxV(n),0)),	    cpn0 );
  n->render->screenProject( (n->points(maxV(n),maxU(n))),  cpnn );
  
  cp0n -= cp00 ; // Make edges into vectors
  cpn0 -= cp00 ; 
  
  
  // Compute the plane equation from two adjacent sides, and
  // measure the distance from the far point to the plane.  If it's
  // larger than tolerance, the surface is twisted.
  
  planeEqn = crossProduct(cpn0,cp0n) ; 
  planeEqn = planeEqn.unitLength() ; // Normalize to keep adds in sync w/ mults 
  
  d = planeEqn * cp00 ; 
  dist = fabs( ( planeEqn * cpnn ) - d );
  
  if ( dist > tolerance ) // Surface is twisted 
    return( FALSE_ );
  else
    return( TRUE_ );
}

/*
 * Turn a sufficiently flat surface into triangles.
 */
template <class T> 
void EmitTriangles( NurbSurface<T> * n )
{
  Point_nD<T,3> vecnn, vec0n;	// Diagonal vectors 
  T len2nn, len20n;		// Diagonal lengths squared 
  T u0, un, v0, vn;		// Texture coords;
				   
  //
  // Measure the distance aint the two diagonals to decide the best
  // way to cut the rectangle into triangles.
  //
  
  vecnn = n->c00.point - n->cnn.point ; 
  vec0n = n->c0n.point - n->cn0.point ; 
  
  len2nn = norm2(vecnn) ; 
  len20n = norm2(vec0n) ; 
  
  if (maximum(len2nn, len20n) < n->epsilon)
    return;				// Triangles are too small to render 
  
  //
  // Assign the texture coordinates
  //
  u0 = n->kvU[n->orderU-1];
  un = n->kvU[n->numU];
  v0 = n->kvV[n->orderV-1];
  vn = n->kvV[n->numV];
  n->c00.u = u0; n->c00.v = v0;
  n->c0n.u = un; n->c0n.v = v0;
  n->cn0.u = u0; n->cn0.v = vn;
  n->cnn.u = un; n->cnn.v = vn;
  
  //
  // If any normals are sick, fix them now.
  //
  if ((n->c00.normLen == 0.0) || (n->cnn.normLen == 0.0) || (n->cn0.normLen == 0.0))
    FixNormals( &(n->c00), &(n->cnn), &(n->cn0) );
  if (n->c0n.normLen == 0.0)
    FixNormals( &(n->c00), &(n->c0n), &(n->cnn) );
  
  if ( len2nn < len20n )
    {
      n->render->drawTriangle( n->c00, n->cnn, n->cn0 );
      n->render->drawTriangle( n->c00, n->c0n, n->cnn );
    }					 	 
  else				 	 
    {					 	 
      n->render->drawTriangle( n->c0n, n->cnn, n->cn0 );
      n->render->drawTriangle( n->c0n, n->cn0, n->c00 );
    }
}

/*
 * The recursive subdivision algorithm.	 Test if the surface is flat.
 * If so, split it into triangles.  Otherwise, split it into two halves,
 * and invoke the procedure on each half.
 */
template <class T> void
DoSubdivision( NurbSurface<T> * n, T tolerance, BOOL dirflag, int level )
{
  NurbSurface<T> *left, *right;
  
  left = new NurbSurface<T>;
  right = new NurbSurface<T>;

  if (TestFlat( n, tolerance ))
    {
      EmitTriangles( n );
    }
  else
    {
      if ( ((! n->flatV) && (! n->flatU)) || ((n->flatV) && (n->flatU)) )
	dirflag = !dirflag;    // If twisted or curved in both directions, 
      else		       // then alternate subdivision direction 
	{
	  if (n->flatU)	       // Only split in directions that aren't flat 
	    dirflag = FALSE_;
	  else
	    dirflag = TRUE_;
	}
      SplitSurface( n, left, right, dirflag );
      DoSubdivision( left, tolerance, dirflag, level + 1 );
      DoSubdivision( right, tolerance, dirflag, level + 1 );
      FreeNurb( left );
      FreeNurb( right );
    }
}

/*
 * Main entry point for subdivision */
template <class T> void
DrawSubdivision( NurbSurface<T> * surf, T tolerance )
{
  surf->flatV = FALSE_;
  surf->flatU = FALSE_;
  surf->strU0 = FALSE_;
  surf->strUn = FALSE_;
  surf->strV0 = FALSE_;
  surf->strVn = FALSE_;
  
  //
  // Initialize the projected corners of the surface
  // and the normals.
  //
  DIVW( &(surf->points(0,0)), &surf->c00.point );
  DIVW( &(surf->points(0,surf->numU-1)), &surf->c0n.point );
  DIVW( &(surf->points(surf->numV-1,0)), &surf->cn0.point );
  DIVW( &(surf->points(surf->numV-1,surf->numU-1)), &surf->cnn.point );

  GetNormal( surf, 0, 0 );
  GetNormal( surf, 0, maxU(surf) );
  GetNormal( surf, maxV(surf), 0 );
  GetNormal( surf, maxV(surf), maxU(surf) );
  
  RenderMesh<T> *render ;

  render = surf->render ;
  render->drawHeader();
  DoSubdivision( surf, tolerance, TRUE_, 0 );
  // Note surf is deallocated by the subdivision process 
  render->drawFooter();
}

/*!
  \brief the copy operator

  \param s the surface sample to copy

  \author Philippe Lavoie
  \date 20 January 1999
*/
template <class T>
SurfSample<T>& SurfSample<T>::operator=(const SurfSample<T>& s) {
  point = s.point ;
  normal = s.normal ;
  normLen = s.normLen ;
  u = s.u ;
  v = s.v ;
  return *this ;
}

/*
template <class T>
class NurbsCurveTess : public NurbsCurve<T,2> {
public:
  NurbsCurveTess(const NurbsSurface<T,3>& rs) ;

  void tesselate(T tol) ; 

protected:
  int isStraight() ; 

  const NurbsSurface<T,3>& rsurf ; 
  T tolerance ; 
}

template <class T>
NurbsCurveTess::NurbsCurveTess(const NurbsSurface<T,3>& rs) : rsurf(rs) {
  tolerance = 0.01 ; 
}

template <class T>
void NurbsCurveTess::tesselate(T tol){
  tolerance = tol ; 
  if(knot.n() != 2*(deg_+1)){
    
  }
}
*/

}
