/*=============================================================================
        File: nurbsSub.h
     Purpose:       
    Revision: $Id: nurbsSub.h,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (20 January, 1999)
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

#ifndef _NURBS_SURFACE_SUBDIVISION_
#define _NURBS_SURFACE_SUBDIVISION_

#include "nurbsS.h"
#include <vector>
#include <iostream>

/*!
 */
namespace PLib {
  
  /*!
    \class SurfSample nurbsSub.h
    \brief A class to represent a NURBS surface sample
    
    A sample point from a surface adds information that are 
    usefull for output routines: the value, the normal, and 
    the texture mapping parametric value.
    
    This class is based on code from the article "Tessellation of
    NURB Surfaces" by John W. Peterson, jp@blowfish.taligent.com in
    "Graphics Gems IV", Academic Press, 1994
    
    \author Philippe Lavoie
    \date 20 January, 1999 
  */
  template <class T>
    struct SurfSample {
      Point_nD<T,3> point ; //!< point on surface
      Point_nD<T,3> normal ; //!< normal at that point
      T normLen ; //!< used for normalizing normals
      T u,v ; //!< parameters used for texture mapping

      SurfSample<T>& operator=(const SurfSample<T>& s) ;

      SurfSample() : normLen(-1),u(0),v(0) {;}

      static T epsilon ;
    };



  /*!
    \class RenderMesh nurbsSub.h nurbs/nurbsSub.h
    \brief a virtual mesh renderer

    The mesh renderer is used by the NurbsSubSurface class to
    perform the writing of a triangle on the screen or on a file.

    \author Philippe Lavoie
    \date 20 January, 1999     
  */
  template <class T>
    class RenderMesh {
    public:
      //virtual ~RenderMesh() = 0 ; 
      virtual void drawHeader() = 0;
      virtual void drawTriangle(const SurfSample<T>&, const SurfSample<T>&, const SurfSample<T>&) = 0;
      virtual void drawFooter() = 0;
      virtual void screenProject(const HPoint_nD<T,3> &worldPt, Point_nD<T,3> &screenPt ) = 0 ; 
    };


  template <class T> class NurbSurface ;

  /*!
    \class NurbsSubSurface nurbsSub.h
    \brief A class to represent a NURBS surface suitable for subdivision
    
    This class adds the methods and the information necessary for
    performing subdivision on the surface.
    
    Subdivision is mainly used to output the surface in diverse formats 
    such as VRML, Post-Sript or a mesh file.
    
    This class is based on code from the article "Tessellation of
    NURB Surfaces" by John W. Peterson, jp@blowfish.taligent.com in
    "Graphics Gems IV", Academic Press, 1994
    
    \author Philippe Lavoie
    \date 20 January, 1999 
  */
  template <class T>
    struct NurbsSubSurface  {
    public:
      NurbsSubSurface(const NurbsSurface<T,3>& s) ;
      ~NurbsSubSurface() ;

      void drawSubdivisionPS(ostream& os, T tolerance) ;
      void drawSubdivisionPS(const char* f, T tolerance) ;

      void drawSubdivisionVRML(ostream& os, T tolerance, const Color& col=Color(0,0,255)) ;
      void drawSubdivisionVRML(const char* f, T tolerance, const Color& col=Color(0,0,255)) ;

      void drawSubdivisionVRML97(ostream& os, T tolerance, const Color& col=Color(0,0,255)) ;
      void drawSubdivisionVRML97(const char* f, T tolerance, const Color& col=Color(0,0,255)) ;

      //void drawSubdivisionPoints(deque<Point_nD<T,3> > &pnts, T tolerance) ;
      void drawSubdivisionPoints(BasicArray<Point_nD<T,3> > &pnts, T tolerance) ;
      void drawSubdivisionPoints(T tolerance) ;

    protected:

      void drawSubdivision(T tolerance) ;
      void initSurf() ;
      RenderMesh<T> *render;
      const NurbsSurface<T,3> &rsurf ; 
      NurbSurface<T> *surf ;
   };
  

  /*!
    \class RenderMeshPS nurbsSub.h
    \brief a mesh renderer for PS files

    \author Philippe Lavoie
    \date 20 January, 1999     
  */
  template <class T>
    class RenderMeshPS : public RenderMesh<T> {
    public:
      RenderMeshPS(ostream& os): out(os) {;}
      virtual ~RenderMeshPS() {;}
      virtual void drawHeader() ;
      virtual void drawTriangle( const SurfSample<T> &v0, const SurfSample<T> &v1, const SurfSample<T> & v2 );
      void drawLine( const SurfSample<T> &v0, const SurfSample<T> &v1);
      virtual void drawFooter() ;
      virtual void screenProject(const HPoint_nD<T,3> &worldPt, Point_nD<T,3> &screenPt ) ; 
    protected:
      ostream& out ;
    };


  //typedef deque<int> IndexSetVector ;

  /*!
    \class RenderMeshVRML nurbsSub.h
    \brief a mesh renderer for VRML files

    \author Philippe Lavoie
    \date 20 January, 1999     
  */
  template <class T>
    class RenderMeshVRML : public RenderMesh<T> {
    public:
      RenderMeshVRML(ostream& os,const Color& col): out(os), color(col) {;}
      virtual ~RenderMeshVRML() { ; }
      virtual void drawHeader() ;
      virtual void drawTriangle( const SurfSample<T> &v0, const SurfSample<T> &v1, const SurfSample<T> & v2 );
      virtual void drawFooter() ;
      virtual void screenProject(const HPoint_nD<T,3> &worldPt, Point_nD<T,3> &screenPt ) ; 
    protected:
      int size ;
      ostream &out ;
      Color color ; 
    };

  /*!
    \class RenderMeshVRML97 nurbsSub.h
    \brief a mesh renderer for VRML files

    \author Philippe Lavoie
    \date 20 January, 1999     
  */
  template <class T>
    class RenderMeshVRML97 : public RenderMesh<T> {
    public:
      RenderMeshVRML97(ostream& os,const Color& col): out(os), color(col) { init = 1 ;}
      virtual ~RenderMeshVRML97() { ; }
      virtual void drawHeader() ;
      virtual void drawTriangle( const SurfSample<T> &v0, const SurfSample<T> &v1, const SurfSample<T> & v2 );
      virtual void drawFooter() ;
      virtual void screenProject(const HPoint_nD<T,3> &worldPt, Point_nD<T,3> &screenPt ) ; 
    protected:
      int size ;
      ostream &out ;
      Color color ; 
      Point_nD<T,3> p_min,p_max ; 
      int init ;
    };

  /*!
    \class RenderMeshPoints nurbsSub.h
    \brief a mesh renderer to a vector of points

    The triangle points are written the the vector specified in the
    constructor call. The points composing the triangle \a n are at
    3n, 3n+1 and 3n+2 in the vector.

    \author Philippe Lavoie
    \date 20 January, 1999 */
  template <class T>
    class RenderMeshPoints : public RenderMesh<T> {
    public:
      //RenderMeshPoints(deque<Point_nD<T,3> >& pts): points(pts) {;}
      RenderMeshPoints(BasicArray<Point_nD<T,3> >& pts): points(pts) {;}
      virtual ~RenderMeshPoints() {; }
      virtual void drawHeader() ;
      virtual void drawTriangle( const SurfSample<T> &v0, const SurfSample<T> &v1, const SurfSample<T> & v2 );
      virtual void drawFooter() ;
      virtual void screenProject(const HPoint_nD<T,3> &worldPt, Point_nD<T,3> &screenPt ) ; 
    protected:
      //deque<Point_nD<T,3> > &points ;
      //vector<Point_nD<T,3> > &points ;
      BasicArray<Point_nD<T,3> > &points;
    };


  

} // end namespace


#endif
