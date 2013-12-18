/*=============================================================================
        File: nurbsGL.H
     Purpose: Describes all OpenGL related classes.
    Revision: $Id: nurbsGL.h,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (28 September, 1997)
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
#ifndef _nurbs_nurbsGL_h_
#define _nurbs_nurbsGL_h_

#include "nurbs_global.h"
#include <list>

#ifdef WITH_OPENGL

#include "nurbs_sp.h"
#include "nurbsS_sp.h"
#include "hnurbsS_sp.h"
#include "nurbsSub.h"
#include <GL/gl.h>
#include <GL/glu.h>

/*!
 */
namespace PLib {

enum ObjectGLState {hideState=0, objectState, selectedState, currentState, selected_currentState };
//enum NurbsGLState { showCurve=0, showCPoints, showBbox } ;

extern Color objectColorDefault ;
extern Color selectColorDefault ;
extern Color currentColorDefault ;

extern Color axisXColorDefault ;
extern Color axisYColorDefault ;
extern Color axisZColorDefault ;

extern Color cpointColorDefault ;
extern Color cpoint0ColorDefault ;
extern Color cPolygonColorDefault ;
extern Color cpointActiveColorDefault ;

extern Color knotColorDefault ;
extern Color knotActiveColorDefault ;

extern ObjectGLState objectStateDefault ;

extern int pSizeDefault ;

extern const int NURBS_FLAGS_AFFECT_ALL ;
extern const int NURBS_FLAGS_AFFECT_ACTIVE_ONLY  ;
extern const int NURBS_FLAGS_AFFECT_SELECTED_ONLY  ;
extern const int NURBS_FLAGS_VIEW_BBOX ; 
		 
extern const int NURBS_DISPLAY_NORMAL ;
extern const int NURBS_DISPLAY_ISOCURVES ;
extern const int NURBS_DISPLAY_SHADED ;
extern const int NURBS_DISPLAY_HIDDEN_LINES ; 
extern const int NURBS_DISPLAY_TESSELATION ;
		 
extern const int NURBSLIST_DELETE_AT_RESET ;
extern const int NURBSLIST_KEEP_AT_RESET ;

extern int NurbsDisplayMode ;
extern float tessel_tolerance ;

extern HPoint3Df dummyCPoint ;

extern Point3Df XAxis_3D ;
extern Point3Df YAxis_3D ;
extern Point3Df ZAxis_3D ;

 struct NurbsSubSurfaceGL : public NurbsSubSurface<float> {
   NurbsSubSurfaceGL(const NurbsSurface<float,3>& s, const Color& c) : NurbsSubSurface<float>(s), color(c) {;}
   ~NurbsSubSurfaceGL() {;}
   void drawSubdivisionGL(float tolerance) ;
 protected:
   Color color ; 
 };


/*!
  \fn void glColor(const Color& color)
  \brief Calls glColor3f

  Calls glColor3f with the proper arguments
  
  \param color  the color to use	       

  \author Philippe Lavoie 
  \date 23 September 1997
*/
inline void glColor(const Color& color){
   glColor3f(GLfloat(color.r)/255.0,GLfloat(color.g)/255.0,GLfloat(color.b)/255.0) ;
}

/*!
  \fn void colorToMaterial(const Color& color, GLfloat* c, GLfloat alpha=1.0)
  \brief Stores the color information

  Stores the color information inside a GLfloat vector.

  \param color  the color to use
  \param c  the GLfloat vector
  \param alpha  the alpha value [1.0]

  \author Philippe Lavoie 
  \date 23 September 1997
*/
inline void colorToMaterial(const Color& color, GLfloat* c, GLfloat alpha=1.0){
  c[0] = GLfloat(color.r)/255.0 ;
  c[1] = GLfloat(color.g)/255.0 ;
  c[2] = GLfloat(color.b)/255.0 ;
  c[3] = alpha ;
}

/*!
  \struct RGBAf nurbsGL.h nurbs/nurbsGL.h
  \brief a class to hold rgba floating point values

  This holds the red, green, blue and alpha floting point
  values. These informations are neede by some renderer.

  \author Philippe Lavoie
  \date 28 September 1997
*/
struct RGBAf {
  float r,g,b,a ;

  RGBAf(){ r=g=b=a=0;}
  RGBAf(const RGBAf& c) { r = c.r ; g =c.g ; b=c.b ; a=c.a; }
  RGBAf(const Color& c) { r = (float)c.r/255.0 ; g = (float)c.g/255.0 ; b = (float)c.b/255.0 ; a = 1 ; }

};

/*!
  \brief a class for the material properties of an object.

  This holds the information about the material properties
  of the object. This is a long list, since it has properties
  which are usefull for more than one rendering method.
  
  You should not use this class yet. It is not used by any 
  other class.

  \author Philippe Lavoie
  \date 28 September 1997
*/
struct Material {
  Material() ; 
  // For OpenGL rendering
  RGBAf frontAmbient, backAmbient ;
  RGBAf frontDiffuse, backDiffuse ;
  RGBAf frontSpecular, backSpecular ;
  RGBAf frontEmission, backEmission ;
  float frontShininess, backShininess ;
  void glMaterial() ;
  
  // for POV-RAY rendering 
  // pigment options
  RGBAf pigment ;
  double pigment_transfer ;
  double pigment_transmit ;
  char *pigment_userdefined ;
  // normal options
  double bump,bump_scale ;
  char *normal_userdefined ;
  // finish options
  Color ambient ;
  double diffuse,brilliance,phong,specular,roughness,metallic ;
  Color reflection ;
  double refraction,ior,caustics,fade_distance,fade_power ;
  double irid_thick ;
  Vector< Point3Df > irid_turbulence ;
  double crand ;

  // for RIB rendering
  char* material ;
};

/*!
  \brief The base class for OpenGL objects

  This is the base virtual class for objects which can be
  displayed using OpenGL.
  
  The class contains informations regarding the color of
  the object and the state of the object (selected, active,
  displayed, hidden). It also associates a color for each 
  state.
  
  Information about local transformations which should be 
  applied to the objects are also given.

  \author Philippe Lavoie
  \date 28 September 1997
*/
class ObjectGL {
public:
  ObjectGL() ; 
  virtual ~ObjectGL() ;

  ObjectGL& operator=(const ObjectGL& a) ;

  Color objectColor ;
  Color selectColor ;
  Color currentColor ;
  GLfloat *materialColor ;
  

  void setObjectColor(const Color& c) { objectColor = c ; }
  void setSelectColor(const Color& c) { selectColor = c ; }
  void setCurrentColor(const Color& c) { currentColor = c ; }

  void glSelectColor() const { glColor(selectColor) ; colorToMaterial(selectColor,materialColor) ;}
  void glObjectColor() const { glColor(objectColor) ; colorToMaterial(objectColor,materialColor) ;}
  void glCurrentColor() const { glColor(currentColor) ; colorToMaterial(currentColor,materialColor) ;}

  void hideObject() { state = hideState ; }
  void selectObject() { state = selectedState ; }
  void viewObject() { state = objectState ; }
  void currentObject() { state = currentState ; }

  virtual void glColor() const 
    { if(isActive())
        glColor(currentColor) ;
        else{
	  if(isSelected()) glColor(selectColor) ; 
	  else glColor(objectColor) ; }
    }
                             

  void glColor(const Color& c) const {
    colorToMaterial(Color(0,0,0),materialColor) ;
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,materialColor) ;
    colorToMaterial(c,materialColor) ;
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,materialColor) ;
    //glColor3f(c) ;
  }

  virtual void glObject() const = 0 ;  

  enum ObjectCategory { badType, nurbsType, pointType, vectorType, listType };
  enum ObjectType {badObject,curveObject,surfaceObject,pointObject,cpointObject,cpointPolygonObject,bboxObject,vectorObject,listObject,hSurfObject,hcpointObject,pointListObject,spointObject} ;
  ObjectType type ;
  ObjectCategory category ;


  // object tranformation information
  GLfloat tx,ty,tz ;
  GLfloat rx,ry,rz ;
  GLfloat sx,sy,sz ;

  virtual void glTransform() const ;

  virtual void display() const { if(state) {glPushMatrix() ; glTransform() ; glObject(); glPopMatrix() ;} }
  virtual void displayList()  { if(state) {glPushMatrix() ; glTransform() ; glColor(); glCallList(callListId); glPopMatrix();} }
  virtual void displayName() { ; }
  
  virtual void glNewList() ; 
  int callListId ; // This is assigned by the user, NOT the class

  // Information for the link list node
  virtual ObjectGL*& previous() { return prev_ ;}
  virtual ObjectGL*& next() { return next_ ;}
  virtual ObjectGL* previous() const { return prev_ ;}
  virtual ObjectGL* next() const { return next_ ;}

  ObjectGLState getState() const { return state ; }

  virtual void select() { selected = 1 ; }
  virtual void deselect() { selected = 0 ; }
  int isSelected() const { return selected ; }

  virtual void activate() { active = 1 ; }
  virtual void deactivate() { active = 0 ; }
  int isActive() const { return active ; }

  virtual ObjectGL* copy() { return 0;} 
  virtual void applyTransform() { ;} 

  // I/O functions
  virtual int read(const char* filename) ;
  virtual int write(const char* filename) const ;
  virtual int writeRIB(const char* filename) const {return 1; }
  virtual int writePOVRAY(const char* filename) const {return 1; }
  virtual int read(ifstream &fin) ;
  virtual int write(ofstream &fout) const ;
  virtual int writeRIB(ofstream &fout) const { return 1; }
  virtual int writePOVRAY(ofstream &fout) const { return 1; }

  // Info functions
  void setName(const char* n) ;
  char* name() const { return name_ ; }
  char* typeName() const ;

protected:
  int selected,active ;
  ObjectGLState state ;  
  ObjectGL *prev_,*next_ ;
  char* name_ ; 
};


/*!
  \brief A reference object for OpenGL

  This class reference other OpenGL objects.

  \author Philippe Lavoie
  \date 28 September 1997
*/
class ObjectRefGL : public ObjectGL {
public:
  ObjectRefGL(ObjectGL* p) : ptr(ptr_) { ptr_ = p ; }

  ObjectGL* &ptr ;
  void glObject() const { ptr->glObject(); }

protected:
  ObjectGL* ptr_ ;
};

/*!
  \brief A link list of ObjectGL

  \author Philippe Lavoie
  \date 28 September 1997
*/
class ObjectListGL : public ObjectGL {
 public:
  ObjectListGL() ;
  virtual ~ObjectListGL() ;
  
  ObjectGL* first() const { return first_ ; }
  ObjectGL* last() const { return last_ ; }
  ObjectGL* current() const { return current_ ; }
  ObjectGL*& first() { return first_ ; }
  ObjectGL*& last() { return last_ ; }
  ObjectGL*& current() { return current_ ; }

  virtual void glObject() const ;
  virtual void display() const ;
  virtual void displayName() const ;

  void reset() ;
  void setResetMode(int m) { resetMode = m ; }
  void add(ObjectGL* obj) ;
  ObjectGL* remove(ObjectGL* obj) ;
  // void insertAfter(ObjectGL* obj) ;
  // void insertBefore(ObjectGL* obj) ;

  void activate() ;
  void deactivate() ;

  void select() ;
  void deselect() ;

  void transformTo(GLfloat x, GLfloat y, GLfloat z, GLfloat a, GLfloat b, GLfloat c, GLfloat sx, GLfloat sy, GLfloat sz, int behavior=NURBS_FLAGS_AFFECT_ALL) ;
  void transformBy(GLfloat x, GLfloat y, GLfloat z, GLfloat a, GLfloat b, GLfloat c, GLfloat sx, GLfloat sy, GLfloat sz, int behavior=NURBS_FLAGS_AFFECT_ALL) ;

  ObjectGL* goTo(int a) ;
  ObjectGL* goToActive(int a) ;

  ObjectGL* goToNext() ; 
  ObjectGL* goToPrevious() ; 

  ObjectGL* goToNextActive() ; 
  ObjectGL* goToPreviousActive() ; 

  ObjectGL* jumpToNext() ;
  ObjectGL* jumpToPrevious() ;

  void setJumpSize(int a) {if(a>0) jumpSize=a ; }

  int size() const { return  n; }

  virtual int read(const char* filename) ;
  virtual int write(const char* filename) const ;
  virtual int writeRIB(const char* filename) const ;
  virtual int writePOVRAY(const char* filename) const ;

  void viewAllObjects() ;
  void hideAllObjects() ;

protected:
  ObjectGL *first_, *last_ ;
  ObjectGL *current_ ;
  int jumpSize ;
  int n ;
  int resetMode ;
};

/*!
  \brief A link list of ObjectRefListGL

  \author Philippe Lavoie
  \date 28 September 1997
*/
class ObjectRefListGL : public ObjectListGL{
public:
  ObjectRefListGL() : ObjectListGL() { resetMode = NURBSLIST_KEEP_AT_RESET ; }
  ~ObjectRefListGL() ;

  void add(ObjectGL* obj) { ObjectRefGL *t = new ObjectRefGL(obj) ; ObjectListGL::add(t) ; }
  ObjectGL* remove(ObjectGL* obj) ;

  void refList(const ObjectListGL* list, int addOnce=1) ;

};


/*!
  \brief A class to hold a list of points.

  \author Philippe Lavoie
  \date 29 Mars 1998
*/
class PointListGL : public ObjectGL {
public:
  PointListGL() :  ObjectGL() { category = pointType ; type = pointListObject ; psize = pSizeDefault ; }
  PointListGL(const PointListGL &pl);
  PointListGL(const BasicList<Point3Df> &l) ;
  virtual void glObject() const ;
  void setPsize(int s) {psize = s ;}

  int read(const char*f) { return ObjectGL::read(f); }
  int write(const char* f) const { return ObjectGL::write(f); }

  int read(ifstream &fin) ;
  int write(ofstream &fout) const ; 

  void applyTransform() ;


  ObjectGL* copy() { PointListGL *t = new PointListGL(*this) ; return t ; }

  mutable BasicList<Point3Df> list ;
protected:
  int psize ;
};


/*!
  \brief A class to hold a control point.

  \author Philippe Lavoie
  \date 23 September 1997
*/
class CPointGL : public ObjectGL  {
protected:
  CPointGL() : ObjectGL(), cpoint(dummyCPoint), symPoint(0) { type = cpointObject;  psize=pSizeDefault ; xCoord = yCoord = zCoord = wCoord = 1.0 ;}
public:  
  CPointGL(HPoint3Df& cp, int i,int j=-1): ObjectGL(), cpoint(cp), symPoint(0) { type = cpointObject; psize=pSizeDefault ; i0= i ; j0=j ; } 
  ~CPointGL() {;}

  virtual void glObject() const;

  virtual void modify(const HPoint3Df& v) { cpoint += v ; }
  virtual void modifySym(const HPoint3Df &v) ;

  void setPsize(int s) {psize = s ; }

  HPoint3Df& point() const { return cpoint ; }

  void setSym(CPointGL* sp, float x, float y, float z, float w) { symPoint = sp ; xCoord = x; yCoord = y ; zCoord = z; wCoord = w ; if(symPoint == this) symPoint = 0 ;}

  int row() const { return i0 ; }
  int col() const { return j0 ; }

protected:
  HPoint3Df& cpoint ;
  int psize ;
  int i0,j0 ; 
  CPointGL *symPoint ;
  float xCoord,yCoord,zCoord,wCoord ;
};

class HNurbsSurfaceGL ;

/*!
  \brief A class to hold a control point from a HNURBS

  \author Philippe Lavoie
  \date 3 November 1997
*/
class HCPointGL : public CPointGL  {
public:
  HCPointGL(HPoint3Df& off, int i, int j, HNurbsSurfaceGL* hs, int mod): CPointGL(), offset(off),s(hs) { type = hcpointObject; psize=pSizeDefault ; i0 = i ; j0 = j ; canModify = mod ;} 
  ~HCPointGL() { ; } 

  virtual void glObject() const;

  virtual void modify(const HPoint3Df& v) ;

protected:
  HPoint3Df& offset ;
  HNurbsSurfaceGL *s ;
  int canModify ;
};

/*!
  \brief A class to hold a control point from a NURBS surface

  \author Philippe Lavoie
  \date 11 May 1998
*/
class SPointGL : public CPointGL {
protected:
  SPointGL(HPoint3Df& cp, int i,int j,int fix) : CPointGL(cp,i,j),editFix(fix) { } ;


public:
  virtual ~SPointGL() { ; }

  void setFixEdit(int f) { editFix = f ; }
  int fixEdit() const { return editFix ; }

protected:
  int editFix ; 

};

/*!
  \brief A class to hold a control point from a NURBS curve or surface

  \author Philippe Lavoie
  \date 11 May 1998
*/
class SPointCurveGL : public SPointGL  {
public:
  SPointCurveGL(int i, NurbsCurveSPf *c, int fix);
  ~SPointCurveGL() { ; } 

  virtual void glObject() const;

  virtual void modify(const HPoint3Df& v) ; 

  void updateOthers() ; 

  void setStartEnd(SPointCurveGL* s, int r) { start = s; rows = r ;  }

protected:
  HPoint3Df spoint ;
  NurbsCurveSPf *curve ;
  SPointCurveGL *start ; 
  int rows ;
};

/*!
  \brief A class to hold a surface point 

  \author Philippe Lavoie
  \date 11 May 1998
*/
class SPointSurfaceGL : public SPointGL  {
public:
  SPointSurfaceGL(int i, int j, NurbsSurfaceSPf *s, ObjectListGL *sp, int fix);
  ~SPointSurfaceGL() { ; } 

  virtual void glObject() const;

  virtual void modify(const HPoint3Df& v) ; 

  void updateOthers() ; 

  void setStartEnd(SPointSurfaceGL* s, int r, int c=0) { start = s; rows = r ; cols = c ;  }

protected:
  HPoint3Df spoint ;
  NurbsSurfaceSPf *surface ;
  SPointSurfaceGL *start ; 
  ObjectListGL *spoints ;
  int rows,cols ; 
};

/*!
  \brief a class to hold a HNURBS surface point 

  \author Philippe Lavoie
  \date 11 May 1998
*/
class SPointHSurfaceGL : public SPointGL  {
public:
  SPointHSurfaceGL(int i, int j, HNurbsSurfaceSPf *s, ObjectListGL *sp, int fix);
  ~SPointHSurfaceGL() { ; } 

  virtual void glObject() const;

  virtual void modify(const HPoint3Df& v) ; 

  void updateOthers() ; 

  void setStartEnd(SPointHSurfaceGL* s, int r, int c=0) { start = s; rows = r ; cols = c ;  }

protected:
  HPoint3Df spoint ;
  HNurbsSurfaceSPf *surface ;
  SPointHSurfaceGL *start ; 
  ObjectListGL *spoints ;
  int rows,cols ; 
};

/*!
  \brief A class to hold a 3D point

  \author Philippe Lavoie
  \date 23 September 1997
*/
class PointGL : public ObjectGL  {
public:  
  PointGL(const Point3Df& p3d): ObjectGL(), p(p3d) { type = pointObject ; psize=pSizeDefault ;}

  void glObject() const ;

  void modify(const Point3Df& v) { p += v ; }
  void set(const Point3Df& v) { p = v ; }

  void setPsize(int s) {psize = s ; }

  float x() const { return p.x() ; }
  float y() const { return p.y() ; }
  float z() const { return p.z() ; }
  const Point3Df& point() const { return p ; }

  ObjectGL* copy() { PointGL* t = new PointGL(p) ; return t ; }

protected:
  Point3Df p ;
  int psize ;
};

/*!
  \brief A class to hold a knot

  \author Philippe Lavoie
  \date 23 September 1997
*/
class KnotGL : public ObjectGL  {
public:  
  KnotGL(const Point3Df& p3d, int kU, int kV): ObjectGL(), p(p3d), ku(kU), kv(kV) { type = pointObject ; psize=pSizeDefault ;}

  void glObject() const ;

  void modify(const Point3Df& v) { p += v ; }
  void set(const Point3Df& v) { p = v ; }

  void setPsize(int s) {psize = s ; }

  float x() const { return p.x() ; }
  float y() const { return p.y() ; }
  float z() const { return p.z() ; }
  const Point3Df& point() const { return p ; }

  ObjectGL* copy() { PointGL* t = new PointGL(p) ; return t ; }
  
  int knotU() { return ku ; }
  int knotV() { return kv ; }

protected:
  Point3Df p ;
  int ku,kv ;
  int psize ;
};

/*!
  \brief Holds the control polygon for a curve

  \author Philippe Lavoie
  \date 23 September 1997
*/
class NurbsCpolygonGL : public ObjectGL  {
public:  
  NurbsCpolygonGL(NurbsCurvef& c): ObjectGL(),curve(c) { objectColor = cPolygonColorDefault;} ;

  void glObject() const;

protected:
  NurbsCurvef& curve ;
};

/*!
  \brief Holds the control polygon for a surface 
  
  \author Philippe Lavoie
  \date 23 September 1997
*/
class NurbsSpolygonGL : public ObjectGL  {
public:  
  NurbsSpolygonGL(NurbsSurfacef& s): ObjectGL(),surface(s) {objectColor = cPolygonColorDefault;}

  void glObject() const;


protected:
  NurbsSurfacef& surface ;
};

/*!
  \brief Holds a bounding box

  \author Philippe Lavoie
  \date 23 September 1997
*/
class BoundingBoxGL : public ObjectGL  {
public:  
  BoundingBoxGL() ;

  void glObject() const;

  Point3Df minP, maxP ;
 
  void setColorXYZ(const Color& colX, const Color& colY, const Color& colZ){
    colorX = colX; colorY = colY; colorZ = colZ ; }

protected:
  Color colorX, colorY, colorZ ;
};

/*!
  \brief a Virtual NURBS object class.

  \author Philippe Lavoie
  \date 28 September 1997
*/
class NurbsGL : public ObjectGL {
public:
  NurbsGL();

  virtual ~NurbsGL() { if(polygon) delete polygon ; }


  virtual NurbsGL& operator=(const NurbsGL& a) ;

  virtual void gluNurbs() const = 0 ;
  
  void setNurbsRenderer(GLUnurbsObj *nobj) { nurbsRenderer = nobj ; }

  virtual void glObject() const;

  virtual void point(float &u, float &v, int pSize, const Color& colorP, int cp_flag=0) const = 0 ;

  virtual void resetAll() { resetBoundingBox(); resetCPoints(); resetPolygon(); resetKnots(); }
  virtual void resetBoundingBox() {} ;
  virtual void resetCPoints() {} ;
  virtual void resetPolygon() {};
  virtual void resetKnots() {} ;
  
  friend NurbsGL* readNurbsObject(const char* filename) ;

  Color colorCP, colorCP0, colorPolygon, colorKnot ;


  void setObjectColor(const Color& a, const Color& b, const Color& c){ objectColor=a ; selectColor = b ; currentColor = c ; }
  void setBBoxColor(const Color& a, const Color& b, const Color& c,const Color& d, const Color& e, const Color& f){ bbox.objectColor=a ; bbox.selectColor = b ; bbox.currentColor = c ; bbox.setColorXYZ(d,e,f) ;}
  void setPolygonColor(const Color& a, const Color& b, const Color& c){ polygon->objectColor=a ; polygon->selectColor = b ; polygon->currentColor = c ; }
  void setCPointColor(const Color& a, const Color& b, const Color& c, const Color& d){ colorCP=a ; colorCP0=b ; cpoints.selectColor = c ; cpoints.currentColor = d ; }
  void setKnotsColor(const Color& a, const Color& b, const Color& c){ knots.objectColor = colorKnot =a ; knots.selectColor = b ; knots.currentColor = c ; }

  void viewBBox()    { bbox.viewObject();}
  void viewCPoints() { cpoints.viewAllObjects();}
  void viewCpolygon(){ if(polygon) polygon->viewObject() ; }
  void viewNurbs()   { nurbsState = objectState ; }
  void viewKnots()   { knots.viewObject();}

  void hideBBox()     { bbox.hideObject();}         
  void hideCPoints()  { cpoints.hideAllObjects();}      
  void hideCpolygon() { if(polygon) polygon->hideObject() ; }   
  void hideNurbs()    { nurbsState = hideState ; }
  void hideKnots()    { knots.hideObject() ; }

  void select() { selected = 1 ; bbox.select() ; }
  void deselect() { selected = 0 ; bbox.deselect() ; }

  void activate() { active = 1 ; bbox.activate() ; }
  void deactivate() { active = 0 ; bbox.deactivate() ; }

  virtual void setSym(int set, int uDir, float x, float y, float z, float w) = 0 ;

  ObjectListGL cpoints;
  ObjectListGL knots ;
  ObjectGL *polygon; 
  BoundingBoxGL bbox ;

  ObjectGLState nurbsState ;

  virtual void modifyPoint(float u, float v, float dx, float dy, float dz) = 0 ;

  int editSurfacePoints() const { return editSP ; }
  int editControlPoints() const { return !editSP; }
  int editSurfacePoints(int a) { editSP = a ; return editSP ; }
  int editControlPoints(int a) { editSP = !a ; return !editSP; }
  int editFixPoints() const { return editFix ; }
  int editFixPoints(int a) { editFix = a ; return editFix ; }

  void setULines(int u) { if(u<=1) u = 2 ; nUlines = u ; }
  void setVLines(int v) { if(v<=2) v = 2 ; nVlines = v ; }
  int ULines() const { return nUlines ; }
  int VLines() const { return nVlines ; }

protected:
  int editSP, editFix ;
  GLUnurbsObj *nurbsRenderer;
  int nUlines,nVlines ; 
};



/*!
  \brief A NURBS curve class with OpenGL interface

  \author Philippe Lavoie
  \date 23 September 1997
*/
class NurbsCurveGL : public NurbsCurveSPf, public NurbsGL {
public:
  NurbsCurveGL() : NurbsCurveSPf(),NurbsGL() { type = curveObject ;  polygon = new NurbsCpolygonGL(*this) ;}
  NurbsCurveGL(const NurbsCurvef& nurb): NurbsCurveSPf(nurb),NurbsGL() { type = curveObject ; polygon = new NurbsCpolygonGL(*this) ;}
  NurbsCurveGL(const NurbsCurveGL& nurb): NurbsCurveSPf((NurbsCurvef)nurb),NurbsGL() { type = curveObject ; polygon = new NurbsCpolygonGL(*this) ;}
  NurbsCurveGL(const Vector<HPoint3Df>& P1, const Vector<float> &U1, int degree=3):NurbsCurveSPf(P1,U1,degree),NurbsGL() { type = curveObject ; polygon = new NurbsCpolygonGL(*this) ;}
  NurbsCurveGL(const Vector<Point3Df>& P1, const Vector<float> &W, const Vector<float> &U1, int degree=3):NurbsCurveSPf(P1,W,U1,degree),NurbsGL() { type = curveObject ;  polygon = new NurbsCpolygonGL(*this) ;}

  void gluNurbs() const;
  void point(float &u, float &v, int pSize, const Color& colorP, int cp_flag=0) const ;

  NurbsCurveGL& operator=(const NurbsCurveGL& a) ;
  NurbsCurveGL& operator=(const NurbsCurvef& a) ;

  void resetBoundingBox() ;
  void resetCPoints() ;
  void resetPolygon() {}
  void resetKnots() ;

  ObjectGL* copy() { NurbsCurveGL *t = new NurbsCurveGL(*this); return t ; }

  int read(ifstream &fin) ;
  int write(ofstream &fout) const ;

  void applyTransform() ;
  void modifyPoint(float u, float v, float dx, float dy, float dz);


  void setSym(int set, int uDir, float x, float y, float z, float w) { }

};

/*!
  \brief A NURBS surface class for OpenGL

  \author Philippe Lavoie
  \date 23 September 1997
*/
class NurbsSurfaceGL : public NurbsSurfaceSPf, public NurbsGL  {
public:  
  NurbsSurfaceGL():NurbsSurfaceSPf(),NurbsGL() { type = surfaceObject ; polygon = new NurbsSpolygonGL(*this) ; image = 0 ; }
  NurbsSurfaceGL(const NurbsSurfacef& nS):NurbsSurfaceSPf(nS),NurbsGL() { type = surfaceObject ; polygon = new NurbsSpolygonGL(*this) ; image = 0 ; }
  NurbsSurfaceGL(const NurbsSurfaceGL& nS):NurbsSurfaceSPf((NurbsSurfacef)nS),NurbsGL() { type = surfaceObject ; polygon = new NurbsSpolygonGL(*this) ; image = 0 ; }
  NurbsSurfaceGL(int DegU, int DegV, const Vector<float>& Uk, const Vector<float>& Vk, const Matrix<HPoint3Df>& Cp):NurbsSurfaceSPf(DegU,DegV,Uk,Vk,Cp),NurbsGL() { type = surfaceObject ; polygon = new NurbsSpolygonGL(*this) ; image = 0 ; }
  NurbsSurfaceGL(int DegU, int DegV, Vector<float>& Uk, Vector<float>& Vk, Matrix< Point3Df >& Cp, Matrix<float>& W):NurbsSurfaceSPf(DegU,DegV,Uk,Vk,Cp,W),NurbsGL() { type = surfaceObject ; polygon = new NurbsSpolygonGL(*this) ; image = 0 ; } ;
  ~NurbsSurfaceGL() { if(image) delete []image ; }

  void gluNurbs() const;
  void point(float &u, float &v, int pSize, const Color& colorP, int cp_flag=0) const ;

  virtual NurbsSurfaceGL& operator=(const NurbsSurfaceGL& a);
  virtual NurbsSurfaceGL& operator=(const NurbsSurfacef& a);


  void resetBoundingBox() ;
  void resetCPoints() ;
  void resetPolygon() {}
  void resetKnots() ;

  int read(ifstream &fin) ;
  int write(ofstream &fout) const ;

  int writeRIB(ofstream &fout) const { return NurbsSurfacef::writeRIB(fout); }
  //  int writePOVRAY(ofstream &fout) const { return NurbsSurface::writePOVRAY(0.1,fout); }
  int writePOVRAY(ofstream &fout) const {  return NurbsSurfacef::writePOVRAY(0.1,fout); }

  ObjectGL* copy() { NurbsSurfaceGL *t = new NurbsSurfaceGL(*this); return t ; }

  void applyTransform() ;

  void modifyPoint(float u, float v, float dx, float dy, float dz);

  void setImage(GLubyte *img,GLint w, GLint h) ;

  void setSym(int set, int uDir, float x, float y, float z, float w) ;


  std::list<NurbsCurve_2Df*> trimmedCurves ; 

protected:
  GLubyte *image ;
  GLint imgW, imgH ;
  //NurbsCurveArray<float> ca ; 
};

/*!
  \brief a HNURBS surface class for OpenGL

  \author Philippe Lavoie
  \date 23 September 1997
*/
class HNurbsSurfaceGL : public HNurbsSurfaceSPf, public NurbsGL  {
public:  
  HNurbsSurfaceGL() ;
  HNurbsSurfaceGL(const NurbsSurfacef& nS);
  HNurbsSurfaceGL(const HNurbsSurfaceGL& bS) ;
  HNurbsSurfaceGL(const HNurbsSurfaceGL* bS);
  virtual ~HNurbsSurfaceGL() { ; }

  void setLevelOfDetail(int l) { lod = l ; }
  int levelOfDetail() const { return lod; }
  void increaseLevelOfDetail() { ++lod ; if(lod>maxLevel()) lod=maxLevel() ;}
  void decreaseLevelOfDetail() ;
  void highestLevelOfDetail() { lod = maxLevel() ; }
  int maxLevelOfDetail() { return maxLevel() ; }

  void gluNurbs() const;
  void point(float &u, float &v, int pSize, const Color& colorP, int cp_flag=0) const ;

  //virtual NurbsSurfaceGL& operator=(const NurbsSurfaceGL& a);
  //virtual NurbsSurfaceGL& operator=(const NurbsSurface& a);

  void resetBoundingBox() ;
  void resetCPoints() ;
  void resetPolygon() ;
  void resetKnots() { }


  int read(const char*f) { return ObjectGL::read(f); }
  int write(const char* f) const { return ObjectGL::write(f); }

  int read(ifstream &fin) ;
  int write(ofstream &fout) const ;


  int writeRIB(ofstream &fout) const { return NurbsSurfacef::writeRIB(fout); }
  //  int writePOVRAY(ofstream &fout) const { return NurbsSurface::writePOVRAY(0.1,fout); }
  int writePOVRAY(ofstream &fout) const { return NurbsSurfacef::writePOVRAY(0.1,fout); }

  //void applyTransform() ;

  void selectBasePatch() { activePatch = this ; }
  void selectNextPatch() ;
  void selectPrevPatch() ;
  void selectHigherLevel() ;
  void selectLowerLevel() ;
  void selectHighestLevel() ;
  int editLevel() { return activePatch->level() ; }

  void updateUpToLOD() { updateLevels(lod) ; }

  HNurbsSurfaceSPf* addLevel() ;

  void applyTransform() ;
  void modifyPoint(float u, float v, float dx, float dy, float dz);
  

  ObjectGL* copy() { HNurbsSurfaceGL *t = new HNurbsSurfaceGL(*this); return t ; }

  void setSym(int set, int uDir, float x, float y, float z, float w) ;

  void axis(int i, int j, Point3Df& xaxis, Point3Df& yaxis, Point3Df& zaxis) const { activePatch->HNurbsSurfacef::axis(i,j,xaxis,yaxis,zaxis) ; }

protected:
  int lod ;
  HNurbsSurfaceGL *activePatch ;
};

/*!
  \brief a linked list of NurbsGL

  \author Philippe Lavoie
  \date 28 September 1997
*/
class NurbsListGL : public ObjectListGL {
 public:
  //void add(NurbsGL* obj) { ObjectListGL::add(obj) ; }
  NurbsGL* remove(NurbsGL* obj) { return (NurbsGL*)ObjectListGL::remove(obj) ; }

  void glObject() const ; 
  void display() const ;

  void resetDisplayFlags(int o, int cp, int p, int b, int k, int behavior=NURBS_FLAGS_AFFECT_ALL) ;
};


void initColorsGL() ;

/*!
  \brief a NURBS curve class for OpenGL

  A NURBS curve class for OpenGL but with not editing 
  capabilities. This class should only be called by other 
  NURBS classes.

  \author Philippe Lavoie
  \date 10 June 1998
*/
class SimpleNurbsCurveGL : public NurbsGL, public NurbsCurvef {
protected:
  SimpleNurbsCurveGL() : NurbsCurvef(),NurbsGL() { type = curveObject ;  polygon = 0 ;}

public:

  void gluNurbs() const;
  void glObject() const { gluNurbs() ; }

  void point(float &u, float &v, int pSize, const Color& colorP, int cp_flag=0) const {}
  void resetBoundingBox() {}
  void resetCPoints() {}
  void resetPolygon() {}
  void resetKnots() {}

  void modifyPoint(float u, float v, float dx, float dy, float dz) { }
  void setSym(int set, int uDir, float x, float y, float z, float w) { }

  friend class NurbsCurveGL ; 
  friend class NurbsSurfaceGL ; 
  friend class HNurbsSurfaceGL ; 

};

} // end namespace

typedef PLib::NurbsCurveGL PlNurbsCurveGL ; 
typedef PLib::NurbsSurfaceGL PlNurbsSurfaceGL ; 
typedef PLib::HNurbsSurfaceGL PlHNurbsSurfaceGL ; 
typedef PLib::ObjectGL PlObjectGL ; 
typedef PLib::NurbsListGL PlNurbsListGL ; 
typedef PLib::ObjectRefGL PlObjectRefGL  ;
typedef PLib::ObjectListGL PlObjectListGL  ;
typedef PLib::ObjectRefListGL PlObjectRefListGL  ;
typedef PLib::PointListGL PlPointListGL  ;
typedef PLib::CPointGL PlCPointGL  ;
typedef PLib::HCPointGL PlHCPointGL  ;
typedef PLib::SPointGL PlSPointGL  ;
typedef PLib::SPointCurveGL PlSPointCurveGL  ;
typedef PLib::SPointSurfaceGL PlSPointSurfaceGL  ;
typedef PLib::SPointHSurfaceGL PlSPointHSurfaceGL  ;
typedef PLib::PointGL PlPointGL  ;
typedef PLib::KnotGL PlKnotGL  ;
typedef PLib::NurbsCpolygonGL PlNurbsCpolygonGL  ;
typedef PLib::NurbsSpolygonGL PlNurbsSpolygonGL  ;
typedef PLib::BoundingBoxGL PlBoundingBoxGL  ;
typedef PLib::NurbsGL PlNurbsGL  ;
typedef PLib::SimpleNurbsCurveGL PlSimpleNurbsCurveGL  ;

#endif // WITH_OPENGL

#endif
