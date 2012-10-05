#include "nurbs.cpp"

namespace PLib {

// float specialization

template <>
Point_nD<float,2> NurbsCurve<float,2>::normal(float u, const Point_nD<float,2>& v) const{
  cerr << "YOU CAN'T COMPUTE THE NORMAL in 2D of a 2D vector!\n" ; 
  return firstDn(u) ;
}

template <>
void NurbsCurve<float,2>::makeCircle(const Point_nD<float,2>& O, float r, double as, double ae){
  makeCircle(O,Point_nD<float,2>(1,0),Point_nD<float,2>(0,1),r,as,ae) ;
}

template <>
int NurbsCurve<float,2>::writeVRML(const char* filename,float radius,int K, const Color& color,int Nu,int Nv, float u_s, float u_e) const{
  NurbsCurve<float,3> C ;
  to3D(*this,C) ; 
  return C.writeVRML(filename,radius,K,color,Nu,Nv,u_s,u_e) ;
}

template <>
int NurbsCurve<float,2>::writeVRML97(const char* filename,float radius,int K, const Color& color,int Nu,int Nv, float u_s, float u_e) const{
  NurbsCurve<float,3> C ;
  to3D(*this,C) ; 
  return C.writeVRML97(filename,radius,K,color,Nu,Nv,u_s,u_e) ;
}

template <>
int NurbsCurve<float,2>::writeVRML(ostream& fout,float radius,int K, const Color& color,int Nu,int Nv, float u_s, float u_e) const{
  NurbsCurve<float,3> C ;
  to3D(*this,C) ; 
  return C.writeVRML(fout,radius,K,color,Nu,Nv,u_s,u_e) ;
}

template <>
int NurbsCurve<float,2>::writeVRML97(ostream& fout,float radius,int K, const Color& color,int Nu,int Nv, float u_s, float u_e) const{
  NurbsCurve<float,3> C ;
  to3D(*this,C) ; 
  return C.writeVRML97(fout,radius,K,color,Nu,Nv,u_s,u_e) ;
}

template <>
void NurbsCurve<float,2>::drawAaImg(Image_Color& Img, const Color& color, int precision, int alpha){
  NurbsCurve<float,3> C ;
  to3D(*this,C) ; 
  C.drawAaImg(Img,color,precision,alpha) ;
}


#ifdef NO_IMPLICIT_TEMPLATES

template class NurbsCurve<float,3>;
template class OpLengthFcn<float,3>;


template float chordLengthParam(const Vector< Point_nD<float,3> >& Q, Vector<float> &ub);
template float chordLengthParamH(const Vector< HPoint_nD<float,3> >& Q, Vector<float> &ub);
template float chordLengthParamClosed(const Vector< Point_nD<float,3> >& Q, Vector<float> &ub, int deg);
template float chordLengthParamClosedH(const Vector< HPoint_nD<float,3> >& Q, Vector<float> &ub, int deg);
template void binomialCoef(Matrix<float>& Bin) ;
template Vector<float> knotUnion(const Vector<float>& Ua, const Vector<float>& Ub);
template float nurbsBasisFun(float u, int i, int p, const Vector<float>& U) ; 
template void nurbsBasisFuns(float u, int span, int deg, const Vector<float>& U, Vector<float>& N);
template void nurbsDersBasisFuns(int n, float u, int span, int deg, const Vector<float>& U, Matrix<float>& ders) ;
template int intersectLine(const Point_nD<float,3>& p1, const Point_nD<float,3>& t1, const Point_nD<float,3>& p2, const Point_nD<float,3>& t2, Point_nD<float,3>& p);
template void knotAveraging(const Vector<float>& uk, int deg, Vector<float>& U) ;
template void knotAveragingClosed(const Vector<float>& uk, int deg, Vector<float>& U) ;
template void averagingKnots(const Vector<float>& U, int deg, Vector<float>& uk);
template int findSpan(float u, const Vector<float>& U, int deg);

template int maxInfluence(int i, const Vector<float>& U, int p, float &u);

template void generateCompatibleCurves(NurbsCurveArray<float,3> &ca);

template void knotApproximationClosed( Vector<float>& U, const  Vector<float>& ub, int n, int p);

template void wrapPointVector(const Vector<Point_nD<float,3> >& Q, int d, Vector<Point_nD<float,3> >& Qw);
template void wrapPointVectorH(const Vector<HPoint_nD<float,3> >& Q, int d, Vector<HPoint_nD<float,3> >& Qw);


template class NurbsCurve<float,2> ;
template class OpLengthFcn<float,2>;

template float chordLengthParam(const Vector< Point_nD<float,2> >& Q, Vector<float> &ub);
template float chordLengthParamH(const Vector< HPoint_nD<float,2> >& Q, Vector<float> &ub);
template float chordLengthParamClosed(const Vector< Point_nD<float,2> >& Q, Vector<float> &ub, int deg);
template float chordLengthParamClosedH(const Vector< HPoint_nD<float,2> >& Q, Vector<float> &ub, int deg);
template int intersectLine(const Point_nD<float,2>& p1, const Point_nD<float,2>& t1, const Point_nD<float,2>& p2, const Point_nD<float,2>& t2, Point_nD<float,2>& p);

template void generateCompatibleCurves(NurbsCurveArray<float,2> &ca);

template void wrapPointVector(const Vector<Point_nD<float,2> >& Q, int d, Vector<Point_nD<float,2> >& Qw);
template void wrapPointVectorH(const Vector<HPoint_nD<float,2> >& Q, int d, Vector<HPoint_nD<float,2> >& Qw);



template void to3D(const NurbsCurve<float,2>&, NurbsCurve<float,3>&);
template void to3D(const NurbsCurve<float,3>&, NurbsCurve<float,3>&);
template void to2D(const NurbsCurve<float,3>&, NurbsCurve<float,2>&);


  // The following is necessary for gcc 2.96
  //  template std::basic_istream<char, std::char_traits<char> >::seekg(std::fpos<__mbstate_t>);

#endif

} // end namespace

