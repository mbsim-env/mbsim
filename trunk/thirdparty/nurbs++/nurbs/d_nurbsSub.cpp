#include "nurbsSub.cpp"

namespace PLib {

  #ifdef NO_IMPLICIT_TEMPLATES

  template class SurfSample<double> ;
  template class NurbsSubSurface<double> ;
  template class NurbSurface<double> ;
  template class RenderMesh<double>;
  template class RenderMeshPS<double>;
  template class RenderMeshVRML<double>;
  template class RenderMeshVRML97<double>;
  template class RenderMeshPoints<double> ;

  
  double NurbSurface<double>::epsilon = 1e-6 ;
  double SurfSample<double>::epsilon = 1e-6 ;

  template void DrawSubdivision( NurbSurface<double> *, double tolerance );
  template void DrawEvaluation( NurbSurface<double> * );
	   
  template int FindBreakPoint( double u, double * kv, int m, int k );
  template void AllocNurb( NurbSurface<double> *, double *, double * );
  template void CloneNurb( NurbSurface<double> *, NurbSurface<double> * );
  template void FreeNurb( NurbSurface<double> * );
  template void RefineSurface( NurbSurface<double> *, NurbSurface<double> *, BOOL );
	   
  template void CalcPoint( double, double, NurbSurface<double> *, Point_nD<double,3> *, Point_nD<double,3> *, Point_nD<double,3> * );


  template void GetNormal( NurbSurface<double> * n, int indV, int indU );
  template void DoSubdivision( NurbSurface<double> * n, double tolerance, BOOL dirflag, int level ) ;
  template void BasisFunctions( double u, int brkPoint, double * kv, int k, double * bvals );
  template void BasisDerivatives( double u, int brkPoint, double * kv, int k, double * dvals );
  template void CalcAlpha( double * ukv, double * wkv, int m, int n, int k, double *** alpha );

  template void AdjustNormal( SurfSample<double> * samp );
  template BOOL TestFlat( NurbSurface<double> * n, double tolerance );
  template void EmitTriangles( NurbSurface<double> * n );
  template void SplitSurface( NurbSurface<double> * parent,
	      NurbSurface<double> * kid0, NurbSurface<double> * kid1,
	      BOOL dirflag );


  template BOOL IsCurveStraight( NurbSurface<double> * n,double tolerance,int crvInd,BOOL dirflag );  
  template void FixNormals( SurfSample<double> * s0, SurfSample<double> * s1, SurfSample<double> * s2 );
  template int SplitKV( double * srckv,double ** destkv,int * splitPt,int m, int k );
  template void MakeNewCorners( NurbSurface<double> * parent,NurbSurface<double> * kid0,NurbSurface<double> * kid1,BOOL dirflag );
  template void ProjectToLine( Point_nD<double,3> * firstPt, Point_nD<double,3> * lastPt, Point_nD<double,3> * midPt );


  //template class deque<Point_nD<double,3> > ;
  //template class deque<int> ; 

#ifdef USING_LINUX
#endif

#ifdef USING_GNU_SOLARIS
  template void fill<__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, PLib::Point_nD<double, 3> >(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, PLib::Point_nD<double, 3> const &);
  template void deque<int, __default_alloc_template<false, 0>, 0>::insert<__deque_iterator<int, int const &, int const &, 0> >(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int const &, int const &, 0>, __deque_iterator<int, int const &, int const &, 0>, forward_iterator_tag);
  template void deque<PLib::Point_nD<double, 3>, __default_alloc_template<false, 0>, 0>::insert<__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0> >(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, forward_iterator_tag);
  template __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0> __uninitialized_copy_aux<__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0> >(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __false_type);
  template void __uninitialized_fill_aux<__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, PLib::Point_nD<double, 3> >(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, PLib::Point_nD<double, 3> const &, __false_type);
  template void __uninitialized_fill_aux<PLib::Point_nD<double, 3> *, PLib::Point_nD<double, 3> >(PLib::Point_nD<double, 3> *, PLib::Point_nD<double, 3> *, PLib::Point_nD<double, 3> const &, __false_type);
  template __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0> __uninitialized_copy_aux<__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0> >(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __false_type);
  template void fill<__deque_iterator<int, int &, int *, 0>, int>(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int &, int *, 0>, int const &);
  template void fill<int *, int>(int *, int *, int const &);
  template void deque<PLib::Point_nD<double, 3>, __default_alloc_template<false, 0>, 0>::insert_aux<__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0> >(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, unsigned int);
  template void deque<int, __default_alloc_template<false, 0>, 0>::insert_aux<__deque_iterator<int, int const &, int const &, 0> >(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int const &, int const &, 0>, __deque_iterator<int, int const &, int const &, 0>, unsigned int);
#endif

#ifdef USING_GNU_DECALPHA
  template void fill<__deque_iterator<Point_nD<double, 3>, Point_nD<double, 3> &, Point_nD<double, 3> *, 0>, Point_nD<double, 3> >(__deque_iterator<Point_nD<double, 3>, Point_nD<double, 3> &, Point_nD<double, 3> *, 0>, __deque_iterator<Point_nD<double, 3>, Point_nD<double, 3> &, Point_nD<double, 3> *, 0>, Point_nD<double, 3> const &);
  template void deque<PLib::Point_nD<double, 3>, __default_alloc_template<0, 0>, 0>::insert<__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0> >(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, forward_iterator_tag);
  template __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0> __uninitialized_copy_aux<__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0> >(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __false_type);
  template __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0> __uninitialized_copy_aux<__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0> >(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __false_type);
  template void __uninitialized_fill_aux<__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, PLib::Point_nD<double, 3> >(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, PLib::Point_nD<double, 3> const &, __false_type);
  template void __uninitialized_fill_aux<PLib::Point_nD<double, 3> *, PLib::Point_nD<double, 3> >(PLib::Point_nD<double, 3> *, PLib::Point_nD<double, 3> *, PLib::Point_nD<double, 3> const &, __false_type);
  template void deque<int, __default_alloc_template<0, 0>, 0>::insert<__deque_iterator<int, int const &, int const &, 0> >(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int const &, int const &, 0>, __deque_iterator<int, int const &, int const &, 0>, forward_iterator_tag);
  template void fill<__deque_iterator<int, int &, int *, 0>, int>(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int &, int *, 0>, int const &);
  template void fill<int *, int>(int *, int *, int const &);
  template void deque<int, __default_alloc_template<true, 0>, 0>::insert<__deque_iterator<int, int const &, int const &, 0> >(__deque_iterator<int, int &, int *, 0>, __deque_iterator<int, int const &, int const &, 0>, __deque_iterator<int, int const &, int const &, 0>, forward_iterator_tag);
  template void deque<PLib::Point_nD<double, 3>, __default_alloc_template<false, 0>, 0>::destroy_nodes_at_back(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>);
  template void deque<PLib::Point_nD<double, 3>, __default_alloc_template<true, 0>, 0>::destroy_nodes_at_back(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>);
  template void deque<PLib::Point_nD<double, 3>, __default_alloc_template<false, 0>, 0>::destroy_nodes_at_front(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>);
  template void deque<PLib::Point_nD<double, 3>, __default_alloc_template<true, 0>, 0>::destroy_nodes_at_front(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>);
  template void deque<PLib::Point_nD<double, 3>, __default_alloc_template<true, 0>, 0>::insert<__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0> >(__deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> &, PLib::Point_nD<double, 3> *, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, __deque_iterator<PLib::Point_nD<double, 3>, PLib::Point_nD<double, 3> const &, PLib::Point_nD<double, 3> const &, 0>, forward_iterator_tag);
  template void* __default_alloc_template<true, 0>::free_list ;
  template char* __default_alloc_template<true, 0>::end_free;
  template char* __default_alloc_template<true, 0>::heap_size;
  template char* __default_alloc_template<true, 0>::start_free;

#endif

#endif

}
