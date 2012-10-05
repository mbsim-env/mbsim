
#ifndef Statistic_SOURCE
#define Statistic_SOURCE

#include "statistic.h"
#include <cmath>

/*!
 */
namespace PLib {

  void initStatistic(){
    MaximumIterations = 100 ; 
  }

  /*!
    \brief returns ln of the gamma function

    The precision of this algorithm is around 1e-10.
   */
  template <class T>
    T lnOfGamma(T xx)
    {
      double x,y,tmp,ser;
      static double cof[6] = { 76.18009172947146, -86.50532032941677,
			       24.01409824083091, -1.231739572450155,
			       0.1208650973866179e-2,-0.5395239384953e-5};
      int j;
      y=x=xx;
      tmp=x+5.5 ;
      tmp -= (x+0.5)*log(tmp);
      ser=1.000000000190015;
      for(j=0;j<=5;++j) ser += cof[j]/++y ;
      return T(-tmp+log(2.5066282746310005*ser/x));
    }
 
  template <class T>
    T factorial(int n)
    {
      static int ntop=4;
      static T a[33] = {1.0,1.0,2.0,6.0,24.0} ;
      if(n<0){
#ifdef USE_EXCEPTION       
	throw MatrixInputError();
#else
	Error error("factorial");
	error << "The input value was lower than 0.\n" ;
	error.fatal();
#endif
      }
      if(n>32)
	return exp(lnOfGamma(n+1.0));
      while(ntop<n){
	int j=ntop++;
	a[ntop]=a[j]*ntop ;
      }
      return a[n] ;
    }

  template <class T>
    T binomialCoefficient(int n, int k)
    {
      return (T)floor(0.5+exp(lnOfFactorial<T>(k)-lnOfFactorial<T>(n-k)));
    }

  template <class T>
    T lnOfFactorial(int n)
    {
      static T a[101] ;
      if(n<0){
#ifdef USE_EXCEPTION       
	throw MatrixInputError();
#else
	Error error("lnOfFactorial");
	error << "The input value was lower than 0.\n" ;
	error.fatal();
#endif
      }
      if(n<=1) return 0;
      if(n <= 100) 
	return a[n] ? a[n] : (a[n] = lnOfGamma(n+1.0)) ;
      return lnOfGamma(n+1.0) ; // out of range of table
    }
  
  template <class T>
    T beta(T z, T w)
    {
      return exp(lnOfGamma(z)+lnOfGamma(w)-lnOfGamma(z+w));
    }

  template <class T> 
    T gammaP(T a, T x)
    {
      T gamser, gammcf, gln ;
      if(x < 0 || a <= 0){
#ifdef USE_EXCEPTION       
	throw MatrixInputError();
#else
	Error error("gammaP");
	error << "Invalid arguments to the incomplete gamma function.\n" ;
	error.fatal();
#endif
      }
      if(x < (a+1)){
	return gammaSerie(a,x,gln);
      }
      return 1.0-gammaSerieCF(a,x,gln);
    }

  template <class T>
    T gammaQ(T a, T x)
    {
      T gamser, gammcf, gln ;
      if(x < 0 || a <= 0){
#ifdef USE_EXCEPTION       
	throw MatrixInputError();
#else
	Error error("gammaQ");
	error << "Invalid arguments to the incomplete gamma function.\n" ;
	error.fatal();
#endif
      }
      if(x < (a+1)){
	return 1.0-gammaSerie(a,x,gln);
      }
      return gammaSerieCF(a,x,gln);
    }

  template <class T>
    T gammaSerie(T a, T x, T& gln)
    {
      int n;
      T sum, del, ap;
      gln=lnOfGamma(a);
      if(x<0){
#ifdef USE_EXCEPTION       
	throw MatrixInputError();
#else
	Error error("gammaSerie");
	error << "x can't be negative.\n" ;
	error.fatal();
	return 0; 
#endif
      }
      
      ap = a ;
      del= sum = T(1)/a;
      for(n=MaximumIterations;n>0;--n){
	++ap;
	del *= x/ap ;
	sum += del ;
	if(absolute(del)< absolute(sum)*Precision<T>())
	  return sum*exp(-x+a*log(x)-gln) ;
      }
#ifdef USING_EXCEPTION
      throw MatrixErr();
#else
      Error error("gammaSerie");
      error << "a too large or MaximumIterations too small.\n" ;
      error.fatal();
      return 0;
#endif
    }

  template <class T>
    T gammaSerieCF(T a, T x, T& gln)
    {
      int i;
      T an,b,c,d,del,h;

      gln=lnOfGamma(a);

      b= x + 1.0 - a;
      c= T(1)/MinimumFloat<T>();
      d= T(1)/b ;
      h= d ;
      for(i=1;i<=MaximumIterations;++i){
	an = -T(i)*T(i-a);
	b += 2 ;
	d = an*d + b ;
	if(absolute(d) < MinimumFloat<T>() )
	  d = MinimumFloat<T>();
	c = b + an/c ;
	if(absolute(c) < MinimumFloat<T>() )
	  c = MinimumFloat<T>();
	d=T(1)/d ;
	del=d*c;
	h *= del ;
	if(absolute(del-1.0) < Precision<T>()) break ;
	
      }

      if(i>MaximumIterations){
#ifdef USING_EXCEPTION
	throw MatrixErr();
#else
	Error error("gammaSerie");
	error << "a too large or MaximumIterations too small.\n" ;
	error.fatal();
	return 0;
#endif
      }
      return exp(-x+a*log(x)-gln)*h ;
    }

  template <class T>
    T errorFcn(T x){
    return x < 0 ? -gammaP<T>(0.5,x*x) : gammaP<T>(0.5,x*x) ;
  }

  template <class T>
    T errorFcnC(T x){
    return x < 0 ? 1+gammaP<T>(0.5,x*x) : gammaQ<T>(0.5,x*x) ; 
  }

  template <class T>
    T errorFcnChebyshevC(T x)
    {
      T t,z,ans ;
      z = absolute(x);
      t = T(1)/(T(1)+0.5*z) ;
      ans=t*exp(-z*z-1.26551223+t*(1.00002368+t*(0.37409196+t*0.09678418+
                t*(-0.18628806+t*(0.27886807+t*(-1.13520398+t*(1.48851587+
                t*(-0.82215223+t*0.17087277))))))));
      return x>0 ? ans : T(2)-ans ;
						 
    }

  template <class T>
    void kendallTau(const BasicArray<T>& data1, const BasicArray<T>& data2, T &tau, T &z, T& prob)
    {
      unsigned int n2=0,n1=0,k,j ;
      int is=0;
      T svar, aa, a2,a1 ;
      int n = data1.n() ;

      for(j=0;j<n-1;++j)
	for(k=j+1;k<n;++k){
	  a1= data1[j]-data1[k] ;
	  a2= data2[j]-data2[k] ;
	  aa=a1*a2;
	  if(aa){
	    ++n1 ; //no tie
	    ++n2 ;
	    aa > 0 ? ++is : --is ;
	  }
	  else{
	    if(a1) ++n1 ; // extra 'x' event
	    if(a2) ++n2 ; // extra 'y' event
	  }
	}
      tau = is/(T(sqrt((double)n1)*sqrt((double)n2))) ;
      svar = T(4*n+10)/T(9*n*(n-1)) ;
      z = tau/T(sqrt(svar)) ;
      prob = errorFcnChebyshevC<T>(absolute(z)/1.4142136) ;
    }

  void kendallTau(const BasicArray<int>& data1, const BasicArray<int>& data2, float &tau, float &z, float& prob)
    {
      unsigned int n2=0,n1=0,k,j ;
      int is=0;
      float svar, aa, a2,a1 ;
      int n = data1.n() ;

      
      for(j=0;j<n-1;++j)
	for(k=j+1;k<n;++k){
	  a1= data1[j]-data1[k] ;
	  a2= data2[j]-data2[k] ;
	  aa=a1*a2;
	  if(aa){
	    ++n1 ; //no tie
	    ++n2 ;
	    aa > 0 ? ++is : --is ;
	  }
	  else{
	    if(a1) ++n1 ; // extra 'x' event
	    if(a2) ++n2 ; // extra 'y' event
	  }
	}
      tau = is/(float(sqrt((double)n1)*sqrt((double)n2))) ;
      svar = float(4*n+10)/float(9*n*(n-1)) ;
      z = tau/float(sqrt(svar)) ;
      prob = errorFcnChebyshevC<float>(absolute(z)/1.4142136) ;
    }

  void kendallTau(const BasicArray<int>& data1, const BasicArray<int>& data2, double &tau, double &z, double& prob)
    {
      unsigned int n2=0,n1=0,k,j ;
      int is=0;
      double svar, aa, a2,a1 ;
      int n = data1.n() ;

      
      for(j=0;j<n-1;++j)
	for(k=j+1;k<n;++k){
	  a1= data1[j]-data1[k] ;
	  a2= data2[j]-data2[k] ;
	  aa=a1*a2;
	  if(aa){
	    ++n1 ; //no tie
	    ++n2 ;
	    aa > 0 ? ++is : --is ;
	  }
	  else{
	    if(a1) ++n1 ; // extra 'x' event
	    if(a2) ++n2 ; // extra 'y' event
	  }
	}
      tau = is/(double(sqrt((double)n1)*sqrt((double)n2))) ;
      svar = double(4*n+10)/double(9*n*(n-1)) ;
      z = tau/double(sqrt(svar)) ;
      prob = errorFcnChebyshevC<double>(absolute(z)/1.4142136) ;
    }

}


#endif
