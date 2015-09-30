#ifndef _NETLIBWRAPPER_H_
#define _NETLIBWRAPPER_H_


// Define as extern "C" if using a C++ compiler
#ifdef __cplusplus
extern "C" {
#endif

#define DDASPK FC_GLOBAL(ddaspk,DDASPK)
void DDASPK(void (*)(double*,double*,double*,double*,double*,int*,double*,int*),int*,double*,
	    double*, double*,double*,int*,double*,double*,int*, double*, int*,
	    int*, int*, double*,int*, void(*)(double*, int*, int*, double*,
	    double*, double*, double*, double*, double*, double*, double*,
	    double*, int*, int*, double*, int*),void(*)(int*, double*,
	    double*, double*, double*, double*, double*, double*,double*,
	    int*, double*,double*, int*,double*,int*));

#define DLSODAR FC_GLOBAL(dlsodar,DLSODAR)
void DLSODAR(void (*)(int*,double*,double*,double*), int*, double*, double*,
            double*, int*, double*, double*, int*, int*, int*, double*, int*,
            int*, int*, void(*)(int*,double*,double*,int*,int*,double*,int*),
	    int*, void(*)(int*, double*, double*,
            int*, double*), int*, int*);

#define DLSODER FC_GLOBAL(dlsoder,DLSODER)
void DLSODER(void (*)(int*,double*,double*,double*), int*, double*, double*,
            double*, int*, double*, double*, int*, int*, int*, double*, int*,
            int*, int*, void(*)(int*,double*,double*,int*,int*,double*,int*),
	    int*, void(*)(int*, double*, double*,
            int*, double*), int*, int*);

#define DLSODE FC_GLOBAL(dlsode,DLSODE)
void DLSODE(void (*)(int*,double*,double*,double*), int*, double*, double*,
            double*, int*, double*, double*, int*, int*, int*, double*, int*,
            int*, int*, void(*)(int*,double*,double*,int*,int*,double*,int*), int*);

#define SETUP FC_GLOBAL(setup,SETUP)
void SETUP(int*, double*, double*, double*,
           double*, double*, int*, char*, int*,
           double*, double*, int*, int*);

#define UT FC_GLOBAL(ut,UT)
void UT(void (*)(double*,double*,double*), double*, double*, double*, double*,
        double*, double*, int*, double*);
#define CT FC_GLOBAL(ct,CT)
void CT(void (*)(double*,double*,double*), double*, double*, 
        double*, double*, int*, double*);
#define INTRP FC_GLOBAL(intrp,INTRP)
void INTRP(double*,char*,int*,double*,double*,void (*)(double*,double*,double*),double*,double*,int*);

#define DOPRI5 FC_GLOBAL(dopri5,DOPRI5)
void DOPRI5(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,
            double*, double*,double*,int*,
	    void(*)(int*,double*,double*,double*,int*,double*,int*,int*,double*,int*,int*),
            int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTD5 FC_GLOBAL(contd5,CONTD5)
double CONTD5(int*,double*,double*,int*,int*);

#define DOP853 FC_GLOBAL(dop853,DOP853)
void DOP853(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,
            double*, double*,double*,int*,
	    void(*)(int*,double*,double*,double*,int*,double*,int*,int*,double*,int*,int*),
            int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTD8 FC_GLOBAL(contd8,CONTD8)
double CONTD8(int*,double*,double*,int*,int*);

#define ODEX FC_GLOBAL(odex,ODEX)
void ODEX(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,
            double*, double*,double*,double*,int*,
	    void(*)(int*,double*,double*,double*,int*,double*,int*,int*,int*,double*,int*,int*),
            int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTEX FC_GLOBAL(contex,CONTEX)
double CONTEX(int*,double*,double*,int*,int*,int*);

#define RADAU5 FC_GLOBAL(radau5,RADAU5)
void RADAU5(int*,void(*)(int*,double*,double*,double*,double*,int*),double*,double*,double*,
    double*,double*,double*,int*,void(*)(int*,double*,double*,double*,int*,double*,int*),
    int*,int*,int*,void(*)(int*,double*,int*,double*,int*),int*,int*,int*,
    void(*)(int*,double*,double*,double*,double*,int*,int*,double*,int*,int*),
    int*,double*,int*,int*,int*,double*,int*,int*);

#define CONTR5 FC_GLOBAL(contr5,CONTR5)
double CONTR5(int*,double*,double*,int*);

#define DDASKR FC_GLOBAL(ddaskr,DDASKR)
void DDASKR(void(*)(double*,double*,double*,double*,double*, double*,int*), int*, double*, double*, double*, double*, int*, double*, double*,
        int*, double*, int*, int*, int*, double*, int*, void(*)(double*, double*, double*, double*, double*, double*, int*), double*,
        void(*)(int*,double*,double*,double*,int*, double*,double*,int*),int*, int*);

#define MEXX FC_GLOBAL(mexx,MEXX)
  void mexx_(int *np,int *nv,int *nl,int *ng,int *nu,void fprob(int *np, int *nv, int *nl, int *ng, int *nu, double *t, double *p, double *v, double *u, double *rlam, int *lda, int *ldg, int *ldf, double *am, double *gp, double *fl, double *f, double *pdot, double *udot, double *g, double *gi, int *lflag, int *ifail), double *t,double *tfin, double *p,double *v,double *u,double *a,double *rlam,int *itol,double *rtol,double *atol,double *h,int *mxjob,int *ierr,int *liwk,int *iwk,int *lrwk,double *rwk,void solout(int *nr, int *np, int *nv, int *nu, int *nl,double *t , double *p, double *v, double *u, double *a, double *rlam, int *irtrn),void denout(int *nr, int *icall, int *ipol, double *t, int *nd, double *yip, int *irtrn),int *nswit,void fswit(int *np,int *nv,int *nu,int *nl,double* t,double* p,double* v,double* u,double *a,double* rlam,int *nswit,double *g),int *iswit);


// END: Define as extern "C" if using a C++ compiler
#ifdef __cplusplus
}
#endif


#endif
