/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__CaPlaneNMDARwMem
#define _nrn_initial _nrn_initial__CaPlaneNMDARwMem
#define nrn_cur _nrn_cur__CaPlaneNMDARwMem
#define _nrn_current _nrn_current__CaPlaneNMDARwMem
#define nrn_jacob _nrn_jacob__CaPlaneNMDARwMem
#define nrn_state _nrn_state__CaPlaneNMDARwMem
#define _net_receive _net_receive__CaPlaneNMDARwMem 
#define kstates kstates__CaPlaneNMDARwMem 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define nrecepts _p[0]
#define nrecepts_columnindex 0
#define scg _p[1]
#define scg_columnindex 1
#define Vrev _p[2]
#define Vrev_columnindex 2
#define kCa1 _p[3]
#define kCa1_columnindex 3
#define kCa0 _p[4]
#define kCa0_columnindex 4
#define ka1 _p[5]
#define ka1_columnindex 5
#define ka0 _p[6]
#define ka0_columnindex 6
#define kg1 _p[7]
#define kg1_columnindex 7
#define kg0 _p[8]
#define kg0_columnindex 8
#define kcdd1 _p[9]
#define kcdd1_columnindex 9
#define kcdd0 _p[10]
#define kcdd0_columnindex 10
#define kd1 _p[11]
#define kd1_columnindex 11
#define kd0 _p[12]
#define kd0_columnindex 12
#define k1M _p[13]
#define k1M_columnindex 13
#define k0M _p[14]
#define k0M_columnindex 14
#define ka1M _p[15]
#define ka1M_columnindex 15
#define ka0M _p[16]
#define ka0M_columnindex 16
#define kg1M _p[17]
#define kg1M_columnindex 17
#define kg0M _p[18]
#define kg0M_columnindex 18
#define kcdd1M _p[19]
#define kcdd1M_columnindex 19
#define kcdd0M _p[20]
#define kcdd0M_columnindex 20
#define kd1M _p[21]
#define kd1M_columnindex 21
#define kd0M _p[22]
#define kd0M_columnindex 22
#define A _p[23]
#define A_columnindex 23
#define M _p[24]
#define M_columnindex 24
#define Inmda _p[25]
#define Inmda_columnindex 25
#define Vm _p[26]
#define Vm_columnindex 26
#define Ca _p[27]
#define Ca_columnindex 27
#define R _p[28]
#define R_columnindex 28
#define AR _p[29]
#define AR_columnindex 29
#define A2R _p[30]
#define A2R_columnindex 30
#define A2Rd _p[31]
#define A2Rd_columnindex 31
#define A2Ro _p[32]
#define A2Ro_columnindex 32
#define RM _p[33]
#define RM_columnindex 33
#define ARM _p[34]
#define ARM_columnindex 34
#define A2RM _p[35]
#define A2RM_columnindex 35
#define A2RdM _p[36]
#define A2RdM_columnindex 36
#define A2RoM _p[37]
#define A2RoM_columnindex 37
#define cR _p[38]
#define cR_columnindex 38
#define cAR _p[39]
#define cAR_columnindex 39
#define cA2R _p[40]
#define cA2R_columnindex 40
#define cA2Rd _p[41]
#define cA2Rd_columnindex 41
#define cA2Rcdd _p[42]
#define cA2Rcdd_columnindex 42
#define cA2Ro _p[43]
#define cA2Ro_columnindex 43
#define cRM _p[44]
#define cRM_columnindex 44
#define cARM _p[45]
#define cARM_columnindex 45
#define cA2RM _p[46]
#define cA2RM_columnindex 46
#define cA2RdM _p[47]
#define cA2RdM_columnindex 47
#define cA2RcddM _p[48]
#define cA2RcddM_columnindex 48
#define cA2RoM _p[49]
#define cA2RoM_columnindex 49
#define Popen _p[50]
#define Popen_columnindex 50
#define DR _p[51]
#define DR_columnindex 51
#define DAR _p[52]
#define DAR_columnindex 52
#define DA2R _p[53]
#define DA2R_columnindex 53
#define DA2Rd _p[54]
#define DA2Rd_columnindex 54
#define DA2Ro _p[55]
#define DA2Ro_columnindex 55
#define DRM _p[56]
#define DRM_columnindex 56
#define DARM _p[57]
#define DARM_columnindex 57
#define DA2RM _p[58]
#define DA2RM_columnindex 58
#define DA2RdM _p[59]
#define DA2RdM_columnindex 59
#define DA2RoM _p[60]
#define DA2RoM_columnindex 60
#define DcR _p[61]
#define DcR_columnindex 61
#define DcAR _p[62]
#define DcAR_columnindex 62
#define DcA2R _p[63]
#define DcA2R_columnindex 63
#define DcA2Rd _p[64]
#define DcA2Rd_columnindex 64
#define DcA2Rcdd _p[65]
#define DcA2Rcdd_columnindex 65
#define DcA2Ro _p[66]
#define DcA2Ro_columnindex 66
#define DcRM _p[67]
#define DcRM_columnindex 67
#define DcARM _p[68]
#define DcARM_columnindex 68
#define DcA2RM _p[69]
#define DcA2RM_columnindex 69
#define DcA2RdM _p[70]
#define DcA2RdM_columnindex 70
#define DcA2RcddM _p[71]
#define DcA2RcddM_columnindex 71
#define DcA2RoM _p[72]
#define DcA2RoM_columnindex 72
#define DPopen _p[73]
#define DPopen_columnindex 73
#define v _p[74]
#define v_columnindex 74
#define _g _p[75]
#define _g_columnindex 75
#define _nd_area  *_ppvar[0]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(Object* _ho) { void* create_point_process(int, Object*);
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt(void*);
 static double _hoc_loc_pnt(void* _vptr) {double loc_point_process(int, void*);
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(void* _vptr) {double has_loc_point(void*);
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(void* _vptr) {
 double get_loc_point_process(void*); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 0, 0
};
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "scg", "pS",
 "Vrev", "mV",
 "kCa1", "uM-1",
 "kCa0", "ms-1",
 "ka1", "uM-1",
 "ka0", "ms-1",
 "kg1", "ms-1",
 "kg0", "ms-1",
 "kcdd1", "ms-1",
 "kcdd0", "ms-1",
 "kd1", "ms-1",
 "kd0", "ms-1",
 "k1M", "uM-1",
 "k0M", "ms-1",
 "ka1M", "uM-1",
 "ka0M", "ms-1",
 "kg1M", "ms-1",
 "kg0M", "ms-1",
 "kcdd1M", "ms-1",
 "kcdd0M", "ms-1",
 "kd1M", "ms-1",
 "kd0M", "ms-1",
 "A", "uM",
 "M", "uM",
 "Inmda", "pA",
 "Vm", "mV",
 "Ca", "uM",
 0,0
};
 static double A2RoM0 = 0;
 static double A2RdM0 = 0;
 static double A2RM0 = 0;
 static double ARM0 = 0;
 static double A2Ro0 = 0;
 static double A2Rd0 = 0;
 static double A2R0 = 0;
 static double AR0 = 0;
 static double Popen0 = 0;
 static double RM0 = 0;
 static double R0 = 0;
 static double cA2RoM0 = 0;
 static double cA2RcddM0 = 0;
 static double cA2RdM0 = 0;
 static double cA2RM0 = 0;
 static double cARM0 = 0;
 static double cRM0 = 0;
 static double cA2Ro0 = 0;
 static double cA2Rcdd0 = 0;
 static double cA2Rd0 = 0;
 static double cA2R0 = 0;
 static double cAR0 = 0;
 static double cR0 = 0;
 static double delta_t = 0.01;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(NrnThread*, _Memb_list*, int);
static void nrn_state(NrnThread*, _Memb_list*, int);
 static void nrn_cur(NrnThread*, _Memb_list*, int);
static void  nrn_jacob(NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(NrnThread*, _Memb_list*, int);
static void _ode_matsol(NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[2]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"CaPlaneNMDARwMem",
 "nrecepts",
 "scg",
 "Vrev",
 "kCa1",
 "kCa0",
 "ka1",
 "ka0",
 "kg1",
 "kg0",
 "kcdd1",
 "kcdd0",
 "kd1",
 "kd0",
 "k1M",
 "k0M",
 "ka1M",
 "ka0M",
 "kg1M",
 "kg0M",
 "kcdd1M",
 "kcdd0M",
 "kd1M",
 "kd0M",
 0,
 "A",
 "M",
 "Inmda",
 "Vm",
 "Ca",
 0,
 "R",
 "AR",
 "A2R",
 "A2Rd",
 "A2Ro",
 "RM",
 "ARM",
 "A2RM",
 "A2RdM",
 "A2RoM",
 "cR",
 "cAR",
 "cA2R",
 "cA2Rd",
 "cA2Rcdd",
 "cA2Ro",
 "cRM",
 "cARM",
 "cA2RM",
 "cA2RdM",
 "cA2RcddM",
 "cA2RoM",
 "Popen",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 76, _prop);
 	/*initialize range parameters*/
 	nrecepts = 300;
 	scg = 50;
 	Vrev = 0;
 	kCa1 = 0.001;
 	kCa0 = 0.00075;
 	ka1 = 31.6;
 	ka0 = 1.01;
 	kg1 = 4.77241;
 	kg0 = 0.557;
 	kcdd1 = 0.03212;
 	kcdd0 = 0.00292;
 	kd1 = 0.003;
 	kd0 = 0.003;
 	k1M = 0.001;
 	k0M = 0.001;
 	ka1M = 31.6;
 	ka0M = 1.01;
 	kg1M = 4.77241;
 	kg0M = 0.053017;
 	kcdd1M = 0.06424;
 	kcdd0M = 0.0006;
 	kd1M = 0.00054;
 	kd0M = 1.25e-05;
  }
 	_prop->param = _p;
 	_prop->param_size = 76;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _thread_cleanup(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _CaPlaneNMDARwMem_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 3,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
  _extcall_thread = (Datum*)ecalloc(2, sizeof(Datum));
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 0, _thread_cleanup);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 76, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 CaPlaneNMDARwMem CaPlaneNMDARwMem.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 extern double *_nrn_thread_getelm(SparseObj*, int, int);
 
#define _MATELM1(_row,_col) *(_nrn_thread_getelm(_so, _row + 1, _col + 1))
 
#define _RHS1(_arg) _rhs[_arg+1]
  
#define _linmat1  0
 static int _spth1 = 1;
 static int _cvspth1 = 0;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[22], _dlist1[22]; static double *_temp1;
 static int kstates();
 
static int kstates (void* _so, double* _rhs, double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt)
 {int _reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=1;_i<22;_i++){
  	_RHS1(_i) = -_dt1*(_p[_slist1[_i]] - _p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 /* ~ A + R <-> AR ( 2.0 * ka1 , ka0 )*/
 f_flux =  2.0 * ka1 * R * A ;
 b_flux =  ka0 * AR ;
 _RHS1( 10) -= (f_flux - b_flux);
 _RHS1( 8) += (f_flux - b_flux);
 
 _term =  2.0 * ka1 * A ;
 _MATELM1( 10 ,10)  += _term;
 _MATELM1( 8 ,10)  -= _term;
 _term =  ka0 ;
 _MATELM1( 10 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ A + AR <-> A2R ( ka1 , 2.0 * ka0 )*/
 f_flux =  ka1 * AR * A ;
 b_flux =  2.0 * ka0 * A2R ;
 _RHS1( 8) -= (f_flux - b_flux);
 _RHS1( 7) += (f_flux - b_flux);
 
 _term =  ka1 * A ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 7 ,8)  -= _term;
 _term =  2.0 * ka0 ;
 _MATELM1( 8 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ A2R <-> A2Ro ( kg1 , kg0 )*/
 f_flux =  kg1 * A2R ;
 b_flux =  kg0 * A2Ro ;
 _RHS1( 7) -= (f_flux - b_flux);
 _RHS1( 5) += (f_flux - b_flux);
 
 _term =  kg1 ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 5 ,7)  -= _term;
 _term =  kg0 ;
 _MATELM1( 7 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ A2R <-> A2Rd ( kd1 , kd0 )*/
 f_flux =  kd1 * A2R ;
 b_flux =  kd0 * A2Rd ;
 _RHS1( 7) -= (f_flux - b_flux);
 _RHS1( 6) += (f_flux - b_flux);
 
 _term =  kd1 ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 6 ,7)  -= _term;
 _term =  kd0 ;
 _MATELM1( 7 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ A2Ro + M <-> A2RoM ( k1M , k0M )*/
 f_flux =  k1M * M * A2Ro ;
 b_flux =  k0M * A2RoM ;
 _RHS1( 5) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  k1M * M ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 1 ,5)  -= _term;
 _term =  k0M ;
 _MATELM1( 5 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ A + RM <-> ARM ( 2.0 * ka1M , ka0M )*/
 f_flux =  2.0 * ka1M * RM * A ;
 b_flux =  ka0M * ARM ;
 _RHS1( 9) -= (f_flux - b_flux);
 _RHS1( 4) += (f_flux - b_flux);
 
 _term =  2.0 * ka1M * A ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 4 ,9)  -= _term;
 _term =  ka0M ;
 _MATELM1( 9 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ A + ARM <-> A2RM ( ka1M , 2.0 * ka0M )*/
 f_flux =  ka1M * ARM * A ;
 b_flux =  2.0 * ka0M * A2RM ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  ka1M * A ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 3 ,4)  -= _term;
 _term =  2.0 * ka0M ;
 _MATELM1( 4 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ A2RM <-> A2RoM ( kg1M , kg0M )*/
 f_flux =  kg1M * A2RM ;
 b_flux =  kg0M * A2RoM ;
 _RHS1( 3) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  kg1M ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 1 ,3)  -= _term;
 _term =  kg0M ;
 _MATELM1( 3 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ A2RM <-> A2RdM ( kd1M , kd0M )*/
 f_flux =  kd1M * A2RM ;
 b_flux =  kd0M * A2RdM ;
 _RHS1( 3) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  kd1M ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 2 ,3)  -= _term;
 _term =  kd0M ;
 _MATELM1( 3 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ A + cR <-> cAR ( 2.0 * ka1 , ka0 )*/
 f_flux =  2.0 * ka1 * cR * A ;
 b_flux =  ka0 * cAR ;
 _RHS1( 21) -= (f_flux - b_flux);
 _RHS1( 20) += (f_flux - b_flux);
 
 _term =  2.0 * ka1 * A ;
 _MATELM1( 21 ,21)  += _term;
 _MATELM1( 20 ,21)  -= _term;
 _term =  ka0 ;
 _MATELM1( 21 ,20)  -= _term;
 _MATELM1( 20 ,20)  += _term;
 /*REACTION*/
  /* ~ A + cAR <-> cA2R ( ka1 , 2.0 * ka0 )*/
 f_flux =  ka1 * cAR * A ;
 b_flux =  2.0 * ka0 * cA2R ;
 _RHS1( 20) -= (f_flux - b_flux);
 _RHS1( 19) += (f_flux - b_flux);
 
 _term =  ka1 * A ;
 _MATELM1( 20 ,20)  += _term;
 _MATELM1( 19 ,20)  -= _term;
 _term =  2.0 * ka0 ;
 _MATELM1( 20 ,19)  -= _term;
 _MATELM1( 19 ,19)  += _term;
 /*REACTION*/
  /* ~ cA2R <-> cA2Ro ( kg1 , kg0 )*/
 f_flux =  kg1 * cA2R ;
 b_flux =  kg0 * cA2Ro ;
 _RHS1( 19) -= (f_flux - b_flux);
 _RHS1( 18) += (f_flux - b_flux);
 
 _term =  kg1 ;
 _MATELM1( 19 ,19)  += _term;
 _MATELM1( 18 ,19)  -= _term;
 _term =  kg0 ;
 _MATELM1( 19 ,18)  -= _term;
 _MATELM1( 18 ,18)  += _term;
 /*REACTION*/
  /* ~ cA2R <-> cA2Rd ( kd1 , kd0 )*/
 f_flux =  kd1 * cA2R ;
 b_flux =  kd0 * cA2Rd ;
 _RHS1( 19) -= (f_flux - b_flux);
 _RHS1( 13) += (f_flux - b_flux);
 
 _term =  kd1 ;
 _MATELM1( 19 ,19)  += _term;
 _MATELM1( 13 ,19)  -= _term;
 _term =  kd0 ;
 _MATELM1( 19 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 /*REACTION*/
  /* ~ cA2R <-> cA2Rcdd ( kcdd1 , kcdd0 )*/
 f_flux =  kcdd1 * cA2R ;
 b_flux =  kcdd0 * cA2Rcdd ;
 _RHS1( 19) -= (f_flux - b_flux);
 _RHS1( 12) += (f_flux - b_flux);
 
 _term =  kcdd1 ;
 _MATELM1( 19 ,19)  += _term;
 _MATELM1( 12 ,19)  -= _term;
 _term =  kcdd0 ;
 _MATELM1( 19 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ cA2Ro + M <-> cA2RoM ( k1M , k0M )*/
 f_flux =  k1M * M * cA2Ro ;
 b_flux =  k0M * cA2RoM ;
 _RHS1( 18) -= (f_flux - b_flux);
 
 _term =  k1M * M ;
 _MATELM1( 18 ,18)  += _term;
 _term =  k0M ;
 _MATELM1( 18 ,0)  -= _term;
 /*REACTION*/
  /* ~ A + cRM <-> cARM ( 2.0 * ka1M , ka0M )*/
 f_flux =  2.0 * ka1M * cRM * A ;
 b_flux =  ka0M * cARM ;
 _RHS1( 17) -= (f_flux - b_flux);
 _RHS1( 16) += (f_flux - b_flux);
 
 _term =  2.0 * ka1M * A ;
 _MATELM1( 17 ,17)  += _term;
 _MATELM1( 16 ,17)  -= _term;
 _term =  ka0M ;
 _MATELM1( 17 ,16)  -= _term;
 _MATELM1( 16 ,16)  += _term;
 /*REACTION*/
  /* ~ A + cARM <-> cA2RM ( ka1M , 2.0 * ka0M )*/
 f_flux =  ka1M * cARM * A ;
 b_flux =  2.0 * ka0M * cA2RM ;
 _RHS1( 16) -= (f_flux - b_flux);
 _RHS1( 15) += (f_flux - b_flux);
 
 _term =  ka1M * A ;
 _MATELM1( 16 ,16)  += _term;
 _MATELM1( 15 ,16)  -= _term;
 _term =  2.0 * ka0M ;
 _MATELM1( 16 ,15)  -= _term;
 _MATELM1( 15 ,15)  += _term;
 /*REACTION*/
  /* ~ cA2RM <-> cA2RoM ( kg1M , kg0M )*/
 f_flux =  kg1M * cA2RM ;
 b_flux =  kg0M * cA2RoM ;
 _RHS1( 15) -= (f_flux - b_flux);
 
 _term =  kg1M ;
 _MATELM1( 15 ,15)  += _term;
 _term =  kg0M ;
 _MATELM1( 15 ,0)  -= _term;
 /*REACTION*/
  /* ~ cA2RM <-> cA2RdM ( kd1M , kd0M )*/
 f_flux =  kd1M * cA2RM ;
 b_flux =  kd0M * cA2RdM ;
 _RHS1( 15) -= (f_flux - b_flux);
 _RHS1( 14) += (f_flux - b_flux);
 
 _term =  kd1M ;
 _MATELM1( 15 ,15)  += _term;
 _MATELM1( 14 ,15)  -= _term;
 _term =  kd0M ;
 _MATELM1( 15 ,14)  -= _term;
 _MATELM1( 14 ,14)  += _term;
 /*REACTION*/
  /* ~ cA2RM <-> cA2RcddM ( kcdd1M , kcdd0M )*/
 f_flux =  kcdd1M * cA2RM ;
 b_flux =  kcdd0M * cA2RcddM ;
 _RHS1( 15) -= (f_flux - b_flux);
 _RHS1( 11) += (f_flux - b_flux);
 
 _term =  kcdd1M ;
 _MATELM1( 15 ,15)  += _term;
 _MATELM1( 11 ,15)  -= _term;
 _term =  kcdd0M ;
 _MATELM1( 15 ,11)  -= _term;
 _MATELM1( 11 ,11)  += _term;
 /*REACTION*/
  /* ~ R <-> cR ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * R ;
 b_flux =  kCa0 * cR ;
 _RHS1( 10) -= (f_flux - b_flux);
 _RHS1( 21) += (f_flux - b_flux);
 
 _term =  Ca * kCa1 ;
 _MATELM1( 10 ,10)  += _term;
 _MATELM1( 21 ,10)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 10 ,21)  -= _term;
 _MATELM1( 21 ,21)  += _term;
 /*REACTION*/
  /* ~ AR <-> cAR ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * AR ;
 b_flux =  kCa0 * cAR ;
 _RHS1( 8) -= (f_flux - b_flux);
 _RHS1( 20) += (f_flux - b_flux);
 
 _term =  Ca * kCa1 ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 20 ,8)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 8 ,20)  -= _term;
 _MATELM1( 20 ,20)  += _term;
 /*REACTION*/
  /* ~ A2R <-> cA2R ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2R ;
 b_flux =  kCa0 * cA2R ;
 _RHS1( 7) -= (f_flux - b_flux);
 _RHS1( 19) += (f_flux - b_flux);
 
 _term =  Ca * kCa1 ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 19 ,7)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 7 ,19)  -= _term;
 _MATELM1( 19 ,19)  += _term;
 /*REACTION*/
  /* ~ A2Rd <-> cA2Rd ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2Rd ;
 b_flux =  kCa0 * cA2Rd ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 13) += (f_flux - b_flux);
 
 _term =  Ca * kCa1 ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 13 ,6)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 6 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 /*REACTION*/
  /* ~ A2Ro <-> cA2Ro ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2Ro ;
 b_flux =  kCa0 * cA2Ro ;
 _RHS1( 5) -= (f_flux - b_flux);
 _RHS1( 18) += (f_flux - b_flux);
 
 _term =  Ca * kCa1 ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 18 ,5)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 5 ,18)  -= _term;
 _MATELM1( 18 ,18)  += _term;
 /*REACTION*/
  /* ~ RM <-> cRM ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * RM ;
 b_flux =  kCa0 * cRM ;
 _RHS1( 9) -= (f_flux - b_flux);
 _RHS1( 17) += (f_flux - b_flux);
 
 _term =  Ca * kCa1 ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 17 ,9)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 9 ,17)  -= _term;
 _MATELM1( 17 ,17)  += _term;
 /*REACTION*/
  /* ~ ARM <-> cARM ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * ARM ;
 b_flux =  kCa0 * cARM ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 16) += (f_flux - b_flux);
 
 _term =  Ca * kCa1 ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 16 ,4)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 4 ,16)  -= _term;
 _MATELM1( 16 ,16)  += _term;
 /*REACTION*/
  /* ~ A2RM <-> cA2RM ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2RM ;
 b_flux =  kCa0 * cA2RM ;
 _RHS1( 3) -= (f_flux - b_flux);
 _RHS1( 15) += (f_flux - b_flux);
 
 _term =  Ca * kCa1 ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 15 ,3)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 3 ,15)  -= _term;
 _MATELM1( 15 ,15)  += _term;
 /*REACTION*/
  /* ~ A2RdM <-> cA2RdM ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2RdM ;
 b_flux =  kCa0 * cA2RdM ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 14) += (f_flux - b_flux);
 
 _term =  Ca * kCa1 ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 14 ,2)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 2 ,14)  -= _term;
 _MATELM1( 14 ,14)  += _term;
 /*REACTION*/
  /* ~ A2RoM <-> cA2RoM ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2RoM ;
 b_flux =  kCa0 * cA2RoM ;
 _RHS1( 1) -= (f_flux - b_flux);
 
 _term =  Ca * kCa1 ;
 _MATELM1( 1 ,1)  += _term;
 _term =  kCa0 ;
 _MATELM1( 1 ,0)  -= _term;
 /*REACTION*/
   /* R + AR + A2R + A2Rd + A2Ro + RM + ARM + A2RM + A2RdM + A2RoM + cR + cAR + cA2R + cA2Rd + cA2Rcdd + cA2Ro + cRM + cARM + cA2RM + cA2RdM + cA2RcddM + cA2RoM = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= cA2RoM ;
 _MATELM1(0, 11) = 1;
 _RHS1(0) -= cA2RcddM ;
 _MATELM1(0, 14) = 1;
 _RHS1(0) -= cA2RdM ;
 _MATELM1(0, 15) = 1;
 _RHS1(0) -= cA2RM ;
 _MATELM1(0, 16) = 1;
 _RHS1(0) -= cARM ;
 _MATELM1(0, 17) = 1;
 _RHS1(0) -= cRM ;
 _MATELM1(0, 18) = 1;
 _RHS1(0) -= cA2Ro ;
 _MATELM1(0, 12) = 1;
 _RHS1(0) -= cA2Rcdd ;
 _MATELM1(0, 13) = 1;
 _RHS1(0) -= cA2Rd ;
 _MATELM1(0, 19) = 1;
 _RHS1(0) -= cA2R ;
 _MATELM1(0, 20) = 1;
 _RHS1(0) -= cAR ;
 _MATELM1(0, 21) = 1;
 _RHS1(0) -= cR ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= A2RoM ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= A2RdM ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= A2RM ;
 _MATELM1(0, 4) = 1;
 _RHS1(0) -= ARM ;
 _MATELM1(0, 9) = 1;
 _RHS1(0) -= RM ;
 _MATELM1(0, 5) = 1;
 _RHS1(0) -= A2Ro ;
 _MATELM1(0, 6) = 1;
 _RHS1(0) -= A2Rd ;
 _MATELM1(0, 7) = 1;
 _RHS1(0) -= A2R ;
 _MATELM1(0, 8) = 1;
 _RHS1(0) -= AR ;
 _MATELM1(0, 10) = 1;
 _RHS1(0) -= R ;
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE ode begin*/
 static int _ode_spec1(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<22;_i++) _p[_dlist1[_i]] = 0.0;}
 /* ~ A + R <-> AR ( 2.0 * ka1 , ka0 )*/
 f_flux =  2.0 * ka1 * R * A ;
 b_flux =  ka0 * AR ;
 DR -= (f_flux - b_flux);
 DAR += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A + AR <-> A2R ( ka1 , 2.0 * ka0 )*/
 f_flux =  ka1 * AR * A ;
 b_flux =  2.0 * ka0 * A2R ;
 DAR -= (f_flux - b_flux);
 DA2R += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A2R <-> A2Ro ( kg1 , kg0 )*/
 f_flux =  kg1 * A2R ;
 b_flux =  kg0 * A2Ro ;
 DA2R -= (f_flux - b_flux);
 DA2Ro += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A2R <-> A2Rd ( kd1 , kd0 )*/
 f_flux =  kd1 * A2R ;
 b_flux =  kd0 * A2Rd ;
 DA2R -= (f_flux - b_flux);
 DA2Rd += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A2Ro + M <-> A2RoM ( k1M , k0M )*/
 f_flux =  k1M * M * A2Ro ;
 b_flux =  k0M * A2RoM ;
 DA2Ro -= (f_flux - b_flux);
 DA2RoM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A + RM <-> ARM ( 2.0 * ka1M , ka0M )*/
 f_flux =  2.0 * ka1M * RM * A ;
 b_flux =  ka0M * ARM ;
 DRM -= (f_flux - b_flux);
 DARM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A + ARM <-> A2RM ( ka1M , 2.0 * ka0M )*/
 f_flux =  ka1M * ARM * A ;
 b_flux =  2.0 * ka0M * A2RM ;
 DARM -= (f_flux - b_flux);
 DA2RM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A2RM <-> A2RoM ( kg1M , kg0M )*/
 f_flux =  kg1M * A2RM ;
 b_flux =  kg0M * A2RoM ;
 DA2RM -= (f_flux - b_flux);
 DA2RoM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A2RM <-> A2RdM ( kd1M , kd0M )*/
 f_flux =  kd1M * A2RM ;
 b_flux =  kd0M * A2RdM ;
 DA2RM -= (f_flux - b_flux);
 DA2RdM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A + cR <-> cAR ( 2.0 * ka1 , ka0 )*/
 f_flux =  2.0 * ka1 * cR * A ;
 b_flux =  ka0 * cAR ;
 DcR -= (f_flux - b_flux);
 DcAR += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A + cAR <-> cA2R ( ka1 , 2.0 * ka0 )*/
 f_flux =  ka1 * cAR * A ;
 b_flux =  2.0 * ka0 * cA2R ;
 DcAR -= (f_flux - b_flux);
 DcA2R += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ cA2R <-> cA2Ro ( kg1 , kg0 )*/
 f_flux =  kg1 * cA2R ;
 b_flux =  kg0 * cA2Ro ;
 DcA2R -= (f_flux - b_flux);
 DcA2Ro += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ cA2R <-> cA2Rd ( kd1 , kd0 )*/
 f_flux =  kd1 * cA2R ;
 b_flux =  kd0 * cA2Rd ;
 DcA2R -= (f_flux - b_flux);
 DcA2Rd += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ cA2R <-> cA2Rcdd ( kcdd1 , kcdd0 )*/
 f_flux =  kcdd1 * cA2R ;
 b_flux =  kcdd0 * cA2Rcdd ;
 DcA2R -= (f_flux - b_flux);
 DcA2Rcdd += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ cA2Ro + M <-> cA2RoM ( k1M , k0M )*/
 f_flux =  k1M * M * cA2Ro ;
 b_flux =  k0M * cA2RoM ;
 DcA2Ro -= (f_flux - b_flux);
 DcA2RoM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A + cRM <-> cARM ( 2.0 * ka1M , ka0M )*/
 f_flux =  2.0 * ka1M * cRM * A ;
 b_flux =  ka0M * cARM ;
 DcRM -= (f_flux - b_flux);
 DcARM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A + cARM <-> cA2RM ( ka1M , 2.0 * ka0M )*/
 f_flux =  ka1M * cARM * A ;
 b_flux =  2.0 * ka0M * cA2RM ;
 DcARM -= (f_flux - b_flux);
 DcA2RM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ cA2RM <-> cA2RoM ( kg1M , kg0M )*/
 f_flux =  kg1M * cA2RM ;
 b_flux =  kg0M * cA2RoM ;
 DcA2RM -= (f_flux - b_flux);
 DcA2RoM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ cA2RM <-> cA2RdM ( kd1M , kd0M )*/
 f_flux =  kd1M * cA2RM ;
 b_flux =  kd0M * cA2RdM ;
 DcA2RM -= (f_flux - b_flux);
 DcA2RdM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ cA2RM <-> cA2RcddM ( kcdd1M , kcdd0M )*/
 f_flux =  kcdd1M * cA2RM ;
 b_flux =  kcdd0M * cA2RcddM ;
 DcA2RM -= (f_flux - b_flux);
 DcA2RcddM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ R <-> cR ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * R ;
 b_flux =  kCa0 * cR ;
 DR -= (f_flux - b_flux);
 DcR += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ AR <-> cAR ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * AR ;
 b_flux =  kCa0 * cAR ;
 DAR -= (f_flux - b_flux);
 DcAR += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A2R <-> cA2R ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2R ;
 b_flux =  kCa0 * cA2R ;
 DA2R -= (f_flux - b_flux);
 DcA2R += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A2Rd <-> cA2Rd ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2Rd ;
 b_flux =  kCa0 * cA2Rd ;
 DA2Rd -= (f_flux - b_flux);
 DcA2Rd += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A2Ro <-> cA2Ro ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2Ro ;
 b_flux =  kCa0 * cA2Ro ;
 DA2Ro -= (f_flux - b_flux);
 DcA2Ro += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ RM <-> cRM ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * RM ;
 b_flux =  kCa0 * cRM ;
 DRM -= (f_flux - b_flux);
 DcRM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ ARM <-> cARM ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * ARM ;
 b_flux =  kCa0 * cARM ;
 DARM -= (f_flux - b_flux);
 DcARM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A2RM <-> cA2RM ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2RM ;
 b_flux =  kCa0 * cA2RM ;
 DA2RM -= (f_flux - b_flux);
 DcA2RM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A2RdM <-> cA2RdM ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2RdM ;
 b_flux =  kCa0 * cA2RdM ;
 DA2RdM -= (f_flux - b_flux);
 DcA2RdM += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ A2RoM <-> cA2RoM ( Ca * kCa1 , kCa0 )*/
 f_flux =  Ca * kCa1 * A2RoM ;
 b_flux =  kCa0 * cA2RoM ;
 DA2RoM -= (f_flux - b_flux);
 DcA2RoM += (f_flux - b_flux);
 
 /*REACTION*/
   /* R + AR + A2R + A2Rd + A2Ro + RM + ARM + A2RM + A2RdM + A2RoM + cR + cAR + cA2R + cA2Rd + cA2Rcdd + cA2Ro + cRM + cARM + cA2RM + cA2RdM + cA2RcddM + cA2RoM = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1(void* _so, double* _rhs, double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<22;_i++){
  	_RHS1(_i) = _dt1*(_p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 /* ~ A + R <-> AR ( 2.0 * ka1 , ka0 )*/
 _term =  2.0 * ka1 * A ;
 _MATELM1( 10 ,10)  += _term;
 _MATELM1( 8 ,10)  -= _term;
 _term =  ka0 ;
 _MATELM1( 10 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ A + AR <-> A2R ( ka1 , 2.0 * ka0 )*/
 _term =  ka1 * A ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 7 ,8)  -= _term;
 _term =  2.0 * ka0 ;
 _MATELM1( 8 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ A2R <-> A2Ro ( kg1 , kg0 )*/
 _term =  kg1 ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 5 ,7)  -= _term;
 _term =  kg0 ;
 _MATELM1( 7 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ A2R <-> A2Rd ( kd1 , kd0 )*/
 _term =  kd1 ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 6 ,7)  -= _term;
 _term =  kd0 ;
 _MATELM1( 7 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ A2Ro + M <-> A2RoM ( k1M , k0M )*/
 _term =  k1M * M ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 1 ,5)  -= _term;
 _term =  k0M ;
 _MATELM1( 5 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ A + RM <-> ARM ( 2.0 * ka1M , ka0M )*/
 _term =  2.0 * ka1M * A ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 4 ,9)  -= _term;
 _term =  ka0M ;
 _MATELM1( 9 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ A + ARM <-> A2RM ( ka1M , 2.0 * ka0M )*/
 _term =  ka1M * A ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 3 ,4)  -= _term;
 _term =  2.0 * ka0M ;
 _MATELM1( 4 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ A2RM <-> A2RoM ( kg1M , kg0M )*/
 _term =  kg1M ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 1 ,3)  -= _term;
 _term =  kg0M ;
 _MATELM1( 3 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ A2RM <-> A2RdM ( kd1M , kd0M )*/
 _term =  kd1M ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 2 ,3)  -= _term;
 _term =  kd0M ;
 _MATELM1( 3 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ A + cR <-> cAR ( 2.0 * ka1 , ka0 )*/
 _term =  2.0 * ka1 * A ;
 _MATELM1( 21 ,21)  += _term;
 _MATELM1( 20 ,21)  -= _term;
 _term =  ka0 ;
 _MATELM1( 21 ,20)  -= _term;
 _MATELM1( 20 ,20)  += _term;
 /*REACTION*/
  /* ~ A + cAR <-> cA2R ( ka1 , 2.0 * ka0 )*/
 _term =  ka1 * A ;
 _MATELM1( 20 ,20)  += _term;
 _MATELM1( 19 ,20)  -= _term;
 _term =  2.0 * ka0 ;
 _MATELM1( 20 ,19)  -= _term;
 _MATELM1( 19 ,19)  += _term;
 /*REACTION*/
  /* ~ cA2R <-> cA2Ro ( kg1 , kg0 )*/
 _term =  kg1 ;
 _MATELM1( 19 ,19)  += _term;
 _MATELM1( 18 ,19)  -= _term;
 _term =  kg0 ;
 _MATELM1( 19 ,18)  -= _term;
 _MATELM1( 18 ,18)  += _term;
 /*REACTION*/
  /* ~ cA2R <-> cA2Rd ( kd1 , kd0 )*/
 _term =  kd1 ;
 _MATELM1( 19 ,19)  += _term;
 _MATELM1( 13 ,19)  -= _term;
 _term =  kd0 ;
 _MATELM1( 19 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 /*REACTION*/
  /* ~ cA2R <-> cA2Rcdd ( kcdd1 , kcdd0 )*/
 _term =  kcdd1 ;
 _MATELM1( 19 ,19)  += _term;
 _MATELM1( 12 ,19)  -= _term;
 _term =  kcdd0 ;
 _MATELM1( 19 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ cA2Ro + M <-> cA2RoM ( k1M , k0M )*/
 _term =  k1M * M ;
 _MATELM1( 18 ,18)  += _term;
 _MATELM1( 0 ,18)  -= _term;
 _term =  k0M ;
 _MATELM1( 18 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ A + cRM <-> cARM ( 2.0 * ka1M , ka0M )*/
 _term =  2.0 * ka1M * A ;
 _MATELM1( 17 ,17)  += _term;
 _MATELM1( 16 ,17)  -= _term;
 _term =  ka0M ;
 _MATELM1( 17 ,16)  -= _term;
 _MATELM1( 16 ,16)  += _term;
 /*REACTION*/
  /* ~ A + cARM <-> cA2RM ( ka1M , 2.0 * ka0M )*/
 _term =  ka1M * A ;
 _MATELM1( 16 ,16)  += _term;
 _MATELM1( 15 ,16)  -= _term;
 _term =  2.0 * ka0M ;
 _MATELM1( 16 ,15)  -= _term;
 _MATELM1( 15 ,15)  += _term;
 /*REACTION*/
  /* ~ cA2RM <-> cA2RoM ( kg1M , kg0M )*/
 _term =  kg1M ;
 _MATELM1( 15 ,15)  += _term;
 _MATELM1( 0 ,15)  -= _term;
 _term =  kg0M ;
 _MATELM1( 15 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ cA2RM <-> cA2RdM ( kd1M , kd0M )*/
 _term =  kd1M ;
 _MATELM1( 15 ,15)  += _term;
 _MATELM1( 14 ,15)  -= _term;
 _term =  kd0M ;
 _MATELM1( 15 ,14)  -= _term;
 _MATELM1( 14 ,14)  += _term;
 /*REACTION*/
  /* ~ cA2RM <-> cA2RcddM ( kcdd1M , kcdd0M )*/
 _term =  kcdd1M ;
 _MATELM1( 15 ,15)  += _term;
 _MATELM1( 11 ,15)  -= _term;
 _term =  kcdd0M ;
 _MATELM1( 15 ,11)  -= _term;
 _MATELM1( 11 ,11)  += _term;
 /*REACTION*/
  /* ~ R <-> cR ( Ca * kCa1 , kCa0 )*/
 _term =  Ca * kCa1 ;
 _MATELM1( 10 ,10)  += _term;
 _MATELM1( 21 ,10)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 10 ,21)  -= _term;
 _MATELM1( 21 ,21)  += _term;
 /*REACTION*/
  /* ~ AR <-> cAR ( Ca * kCa1 , kCa0 )*/
 _term =  Ca * kCa1 ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 20 ,8)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 8 ,20)  -= _term;
 _MATELM1( 20 ,20)  += _term;
 /*REACTION*/
  /* ~ A2R <-> cA2R ( Ca * kCa1 , kCa0 )*/
 _term =  Ca * kCa1 ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 19 ,7)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 7 ,19)  -= _term;
 _MATELM1( 19 ,19)  += _term;
 /*REACTION*/
  /* ~ A2Rd <-> cA2Rd ( Ca * kCa1 , kCa0 )*/
 _term =  Ca * kCa1 ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 13 ,6)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 6 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 /*REACTION*/
  /* ~ A2Ro <-> cA2Ro ( Ca * kCa1 , kCa0 )*/
 _term =  Ca * kCa1 ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 18 ,5)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 5 ,18)  -= _term;
 _MATELM1( 18 ,18)  += _term;
 /*REACTION*/
  /* ~ RM <-> cRM ( Ca * kCa1 , kCa0 )*/
 _term =  Ca * kCa1 ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 17 ,9)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 9 ,17)  -= _term;
 _MATELM1( 17 ,17)  += _term;
 /*REACTION*/
  /* ~ ARM <-> cARM ( Ca * kCa1 , kCa0 )*/
 _term =  Ca * kCa1 ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 16 ,4)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 4 ,16)  -= _term;
 _MATELM1( 16 ,16)  += _term;
 /*REACTION*/
  /* ~ A2RM <-> cA2RM ( Ca * kCa1 , kCa0 )*/
 _term =  Ca * kCa1 ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 15 ,3)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 3 ,15)  -= _term;
 _MATELM1( 15 ,15)  += _term;
 /*REACTION*/
  /* ~ A2RdM <-> cA2RdM ( Ca * kCa1 , kCa0 )*/
 _term =  Ca * kCa1 ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 14 ,2)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 2 ,14)  -= _term;
 _MATELM1( 14 ,14)  += _term;
 /*REACTION*/
  /* ~ A2RoM <-> cA2RoM ( Ca * kCa1 , kCa0 )*/
 _term =  Ca * kCa1 ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 0 ,1)  -= _term;
 _term =  kCa0 ;
 _MATELM1( 1 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
   /* R + AR + A2R + A2Rd + A2Ro + RM + ARM + A2RM + A2RdM + A2RoM + cR + cAR + cA2R + cA2Rd + cA2Rcdd + cA2Ro + cRM + cARM + cA2RM + cA2RdM + cA2RcddM + cA2RoM = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 22;}
 
static void _ode_spec(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 22; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _cvode_sparse_thread(&_thread[_cvspth1]._pvoid, 22, _dlist1, _p, _ode_matsol1, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _thread_cleanup(Datum* _thread) {
   _nrn_destroy_sparseobj_thread(_thread[_cvspth1]._pvoid);
   _nrn_destroy_sparseobj_thread(_thread[_spth1]._pvoid);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
  int _i; double _save;{
  A2RoM = A2RoM0;
  A2RdM = A2RdM0;
  A2RM = A2RM0;
  ARM = ARM0;
  A2Ro = A2Ro0;
  A2Rd = A2Rd0;
  A2R = A2R0;
  AR = AR0;
  Popen = Popen0;
  RM = RM0;
  R = R0;
  cA2RcddM = cA2RcddM0;
  cA2Rcdd = cA2Rcdd0;
  cA2Rd = cA2Rd0;
  cA2RoM = cA2RoM0;
  cA2RdM = cA2RdM0;
  cA2RM = cA2RM0;
  cARM = cARM0;
  cRM = cRM0;
  cA2Ro = cA2Ro0;
  cA2R = cA2R0;
  cAR = cAR0;
  cR = cR0;
 {
   R = 1.0 ;
   }
 
}
}

static void nrn_init(NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   Vm = - 65.0 ;
   Popen = A2Ro + cA2Ro ;
   Inmda = nrecepts * Popen * scg * ( ( Vm - Vrev ) / 1000.0 ) ;
   }
 _current += Inmda;

} return _current;
}

static void nrn_cur(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
double _dtsav = dt;
if (secondorder) { dt *= 0.5; }
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {  sparse_thread(&_thread[_spth1]._pvoid, 22, _slist1, _dlist1, _p, &t, dt, kstates, _linmat1, _ppvar, _thread, _nt);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 22; ++_i) {
      _p[_slist1[_i]] += dt*_p[_dlist1[_i]];
    }}
 }}}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = cA2RoM_columnindex;  _dlist1[0] = DcA2RoM_columnindex;
 _slist1[1] = A2RoM_columnindex;  _dlist1[1] = DA2RoM_columnindex;
 _slist1[2] = A2RdM_columnindex;  _dlist1[2] = DA2RdM_columnindex;
 _slist1[3] = A2RM_columnindex;  _dlist1[3] = DA2RM_columnindex;
 _slist1[4] = ARM_columnindex;  _dlist1[4] = DARM_columnindex;
 _slist1[5] = A2Ro_columnindex;  _dlist1[5] = DA2Ro_columnindex;
 _slist1[6] = A2Rd_columnindex;  _dlist1[6] = DA2Rd_columnindex;
 _slist1[7] = A2R_columnindex;  _dlist1[7] = DA2R_columnindex;
 _slist1[8] = AR_columnindex;  _dlist1[8] = DAR_columnindex;
 _slist1[9] = RM_columnindex;  _dlist1[9] = DRM_columnindex;
 _slist1[10] = R_columnindex;  _dlist1[10] = DR_columnindex;
 _slist1[11] = cA2RcddM_columnindex;  _dlist1[11] = DcA2RcddM_columnindex;
 _slist1[12] = cA2Rcdd_columnindex;  _dlist1[12] = DcA2Rcdd_columnindex;
 _slist1[13] = cA2Rd_columnindex;  _dlist1[13] = DcA2Rd_columnindex;
 _slist1[14] = cA2RdM_columnindex;  _dlist1[14] = DcA2RdM_columnindex;
 _slist1[15] = cA2RM_columnindex;  _dlist1[15] = DcA2RM_columnindex;
 _slist1[16] = cARM_columnindex;  _dlist1[16] = DcARM_columnindex;
 _slist1[17] = cRM_columnindex;  _dlist1[17] = DcRM_columnindex;
 _slist1[18] = cA2Ro_columnindex;  _dlist1[18] = DcA2Ro_columnindex;
 _slist1[19] = cA2R_columnindex;  _dlist1[19] = DcA2R_columnindex;
 _slist1[20] = cAR_columnindex;  _dlist1[20] = DcAR_columnindex;
 _slist1[21] = cR_columnindex;  _dlist1[21] = DcR_columnindex;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "CaPlaneNMDARwMem.mod";
static const char* nmodl_file_text = 
  ": Altered version of channel block model described in Glasgow et al. (2017) JNeuro paper.\n"
  ": Model originally based on Erreger, Dravid, Banke, Wyllie, & Traynelis (2005). \n"
  ": Built from 2OpStCaNMDAR.mod\n"
  ": 2 Symmetric \"Arms\" - Mem-bound and unbound. \n"
  ": 2 \"Planes\" - Ca2+-bound and unbound. Transition b/t planes is dependent on intracellular [Ca2+]. Ca2+-bound plane possesses additional\n"
  ": desensitized state, otherwise rates are identical b/t planes (prevents violation of microscopic reversibility).\n"
  ": \n"
  ": \n"
  ":\n"
  ":                                        SUMMARY OF MODEL AND RATE CONSTANTS\n"
  ":\n"
  ":\n"
  ":                                     A2Rd                                      A2RdM\n"
  ":                                   /   |                                        |    \\\n"
  ":                                  /    |                                        |     \\\n"
  ":                                 /     |                                    kd1M|      \\\n"
  ":                                |   kd1|kd0                                     |kd0M   |\n"
  ":                 +2A          +A       |                  +M                    |       |\n"
  ":                 2*ka1        ka1      |     kg1          k1M           kg0M    |    2*ka0M        ka0M\n"
  ":             R <------> AR <--------> A2R <------> A2Ro <---_--> A2RoM <----> A2RM <--------> ARM <------> RM\n"
  ":             |   ka0    |   2*ka0      |     kg0    |     k0M     |     kg1M    |     ka1M     |   2*ka1M  |\n"
  ":             |          |       |      |            |             |             |     +A       |    +2A    |\n"
  ":             |kCa0      |kCa0   |      |kCa0        |kCa0         |kCa0         |kCa0   |      |kCa0       |kCa0\n"
  ":         kCa1|      kCa1|   kCa1|kCa0  |        kCa1|         kCa1|             |       |  kCa1|       kCa1|\n"
  ":             |          |       |      |            |             |         kCa1|       |      |           |\n"
  ":             |          |       |  kCa1|            |             |             |   kCa1|kCa0  |           |\n"
  ":             |  +2A     |     +A       |            |     +M      |             |       |      |           |\n"
  ":             |  2*ka1   |     ka1      |     kg1    |     k1M     |     kg0M    |     2*ka0M   |    ka0M   |\n"
  ":            cR <-----> cAR <--------> cA2R <-----> cA2Ro <----> cA2RoM <----> cA2RM <-------> cARM <----> cRM\n"
  ":                ka0         2*ka0      / \\   kg0          k0M           kg1M   / \\     ka1M         2*ka1M\n"
  ":                                |     /   \\                                   /   \\    +A           +2A\n"
  ":                                | kd1/     \\                           kcdd0M/     \\    |\n"
  ":                                |   /kd0    \\kcdd0                          /kcdd1M \\kd1M\n"
  ":                                |  /    kcdd1\\                             /     kd0M\\  |\n"
  ":                                | /           \\                           /           \\ |\n"
  ":                              cA2Rd         cA2Rcdd                    cA2RcddM        cA2RdM  \n"
  ":\n"
  ":       \n"
  "NEURON {\n"
  "    POINT_PROCESS CaPlaneNMDARwMem\n"
  "	RANGE A, M, Ca, Vm, nrecepts, scg, Vrev, Popen\n"
  "	RANGE ka1, ka0, kd1, kd0, kg0, kg1, kcdd1, kcdd0\n"
  "    RANGE k1M, k0M, ka1M, ka0M, kd1M, kd0M, kg0M, kg1M, kcdd1M, kcdd0M\n"
  "    RANGE R, AR, A2R, A2Rd, A2Ro \n"
  "    RANGE RM, ARM, A2RM, A2RdM, A2RoM\n"
  "    RANGE kCa1, kCa0, cR, cAR, cA2R, cA2dR, A2Rcdd, cA2Ro \n"
  "    RANGE cRM, cARM, cA2RM, cA2RdM, A2RcddM, cA2RoM\n"
  "	NONSPECIFIC_CURRENT Inmda\n"
  "	THREADSAFE\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(nA) = (nanoamp)\n"
  "	(mV) = (millivolt)\n"
  "	(uS) = (microsiemens)\n"
  "    (pS) = (picosiemens)\n"
  "	(umho) = (micromho)\n"
  "	(mM) = (milli/liter)\n"
  "	(uM) = (micro/liter)\n"
  "}\n"
  "\n"
  "\n"
  "PARAMETER\n"
  "{    \n"
  ":  CURRENT CONTROL\n"
  "    nrecepts = 300                    :number of NMDA receptors\n"
  "    scg = 50             (pS)         :single channel conductance \n"
  "    Vrev = 0             (mV)         :reversal potential\n"
  "     \n"
  ":RATES\n"
  ": Transition b/t planes\n"
  "    kCa1 = 1e-3         (uM-1 s-1)    :Ca \"binding\"\n"
  "    kCa0 = 0.75e-3      (ms-1)        :Ca \"unbinding\"\n"
  "\n"
  ":Unblocked Arm\n"
  "    ka1 = 31.6           (uM-1 s-1)   :Agonist binding\n"
  "    ka0 = 1010e-3        (ms-1)       :Agonist unbinding\n"
  ":  Gating\n"
  "    kg1 = 4772.41e-3     (ms-1)       :opening\n"
  "    kg0 = 557e-3         (ms-1)       :closing  \n"
  " \n"
  ":  Desensitization\n"
  "    kcdd1 = 32.12e-3     (ms-1)       :into Ca-dependent desen\n"
  "    kcdd0 = 2.92e-3      (ms-1)       :out of CDD\n"
  "    kd1 = 3e-3        (ms-1)       :into desens\n"
  "    kd0 = 3e-3        (ms-1)       :out of desens\n"
  "\n"
  ":Blocked Arm\n"
  "    k1M = 1e-3           (uM-1 s-1)  :Block rate\n"
  "    k0M = 1e-3           (ms-1)      :Unblock rate\n"
  "    ka1M = 31.6           (uM-1 s-1)  :Agonist binding\n"
  "    ka0M = 1010e-3        (ms-1)      :Agonist unbinding\n"
  ":  Gating                                                \n"
  "    kg1M = 4772.41e-3     (ms-1)      :opening\n"
  "    kg0M = 53.017e-3      (ms-1)      :closing  \n"
  ":  Desensitization\n"
  "    kcdd1M = 64.24e-3     (ms-1)      :into Ca-dependent desen\n"
  "    kcdd0M = 0.6e-3       (ms-1)      :out of CDD\n"
  "    kd1M = 0.54e-3        (ms-1)      :into desens\n"
  "    kd0M = 0.0125e-3      (ms-1)      :out of desens\n"
  "  \n"
  "}\n"
  "\n"
  "\n"
  "ASSIGNED \n"
  "{\n"
  "  A       (uM)        :FROM 0 TO 1000\n"
  "  M       (uM)        :FROM 0 TO 100\n"
  "  Inmda   (pA)        :FROM -1000 TO 10 (pA)\n"
  "  Vm      (mV)        :FROM -70 TO 40 (mV)\n"
  "  Ca      (uM)        :FROM 0 TO 100\n"
  "    \n"
  "}\n"
  "\n"
  "STATE\n"
  "\n"
  "{: Value of each state = fraction of all receptors in state\n"
  "\n"
  ":Unbound Plane   \n"
  "    :Unblocked arm\n"
  "      R       \n"
  "      AR     \n"
  "      A2R  \n"
  "      A2Rd      \n"
  "      A2Ro   \n"
  "    :Blocked arm\n"
  "      RM      \n"
  "      ARM\n"
  "      A2RM\n"
  "      A2RdM    \n"
  "      A2RoM   \n"
  ":Ca2+-bound plane\n"
  "     :Unblocked arm\n"
  "      cR       \n"
  "      cAR     \n"
  "      cA2R  \n"
  "      cA2Rd\n"
  "      cA2Rcdd\n"
  "      cA2Ro   \n"
  "    :Blocked arm\n"
  "      cRM      \n"
  "      cARM \n"
  "      cA2RM\n"
  "      cA2RdM\n"
  "      cA2RcddM\n"
  "      cA2RoM   \n"
  ":General\n"
  "      Popen\n"
  "}\n"
  "\n"
  "INITIAL\n"
  "{\n"
  "  R = 1\n"
  "}\n"
  "\n"
  "BREAKPOINT\n"
  "{\n"
  "    SOLVE kstates METHOD sparse\n"
  "   \n"
  "    Vm = -65\n"
  "    \n"
  "    Popen = A2Ro + cA2Ro\n"
  "    Inmda = nrecepts * Popen * scg * ((Vm - Vrev)/1000)\n"
  "}\n"
  "\n"
  "KINETIC kstates {\n"
  ":Unbound Plane\n"
  "   :Unblocked, agonist binding\n"
  "       ~ A + R <-> AR           (2*ka1, ka0)\n"
  "       ~ A + AR <-> A2R         (ka1, 2*ka0)\n"
  "   :Unblocked, gating\n"
  "       ~ A2R <-> A2Ro           (kg1, kg0)\n"
  "   :Unblocked, desensitization\n"
  "       ~ A2R <-> A2Rd           (kd1, kd0)\n"
  "   :Blocker binding\n"
  "       ~ A2Ro + M <-> A2RoM     (k1M, k0M)\n"
  "   :Blocked, agonist binding\n"
  "       ~ A + RM <-> ARM         (2*ka1M, ka0M)\n"
  "       ~ A + ARM <-> A2RM       (ka1M, 2*ka0M)\n"
  "   :Blocked, gating\n"
  "       ~ A2RM <-> A2RoM         (kg1M, kg0M)\n"
  "   :Blocked, desensitization\n"
  "       ~ A2RM <-> A2RdM         (kd1M, kd0M)\n"
  ":Ca2+-bound Plane\n"
  "   :Unblocked, agonist binding\n"
  "       ~ A + cR <-> cAR         (2*ka1, ka0)\n"
  "       ~ A + cAR <-> cA2R       (ka1, 2*ka0)\n"
  "   :Unblocked, gating\n"
  "       ~ cA2R <-> cA2Ro         (kg1, kg0)\n"
  "   :Unblocked, desensitization\n"
  "       ~ cA2R <-> cA2Rd         (kd1, kd0)\n"
  "       ~ cA2R <-> cA2Rcdd        (kcdd1, kcdd0)\n"
  "   :Blocker binding\n"
  "       ~ cA2Ro + M <-> cA2RoM   (k1M, k0M)\n"
  "   :Blocked, agonist binding\n"
  "       ~ A + cRM <-> cARM       (2*ka1M, ka0M)\n"
  "       ~ A + cARM <-> cA2RM     (ka1M, 2*ka0M)\n"
  "   :Blocked, gating\n"
  "       ~ cA2RM <-> cA2RoM       (kg1M, kg0M)\n"
  "   :Blocked, desensitization\n"
  "       ~ cA2RM <-> cA2RdM       (kd1M, kd0M)\n"
  "       ~ cA2RM <-> cA2RcddM      (kcdd1M, kcdd0M)\n"
  ":Transitions b/t Planes\n"
  "       ~ R <-> cR              (Ca*kCa1, kCa0)\n"
  "       ~ AR <-> cAR            (Ca*kCa1, kCa0)\n"
  "       ~ A2R <-> cA2R          (Ca*kCa1, kCa0)\n"
  "       ~ A2Rd <-> cA2Rd        (Ca*kCa1, kCa0)\n"
  "       ~ A2Ro <-> cA2Ro        (Ca*kCa1, kCa0)\n"
  "       ~ RM <-> cRM            (Ca*kCa1, kCa0)\n"
  "       ~ ARM <-> cARM          (Ca*kCa1, kCa0)\n"
  "       ~ A2RM <-> cA2RM        (Ca*kCa1, kCa0)\n"
  "       ~ A2RdM <-> cA2RdM      (Ca*kCa1, kCa0)\n"
  "       ~ A2RoM <-> cA2RoM      (Ca*kCa1, kCa0)\n"
  "          \n"
  "CONSERVE R + AR + A2R + A2Rd + A2Ro + RM + ARM + A2RM + A2RdM + A2RoM + cR + cAR + cA2R + cA2Rd + cA2Rcdd + cA2Ro + cRM + cARM + cA2RM + cA2RdM + cA2RcddM + cA2RoM = 1\n"
  "}  \n"
  "\n"
  ;
#endif
