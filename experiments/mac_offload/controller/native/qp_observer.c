#include <math.h>
#include <string.h>
#include "acados_c/ocp_nlp_interface.h"
#include "acados/ocp_nlp/ocp_nlp_sqp.h"
#include "blasfeo_d_aux.h"
/* Phase 0: newly evaluated iterate, before QP. Phase 3: convergence exit.
 * MAXITER leaves a newer nlp_out with OLD residuals, so never emit that point.
 * All values below are read from caches; no model or residual evaluation. */
typedef void(*callback_t)(int,int,int,double,double,double,double,double,double,double);
typedef int(*evaluate_t)(void*,ocp_qp_xcond_solver_dims*,ocp_qp_in*,ocp_qp_out*,void*,void*,void*);
static callback_t cb=0;static evaluate_t original=0;static ocp_nlp_solver *active=0;
static int count=0,abort_requested=0,proximity_count=0,proximity_rows[64];
int qp_observer_api_version(void){return 3;}
/* Original output-feasibility norm, including the ORCA grad_Lagr and lambda*g
 * equations already embedded in h. Unlike NLP res_stat/res_comp this does not
 * add the OUTER objective's stationarity or interior-point complementarity. */
double qp_observer_cached_feasibility(void){
 if(!active)return NAN;
 ocp_nlp_dims *dims=active->dims;
 ocp_nlp_sqp_memory *s=active->mem;ocp_nlp_memory *m=s->nlp_mem;
 double maximum=0.;
 for(int i=0;i<=dims->N;i++){
  for(int j=0;j<2*dims->ni[i];j++){
   double v=blasfeo_dvecex1(m->ineq_fun+i,j);if(!isfinite(v))return NAN;
   maximum=fmax(maximum,v);
  }
  if(i<dims->N)for(int j=0;j<dims->nx[i+1];j++){
   double v=blasfeo_dvecex1(m->dyn_fun+i,j);if(!isfinite(v))return NAN;
   maximum=fmax(maximum,fabs(v));
  }
 }
 return maximum;
}
double qp_observer_cached_h_rows_max(int n,const int *rows){
 if(!active)return NAN;
 ocp_nlp_dims *dims=active->dims;
 ocp_nlp_sqp_memory *s=active->mem;ocp_nlp_memory *m=s->nlp_mem;
 double maximum=0.;
 for(int i=0;i<dims->N;i++){
  int nb=0,ng=0,nh=0;
  active->config->constraints[i]->dims_get(active->config->constraints[i],dims->constraints[i],"nb",&nb);
  active->config->constraints[i]->dims_get(active->config->constraints[i],dims->constraints[i],"ng",&ng);
  active->config->constraints[i]->dims_get(active->config->constraints[i],dims->constraints[i],"nh",&nh);
  for(int j=0;j<n;j++){
   if(rows[j]<0||rows[j]>=nh)return NAN;
   double a=blasfeo_dvecex1(m->ineq_fun+i,nb+ng+rows[j]);
   double b=blasfeo_dvecex1(m->ineq_fun+i,2*nb+2*ng+nh+rows[j]);
   if(!isfinite(a)||!isfinite(b))return NAN;
   maximum=fmax(maximum,fmax(a,b));
  }
 }
 return maximum;
}
int qp_observer_set_proximity_rows(int n,const int *rows){
 if(n<0||n>64)return -1;
 proximity_count=n;if(n)memcpy(proximity_rows,rows,n*sizeof(int));return 0;
}
void qp_observer_request_abort(void){abort_requested=1;}
static void emit(int phase,int iteration,int qp_status){
 ocp_nlp_sqp_memory *s=active->mem;ocp_nlp_memory *m=s->nlp_mem;ocp_nlp_res *r=m->nlp_res;
 ocp_nlp_dims *dims=active->dims;
 double cost=0.,proximity=0.;
 for(int i=0;i<=dims->N;i++)cost+=*active->config->cost[i]->memory_get_fun_ptr(m->cost[i]);
 for(int i=0;i<dims->N;i++){
  int nb=0,ng=0,nh=0;
  active->config->constraints[i]->dims_get(active->config->constraints[i],dims->constraints[i],"nb",&nb);
  active->config->constraints[i]->dims_get(active->config->constraints[i],dims->constraints[i],"ng",&ng);
  active->config->constraints[i]->dims_get(active->config->constraints[i],dims->constraints[i],"nh",&nh);
  for(int j=0;j<proximity_count;j++){
   int row=proximity_rows[j];if(row<0||row>=nh){proximity=NAN;break;}
   double lower=blasfeo_dvecex1(m->ineq_fun+i,nb+ng+row);
   double upper=blasfeo_dvecex1(m->ineq_fun+i,2*nb+2*ng+nh+row);
   if(!isfinite(lower)||!isfinite(upper)){proximity=NAN;break;}
   proximity=fmax(proximity,fmax(lower,upper));
  }
 }
 if(cb)cb(phase,iteration,qp_status,0.,r->inf_norm_res_stat,r->inf_norm_res_eq,
           r->inf_norm_res_ineq,r->inf_norm_res_comp,cost,proximity);
}
static int observed_evaluate(void*c,ocp_qp_xcond_solver_dims*d,ocp_qp_in*in,ocp_qp_out*out,void*o,void*m,void*w){
 int iteration=count++;emit(0,iteration,-1);
 /* Abort before QP/variable update, only to switch models using the cached probe. */
 if(abort_requested)return 99;
 return original(c,d,in,out,o,m,w);
}
int qp_observer_begin(void*solver,void*callback){
 if(active)return -1;active=solver;cb=(callback_t)callback;count=0;abort_requested=0;
 original=active->config->qp_solver->evaluate;active->config->qp_solver->evaluate=observed_evaluate;return 0;
}
void qp_observer_finish(void){
 if(!active)return;
 ocp_nlp_sqp_memory *s=active->mem;
 if(!abort_requested && s->status==0)emit(3,s->sqp_iter,0);
 active->config->qp_solver->evaluate=original;active=0;cb=0;original=0;abort_requested=0;
}
