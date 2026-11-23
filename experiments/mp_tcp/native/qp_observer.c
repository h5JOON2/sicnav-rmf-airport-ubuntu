#include "acados_c/ocp_nlp_interface.h"
#include "acados/ocp_nlp/ocp_nlp_sqp.h"
typedef void(*callback_t)(int,int,int,double,double,double,double,double);
typedef int(*evaluate_t)(void*,ocp_qp_xcond_solver_dims*,ocp_qp_in*,ocp_qp_out*,void*,void*,void*);
static callback_t cb=0;static evaluate_t original=0;static ocp_nlp_solver *active=0;static int count=0,last_status=-1;
static void emit(int phase,int k,int q,double alpha){ocp_nlp_sqp_memory*m=active->mem;ocp_nlp_res*r=m->nlp_mem->nlp_res;if(cb)cb(phase,k,q,alpha,r->inf_norm_res_stat,r->inf_norm_res_eq,r->inf_norm_res_ineq,r->inf_norm_res_comp);}
static int observed_evaluate(void*c,ocp_qp_xcond_solver_dims*d,ocp_qp_in*in,ocp_qp_out*out,void*o,void*m,void*w){
 int k=count++;
 if(k>0&&(last_status==0||last_status==2)){ocp_nlp_sqp_memory*s=active->mem;emit(2,k-1,last_status,s->stat[s->stat_n*k+6]);}
 emit(0,k,-1,0.0);int result=original(c,d,in,out,o,m,w);last_status=result;emit(1,k,result,0.0);return result;
}
int qp_observer_begin(void*solver,void*callback){if(active)return -1;active=solver;cb=(callback_t)callback;count=0;last_status=-1;original=active->config->qp_solver->evaluate;active->config->qp_solver->evaluate=observed_evaluate;return 0;}
void qp_observer_finish(void){if(!active)return;if(count>0&&(last_status==0||last_status==2)){ocp_nlp_sqp_memory*s=active->mem;emit(2,count-1,last_status,s->stat[s->stat_n*count+6]);}active->config->qp_solver->evaluate=original;active=0;cb=0;original=0;}
