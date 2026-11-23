#include "acados/ocp_nlp/ocp_nlp_common.h"
#include "acados/dense_qp/dense_qp_hpipm.h"
int hpipm_trace_stats(void *ptr,int *dims,double *out) {
 ocp_nlp_memory *m=(ocp_nlp_memory*)ptr;
 dense_qp_hpipm_memory *q=(dense_qp_hpipm_memory*)m->qp_solver_mem->solver_memory;
 struct d_dense_qp_ipm_ws *w=q->hpipm_workspace;
 int n=w->iter+1;if(n>w->stat_max)n=w->stat_max;
 dims[0]=n;dims[1]=w->stat_m;dims[2]=w->status;
 if(out)for(int i=0;i<n*w->stat_m;i++)out[i]=w->stat[i];
 return n*w->stat_m;
}
