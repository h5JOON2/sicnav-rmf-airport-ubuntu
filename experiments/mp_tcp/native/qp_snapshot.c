/* Read-only access to the installed HPIPM ABI; no solver modifications. */
#include <string.h>
#include <blasfeo_d_aux.h>
#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_sol.h>
void snapshot_dense_dims(struct d_dense_qp *q, int *d) {
 d[0]=q->dim->nv;d[1]=q->dim->ne;d[2]=q->dim->nb;d[3]=q->dim->ng;d[4]=q->dim->ns;
}
int snapshot_dense_raw(struct d_dense_qp *q,const char *field,double *out) {
 int nv=q->dim->nv,ne=q->dim->ne,nb=q->dim->nb,ng=q->dim->ng,ns=q->dim->ns,n=0;
 struct blasfeo_dvec *v=0;
 if(!strcmp(field,"gz")){v=q->gz;n=nv+2*ns;}
 else if(!strcmp(field,"b")){v=q->b;n=ne;}
 else if(!strcmp(field,"d")){v=q->d;n=2*(nb+ng+ns);}
 else if(!strcmp(field,"d_mask")){v=q->d_mask;n=2*(nb+ng+ns);}
 else if(!strcmp(field,"m")){v=q->m;n=2*(nb+ng+ns);}
 else if(!strcmp(field,"Z")){v=q->Z;n=2*ns;}
 else return -1;
 if(n)blasfeo_unpack_dvec(n,v,0,out,1);return n;
}
int snapshot_dense_sol(struct d_dense_qp_sol *q,const char *field,double *out) {
 int nv=q->dim->nv,ne=q->dim->ne,nb=q->dim->nb,ng=q->dim->ng,ns=q->dim->ns,n=0;
 struct blasfeo_dvec *v=0;
 if(!strcmp(field,"v")){v=q->v;n=nv+2*ns;}
 else if(!strcmp(field,"pi")){v=q->pi;n=ne;}
 else if(!strcmp(field,"lam")){v=q->lam;n=2*(nb+ng+ns);}
 else if(!strcmp(field,"t")){v=q->t;n=2*(nb+ng+ns);}
 else return -1;
 if(n)blasfeo_unpack_dvec(n,v,0,out,1);return n;
}
