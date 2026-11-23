"""Rebuild the observer against the exact Acados installation used by MP."""
import os,subprocess
from pathlib import Path
root=Path(os.environ.get('SICNAV_MP_ROOT',Path(__file__).resolve().parents[2]))
acados=Path(os.environ.get('ACADOS_SOURCE_DIR',str(root/'work/acados')))
here=Path(__file__).resolve().parent/'native'
includes=[acados/'include',acados/'include/acados',acados/'include/blasfeo/include',acados/'include/hpipm/include']
for source,target in [('qp_observer.c','qp_observer.dylib'),('hpipm_trace_stats.c','hpipm_trace_stats.dylib'),('qp_snapshot.c','libqp_snapshot.dylib')]:
 cmd=['cc','-dynamiclib','-fPIC','-O2',str(here/source),'-o',str(here/target)]
 cmd += [item for p in includes for item in ['-I',str(p)]]
 cmd += ['-L',str(acados/'lib'),'-lacados','-lhpipm','-lblasfeo']
 subprocess.run(cmd,check=True)
 print('Built',target)
