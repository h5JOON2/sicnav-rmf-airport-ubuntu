import ast,sys,unittest
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'controller'))
from lean_runtime import strip_print_diagnostics
from mp_solver_adapter import native_residual_eligible

class LeanRuntimeTests(unittest.TestCase):
    def test_global_native_residual_comparison(self):
        self.assertTrue(native_residual_eligible([.05,.05,.05,.05]))
        for index in range(4):
            values=[.001]*4;values[index]=.051
            self.assertFalse(native_residual_eligible(values))
    def test_nonfinite_invalid_shape_and_negative_values_rejected(self):
        for values in ([0,0,0,float('nan')],[0,0,float('inf'),0],[0,0,0],[-1,0,0,0]):
            self.assertFalse(native_residual_eligible(values))
    def test_two_comparisons_can_read_the_same_native_residual_tuple(self):
        values=[.01,.005,.02,.03]
        self.assertTrue(native_residual_eligible(values,.05))
        self.assertFalse(native_residual_eligible(values,.001))
    def test_print_only_blocks_and_debug_costs_do_not_run(self):
        source='''def select_action(self):
 init_obj_val=self.solver.get_cost()
 if True:
  print('GUESS')
  self.diagnostic()
 if True:
  print('FINAL')
  self.diagnostic()
 final_obj_val=self.solver.get_cost()
 return 3
'''
        namespace={};exec(compile(strip_print_diagnostics(source),'test','exec'),namespace)
        def fail():raise AssertionError('diagnostic or cost reevaluation still ran')
        self.assertEqual(namespace['select_action'](S(diagnostic=fail,solver=S(get_cost=fail),mpc_env=S(human_pred_MID=True))),3)
    def test_no_python_constraint_or_cost_reevaluation_in_adapter(self):
        source=(ROOT/'controller/mp_solver_adapter.py').read_text()
        for marker in ('savez_compressed','events.jsonl','summary.jsonl','dense_check','kkt_check','control_candidate','control_cost','hf(','he(','active_f('):
            self.assertNotIn(marker,source)
        self.assertIn("residual_source='native_cached'",source)
    def test_native_hook_reads_caches_without_residual_computation(self):
        source=(ROOT/'controller/native/qp_observer.c').read_text()
        for marker in ('ocp_nlp_res_compute(', 'ocp_nlp_approximate_qp_matrices(', 'compute_fun(', 'eval_cost('):
            self.assertNotIn(marker,source)
        self.assertIn('s->status==0',source)
        self.assertIn('m->ineq_fun',source)
        self.assertIn('m->dyn_fun',source)
        adapter=(ROOT/'controller/mp_solver_adapter.py').read_text()
        self.assertIn('eq_grad_Lagr',adapter)
        self.assertIn('eq_comp_slack',adapter)
        self.assertIn('cached_model_feasibility_including_orca_kkt',adapter)

if __name__=='__main__':unittest.main()
