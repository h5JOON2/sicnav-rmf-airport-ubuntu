"""Remove upstream print-only constraint reevaluation without changing control."""
import ast,inspect,textwrap

def strip_print_diagnostics(source):
    tree=ast.parse(textwrap.dedent(source));removed=[]
    class Strip(ast.NodeTransformer):
        def visit_Assign(self,node):
            # In MID mode these two costs only format debug_text; succ_cond uses
            # cached residuals, not costs. Do not trigger ocp_nlp_eval_cost again.
            if (len(node.targets)==1 and isinstance(node.targets[0],ast.Name)
                and node.targets[0].id in ('init_obj_val','final_obj_val')
                and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute)
                and node.value.func.attr=='get_cost'):
                node.value=ast.IfExp(test=ast.Attribute(value=ast.Attribute(value=ast.Name(id='self',ctx=ast.Load()),attr='mpc_env',ctx=ast.Load()),attr='human_pred_MID',ctx=ast.Load()),body=ast.Constant(0.),orelse=node.value)
            return self.generic_visit(node)
        def visit_If(self,node):
            first=node.body[0] if node.body else None
            if (isinstance(first,ast.Expr) and isinstance(first.value,ast.Call)
                and isinstance(first.value.func,ast.Name) and first.value.func.id=='print'
                and first.value.args and isinstance(first.value.args[0],ast.Constant)
                and first.value.args[0].value in ('GUESS','FINAL')):
                removed.append(first.value.args[0].value)
                return ast.Pass()
            return self.generic_visit(node)
        def visit_Expr(self,node):
            if isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Name) and node.value.func.id=='print':
                return ast.Pass()
            return self.generic_visit(node)
    tree=Strip().visit(tree)
    if sorted(removed)!=['FINAL','GUESS']:raise RuntimeError('Upstream diagnostics changed; refusing an unverified policy patch')
    return ast.fix_missing_locations(tree)

def install(policy_class):
    if getattr(policy_class,'_print_diagnostics_removed',False):return
    original=policy_class.select_action
    tree=strip_print_diagnostics(inspect.getsource(original))
    namespace={}
    exec(compile(tree,original.__code__.co_filename,'exec'),original.__globals__,namespace)
    policy_class.select_action=namespace[original.__name__]
    policy_class._print_diagnostics_removed=True
