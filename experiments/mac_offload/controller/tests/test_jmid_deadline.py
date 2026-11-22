import sys,time,threading,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from jmid_deadline import DeadlineForecast

class DeadlineTests(unittest.TestCase):
 def test_timely_result_used_without_orca(self):
  c=DeadlineForecast(150)
  try:
   r=c.step(lambda:(lambda:'fresh',{'stamp':1}))
   self.assertEqual(r['status'],'ready');self.assertEqual(r['result'],'fresh')
   self.assertIsNone(c.future)
  finally:c.close()
 def test_timeout_busy_no_queue_discard_then_resume_with_new_input(self):
  release=threading.Event();started=threading.Event();calls=[];c=DeadlineForecast(15)
  def slow():
   calls.append('old');started.set();release.wait(2);return 'late old prediction'
  try:
   r=c.step(lambda:(slow,{'stamp':1}))
   self.assertEqual(r['status'],'deadline');self.assertIsNone(r['result'])
   self.assertTrue(started.is_set())
   def forbidden():raise AssertionError('queued inference while busy')
   for _ in range(3):
    r=c.step(forbidden);self.assertEqual(r['status'],'inflight');self.assertIsNone(r['result'])
   self.assertEqual(calls,['old'])
   release.set();c.future.result(timeout=1)
   r=c.step(lambda:(lambda:'new current prediction',{'stamp':2}))
   self.assertEqual(r['status'],'ready');self.assertEqual(r['result'],'new current prediction')
   self.assertEqual(r['metadata']['stamp'],2)
   self.assertEqual(r['discarded']['metadata']['stamp'],1)
   self.assertEqual(r['discarded']['result']['value'],'late old prediction')
  finally:release.set();c.close()
 def test_inference_failure_is_not_hidden_as_timeout(self):
  c=DeadlineForecast(150)
  def bad():raise RuntimeError('model failure')
  try:
   with self.assertRaisesRegex(RuntimeError,'model failure'):c.step(lambda:(bad,{}))
  finally:c.close()
 def test_worker_timeout_exception_is_not_control_deadline(self):
  c=DeadlineForecast(150)
  def bad():raise TimeoutError('worker error')
  try:
   with self.assertRaisesRegex(TimeoutError,'worker error'):c.step(lambda:(bad,{}))
  finally:c.close()
 def test_invalid_deadline(self):
  for value in [0,-1,float('nan'),float('inf')]:
   with self.assertRaises(ValueError):DeadlineForecast(value)

if __name__=='__main__':unittest.main()
