import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from remote_runtime.tracker import PersonSlots,observation_compatible

class SlotsTest(unittest.TestCase):
    def test_detection_order_does_not_swap_history(self):
        tracker=PersonSlots(2)
        first=tracker.update(1.,[[0.,0.],[3.,0.]])
        second=tracker.update(1.2,[[3.1,0.],[.1,0.]])
        self.assertEqual([v['id'] for v in first],[v['id'] for v in second])
        self.assertAlmostEqual(second[0]['px'],.1)
        self.assertAlmostEqual(second[1]['px'],3.1)

    def test_inflight_reply_cannot_use_changed_or_incomplete_people(self):
        valid={'capacity_exceeded':False,'slots':[{'id':1},{'id':2}]}
        self.assertTrue(observation_compatible(valid,[1,2]))
        self.assertFalse(observation_compatible(valid,[1,3]))
        self.assertFalse(observation_compatible({**valid,'capacity_exceeded':True},[1,2]))
        self.assertFalse(observation_compatible({**valid,'slots':[{'id':1},None]},[1,2]))
        self.assertFalse(observation_compatible(None,[1,2]))

    def test_expired_and_reset_tracks_are_not_reused(self):
        tracker=PersonSlots(1,expiry=.5)
        old=tracker.update(1.,[[0.,0.]])[0]['id']
        self.assertIsNone(tracker.update(2.,[])[0])
        new=tracker.update(2.1,[[0.,0.]])[0]['id']
        self.assertNotEqual(old,new)
        reset=tracker.update(0.,[[0.,0.]])[0]['id']
        self.assertNotEqual(new,reset)

if __name__=='__main__':unittest.main()
