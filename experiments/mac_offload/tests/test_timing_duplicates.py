import unittest
from remote_runtime.tracker import PersonSlots
from remote_runtime.command_guard import CommandGuard

class DuplicateStampTests(unittest.TestCase):
    def test_repeated_timestamp_does_not_create_phantom_person(self):
        tracker=PersonSlots(3)
        first=tracker.update(1.,[[2.,3.]])
        repeated=tracker.update(1.,[[2.,3.]])
        self.assertEqual(first,repeated)
        self.assertEqual(sum(v is not None for v in repeated),1)
        following=tracker.update(1.1,[[2.01,3.]])
        self.assertEqual(following[0]['id'],first[0]['id'])
        self.assertEqual(sum(v is not None for v in following),1)

    def test_clock_reset_starts_new_tracks(self):
        tracker=PersonSlots(3)
        first=tracker.update(10.,[[2.,3.]])
        reset=tracker.update(0.,[[2.,3.]])
        self.assertNotEqual(first[0]['id'],reset[0]['id'])

    def test_velocity_rejection_is_reported_without_relaxing_limits(self):
        guard=CommandGuard('test')
        guard.note_state(1,0,10.,100.)
        packet={'session':'test','epoch':0,'state_seq':1,'action_timestamp':10.,'action':{'v':.46,'r':0.}}
        self.assertFalse(guard.accept(packet,100.1))
        self.assertEqual(guard.last_rejection,'velocity_limit')
        self.assertEqual(guard.command(100.1,10.1),(0.,0.))
        packet['action']['v']=.2
        self.assertTrue(guard.accept(packet,100.2))
        self.assertIsNone(guard.last_rejection)

if __name__=='__main__':unittest.main()
