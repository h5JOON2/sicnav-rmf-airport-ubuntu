import sys
import threading
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from perception.latest_frame import LatestFrame


class LatestFrameTests(unittest.TestCase):
    def test_slow_consumer_gets_newest_complete_bundle(self):
        frames=LatestFrame()
        a=('rgb1','depth1','odom1','info1')
        b=('rgb2','depth2','odom2','info2')
        c=('rgb3','depth3','odom3','info3')
        frames.put(1,a)
        active=frames.take()
        frames.put(2,b)
        frames.put(3,c)
        self.assertEqual(active[1],a)
        self.assertEqual(frames.take()[1],c)
        self.assertEqual(frames.replaced,1)

    def test_reset_invalidates_active_frame(self):
        frames=LatestFrame()
        frames.put(10,'before');active=frames.take()
        self.assertTrue(frames.put(0,'after'))
        self.assertNotEqual(active[0],frames.generation)
        self.assertEqual(frames.take()[1],'after')

    def test_close_wakes_waiting_worker(self):
        frames=LatestFrame();result=[]
        t=threading.Thread(target=lambda:result.append(frames.take()))
        t.start();frames.close();t.join(timeout=1)
        self.assertFalse(t.is_alive())
        self.assertEqual(result,[None])
        frames.put(1,'ignored')
        self.assertIsNone(frames.take())

if __name__=='__main__':unittest.main()
