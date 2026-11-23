import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from perception.ground_truth import gt_observation

def packet(n):
    return {'schema':'crowd_gt/v1','session':'test','epoch':0,'stamp':1.,'odom_stamp':1.012,
            'scan_seq':1,'frame_id':'world','source_topic':'/crowd_sim/states',
            'humans':[{'id':i+1,'px':float(i),'py':2.,'vx':.1,'vy':.2,'velocity_valid':True} for i in range(n)]}

class GroundTruthTests(unittest.TestCase):
    def test_gt_preserves_ids_coordinates_and_velocity(self):
        h=packet(3);r=gt_observation(h,3)
        self.assertEqual(r['sensor'],'simulator_gt')
        self.assertEqual([s['id'] for s in r['slots']],[1,2,3])
        self.assertEqual(r['slots'][2]['px'],2.)
        self.assertEqual(r['slots'][2]['vx'],.1)
        self.assertFalse(r['capacity_exceeded'])
    def test_gt_does_not_pad_population(self):
        self.assertEqual(len(gt_observation(packet(1),3)['slots']),1)
        self.assertTrue(gt_observation(packet(4),3)['capacity_exceeded'])
    def test_invalid_state_and_duplicate_ids_are_rejected(self):
        for field,value in [('px',float('nan')),('velocity_valid',False)]:
            h=packet(1);h['humans'][0][field]=value
            with self.assertRaises(ValueError):gt_observation(h,3)
        h=packet(2);h['humans'][1]['id']=1
        with self.assertRaises(ValueError):gt_observation(h,3)
    def test_lidar_packet_never_enters_gt_mode(self):
        h=packet(1);h['schema']='laser_scan/v1'
        with self.assertRaises(ValueError):gt_observation(h,3)

if __name__=='__main__':unittest.main()
