import sys,math,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from perception.lidar import decode_scan,prepare_drow_scan,drow_xy_to_ros,lidar_points_to_odom
from remote_runtime.wire import send_packet,receive_packet
import socket


def header(n=720):
    return {'schema':'laser_scan/v1','stamp':1.,'odom_stamp':1.,'count':n,'angle_min':-math.pi,
            'angle_increment':2*math.pi/(n-1),'range_min':.15,'range_max':12.,
            'odom_position':[10.,20.,0.],'odom_quaternion':[0.,0.,0.,1.],
            'lidar_position_base':[.1,0.,.265],'lidar_quaternion_base':[0.,0.,0.,1.]}

class LidarTests(unittest.TestCase):
    def test_wire_roundtrip_preserves_scan_invalid_values_for_sanitizing(self):
        h=header();r=np.full(720,2.,dtype='<f4');r[:3]=[np.inf,np.nan,0]
        a,b=socket.socketpair()
        try:
            send_packet(a,h,r.tobytes());rh,p=receive_packet(b);parsed=decode_scan(rh,p)
            self.assertTrue(np.isinf(parsed[0]));self.assertTrue(np.isnan(parsed[1]))
            clean,_,_=prepare_drow_scan(parsed,h['angle_min'],h['angle_increment'],.15,12)
            self.assertTrue(np.isfinite(clean).all());np.testing.assert_allclose(clean[-3:],29.99)
        finally:a.close();b.close()

    def test_positive_ros_angle_is_left_after_drow_conversion(self):
        theta=math.radians(30);raw=[[3*math.cos(theta),-3*math.sin(theta)]]
        ros=drow_xy_to_ros(raw,0)[0]
        self.assertAlmostEqual(math.degrees(math.atan2(ros[1],ros[0])),30)
        self.assertAlmostEqual(np.linalg.norm(ros),3)
        # A scan whose midpoint looks left by 90 degrees.
        np.testing.assert_allclose(drow_xy_to_ros([[2,0]],math.pi/2),[[0,2]],atol=1e-9)

    def test_scan_order_normalization(self):
        r=np.linspace(1,4,32,dtype=np.float32)
        ccw,fov,c=prepare_drow_scan(r,-math.pi/2,math.pi/31,.15,12)
        np.testing.assert_array_equal(ccw,r[::-1]);self.assertAlmostEqual(fov,180);self.assertAlmostEqual(c,0)
        cw,_,_=prepare_drow_scan(r,math.pi/2,-math.pi/31,.15,12)
        np.testing.assert_array_equal(cw,r)

    def test_robot_rotation_and_mount_are_applied_once(self):
        h=header();a=math.sqrt(.5);h['odom_quaternion']=[0,0,a,a]
        np.testing.assert_allclose(lidar_points_to_odom([[2,0]],h),[[10,22.1,.265]],atol=1e-9)
        h=header();h['lidar_quaternion_base']=[0,0,a,a]
        np.testing.assert_allclose(lidar_points_to_odom([[2,0]],h),[[10.1,22,.265]],atol=1e-9)

    def test_wrong_camera_packet_and_invalid_geometry_rejected(self):
        h=header();payload=np.ones(720,dtype='<f4').tobytes()
        for changed in [{'schema':'rgbd/v1'},{'count':100000},{'angle_increment':0},
                        {'range_max':float('inf')},{'stamp':float('nan')},{'odom_stamp':1.1},{'odom_quaternion':[0,0,0,0]}]:
            with self.assertRaises(ValueError):decode_scan({**h,**changed},payload)
        with self.assertRaises(ValueError):decode_scan(h,payload[:-4])

    def test_active_scripts_have_no_camera_dependency(self):
        root=Path(__file__).resolve().parents[1]
        for path in ['scripts/robot_offload_gateway.py','scripts/mac_offload_server.py']:
            text=(root/path).read_text()
            for forbidden in ['from ultralytics','from cv_bridge','CameraInfo','sensor_msgs.msg import Image','jpeg_bytes']:
                self.assertNotIn(forbidden,text)


class LidarWorldTests(unittest.TestCase):
    def test_world_has_lidar_and_no_image_sensors(self):
        import xml.etree.ElementTree as ET
        root=Path(__file__).resolve().parents[1]
        world=ET.parse(root/'worlds/airport_lidar_world.sdf')
        sensors=world.findall('.//sensor')
        self.assertEqual([s.get('type') for s in sensors],['gpu_lidar'])
        sensor=sensors[0]
        self.assertEqual(sensor.findtext('topic'),'/scan')
        self.assertEqual(sensor.findtext('lidar/scan/horizontal/samples'),'720')
        self.assertEqual(sensor.findtext('pose'),'0 0 0.265 0 0 0')
        self.assertEqual(sensor.findtext('gz_frame_id'),'tb4_lidar')

if __name__=='__main__':unittest.main()
