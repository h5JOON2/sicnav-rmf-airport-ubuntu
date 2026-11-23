"""Derived camera-free airport world; never overwrite the existing base world."""
import argparse,re,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def build(base,output):
 text=Path(base).read_text()
 text=re.sub(r'<!--\s*Disabled gpu_lidar.*?(<sensor name="rplidar".*?</sensor>)\s*-->',r'\1',text,flags=re.S)
 tree=ET.fromstring(text)
 # The crowd scene already spawns three pedestrians. Do not add the two
 # static passenger props on top of that three-person experiment.
 world=tree.find('world')
 for include in list(world.findall('include')):
  if include.findtext('name') in ('passenger_standing_1','passenger_standing_2'):
   world.remove(include)
 robot=tree.find("world/model[@name='turtlebot4']")
 if robot is None:raise ValueError('Expected project TurtleBot4 model')
 for link in robot.findall('link'):
  for sensor in list(link.findall('sensor')):
   if sensor.get('type') in ('camera','depth_camera','rgbd_camera'):link.remove(sensor)
 sensor=robot.find("link[@name='base_link']/sensor[@name='rplidar']")
 if sensor is None:raise ValueError('Expected rplidar sensor in base_link')
 sensor.find('topic').text='/scan'
 sensor.find('lidar/scan/horizontal/samples').text='720'
 sensor.find('lidar/scan/horizontal/min_angle').text='-3.141592653589793'
 sensor.find('lidar/scan/horizontal/max_angle').text='3.141592653589793'
 sensor.find('update_rate').text='10'
 sensor.find('pose').text='0 0 0.265 0 0 0'
 sensor.find('visualize').text='false'
 frame=sensor.find('gz_frame_id')
 if frame is None:frame=ET.SubElement(sensor,'gz_frame_id')
 frame.text='tb4_lidar'
 plugin=tree.find("world/plugin[@name='gz::sim::systems::Sensors']/render_engine")
 if plugin is None:raise ValueError('Expected rendering sensor system')
 plugin.text='ogre2'
 path=Path(output);path.parent.mkdir(parents=True,exist_ok=True)
 ET.indent(tree);ET.ElementTree(tree).write(path,encoding='unicode',xml_declaration=True)
 return path
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True)
 p.add_argument('--output',type=Path,default=ROOT/'worlds/airport_lidar_world.sdf');a=p.parse_args()
 print(build(a.base,a.output))
