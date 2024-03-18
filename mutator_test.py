'''test for mutator'''
import mutator as mttr
import unittest
import logging as log

class TestMutators(unittest.TestCase):
  def test_time_mutator(self):
    time = {'sec': 123, 'nanosec': 456}
    for _ in range(10):
      new_time = mttr.time_mutator(time)
    self.assertIsNot(time, new_time, "Time should be mutated.")
    log.info(new_time)

  def test_header_mutator(self):
    header = {'stamp': {'sec': 123, 'nanosec': 456}, 'frame_id': 'frame_123'}
    for _ in range(10):
      new_header = mttr.header_mutator(header)
    self.assertIsNot(new_header, header, "Header should be mutated.")
    self.assertIsNot(new_header['stamp'], header['stamp'], "Stamp should be mutated.")
    log.info(new_header)

  def test_status_mutator(self):
    status = {
      'header': {'stamp': {'sec': 123, 'nanosec': 456}, 'frame_id': 'frame_123'},
      'id': 'id_123',
      'instance_id': 'instance_456',
      'active': True,
      'heartbeat_timeout': 1.23,
      'heartbeat_period': 4.56
    }
    for _ in range(10):
      new_status = mttr.status_mutator(status)
    self.assertIsNot(new_status, status, "Status should be mutated.")
    self.assertIsNot(new_status['header'], status['header'], "Header should be mutated.")
    log.info(new_status)

  def test_laserscan_mutator(self):
    laserscan = {
      'header': {
        'stamp': {
          'sec': 123,
          'nanosec': 456
        },
        'frame_id': 'test_frame'
      },
      'angle_min': -1.57,
      'angle_max': 1.57,
      'angle_increment': 0.01,
      'time_increment': 0.0,
      'scan_time': 0.1,
      'range_min': 0.1,
      'range_max': 100.0,
      'ranges': [1.0] * 100, 
      'intensities': [100.0] * 100  
    }
    for _ in range(10):
      new_laserscan = mttr.laserscan_mutator(laserscan)
    self.assertIsNot(new_laserscan, laserscan, "Laserscan should be mutated.")
    self.assertIsNot(new_laserscan['header'], laserscan['header'], "Header should be mutated.")
    self.assertIsNot(new_laserscan['ranges'], laserscan['ranges'], "Ranges should be mutated.")
    self.assertIsNot(new_laserscan['intensities'], laserscan['intensities'], "Intensities should be mutated.")
    log.info(new_laserscan)
  
  def test_odometry(self):
    odometry = {
      'header': {
        'stamp': {
          'sec': 123,
          'nanosec': 456
        },
        'frame_id': 'odom_frame'
      },
      'child_frame_id': 'base_link',
      'pose': {
        'pose': {
          'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
          'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        },
        'covariance': [0.0] * 36  # 6x6 矩阵展平
      },
      'twist': {
        'twist': {
          'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
          'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        },
        'covariance': [0.0] * 36  # 6x6 矩阵展平
      }
    }
    for _ in range(10):
      new_odometry = mttr.odometry_mutator(odometry)
    self.assertIsNot(new_odometry, odometry, "Odometry should be mutated.") 
    self.assertIsNot(new_odometry['header'], odometry['header'], "Header should be mutated.")
    self.assertIsNot(new_odometry['pose'], odometry['pose'], "Pose should be mutated.")
    self.assertIsNot(new_odometry['twist'], odometry['twist'], "Twist should be mutated.")
    log.info(new_odometry)

if __name__ == '__main__':
  log.basicConfig(level=log.INFO) #! only setup once
  unittest.main()

