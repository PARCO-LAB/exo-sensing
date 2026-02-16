import json
from distutils.util import strtobool
import sys
from bluepy.btle import DefaultDelegate
import struct
from datetime import datetime
import binascii
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

DEFAULT_UNKNOWN_COV = [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0]

class MotionDelegate(DefaultDelegate):

    def __init__(self, name: str, mac_adr: str):
        super().__init__()
        # Load configuration
        self.config = json.load(open(os.path.join(os.getcwd(), 'config.json')))
        self.sensor_sampling_frequency = self.config['sensor_sampling_frequency']
        self.fall_time_window_for_detection = self.config['fall_time_window_for_detection']
        self.shake_time_window_for_detection = self.config['shake_time_window_for_detection']

        # Basic info
        self.sensor_nickname = name
        self.time_start = str(datetime.now().microsecond)
        self.mac = mac_adr
        self.transmission_mode = bool(strtobool(self.config['transmission_mode']))
        self.previously_sent = {'shake': True}
        self.initialized = datetime.now()
        self.delta = None

        # Movements data
        self.accelerometer = {}
        self.gyroscope = {}
        self.compass = {}

        # For shaking detection
        self.shake_history_max_length = int(self.sensor_sampling_frequency * self.shake_time_window_for_detection)
        self.possible_shake = False
        self.is_shaking = False
        self.in_use = False
        self.shake_threshold = 1.5
        self.shaking_history = {}
        self.last_magnitude = None
        self.counter = 0

        # Fall detection
        self.fall_history_max_length = int(self.sensor_sampling_frequency * self.fall_time_window_for_detection)
        self.fall_sent_alert = False
        self.fall_detect = False
        self.alarm = False
        self.acc_history = []
        self.gyro_history = []

        # Heading
        self.heading = 'loading...'
        self.heading_is_changed = False
        self.heading_deg = 0

        # Time of event
        self.start_event_time = None
        self.end_event_time = None

        # Set handlers
        self.handlers = {
            #77: self._unpack_gravity_vector,
            #74: self._unpack_heading,
            65: self._unpack_raw_data,
            #56: self._unpack_orientation
        }

        
        self.node = Node(f'node_{self.sensor_nickname}')
        qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                )

        self.pub = self.node.create_publisher(Imu, f'/imu/{self.sensor_nickname}', qos)

    # TODO: transform the log to file part of the below functions into ros message.
    def _unpack_gravity_vector(self, d):
        """ Transform byte into float x, y, z gravity vector. """
        # Get Time
        now = datetime.now()

        gravity_vector = [struct.unpack('f', d[i: i + 4])[0] for i in range(0, len(d), 4)]

        # Log to file
        with open('{}_gravity_vector_{}.csv'.format(self.sensor_nickname, self.time_start), 'a+') as file:
            # time, x, y, z
            file.write('%s,%s,%s,%s\n' % (now, gravity_vector[0], gravity_vector[1], gravity_vector[2]))

    @staticmethod
    def _unpack_orientation(d):
        tmp = binascii.b2a_hex(d)
        orientation = 'Unknown'

        if tmp == b'00':
            orientation = 'Portrait'
        elif tmp == b'01':
            orientation = 'Landscape'
        elif tmp == b'02':
            orientation = 'Reverse Portrait'
        elif tmp == b'03':
            orientation = 'Reverse Landscape'

        print(orientation)

    def _unpack_heading(self, d):
        # Get Time
        now = datetime.now()

        # Unpack heading degree
        deg = struct.unpack('i', d)[0] / 65536

        # Convert to ordinal
        if 22.5 <= deg < 67.5:
            ordinal_name = 'north-east'

        elif 67.5 <= deg < 112.5:
            ordinal_name = 'east'

        elif 112.5 <= deg < 157.5:
            ordinal_name = 'south-east'

        elif 157.5 <= deg < 202.5:
            ordinal_name = 'south'

        elif 202.5 <= deg < 247.5:
            ordinal_name = 'south-west'

        elif 247.5 <= deg < 292.5:
            ordinal_name = 'west'

        elif 292.5 <= deg < 337.5:
            ordinal_name = 'north-west'

        else:
            ordinal_name = 'north'

        if ordinal_name != self.heading:
            self.heading = ordinal_name
            self.heading_deg = deg
            self.heading_is_changed = True
            # Log to file
            with open('{}_heading_{}.csv'.format(self.sensor_nickname, self.time_start), 'a+') as file:
                file.write('%s,%s,%s,\n' % (now, deg, ordinal_name))

        else:
            self.heading_is_changed = False

    def _unpack_raw_data(self, d):

        # TODO: write a new message that fits better your purposes
        # TODO: And the nordic timestamp?
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.mac
        
        # Get Time
        # now = datetime.now()

        # if self.delta is None:
        #     self.delta = now - self.initialized

        # Accelerometer
        acc_x = (struct.unpack('h', d[0:2])[0] * 1.0) / 2 ** 10
        acc_y = (struct.unpack('h', d[2:4])[0] * 1.0) / 2 ** 10
        acc_z = (struct.unpack('h', d[4:6])[0] * 1.0) / 2 ** 10

        # self.accelerometer = {
        #     'x': acc_x,
        #     'y': acc_y,
        #     'z': acc_z
        # }

        # # Gyroscope
        gyro_x = (struct.unpack('h', d[6:8])[0] * 1.0) / 2 ** 5
        gyro_y = (struct.unpack('h', d[8:10])[0] * 1.0) / 2 ** 5
        gyro_z = (struct.unpack('h', d[10:12])[0] * 1.0) / 2 ** 5

        # self.gyroscope = {
        #     'x': gyro_x,
        #     'y': gyro_y,
        #     'z': gyro_z
        # }

        # Compass
        # comp_x = (struct.unpack('h', d[12:14])[0] * 1.0) / 2 ** 4
        # comp_y = (struct.unpack('h', d[14:16])[0] * 1.0) / 2 ** 4
        # comp_z = (struct.unpack('h', d[16:18])[0] * 1.0) / 2 ** 4

        # self.compass = {
        #     'x': comp_x,
        #     'y': comp_y,
        #     'z': comp_z
        # }

        msg.linear_acceleration.x = float(acc_x)
        msg.linear_acceleration.y = float(acc_y)
        msg.linear_acceleration.z = float(acc_z)

        msg.angular_velocity.x = float(gyro_x)
        msg.angular_velocity.y = float(gyro_y)
        msg.angular_velocity.z = float(gyro_z)

        
        msg.orientation_covariance = DEFAULT_UNKNOWN_COV
        msg.angular_velocity_covariance = DEFAULT_UNKNOWN_COV
        msg.linear_acceleration_covariance = DEFAULT_UNKNOWN_COV


        self.pub.publish(msg)



        # Log to file
        # with open('{}_raw_data_{}.csv'.format(self.sensor_nickname, self.time_start), 'a+') as file:
        #     file.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %
        #                (now, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, comp_x, comp_y, comp_z)
        #                )

    @staticmethod
    def datetime_to_timestamp(time):
        return int(time.timestamp() * 1000000)

    def handleNotification(self, handle_code, data):
        self.handlers[handle_code](data)

        # now = datetime.now()

        # # Send data to Kafka
        # if self.transmission_mode:
        #     payload = {
        #         'id': self.mac,
        #         'timestamp_gate': self.datetime_to_timestamp(now),
        #         'timestamp_sensor': self.datetime_to_timestamp(now - self.delta),
        #         'accelerometer': self.accelerometer,
        #         'gyroscope': self.gyroscope,
        #         'compass': self.compass,
        #         'shake': self.is_shaking,
        #         'fall': self.alarm,
        #         'heading': self.heading,
        #         'orientation': int(self.heading_deg),
        #         'in_use': self.in_use
        #     }

        # else:
        #     payload = {
        #         'id': self.mac,
        #         'timestamp_gate': self.datetime_to_timestamp(now),
        #         'timestamp_sensor': self.datetime_to_timestamp(now - self.delta),
        #         'start': self.start_event_time,
        #         'end': self.end_event_time,
        #         'shake': self.is_shaking,
        #         'fall': self.alarm,
        #         'activity': None,
        #         'heading': self.heading,
        #         'orientation': int(self.heading_deg),
        #         'in_use': self.in_use
        #     }

        # # Send the packet when the shake is just ended
        # if self.in_use and not payload['shake'] and self.previously_sent['shake']:
        #     sys.stdout.write(f'{self.previously_sent}\n')
        #     sys.stdout.flush()

        # self.previously_sent = payload

        # if payload['fall'] and not self.fall_sent_alert:
        #     sys.stdout.write(f'{payload}\n')
        #     sys.stdout.flush()
        #     self.fall_sent_alert = True

        # if not payload['fall']:
        #     self.fall_sent_alert = False
