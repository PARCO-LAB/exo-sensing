import json
import threading
import multiprocessing as mp
import time
import signal
import rclpy
from sensor import Sensor
from delegates.motion_delegate import MotionDelegate
from delegates.scanner_delegate import ScanDelegate
from bluepy.btle import Scanner


def start_sensor(nickname, mac, thread_sync, iface):
    rclpy.init()
    thingy = Sensor(
                    nickname=nickname,
                    mac=mac,
                    thread_sync=thread_sync,
                    iface = iface,
                    sensor_delegates=[MotionDelegate],
                    services={'rawdata': True}, # TODO: correct with useful data for us
                )

    thingy.recording()
    rclpy.shutdown()


class Controller(object):

    def __init__(self):
        self.__scanner = Scanner().withDelegate(ScanDelegate())
        self.__allowed_devices: dict = json.load(open('devices.json'))
        self.__ifaces: dict = json.load(open('iface.json'))
        self.__connected_devices = []
        self.__is_active = True
        self.__synchronizer = mp.Event()

        self.available_func = {
            'help': self.__help,
            'add_device': self.__add_allowed_device,
            'connect': self.__add_connected_device,
            'allowed_devices': self.__get_allowed_devices,
            'connected_devices': self.__get_connected_devices,
            'end': self.__end_session
        }

    # RPC methods
    def __help(self):
        return list(self.available_func.keys())

    def __add_allowed_device(self, device: dict):
        self.__allowed_devices.update(device)

    def __get_allowed_devices(self):
        return self.__allowed_devices

    def __get_connected_devices(self):
        return self.__connected_devices

    def __add_connected_device(self, device):
        self.__connected_devices.append(device)

    def __end_session(self):
        self.__is_active = False

    # Controller start session
    def start_session(self):

        print('Session started')
        
        # Scan for IMUs
        self.__scanner.scan(timeout=10,passive=True)
        discovered_devices = list(self.__scanner.getDevices())
        # Connecting the IMUs
        for device in discovered_devices:
            # Connect only allowed and connectable devices
            if device.addr.upper() in self.__allowed_devices.keys() and device.connectable:
                try:
                    # # Connection and setup
                    # thingy = Sensor(
                    #     nickname=self.__allowed_devices[device.addr.upper()],
                    #     mac=device.addr,
                    #     thread_sync=self.__synchronizer,
                    #     sensor_delegates=[MotionDelegate],
                    #     services={'rawdata': True}, # TODO: correct with useful data for us
                    # )

                    # # Keep track of connected devices
                    # self.__add_connected_device(device.addr)

                    # # Wait for threads synchronization
                    # thingy.start()
                    self.__add_connected_device(device.addr)
                    process = mp.Process(target = start_sensor, args= (self.__allowed_devices[device.addr.upper()], device.addr, self.__synchronizer, self.__ifaces[self.__allowed_devices[device.addr.upper()]]))
                    process.daemon = True
                    process.start()

                except Exception as e:
                    print(e.args)

        # Start recording
        time.sleep(2)
        self.__synchronizer.set()

        #time.sleep(60)
        signals = {signal.SIGINT, signal.SIGTERM}
        signal.pthread_sigmask(signal.SIG_BLOCK, signals)
        signum = signal.sigwait(signals)

        self.__synchronizer.clear()

        print('Session ended')
