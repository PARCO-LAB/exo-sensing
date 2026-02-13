import sys
import threading
from bluepy.thingy52 import Thingy52
import time


class Sensor(object):
    # TODO: correct default values
    def __init__(
            self,
            mac,
            thread_sync,
            nickname: str = 'NordicThingy',
            sampling_frequency: int = 32,
            sensor_delegates: list = None,
            services: dict = None
    ):
        self.nickname = nickname
        self.thingy = Thingy52(mac)
        self.mac = mac
        self.synchronizer = thread_sync
        self.sampling_frequency = sampling_frequency
        self.sensor_delegates = sensor_delegates
        self.services = services

        # Set delegates
        for delegate in sensor_delegates:
            self.thingy.setDelegate(delegate(self.nickname, self.thingy.addr))

        self.thingy.ui.enable()
        self.thingy.ui.set_led_mode_constant(0, 255, 0)

    def enabling_selected_sensors(self, services: dict):
        # Enabling selected sensors
        print('Enabling selected sensors...')

        # Environment Service
        if services.get('temperature', False):
            self.thingy.environment.enable()
            self.thingy.environment.configure(temp_int=1000)
            self.thingy.environment.set_temperature_notification(True)
        if services.get('pressure', False):
            self.thingy.environment.enable()
            self.thingy.environment.configure(press_int=1000)
            self.thingy.environment.set_pressure_notification(True)
        if services.get('humidity', False):
            self.thingy.environment.enable()
            self.thingy.environment.configure(humid_int=1000)
            self.thingy.environment.set_humidity_notification(True)
        if services.get('gas', False):
            self.thingy.environment.enable()
            self.thingy.environment.configure(gas_mode_int=1)
            self.thingy.environment.set_gas_notification(True)
        if services.get('color', False):
            self.thingy.environment.enable()
            self.thingy.environment.configure(color_int=1000)
            self.thingy.environment.configure(color_sens_calib=[0, 0, 0])
            self.thingy.environment.set_color_notification(True)

        # User Interface Service
        if services.get('keypress', False):
            self.thingy.ui.enable()
            self.thingy.ui.set_btn_notification(True)
        if services.get('battery', False):
            self.thingy.battery.enable()

        # Motion Service
        if services.get('tap', False):
            self.thingy.motion.enable()
            self.thingy.motion.configure(motion_freq=200)
            self.thingy.motion.set_tap_notification(True)
        if services.get('orientation', False):
            self.thingy.motion.enable()
            self.thingy.motion.set_orient_notification(True)
        if services.get('quaternion', False):
            self.thingy.motion.enable()
            self.thingy.motion.set_quaternion_notification(True)
        if services.get('stepcnt', False):
            self.thingy.motion.enable()
            self.thingy.motion.configure(step_int=100)
            self.thingy.motion.set_stepcnt_notification(True)
        if services.get('rawdata', False):
            self.thingy.motion.enable()
            self.thingy.motion.set_rawdata_notification(True)
        if services.get('euler', False):
            self.thingy.motion.enable()
            self.thingy.motion.set_euler_notification(True)
        if services.get('rotation', False):
            self.thingy.motion.enable()
            self.thingy.motion.set_rotation_notification(True)
        if services.get('heading', False):
            self.thingy.motion.enable()
            self.thingy.motion.set_heading_notification(True)
        if services.get('gravity', False):
            self.thingy.motion.enable()
            self.thingy.motion.set_gravity_notification(True)

        # Sound Service
        if services.get('speaker', False):
            self.thingy.sound.enable()
            self.thingy.sound.configure(speaker_mode=0x03)
            self.thingy.sound.set_speaker_status_notification(True)
            # Test speaker
            self.thingy.sound.play_speaker_sample(1)
        if services.get('microphone', False):
            self.thingy.sound.enable()
            self.thingy.sound.configure(microphone_mode=0x01)
            self.thingy.sound.set_microphone_notification(True)

        # Allow sensors time to start up (might need more time for some sensors to be ready)
        print('All requested sensors and notifications are enabled...')
        time.sleep(1.0)
        self.synchronizer.wait()

    def start(self):
        """ Start sensor thread """
        thread = threading.Thread(target=self.recording)
        thread.daemon = True
        thread.start()

    def recording(self):
        """ Start the recording of a sensor """
        try:

            # Enable sensors
            self.enabling_selected_sensors(self.services)

            # Set LED in recording mode
            self.thingy.ui.set_led_mode_breathe(0x01, 50, 100)

            while self.synchronizer.is_set():
                self.thingy.waitForNotifications(1.)  # 1 seconds

        except Exception as e:
            print(e.args)

        finally:
            self.thingy.disconnect()
            print('Recording %s ended...' % self.nickname)
            print('%s disconnected...' % self.nickname)
