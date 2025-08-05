#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
import smbus
import time
import math

class IMUPublisher:
    def __init__(self):
        # --- Parametri del Sensore e Indirizzi ---
        self.MPU6050_ADDR = 0x68
        # Registri chiave
        self.PWR_MGMT_1 = 0x6B
        self.CONFIG = 0x1A
        self.ACCEL_CONFIG = 0x1C
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43

        # --- Fattori di Scala (Default: AFS_SEL=0, FS_SEL=0) ---
        # Accel: ±2g -> 16384 LSB/g
        # Gyro:  ±250 °/s -> 131 LSB/(°/s)
        self.ACCEL_SCALE = 16384.0
        self.GYRO_SCALE = 131.0

        # --- Costanti Fisiche ---
        self.G_STD = 9.80665  # Accelerazione di gravità standard (m/s^2)

        # --- Parametri del Filtro Complementare ---
        self.FILTER_COEFFICIENT = 0.98

        # --- Bus I2C ---
        self.bus = smbus.SMBus(1)  # Bus I2C 1 (standard su Raspberry Pi/Jetson)

        # --- Variabili di Stato del Filtro ---
        self.last_time = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Valori di Calibrazione Giroscopio (Offset)
        self.gyro_offset_x = -218.295
        self.gyro_offset_y = 171.2
        self.gyro_offset_z = -86.41

        # Valori di Calibrazione Accelerometro (Offset e Scala)
        self.accel_offset_x = 79.2450000000008
        self.accel_offset_y = -47.594999999999345
        self.accel_offset_z = 2857.79
        self.accel_scale_x = 1.0011484732394125
        self.accel_scale_y = 0.9921498204362637
        self.accel_scale_z = 0.9913414936679757
        
        # --- Inizializzazione del Nodo ROS ---
        rospy.init_node('advanced_imu_publisher', anonymous=True)
        self.pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.rate = rospy.Rate(100)  # Aumentato a 100 Hz

    def read_word_2c(self, reg):
        """ Legge un valore a 16 bit in complemento a due da due registri consecutivi. """
        high = self.bus.read_byte_data(self.MPU6050_ADDR, reg)
        low = self.bus.read_byte_data(self.MPU6050_ADDR, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def setup_mpu6050(self):
        """ Inizializza e configura l'MPU6050. """
        # Risveglia il sensore (esce dalla modalità sleep)
        self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0)
        rospy.loginfo("MPU6050 attivato.")
        
        # Attiva il Digital Low-Pass Filter (DLPF)
        # DLPF_CFG = 1 -> Accel BW: 184Hz, Gyro BW: 188Hz
        # Fornisce letture più stabili, riducendo il rumore.
        self.bus.write_byte_data(self.MPU6050_ADDR, self.CONFIG, 1)
        rospy.loginfo("Filtro DLPF interno configurato.")

    def calibrate_gyro(self, samples=200):
        """ Calcola l'offset del giroscopio leggendo N campioni a riposo. """
        rospy.loginfo("Calibrazione giroscopio... Non muovere il sensore.")
        sum_x, sum_y, sum_z = 0, 0, 0
        for _ in range(samples):
            sum_x += self.read_word_2c(self.GYRO_XOUT_H)
            sum_y += self.read_word_2c(self.GYRO_XOUT_H + 2)
            sum_z += self.read_word_2c(self.GYRO_XOUT_H + 4)
            time.sleep(0.01) # Attende tra una lettura e l'altra
        
        self.gyro_offset_x = sum_x / samples
        self.gyro_offset_y = sum_y / samples
        self.gyro_offset_z = sum_z / samples
        rospy.loginfo("Calibrazione completata. Offset: [x: %.2f, y: %.2f, z: %.2f]", 
                      self.gyro_offset_x, self.gyro_offset_y, self.gyro_offset_z)

    def run(self):
        """ Ciclo principale di pubblicazione. """
        self.setup_mpu6050()
        self.calibrate_gyro()
        self.last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            if dt <= 0: # Salta il ciclo se il tempo non è avanzato
                self.rate.sleep()
                continue
                
            # --- Lettura dati grezzi ---
            accel_x_raw = self.read_word_2c(self.ACCEL_XOUT_H)
            accel_y_raw = self.read_word_2c(self.ACCEL_XOUT_H + 2)
            accel_z_raw = self.read_word_2c(self.ACCEL_XOUT_H + 4)
                        
            # Applica solo l'offset al giroscopio
            gyro_x_raw = self.read_word_2c(self.GYRO_XOUT_H) - self.gyro_offset_x
            gyro_y_raw = self.read_word_2c(self.GYRO_XOUT_H + 2) - self.gyro_offset_y
            gyro_z_raw = self.read_word_2c(self.GYRO_XOUT_H + 4) - self.gyro_offset_z

            # Applica prima l'offset e poi la scala
            accel_x_cal = (accel_x_raw - self.accel_offset_x) * self.accel_scale_x
            accel_y_cal = (accel_y_raw - self.accel_offset_y) * self.accel_scale_y
            accel_z_cal = (accel_z_raw - self.accel_offset_z) * self.accel_scale_z
            
            # --- Conversione in Unità SI ---
            # --- Conversione in Unità SI (USA I VALORI CALIBRATI) ---
            # Accelerazione lineare in m/s^2
            accel_x = (accel_x_cal / self.ACCEL_SCALE) * self.G_STD
            accel_y = (accel_y_cal / self.ACCEL_SCALE) * self.G_STD
            accel_z = (accel_z_cal / self.ACCEL_SCALE) * self.G_STD
            
            # Velocità angolare in rad/s
            gyro_x = (gyro_x_raw / self.GYRO_SCALE) * (math.pi / 180.0)
            gyro_y = (gyro_y_raw / self.GYRO_SCALE) * (math.pi / 180.0)
            gyro_z = (gyro_z_raw / self.GYRO_SCALE) * (math.pi / 180.0)

            # --- Filtro Complementare ---
            # Stima dell'orientamento dall'accelerometro (affidabile a lungo termine)
            roll_acc = math.atan2(accel_y, accel_z)
            pitch_acc = math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z))
            
            # Integrazione del giroscopio (affidabile a breve termine) e fusione
            self.roll = self.FILTER_COEFFICIENT * (self.roll + gyro_x * dt) + (1 - self.FILTER_COEFFICIENT) * roll_acc
            self.pitch = self.FILTER_COEFFICIENT * (self.pitch + gyro_y * dt) + (1 - self.FILTER_COEFFICIENT) * pitch_acc
            self.yaw += gyro_z * dt # Lo yaw viene solo integrato, soggetto a deriva

            # --- Creazione e Popolamento del Messaggio IMU ---
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = "imu_link"

            # Orientamento come quaternione
            q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            # Covarianza dell'orientamento (0 se sconosciuta)
            imu_msg.orientation_covariance = [0.0] * 9 # Inizializzato a zero

            # Velocità Angolare
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            imu_msg.angular_velocity_covariance = [0.0] * 9

            # Accelerazione Lineare
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            imu_msg.linear_acceleration_covariance = [0.0] * 9

            # --- Pubblicazione ---
            self.pub.publish(imu_msg)
            
            self.last_time = current_time
            self.rate.sleep()

if __name__ == '__main__':
    try:
        imu_pub = IMUPublisher()
        imu_pub.run()
    except rospy.ROSInterruptException:
        pass
    except IOError as e:
        rospy.logerr("Errore I/O: Controllare la connessione dell'MPU6050. Dettagli: %s", e)