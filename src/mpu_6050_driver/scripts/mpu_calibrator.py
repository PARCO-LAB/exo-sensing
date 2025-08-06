#!/home/nvidia/exo-sensing/.venv/bin/python
# -*- coding: utf-8 -*-
import smbus
import time
import math

class MPUCalibrator:
    def __init__(self):
        self.MPU6050_ADDR = 0x68
        self.PWR_MGMT_1 = 0x6B
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        
        # Valore teorico che il sensore dovrebbe leggere per 1g con fondo scala a +/-2g
        self.G_RAW_IDEAL = 16384.0 

        self.bus = smbus.SMBus(1)
        self.accel_data = {"x+": [], "x-": [], "y+": [], "y-": [], "z+": [], "z-": []}
        self.gyro_offset = [0, 0, 0]
        self.accel_offset = [0, 0, 0]
        self.accel_scale = [1.0, 1.0, 1.0]

    def read_word_2c(self, reg):
        """Legge un valore a 16 bit in complemento a due."""
        high = self.bus.read_byte_data(self.MPU6050_ADDR, reg)
        low = self.bus.read_byte_data(self.MPU6050_ADDR, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def setup_mpu6050(self):
        """Attiva il sensore MPU6050."""
        self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0)
        time.sleep(0.1)
        print("MPU6050 attivato.")

    def get_avg_data(self, samples=200):
        """Legge e media i dati grezzi per ridurre il rumore."""
        ax_sum, ay_sum, az_sum = 0, 0, 0
        gx_sum, gy_sum, gz_sum = 0, 0, 0

        print(f"Collezionando {samples} campioni...")
        for _ in range(samples):
            ax_sum += self.read_word_2c(self.ACCEL_XOUT_H)
            ay_sum += self.read_word_2c(self.ACCEL_XOUT_H + 2)
            az_sum += self.read_word_2c(self.ACCEL_XOUT_H + 4)
            gx_sum += self.read_word_2c(self.GYRO_XOUT_H)
            gy_sum += self.read_word_2c(self.GYRO_XOUT_H + 2)
            gz_sum += self.read_word_2c(self.GYRO_XOUT_H + 4)
            time.sleep(0.002) # Pausa breve tra i campioni

        avg_data = {
            "ax": ax_sum / samples, "ay": ay_sum / samples, "az": az_sum / samples,
            "gx": gx_sum / samples, "gy": gy_sum / samples, "gz": gz_sum / samples,
        }
        print("Collezione completata.")
        return avg_data

    def calibrate_gyro(self):
        """Calcola l'offset del giroscopio a riposo."""
        print("\n--- CALIBRAZIONE GIROSCOPIO ---")
        print("Per favore, posiziona il sensore su una superficie piana e non muoverlo.")
        input("Premi Invio per iniziare...")

        avg_data = self.get_avg_data()
        self.gyro_offset = [avg_data["gx"], avg_data["gy"], avg_data["gz"]]
        
        print("\nCalibrazione giroscopio completata.")
        print(f"Offset Giroscopio [X, Y, Z]: {self.gyro_offset}")

    def calibrate_accel(self):
        """Guida l'utente attraverso la calibrazione a 6 posizioni dell'accelerometro."""
        print("\n--- CALIBRAZIONE ACCELEROMETRO (6 POSIZIONI) ---")
        print("Dovrai posizionare il sensore in 6 orientamenti.")
        print("Assicurati che ogni asse sia il più possibile perpendicolare al suolo.")

        positions = {
            "z+": "Z rivolto VERSO L'ALTO (+1g)",
            "z-": "Z rivolto VERSO IL BASSO (-1g)",
            "y+": "Y rivolto VERSO L'ALTO (+1g)",
            "y-": "Y rivolto VERSO IL BASSO (-1g)",
            "x+": "X rivolto VERSO L'ALTO (+1g)",
            "x-": "X rivolto VERSO IL BASSO (-1g)",
        }

        for key, description in positions.items():
            print(f"\nPasso: Posiziona l'asse {description}")
            input("Premi Invio quando sei pronto...")
            avg_data = self.get_avg_data()
            self.accel_data[key] = [avg_data["ax"], avg_data["ay"], avg_data["az"]]
            print(f"Dati grezzi registrati per la posizione {key}: {self.accel_data[key]}")

        # Calcolo di offset e scala
        # Offset = (Lettura a +1g + Lettura a -1g) / 2
        self.accel_offset[0] = (self.accel_data["x+"][0] + self.accel_data["x-"][0]) / 2.0
        self.accel_offset[1] = (self.accel_data["y+"][1] + self.accel_data["y-"][1]) / 2.0
        self.accel_offset[2] = (self.accel_data["z+"][2] + self.accel_data["z-"][2]) / 2.0
        
        # Scala = G_RAW_IDEAL / ((Lettura a +1g - Lettura a -1g) / 2)
        # Lo riscriviamo come: Scala = (2 * G_RAW_IDEAL) / (Lettura a +1g - Lettura a -1g)
        self.accel_scale[0] = (2 * self.G_RAW_IDEAL) / (self.accel_data["x+"][0] - self.accel_data["x-"][0])
        self.accel_scale[1] = (2 * self.G_RAW_IDEAL) / (self.accel_data["y+"][1] - self.accel_data["y-"][1])
        self.accel_scale[2] = (2 * self.G_RAW_IDEAL) / (self.accel_data["z+"][2] - self.accel_data["z-"][2])

        print("\n\n--- CALIBRAZIONE COMPLETATA ---")

    def print_results(self):
        """Stampa i risultati in un formato facile da copiare."""
        print("\nCopia e incolla questi valori nel tuo script principale:")
        print("-" * 50)
        print("# Valori di Calibrazione Giroscopio (Offset)")
        print(f"self.gyro_offset_x = {self.gyro_offset[0]}")
        print(f"self.gyro_offset_y = {self.gyro_offset[1]}")
        print(f"self.gyro_offset_z = {self.gyro_offset[2]}")
        print("\n# Valori di Calibrazione Accelerometro (Offset e Scala)")
        print(f"self.accel_offset_x = {self.accel_offset[0]}")
        print(f"self.accel_offset_y = {self.accel_offset[1]}")
        print(f"self.accel_offset_z = {self.accel_offset[2]}")
        print(f"self.accel_scale_x = {self.accel_scale[0]}")
        print(f"self.accel_scale_y = {self.accel_scale[1]}")
        print(f"self.accel_scale_z = {self.accel_scale[2]}")
        print("-" * 50)


if __name__ == '__main__':
    calibrator = MPUCalibrator()
    calibrator.setup_mpu6050()
    calibrator.calibrate_gyro()
    calibrator.calibrate_accel()
    calibrator.print_results()