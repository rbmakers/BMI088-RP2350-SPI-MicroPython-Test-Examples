#  Bosch BMI088 範例程式 , 火箭鳥創客倉庫 #

from machine import SPI, Pin
import time

class BMI088_SPI:
    def __init__(self, spi, cs_acc, cs_gyro):
        self.spi = spi
        self.cs_acc = Pin(cs_acc, Pin.OUT, value=1)
        self.cs_gyro = Pin(cs_gyro, Pin.OUT, value=1)
        
        # Scaling factors for 16-bit signed integers
        self.acc_scale = 6 / 32768    # ±6g Range
        self.gyro_scale = 2000 / 32768 # ±2000 dps Range
        
        # Offset storage for calibration
        self.acc_offsets = [0.0, 0.0, 0.0]
        self.gyro_offsets = [0.0, 0.0, 0.0]

    def _write_reg(self, cs, reg, data):
        cs.value(0)
        self.spi.write(bytearray([reg & 0x7F, data]))
        cs.value(1)

    def _read_reg(self, cs, reg, nbytes=1, is_accel=False):
        cs.value(0)
        if is_accel:
            # Accel SPI requires 1 dummy byte after the address
            self.spi.write(bytearray([reg | 0x80]))
            self.spi.read(1) 
            result = self.spi.read(nbytes)
        else:
            # Gyro SPI does not require a dummy byte
            self.spi.write(bytearray([reg | 0x80]))
            result = self.spi.read(nbytes)
        cs.value(1)
        return result

    def begin(self):
        # 1. Hardware Toggle: Forces Gyro die to detect SPI mode
        self.cs_gyro.value(1); time.sleep_ms(5)
        self.cs_gyro.value(0); time.sleep_ms(5)
        self.cs_gyro.value(1); time.sleep_ms(20)

        # 2. Soft Reset
        self._write_reg(self.cs_acc, 0x7E, 0xB6)
        self._write_reg(self.cs_gyro, 0x14, 0xB6)
        time.sleep_ms(100)
        
        # 3. Accelerometer Power Sequence (Critical Order)
        self._write_reg(self.cs_acc, 0x7D, 0x04) # Power ON
        time.sleep_ms(10)
        self._write_reg(self.cs_acc, 0x7C, 0x00) # Active Mode
        time.sleep_ms(50)
        
        # 4. Sensor Configurations
        self._write_reg(self.cs_acc, 0x41, 0x01) # Range ±6g
        self._write_reg(self.cs_acc, 0x40, 0x88) # ODR 100Hz
        self._write_reg(self.cs_gyro, 0x0F, 0x00) # Range ±2000dps
        self._write_reg(self.cs_gyro, 0x10, 0x07) # ODR 100Hz
        
        # 5. Check Chip IDs
        try:
            acc_id = self._read_reg(self.cs_acc, 0x00, 1, True)[0]
            gyro_id = self._read_reg(self.cs_gyro, 0x00, 1, False)[0]
            print(f"Detected IDs - ACC: {hex(acc_id)}, GYRO: {hex(gyro_id)}")
            return acc_id == 0x1E and gyro_id == 0x0F
        except:
            return False

    def calibrate(self, samples=100):
        print(f"Starting Calibration ({samples} samples)... Keep Still.")
        sum_ax, sum_ay, sum_az = 0, 0, 0
        sum_gx, sum_gy, sum_gz = 0, 0, 0
        
        # Clear existing offsets
        self.acc_offsets = [0, 0, 0]
        self.gyro_offsets = [0, 0, 0]
        
        for _ in range(samples):
            ax, ay, az = self.read_accel()
            gx, gy, gz = self.read_gyro()
            sum_ax += ax; sum_ay += ay; sum_az += (az - 1.0) # Assume 1g on Z
            sum_gx += gx; sum_gy += gy; sum_gz += gz
            time.sleep_ms(5)
            
        self.acc_offsets = [sum_ax/samples, sum_ay/samples, sum_az/samples]
        self.gyro_offsets = [sum_gx/samples, sum_gy/samples, sum_gz/samples]
        print("Calibration Finished.")

    def read_accel(self):
        data = self._read_reg(self.cs_acc, 0x12, 6, True)
        x = self._to_signed(data[1] << 8 | data[0]) * self.acc_scale
        y = self._to_signed(data[3] << 8 | data[2]) * self.acc_scale
        z = self._to_signed(data[5] << 8 | data[4]) * self.acc_scale
        return (x - self.acc_offsets[0], y - self.acc_offsets[1], z - self.acc_offsets[2])

    def read_gyro(self):
        data = self._read_reg(self.cs_gyro, 0x02, 6, False)
        x = self._to_signed(data[1] << 8 | data[0]) * self.gyro_scale
        y = self._to_signed(data[3] << 8 | data[2]) * self.gyro_scale
        z = self._to_signed(data[5] << 8 | data[4]) * self.gyro_scale
        return (x - self.gyro_offsets[0], y - self.gyro_offsets[1], z - self.gyro_offsets[2])

    def _to_signed(self, val):
        return val - 65536 if val > 32767 else val
