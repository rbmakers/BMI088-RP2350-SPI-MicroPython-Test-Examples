//   Bosch BMI088 範例程式 , 火箭鳥創客倉庫

from machine import SPI, Pin
import time
from bmi088_spi import BMI088_SPI

# Pin Assignments (XIAO RP2350)
PIN_MOSI = 3   
PIN_MISO = 4   
PIN_SCK  = 2   
CS_ACCEL = 5   
CS_GYRO  = 6   

# Init SPI Bus 0 (Hardware SPI)
# 1MHz is used for reliable initialization
spi = SPI(0, baudrate=1000000, polarity=0, phase=0, 
          sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))

# Create IMU object
imu = BMI088_SPI(spi, CS_ACCEL, CS_GYRO)

def safe_init():
    print("--- BMI088 SPI SYSTEM START ---")
    while not imu.begin():
        print("Initialization failed. Retrying in 1s...")
        time.sleep(1)
    
    imu.calibrate(samples=100)
    print("System Ready.")

# Run initial setup
safe_init()

while True:
    # Read sensor data
    ax, ay, az = imu.read_accel()
    gx, gy, gz = imu.read_gyro()
    
    # Anti-Freeze/Error Check
    # If Gyro reads exactly -0.061 repeatedly or Accel is static 0, restart.
    if abs(gx + 0.061) < 0.0001 or abs(ax) > 15.0:
        print("!!! SENSOR ERROR DETECTED - COLD RESTARTING !!!")
        safe_init()
        continue

    # Clean Output
    print("-" * 50)
    print(f"ACC (g):  X:{ax:>8.3f} | Y:{ay:>8.3f} | Z:{az:>8.3f}")
    print(f"GYRO(dps):X:{gx:>8.3f} | Y:{gy:>8.3f} | Z:{gz:>8.3f}")
    
    time.sleep(0.1)
