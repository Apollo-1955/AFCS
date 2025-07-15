from machine import I2C, Pin
from imu import MPU6050
from madgwick import MadgwickAHRS
from time import ticks_us, ticks_diff, sleep_ms
from math import atan2, asin, degrees

# I2C setup
i2c = I2C(0, scl=Pin(1), sda=Pin(0))
mpu = MPU6050(i2c)
filt = MadgwickAHRS(sample_period=1/100, beta=0.02)

# Timing
last_time = ticks_us()

# Low-pass filter setup
filtered_pitch = 0.0
alpha = 0.95  # Higher = smoother but slower

def get_filtered_pitch():
    global last_time, filtered_pitch

    # Get raw sensor data
    ax, ay, az = mpu.accel.x, mpu.accel.y, mpu.accel.z
    gx, gy, gz = mpu.gyro.x, mpu.gyro.y, mpu.gyro.z

    # Delta time
    now = ticks_us()
    dt = ticks_diff(now, last_time) / 1_000_000.0
    last_time = now
    filt.sample_period = dt

    # Update filter
    filt.update_imu(gx, gy, gz, ax, ay, az)

    # Quaternion to pitch (Euler)
    q = filt.quaternion
    w, x, y, z = q
    pitch_rad = asin(-2.0 * (x * z - w * y))
    pitch_deg = degrees(pitch_rad)

    # Apply low-pass filter
    filtered_pitch = alpha * filtered_pitch + (1 - alpha) * pitch_deg
    return filtered_pitch

# Main loop
while True:
    pitch = get_filtered_pitch()
    print("Filtered Pitch: {:.2f}°".format(pitch), end="\r")
    sleep_ms(10)
