import math
import os
from time import ticks_us, ticks_diff, ticks_ms, time, sleep_ms
from machine import I2C, Pin, PWM
from mpu6500 import MPU6500
from bmp180 import BMP180

# === I2C Setup ===
i2c_imu = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
i2c_bmp = I2C(1, sda=Pin(2), scl=Pin(3), freq=100_000)

# === Servo Setup ===
servo1 = PWM(Pin(9))
servo2 = PWM(Pin(21))
servo1.freq(50)
servo2.freq(50)

def angle_to_duty(angle):
    angle = max(135, min(225, angle))  # ±45° limit
    return int((angle / 180.0) * 3276) + 3277

servo1.duty_u16(angle_to_duty(180))
servo2.duty_u16(angle_to_duty(180))

# === MPU Initialization ===
def init_mpu():
    while True:
        try:
            mpu = MPU6500(i2c_imu)
            print("✅ MPU6500 connected.")
            return mpu
        except:
            print("⚠️ MPU6500 connection failed. Retrying...")
            sleep_ms(500)

mpu = init_mpu()

# === BMP180 Initialization ===
bmp = None
bmp_last_attempt = ticks_ms()
bmp_retry_interval = 2000

def try_connect_bmp180():
    global bmp
    try:
        bmp = BMP180(i2c_bmp)
        bmp.oversample_sett = 2
        bmp.sea_level_pressure = 101325
        print("✅ BMP180 connected.")
    except Exception as e:
        bmp = None
        print("⚠️ BMP180 connection failed:", e)

try_connect_bmp180()

# === Gyro Calibration ===
def calibrate_gyro(samples=500, delay_ms=5):
    print("🔧 Calibrating gyro... Keep the board still.")
    gx_total = gy_total = gz_total = 0.0
    for _ in range(samples):
        gx, gy, gz = mpu.gyro
        gx_total += gx
        gy_total += gy
        gz_total += gz
        sleep_ms(delay_ms)
    print("✅ Calibration complete.")
    return gx_total / samples, gy_total / samples, gz_total / samples

gx_bias, gy_bias, gz_bias = calibrate_gyro()

# === Complementary Filter Setup ===
roll = pitch = yaw = 0.0
alpha = 0.98
last_time = ticks_us()

# === Yaw PID Setup ===
desired_yaw = 0.0
Kp = 0.6
Ki = 0.2
Kd = 0.05
integral = 0.0
last_error = 0.0

# === Logging Setup ===
def get_next_log_filename():
    n = 1
    while True:
        name = f"log{n}.csv"
        if name not in os.listdir():
            return name
        n += 1

filename = get_next_log_filename()
log_file = open(filename, "w")
log_file.write("Time (s), Altitude (m), Roll (deg), Pitch (deg), Yaw (deg), Servo1 (deg), Servo2 (deg)\n")
log_file.flush()
boot_time_us = ticks_us()
last_flush_time = time()

# === Main Loop ===
while True:
    now = ticks_us()
    dt = ticks_diff(now, last_time) / 1_000_000
    last_time = now
    if dt <= 0 or dt > 0.1:
        dt = 0.01

    try:
        ax, ay, az = mpu.acceleration
        gx, gy, gz = mpu.gyro
    except:
        print("⚠️ MPU read failed. Reconnecting...")
        mpu = init_mpu()
        continue

    try:
        if bmp:
            bmp.measure()
            altitude = bmp.altitude
        else:
            if ticks_diff(ticks_ms(), bmp_last_attempt) > bmp_retry_interval:
                bmp_last_attempt = ticks_ms()
                try_connect_bmp180()
            altitude = None
    except:
        altitude = None
        bmp = None

    gx -= gx_bias
    gy -= gy_bias
    gz -= gz_bias

    gx_deg = math.degrees(gx)
    gy_deg = math.degrees(gy)
    gz_deg = math.degrees(gz)

    def get_accel_angles(ax, ay, az):
        ay -= 9.81
        norm = math.sqrt(ax**2 + ay**2 + az**2)
        if norm == 0: return 0.0, 0.0
        ax /= norm; ay /= norm; az /= norm
        return math.degrees(math.atan2(ay, az)), math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

    roll_acc, pitch_acc = get_accel_angles(ax, ay, az)

    roll_gyro = roll + gx_deg * dt
    pitch_gyro = pitch + gy_deg * dt
    yaw += gz_deg * dt

    # Wrap yaw
    if yaw > 180: yaw -= 360
    elif yaw < -180: yaw += 360

    roll = alpha * roll_gyro + (1 - alpha) * roll_acc
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc

    # PID Yaw Control
    error = desired_yaw - yaw
    error = (error + 180) % 360 - 180
    integral += error * dt
    derivative = (error - last_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error

    servo1_angle = 180 - output
    servo2_angle = 180 - output

    servo1.duty_u16(angle_to_duty(servo1_angle))
    servo2.duty_u16(angle_to_duty(servo2_angle))

    # Time and logging
    timestamp = ticks_diff(now, boot_time_us) / 1_000_000.0
    altitude_str = f"{altitude:.2f}" if altitude else "N/A"

    log_file.write(f"{timestamp:.2f}, {altitude_str}, {roll:.2f}, {pitch:.2f}, {yaw:.2f}, {servo1_angle:.2f}, {servo2_angle:.2f}\n")

    if time() - last_flush_time >= 10:
        log_file.flush()
        last_flush_time = time()
        print(f"[LOG] ⏱ {timestamp:.2f}s | Alt: {altitude_str}m")

    print(f"Yaw: {yaw:.2f}° | Err: {error:.2f}° | S1: {servo1_angle:.1f}° | S2: {servo2_angle:.1f}°", end="\r")

    sleep_ms(10)
