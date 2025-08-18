import math
import os
import uos
from machine import I2C, Pin, PWM
from mpu6500 import MPU6500
from time import ticks_us, ticks_diff, sleep_ms, ticks_ms

# === I2C Init ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)

# === Sensor Init with retry ===
def init_mpu():
    while True:
        try:
            mpu = MPU6500(i2c)
            print("MPU6500 connected.")
            return mpu
        except:
            print("MPU6500 connection failed. Retrying...")
            sleep_ms(500)

mpu = init_mpu()

# === Gyro Calibration ===
def calibrate_gyro(samples=500, delay_ms=5):
    print("Calibrating gyro... Keep sensor still.")
    gx_offset = gy_offset = gz_offset = 0.0
    for _ in range(samples):
        gx, gy, gz = mpu.gyro
        gx_offset += gx
        gy_offset += gy
        gz_offset += gz
        sleep_ms(delay_ms)
    return gx_offset / samples, gy_offset / samples, gz_offset / samples

gx_bias, gy_bias, gz_bias = calibrate_gyro()

# === Complementary Filter Setup ===
roll = pitch = yaw = 0.0
alpha = 0.98
last = ticks_us()

# === Desired Yaw ===
desired_yaw = 0.0
yaw_updated = False  # flag to ensure 50° is only added once

# === Servo Setup ===
servo1 = PWM(Pin(9))
servo2 = PWM(Pin(21))
servo1.freq(50)
servo2.freq(50)

def angle_to_duty(angle):
    angle = max(135, min(225, angle))  # ±45° range around center
    return int((angle / 180.0) * 3276) + 3277

# Center servos initially
servo1.duty_u16(angle_to_duty(180))
servo2.duty_u16(angle_to_duty(180))

# === Accelerometer Angles ===
def get_accel_angles(ax, ay, az):
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0:
        return 0.0, 0.0
    ax /= norm; ay /= norm; az /= norm
    roll_acc = math.atan2(ay, az)
    pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    return math.degrees(roll_acc), math.degrees(pitch_acc)

# === Flight Log Setup ===
def next_log_filename():
    n = 1
    while True:
        fname = f"flight{n}.csv"
        if fname not in os.listdir():
            return fname
        n += 1

log_filename = next_log_filename()
log_file = open(log_filename, "w")

# CSV header
log_file.write("time,altitude,velocity,yaw,servo1,servo2,setpoint,apogee\n")
log_file.flush()
print(f"Logging to {log_filename}")

last_save = ticks_ms()
apogee_reached = False
max_alt = -1e9

# === Boot time reference ===
boot_time = ticks_us()

# === Main Loop ===
while True:
    now = ticks_us()
    dt = ticks_diff(now, last) / 1_000_000
    last = now
    if dt <= 0 or dt > 0.1:
        dt = 0.01

    elapsed = ticks_diff(now, boot_time) / 1_000_000  # in seconds

    # === Add 50 degrees to desired_yaw after 15 seconds
    if elapsed >= 15 and not yaw_updated:
        desired_yaw += 50
        yaw_updated = True
        print(f"⏱ 15 seconds passed — desired_yaw set to {desired_yaw}°")

    try:
        ax, ay, az = mpu.acceleration
        gx, gy, gz = mpu.gyro
    except:
        print("Sensor read failed. Attempting reconnect...")
        mpu = init_mpu()
        continue

    gx -= gx_bias
    gy -= gy_bias
    gz -= gz_bias

    gx_deg = math.degrees(gx)
    gy_deg = math.degrees(gy)
    gz_deg = math.degrees(gz)

    roll_acc, pitch_acc = get_accel_angles(ax, ay, az)

    roll_gyro = roll + gx_deg * dt
    pitch_gyro = pitch + gy_deg * dt
    yaw += gz_deg * dt

    if yaw > 180:
        yaw -= 360
    elif yaw < -180:
        yaw += 360

    roll = alpha * roll_gyro + (1 - alpha) * roll_acc
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc

    # === Yaw Correction ===
    yaw_error = desired_yaw - yaw
    yaw_error = max(-90, min(90, yaw_error))  # Clamp to ±90

    servo1_angle = 180 - yaw_error
    servo2_angle = 180 - yaw_error

    servo1.duty_u16(angle_to_duty(servo1_angle))
    servo2.duty_u16(angle_to_duty(servo2_angle))

    # === Altitude & Velocity (stubbed, replace with sensor) ===
    altitude = 0.0
    velocity = 0.0

    # === Apogee detection ===
    if altitude > max_alt:
        max_alt = altitude
    elif altitude < max_alt - 0.5 and not apogee_reached:  # falling 0.5m
        apogee_reached = True
        print("🚀 Apogee detected!")

    apogee_flag = "APOGEE" if apogee_reached else ""

    # === Log Data ===
    log_file.write(f"{elapsed:.2f},{altitude:.2f},{velocity:.2f},{yaw:.2f},{servo1_angle:.1f},{servo2_angle:.1f},{desired_yaw:.2f},{apogee_flag}\n")

    # === Save every 10 seconds ===
    if ticks_diff(ticks_ms(), last_save) >= 10000:
        log_file.flush()
        last_save = ticks_ms()
        print("💾 Data flushed to SD/Flash")

    print(f"Yaw: {yaw:.2f}° | Error: {yaw_error:.2f}° | S1: {servo1_angle:.1f}° | S2: {servo2_angle:.1f}°")

    sleep_ms(10)
