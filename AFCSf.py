from imu import MPU6050
from bmp180 import BMP180
from time import ticks_us, time, sleep_ms
from machine import Pin, I2C, PWM
import math
import os

# === Initialize I2C, MPU6050, BMP180 ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
i2C = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)  # Slower for reliability
imu = MPU6050(i2c)

# BMP180 with reconnect logic
bmp = None
while bmp is None:
    try:
        bmp = BMP180(i2C)
        bmp.oversample_sett = 2
        bmp.sea_level_pressure = 101325  # Pa
    except Exception as e:
        print("BMP180 init failed:", e)
        sleep_ms(2000)

# === Kalman Filter Setup ===
Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03

pitch_angle = 0.0
bias_pitch = 0.0
P = [[0, 0], [0, 0]]

prev_time = ticks_us()

# === Servo Setup ===
servo1 = PWM(Pin(9))
servo2 = PWM(Pin(21))
servo1.freq(50)
servo2.freq(50)

def kalman_update(new_angle, new_rate, dt):
    global pitch_angle, bias_pitch, P

    rate = new_rate - bias_pitch
    pitch_angle += dt * rate

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += Q_bias * dt

    S = P[0][0] + R_measure
    K = [P[0][0] / S, P[1][0] / S]

    y = new_angle - pitch_angle
    pitch_angle += K[0] * y
    bias_pitch += K[1] * y

    P[0][0] -= K[0] * P[0][0]
    P[0][1] -= K[0] * P[0][1]
    P[1][0] -= K[1] * P[0][0]
    P[1][1] -= K[1] * P[0][1]

    return pitch_angle

def map_pitch_to_servo(pitch):
    servo_angle = 180 - pitch
    return int((servo_angle / 180.0) * 3276) + 3277

# === Move Servos to Neutral ===
servo1.duty_u16(map_pitch_to_servo(0))
servo2.duty_u16(map_pitch_to_servo(0))

# === IMU Reconnect Fallback ===
def reconnect_imu():
    global i2c, imu
    try:
        i2c.deinit()
    except:
        pass
    try:
        i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
        imu = MPU6050(i2c)
        print("IMU reconnected successfully.")
    except Exception as e:
        print("IMU reconnection failed:", e)

# === Auto-Increment CSV Filename ===
def get_next_log_filename():
    n = 1
    while True:
        fname = f"log{n}.csv"
        if fname not in os.listdir():
            return fname
        n += 1

# === Create CSV File and Write Header ===
filename = get_next_log_filename()
log_file = open(filename, "w")
log_file.write("Timestamp (s) ,  Altitude (m) ,  Altitude Rate (m/s) ,  Roll (deg) ,  Velocity Z (m/s) ,  Servo Angle (deg) ,  Event\n")
log_file.flush()

last_flush_time = time()

# === Flight Event Flags and Variables ===
liftoff_detected = False
apogee_detected = False
landed_detected = False

prev_altitude = None
prev_time_s = None
velocity_z = 0.0
stable_time = 0.0

# === MAIN LOOP ===
while True:
    try:
        # --- Sensor Reading ---
        ax, ay, az = imu.accel.x, imu.accel.y, imu.accel.z
        gy = imu.gyro.y

        bmp.measure()
        altitude = bmp.altitude

        roll = math.atan2(imu.accel.y, imu.accel.z) * (180 / math.pi)

        # --- Time management ---
        current_time_us = ticks_us()
        dt = (current_time_us - prev_time) / 1_000_000.0
        prev_time = current_time_us

        current_time_s = time()
        if prev_time_s is None:
            prev_time_s = current_time_s

        # --- Kalman Pitch Calculation ---
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
        pitch_angle = kalman_update(pitch_acc, gy, dt)
        pitch_angle = max(-90, min(90, pitch_angle))

        # --- Velocity estimation along z-axis ---
        if prev_altitude is not None:
            dz = altitude - prev_altitude
            velocity_z = dz / dt
            # altitude rate derivative for redundancy is dz/dt (same as velocity_z)
        else:
            dz = 0.0
            velocity_z = 0.0

        prev_altitude = altitude

        # --- Servo Control ---
        servo_val = map_pitch_to_servo(pitch_angle)
        servo1.duty_u16(servo_val)
        servo2.duty_u16(servo_val)

        # --- Event detection with more sensitive thresholds ---
        event = ""

        # Liftoff: detect when upward acceleration > 0.2g (more sensitive)
        vertical_accel = az  # Assuming z-axis points up/down (adjust if needed)
        if not liftoff_detected and vertical_accel > 0.2:
            liftoff_detected = True
            event = "LIFTOFF"

        # Apogee: altitude peaked and now descending by more than 0.05 m/s
        if liftoff_detected and not apogee_detected:
            if prev_altitude is not None and dz < -0.05:
                apogee_detected = True
                event = "APOGEE"

        # Landed: velocity < 0.15 m/s and accel < 0.1g stable for 1 second
        if liftoff_detected and apogee_detected and not landed_detected:
            if abs(velocity_z) < 0.15 and abs(vertical_accel) < 0.1:
                stable_time += dt
                if stable_time >= 1.0:
                    landed_detected = True
                    event = "LANDED"
            else:
                stable_time = 0.0

        # --- Write data with spaces for readability ---
        log_file.write(f"{current_time_s:0.2f} ,  {altitude:0.2f} ,  {dz/dt if dt > 0 else 0:0.3f} ,  {roll:0.2f} ,  {velocity_z:0.3f} ,  {(servo_val - 3277) * 180 / 3276:0.2f} ,  {event}\n")

        # --- Flush every 10 seconds ---
        if time() - last_flush_time >= 10:
            log_file.flush()
            last_flush_time = time()
            print(f"\n[FLUSHED] Altitude={altitude:.2f} m | Roll={roll:.2f}° | Event={event}")

        print(f"Pitch: {pitch_angle:.2f}° | Roll: {roll:.2f}° | Alt: {altitude:.2f} m | Event: {event}", end="\r")

    except Exception as e:
        print("\n[ERROR]:", e)
        reconnect_imu()
        sleep_ms(500)
