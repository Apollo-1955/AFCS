from imu import MPU6050  # MPU6050 driver
from bmp180 import BMP180
from time import ticks_us, ticks_diff, time, sleep_ms, sleep, ticks_ms
from machine import Pin, I2C, PWM
import math
import os
import gc  # For RAM monitoring

# === Initialize I2C, MPU6050 ===
i2c_imu = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c_imu)

# === Initialize I2C for BMP180 ===
i2c_bmp = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)
bmp = None
bmp_last_attempt = ticks_ms()
bmp_retry_interval = 2000  # milliseconds

def try_connect_bmp180():
    global bmp
    try:
        bmp = BMP180(i2c_bmp)
        bmp.oversample_sett = 2
        bmp.sea_level_pressure = 101325
        print("BMP180 connected successfully!")
    except Exception as e:
        bmp = None
        print("BMP180 connection failed:", e)

# Try to connect BMP180 once at start
try_connect_bmp180()

# === MPU6050 Calibration ===
def calibrate_imu(samples=200):
    print("Calibrating MPU6050...")
    accel_offsets = [0.0, 0.0, 0.0]
    gyro_offsets = [0.0, 0.0, 0.0]
    for _ in range(samples):
        ax, ay, az = imu.accel.x, imu.accel.y, imu.accel.z
        gx, gy, gz = imu.gyro.x, imu.gyro.y, imu.gyro.z
        accel_offsets[0] += ax
        accel_offsets[1] += ay
        accel_offsets[2] += az
        gyro_offsets[0] += gx
        gyro_offsets[1] += gy
        gyro_offsets[2] += gz
        sleep_ms(5)
    accel_offsets = [x / samples for x in accel_offsets]
    gyro_offsets = [x / samples for x in gyro_offsets]
    accel_offsets[2] -= 1.0  # remove gravity from Z
    print(f"Accel offsets: {accel_offsets}")
    print(f"Gyro offsets: {gyro_offsets}")
    return accel_offsets, gyro_offsets

accel_offset, gyro_offset = calibrate_imu()

# === BMP180 Calibration ===
def calibrate_bmp180():
    if not bmp:
        return None
    print("Calibrating BMP180...")
    pressures = []
    for _ in range(10):
        bmp.measure()
        pressures.append(bmp.pressure)
        sleep_ms(100)
    avg_pressure = sum(pressures) / len(pressures)
    print(f"Baseline pressure: {avg_pressure} Pa")
    return avg_pressure

baseline_pressure = calibrate_bmp180()
if baseline_pressure is not None:
    bmp.sea_level_pressure = baseline_pressure

# === Kalman Filter ===
Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03

pitch_angle = 0.0
bias_pitch = 0.0
P = [[0, 0], [0, 0]]
prev_time_us = ticks_us()
boot_time_us = prev_time_us

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

servo1.duty_u16(map_pitch_to_servo(0))
servo2.duty_u16(map_pitch_to_servo(0))

def reconnect_imu():
    global i2c_imu, imu
    try:
        i2c_imu.deinit()
    except:
        pass
    try:
        i2c_imu = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
        imu = MPU6050(i2c_imu)
        print("IMU reconnected successfully.")
    except Exception as e:
        print("IMU reconnection failed:", e)

def get_next_log_filename():
    n = 1
    while True:
        fname = f"log{n}.csv"
        if fname not in os.listdir():
            return fname
        n += 1

filename = get_next_log_filename()
log_file = open(filename, "w")
log_file.write("Timestamp (s),   Altitude (m),   Roll (deg),   Pitch (deg),   Velocity Z (deg/s),   Servo Angle (deg),   Event\n\n")
log_file.flush()
last_flush_time = time()

liftoff_detected = False
apogee_detected = False
landed_detected = False
stable_time = 0
prev_velocity_z = 0

# === MAIN LOOP ===
while True:
    try:
        # === START: RAM & CPU monitoring ===
        loop_start_us = ticks_us()
        gc.collect()
        free_ram_before = gc.mem_free()

        # === Sensor Read with offsets ===
        ax = imu.accel.x - accel_offset[0]
        ay = imu.accel.y - accel_offset[1]
        az = imu.accel.z - accel_offset[2]
        gx = imu.gyro.x - gyro_offset[0]
        gy = imu.gyro.y - gyro_offset[1]
        gz = imu.gyro.z - gyro_offset[2]

        # === BMP180 Altitude (non-blocking) ===
        altitude = None
        if bmp:
            try:
                bmp.measure()
                altitude = bmp.altitude
            except Exception as e:
                print("BMP180 read error:", e)
                bmp = None
        else:
            now = ticks_ms()
            if (now - bmp_last_attempt) > bmp_retry_interval:
                bmp_last_attempt = now
                try_connect_bmp180()

        # === Kalman Filter Pitch ===
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
        current_time_us = ticks_us()
        dt = ticks_diff(current_time_us, prev_time_us) / 1_000_000.0
        prev_time_us = current_time_us
        pitch_angle = kalman_update(pitch_acc, gy, dt)
        pitch_angle = max(-90, min(90, pitch_angle))

        # === Roll and Angular Velocity ===
        roll = math.atan2(ay, az) * (180 / math.pi)
        velocity_z = gz

        # === Servo Control ===
        servo_deg = 180 - pitch_angle
        servo_val = map_pitch_to_servo(pitch_angle)
        servo1.duty_u16(servo_val)
        servo2.duty_u16(servo_val)

        # === Timestamp & Event Detection ===
        timestamp = ticks_diff(current_time_us, boot_time_us) / 1_000_000.0
        event = ""
        vertical_accel = az - 1
        if not liftoff_detected and vertical_accel > 0.5:
            liftoff_detected = True
            event = "🚀 Liftoff"
        if liftoff_detected and not apogee_detected and prev_velocity_z > 0 and velocity_z <= 0:
            apogee_detected = True
            event = "📍 Apogee"
        if apogee_detected and not landed_detected:
            if abs(velocity_z) < 0.3 and abs(vertical_accel) < 0.2:
                stable_time += dt
                if stable_time >= 2:
                    landed_detected = True
                    event = "✅ Landed"
            else:
                stable_time = 0

        # === Logging ===
        altitude_str = f"{altitude:.2f}" if altitude is not None else "N/A"
        log_file.write(f"{timestamp:.2f},           {altitude_str},          {roll:.2f},         {pitch_angle:.2f},           {velocity_z:.2f},             {servo_deg:.2f},         {event}\n")
        prev_velocity_z = velocity_z

        # === Flush every 10 sec ===
        if time() - last_flush_time >= 10:
            log_file.flush()
            last_flush_time = time()
            print(f"\n[FLUSHED] ⏱ {timestamp:.2f}s | Alt={altitude_str} m | Roll={roll:.2f}°")

        # === RAM & CPU stats print every 5 seconds ===
        free_ram_after = gc.mem_free()
        loop_end_us = ticks_us()
        loop_time_ms = ticks_diff(loop_end_us, loop_start_us) / 1000
        if int(timestamp) % 5 == 0:
            print(f"\n[DEBUG] Loop: {loop_time_ms:.2f} ms | Free RAM: {free_ram_after} B")

        print(f"Pitch: {pitch_angle:.2f}° | Roll: {roll:.2f}° | Alt: {altitude_str} m", end="\r")

    except Exception as e:
        print("\n[ERROR]:", e)
        reconnect_imu()
        sleep_ms(500)
