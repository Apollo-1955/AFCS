from imu import MPU6050
from bmp180 import BMP180
from time import ticks_us, ticks_diff, time, sleep_ms, sleep, ticks_ms
from machine import Pin, I2C, PWM
import math
import os

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

try_connect_bmp180()

# === Kalman Filter Setup ===
Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03

roll_angle = 0.0    # This is now the Kalman filtered pitch (treated as roll)
bias_roll = 0.0
P = [[0, 0], [0, 0]]
prev_time_us = ticks_us()
boot_time_us = prev_time_us

# === Servo Setup ===
servo1 = PWM(Pin(9))
servo2 = PWM(Pin(21))
servo1.freq(50)
servo2.freq(50)

def kalman_update(new_angle, new_rate, dt):
    global roll_angle, bias_roll, P
    rate = new_rate - bias_roll
    roll_angle += dt * rate

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += Q_bias * dt

    S = P[0][0] + R_measure
    K = [P[0][0] / S, P[1][0] / S]
    y = new_angle - roll_angle
    roll_angle += K[0] * y
    bias_roll += K[1] * y

    P[0][0] -= K[0] * P[0][0]
    P[0][1] -= K[0] * P[0][1]
    P[1][0] -= K[1] * P[0][0]
    P[1][1] -= K[1] * P[0][1]

    return roll_angle

def map_pitch_to_servo(angle):
    # Map from -90..90 degrees to servo duty cycle roughly between 90° and 270°
    servo_angle = 180 - angle  # Invert angle so positive pitch moves servo correctly
    return int((servo_angle / 180.0) * 3276) + 3277

# Center servos at startup
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

# === Create log file with header ===
filename = get_next_log_filename()
log_file = open(filename, "w")
log_file.write(
    "Timestamp (s),   Altitude (m),   Roll (deg),   Pitch (deg),   Velocity Z (deg/s),   Servo Angle (deg),   Event\n\n"
)
log_file.flush()
last_flush_time = time()

# === Flight Detection Flags ===
liftoff_detected = False
apogee_detected = False
landed_detected = False
stable_time = 0
prev_velocity_z = 0

# === MAIN LOOP ===
while True:
    try:
        # === Sensor Read ===
        ax, ay, az = imu.accel.x, imu.accel.y, imu.accel.z
        gy = imu.gyro.y
        gz = imu.gyro.z

        # === BMP180 reading, non-blocking ===
        altitude = None
        if bmp:
            try:
                bmp.measure()
                altitude = bmp.altitude
            except Exception as e:
                print("BMP180 read error:", e)
                bmp = None
                altitude = None
        else:
            now = ticks_ms()
            if (now - bmp_last_attempt) > bmp_retry_interval:
                bmp_last_attempt = now
                try_connect_bmp180()

        # === Kalman filtered 'roll' from pitch accelerometer ===
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
        current_time_us = ticks_us()
        dt = ticks_diff(current_time_us, prev_time_us) / 1_000_000.0
        prev_time_us = current_time_us

        roll_angle = kalman_update(pitch_acc, gy, dt)
        roll_angle = max(-90, min(90, roll_angle))  # Clamp if desired

        # === Calculate actual pitch (for info/logging only) ===
        pitch = math.atan2(ay, az) * (180 / math.pi)

        # === Velocity ===
        velocity_z = gz  # raw Z-axis rotational velocity

        # === Servo Control using roll_angle (the Kalman filtered pitch) ===
        servo_deg = 180 - roll_angle
        servo_val = map_pitch_to_servo(roll_angle)
        servo1.duty_u16(servo_val)
        servo2.duty_u16(servo_val)

        # === Timestamp ===
        timestamp = ticks_diff(current_time_us, boot_time_us) / 1_000_000.0

        # === Event Detection ===
        event = ""
        vertical_accel = az - 1  # approximate net upward acceleration in g's

        # Liftoff detection
        if not liftoff_detected and vertical_accel > 0.5:
            liftoff_detected = True
            event = "🚀 Liftoff"

        # Apogee detection
        if liftoff_detected and not apogee_detected and prev_velocity_z > 0 and velocity_z <= 0:
            apogee_detected = True
            event = "📍 Apogee"

        # Landed detection (after apogee, near 0 velocity and acceleration for 2s)
        if apogee_detected and not landed_detected:
            if abs(velocity_z) < 0.3 and abs(vertical_accel) < 0.2:
                stable_time += dt
                if stable_time >= 2:
                    landed_detected = True
                    event = "✅ Landed"
            else:
                stable_time = 0

        # === Log ===
        altitude_str = f"{altitude:.2f}" if altitude is not None else "N/A"
        log_file.write(
            f"{timestamp:.2f},           {altitude_str},          {roll_angle:.2f},         {pitch:.2f},           {velocity_z:.2f},             {servo_deg:.2f},         {event}\n"
        )

        prev_velocity_z = velocity_z

        # === Flush every 10 sec ===
        if time() - last_flush_time >= 10:
            log_file.flush()
            last_flush_time = time()
            print(f"\n[FLUSHED] ⏱ {timestamp:.2f}s | Alt={altitude_str} m | Roll={roll_angle:.2f}°")

        print(f"Roll (Kalman pitch): {roll_angle:.2f}° | Pitch (raw): {pitch:.2f}° | Alt: {altitude_str} m", end="\r")

    except Exception as e:
        print("\n[ERROR]:", e)
        reconnect_imu()
        sleep_ms(500)
