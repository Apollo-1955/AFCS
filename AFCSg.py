from imu import MPU6050
from bmp180 import BMP180
from time import ticks_us, ticks_ms, sleep_ms
from machine import Pin, I2C, PWM
import math
import os

# === Initialize I2C, MPU6050, BMP180 ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
i2C = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)  # Slower for reliability
imu = MPU6050(i2c)
bmp = BMP180(i2C)
bmp.oversample_sett = 2  # high resolution

# === Kalman Filter Setup ===
Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03

pitch_angle = 0.0
bias_pitch = 0.0
P = [[0, 0], [0, 0]]

prev_time = ticks_us()
start_ms = ticks_ms()

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
log_file.write("Timestamp (s),  Altitude (m),  Altitude Rate (m/s),  Roll (deg),  Velocity (m/s),  Servo Angle (deg),  Event\n")
log_file.flush()

last_flush_time = ticks_ms()

# === Event Detection Thresholds (more sensitive) ===
ACC_THRESHOLD_LIFTOFF = 0.5    # m/s² upward acceleration threshold
VEL_THRESHOLD_APOGEE = 0.1     # m/s upward velocity near zero to detect apogee
VEL_THRESHOLD_LANDED = 0.05    # m/s near zero velocity to detect landed
ACC_THRESHOLD_LANDED = 0.1     # m/s² near zero acceleration to detect landed

prev_altitude = None
prev_altitude_time = None
prev_velocity = 0
last_event = "None"

# === MAIN LOOP ===
while True:
    try:
        # === Sensor Reading ===
        ax, ay, az = imu.accel.x, imu.accel.y, imu.accel.z
        gy = imu.gyro.y
        bmp.measure()
        altitude = bmp.altitude
        roll = math.atan2(imu.accel.y, imu.accel.z) * (180 / math.pi)

        # === Kalman Pitch Calculation ===
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
        current_time_us = ticks_us()
        dt = (current_time_us - prev_time) / 1_000_000.0
        prev_time = current_time_us
        pitch_angle = kalman_update(pitch_acc, gy, dt)
        pitch_angle = max(-90, min(90, pitch_angle))

        # === Calculate time elapsed for timestamp ===
        elapsed_ms = ticks_ms() - start_ms
        timestamp = elapsed_ms / 1000  # seconds with decimals

        # === Calculate altitude rate (derivative) ===
        if prev_altitude is not None and prev_altitude_time is not None:
            altitude_rate = (altitude - prev_altitude) / ((elapsed_ms - prev_altitude_time) / 1000)
        else:
            altitude_rate = 0

        prev_altitude = altitude
        prev_altitude_time = elapsed_ms

        # === Calculate velocity from accelerometer y-axis (assuming upward axis) ===
        # Convert raw acceleration to m/s² assuming imu.accel.y is in g, multiply by 9.81
        velocity = prev_velocity + (ay * 9.81) * dt
        prev_velocity = velocity

        # === Servo Control ===
        servo_val = map_pitch_to_servo(pitch_angle)
        servo1.duty_u16(servo_val)
        servo2.duty_u16(servo_val)

        servo_degrees = 180 - pitch_angle  # Same as map_pitch_to_servo logic but degrees

        # === Event detection ===
        event = ""
        if last_event == "None" or last_event == "Landed":
            if ay * 9.81 > ACC_THRESHOLD_LIFTOFF and velocity > 0.1:
                event = "Liftoff"
                last_event = "Liftoff"
        elif last_event == "Liftoff":
            if velocity < -VEL_THRESHOLD_APOGEE:
                event = "Apogee"
                last_event = "Apogee"
        elif last_event == "Apogee":
            if abs(velocity) < VEL_THRESHOLD_LANDED and abs(ay * 9.81) < ACC_THRESHOLD_LANDED:
                event = "Landed"
                last_event = "Landed"

        # === Write data to CSV ===
        # Columns spaced with double commas and spaces for readability
        log_file.write(
            f"{timestamp:.2f},  {altitude:.2f},  {altitude_rate:.2f},  {roll:.2f},  {velocity:.2f},  {servo_degrees:.2f},  {event}\n"
        )

        # === Periodic flush every 10 seconds to save data safely ===
        if ticks_ms() - last_flush_time >= 10_000:
            log_file.flush()
            last_flush_time = ticks_ms()
            print(f"\n[FLUSHED] Time={timestamp:.2f}s Alt={altitude:.2f}m Roll={roll:.2f}° Event={event}")

        print(f"Time: {timestamp:.2f}s | Pitch: {pitch_angle:.2f}° | Roll: {roll:.2f}° | Alt: {altitude:.2f} m | Vel: {velocity:.2f} m/s | Event: {event}", end="\r")

    except Exception as e:
        print("\n[ERROR]:", e)
        reconnect_imu()
        sleep_ms(500)
