from imu import MPU6050
from bmp180 import BMP180
from time import ticks_us, time, sleep_ms
from machine import Pin, I2C, PWM
import math
import os

# === Initialize I2C, MPU6050, BMP180 ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
i2C = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)  # Slower for more reliability
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

# === Servo Setup ===
servo1 = PWM(Pin(9))
servo2 = PWM(Pin(21))
servo1.freq(50)
servo2.freq(50)

# === Constants for velocity calculation ===
G = 9.81  # gravity m/s^2
ACC_THRESHOLD_NOISE = 0.1  # Threshold to filter out noise in m/s^2
prev_velocity = 0.0

# === Flight event detection thresholds ===
LIFTOFF_ACC_THRESHOLD = 0.5  # m/s^2
APOGEE_VEL_THRESHOLD = 0.1  # m/s
LANDED_ACC_THRESHOLD = 0.05  # m/s^2
LANDED_VEL_THRESHOLD = 0.05  # m/s

last_event = "Waiting for Liftoff"
prev_altitude = None
last_flush_time = time()

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

def safe_str(val):
    return f"{val:.3f}" if isinstance(val, (int, float)) else "N/A"

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

# === Create CSV File and Write Header with spacing for readability ===
filename = get_next_log_filename()
log_file = open(filename, "w")
log_file.write("Time (s)  ,  Altitude (m)  ,  Baro Altitude Derivative (m/s)  ,  Roll (deg)  ,  Pitch (deg)  ,  Velocity (m/s)  ,  Servo Angle (deg)  ,  Event\n")
log_file.flush()

# === MAIN LOOP ===
while True:
    try:
        global prev_velocity  # ensure updates affect global variable

        # Initialize placeholders for data
        ax = ay = az = gy = None
        altitude = None
        baro_vel = None
        roll = None
        pitch_deg = None
        pitch_angle_val = None
        velocity = None
        servo_val = None
        lin_acc_y = 0  # default to zero if no MPU data

        # === MPU6050 Reading and calculations ===
        try:
            ax, ay, az = imu.accel.x, imu.accel.y, imu.accel.z
            gy = imu.gyro.y

            pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            roll = math.atan2(ay, az) * (180 / math.pi)
            pitch_deg = pitch_acc * (180 / math.pi)

            current_time_us = ticks_us()
            dt = (current_time_us - prev_time) / 1_000_000.0
            prev_time = current_time_us

            pitch_angle_val = kalman_update(pitch_deg, gy, dt)
            pitch_angle_val = max(-90, min(90, pitch_angle_val))

            # Gravity compensation for velocity calc
            pitch_rad = pitch_acc
            roll_rad = math.atan2(ay, az)

            g_y = -G * math.cos(pitch_rad) * math.sin(roll_rad)
            lin_acc_y = (ay * G) - g_y

            # Filter noise
            if abs(lin_acc_y) < ACC_THRESHOLD_NOISE:
                lin_acc_y = 0

            # Integrate velocity, reset on landed
            if last_event == "Landed":
                velocity = 0
            else:
                velocity = prev_velocity + lin_acc_y * dt
            prev_velocity = velocity

            # Servo control
            servo_val = map_pitch_to_servo(pitch_angle_val)
            servo1.duty_u16(servo_val)
            servo2.duty_u16(servo_val)

        except Exception as e_mpu:
            print("[MPU6050 ERROR]:", e_mpu)
            reconnect_imu()
            pitch_angle_val = None
            velocity = None
            servo_val = None
            roll = None
            lin_acc_y = 0

        # === BMP180 Reading and calculations ===
        try:
            bmp.measure()
            altitude = bmp.altitude

            now_time = time()
            dt_baro = max(now_time - last_flush_time, 0.1)
            if prev_altitude is not None:
                baro_vel = (altitude - prev_altitude) / dt_baro
            else:
                baro_vel = 0.0
            prev_altitude = altitude

        except Exception as e_bmp:
            print("[BMP180 ERROR]:", e_bmp)
            altitude = None
            baro_vel = None

        # === Flight Event Detection ===
        event = last_event
        try:
            acc_check = lin_acc_y
            velocity_check = velocity if velocity is not None else 0
            baro_vel_check = baro_vel if baro_vel is not None else 0

            if last_event == "Waiting for Liftoff":
                if acc_check > LIFTOFF_ACC_THRESHOLD:
                    event = "Liftoff"
            elif last_event == "Liftoff":
                if velocity_check < -APOGEE_VEL_THRESHOLD and baro_vel_check < -APOGEE_VEL_THRESHOLD:
                    event = "Apogee"
            elif last_event == "Apogee":
                if abs(acc_check) < LANDED_ACC_THRESHOLD and abs(velocity_check) < LANDED_VEL_THRESHOLD:
                    event = "Landed"
        except Exception as e_evt:
            print("[EVENT DETECTION ERROR]:", e_evt)

        last_event = event

        # === Format time as seconds.milliseconds (s.ms) ===
        elapsed_ms = int(time() * 1000)
        seconds = elapsed_ms // 1000
        milliseconds = elapsed_ms % 1000
        time_str = f"{seconds}.{milliseconds:03d}"

        servo_angle_deg = 180 - pitch_angle_val if pitch_angle_val is not None else None

        # === Write data line with spaces between columns ===
        log_file.write(
            f"{time_str}  ,  "
            f"{safe_str(altitude)}  ,  "
            f"{safe_str(baro_vel)}  ,  "
            f"{safe_str(roll)}  ,  "
            f"{safe_str(pitch_angle_val)}  ,  "
            f"{safe_str(velocity)}  ,  "
            f"{safe_str(servo_angle_deg)}  ,  "
            f"{event}\n"
        )

        # === Flush to disk every 10 seconds ===
        if time() - last_flush_time >= 10:
            log_file.flush()
            last_flush_time = time()
            print(f"\n[FLUSHED] Time={time_str} | Altitude={safe_str(altitude)} m | Velocity={safe_str(velocity)} m/s | Event={event}")

        print(f"Pitch: {safe_str(pitch_angle_val)}° | Roll: {safe_str(roll)}° | Alt: {safe_str(altitude)} m | Vel: {safe_str(velocity)} m/s | Event: {event}", end="\r")

    except Exception as e:
        print("\n[MAIN LOOP ERROR]:", e)
        reconnect_imu()
        sleep_ms(500)
