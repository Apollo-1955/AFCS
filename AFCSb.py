from imu import MPU6050
from bmp180 import BMP180
from time import ticks_us, ticks_diff, time, sleep_ms
from machine import Pin, I2C, PWM
import math
import os

# === Initialize I2C, MPU6050, BMP180 ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
i2C = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)  # Separate I2C for BMP180
imu = MPU6050(i2c)
bmp = BMP180(i2C)
bmp.oversample_sett = 2
bmp.sea_level_pressure = 101325  # Adjust if needed

# === Kalman Filter Setup ===
Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03

pitch_angle = 0.0
bias_pitch = 0.0
P = [[0, 0], [0, 0]]

prev_time_us = ticks_us()
velocity_z = 0.0

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
    pitch = max(-90, min(90, pitch))
    servo_angle = 180 - pitch
    return int((servo_angle / 180.0) * 3276) + 3277, servo_angle

# === Neutral Servo Start ===
servo1.duty_u16(map_pitch_to_servo(0)[0])
servo2.duty_u16(map_pitch_to_servo(0)[0])

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

def get_next_log_filename():
    n = 1
    while True:
        fname = f"log{n}.csv"
        if fname not in os.listdir():
            return fname
        n += 1

# === Create CSV File with Clear Headers ===
filename = get_next_log_filename()
log_file = open(filename, "w")
log_file.write("Time (s), Altitude (m), Roll (deg), Pitch (deg), Velocity Z (m/s), Servo Angle (deg)\n\n")
log_file.flush()

last_flush_time = time()

# === MAIN LOOP ===
while True:
    try:
        current_time_us = ticks_us()
        dt = ticks_diff(current_time_us, prev_time_us) / 1_000_000.0
        prev_time_us = current_time_us
        timestamp = time()

        ax, ay, az = imu.accel.x, imu.accel.y, imu.accel.z
        gy = imu.gyro.y
        bmp.measure()
        altitude = bmp.altitude
        roll = math.atan2(ay, az) * (180 / math.pi)

        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
        pitch_angle = kalman_update(pitch_acc, gy, dt)

        g_force = 9.80665
        acc_z = az * g_force
        velocity_z += acc_z * dt

        duty_val, servo_deg = map_pitch_to_servo(pitch_angle)
        servo1.duty_u16(duty_val)
        servo2.duty_u16(duty_val)

        log_file.write(f"{timestamp}, {altitude:.2f}, {roll:.2f}, {pitch_angle:.2f}, {velocity_z:.2f}, {servo_deg:.2f}\n")

        if timestamp - last_flush_time >= 10:
            log_file.flush()
            last_flush_time = timestamp
            print(f"\n[FLUSHED] Time={timestamp}s | Alt={altitude:.2f} m | Roll={roll:.2f}°")

        print(f"t={timestamp}s | Pitch={pitch_angle:.2f}° | Roll={roll:.2f}° | Alt={altitude:.2f} m | Vz={velocity_z:.2f} m/s | Servo={servo_deg:.1f}°", end="\r")

    except Exception as e:
        print("\n[ERROR]:", e)
        reconnect_imu()
        sleep_ms(500)
