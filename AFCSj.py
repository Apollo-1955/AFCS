from imu import MPU6050
from bmp180 import BMP180
from time import ticks_us, ticks_ms, sleep_ms
from machine import Pin, I2C, PWM, UART
import math
import os

# === Sensor Initialization ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
i2C = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)
imu = MPU6050(i2c)
bmp = BMP180(i2C)
bmp.oversample_sett = 2

# === GPS UART Setup ===
uart = UART(0, baudrate=9600, tx=None, rx=13)
uart.write(b'$PMTK220,100*2F\r\n')  # 10Hz update

gps_alt = None
gps_sats = None
gps_speed = None

def parse_gpgga(sentence):
    parts = sentence.split(',')
    if parts[0] != "$GPGGA":
        return None
    try:
        alt = float(parts[9])
        sats = int(parts[7])
        return alt, sats
    except:
        return None

def parse_gprmc(sentence):
    parts = sentence.split(',')
    if parts[0] != "$GPRMC":
        return None
    try:
        speed_knots = float(parts[7])
        speed_mps = speed_knots * 0.514444
        return speed_mps
    except:
        return None

def update_gps():
    global gps_alt, gps_sats, gps_speed
    try:
        if uart.any():
            line = uart.readline()
            if line:
                try:
                    line = line.decode('ascii').strip()
                    if line.startswith('$GPGGA'):
                        parsed = parse_gpgga(line)
                        if parsed:
                            gps_alt, gps_sats = parsed
                    elif line.startswith('$GPRMC'):
                        speed = parse_gprmc(line)
                        if speed is not None:
                            gps_speed = speed
                except:
                    pass
    except:
        pass

# === Kalman Filter ===
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

def reconnect_imu():
    global i2c, imu
    try:
        i2c.deinit()
    except:
        pass
    try:
        i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
        imu = MPU6050(i2c)
    except Exception as e:
        print("IMU reconnect failed:", e)

# Move servos to neutral
servo1.duty_u16(map_pitch_to_servo(0))
servo2.duty_u16(map_pitch_to_servo(0))

# Log file
def get_next_log_filename():
    n = 1
    while True:
        name = f"log{n}.csv"
        if name not in os.listdir():
            return name
        n += 1

filename = get_next_log_filename()
log_file = open(filename, "w")
log_file.write("Time(s),Altitude(m),AltRate(m/s),Roll(deg),Velocity(m/s),Servo(deg),Event,GPS_Alt(m),GPS_Speed(m/s),GPS_Sats\n")
log_file.flush()

last_flush_time = ticks_ms()
prev_altitude = None
prev_altitude_time = None
prev_velocity = 0
last_event = "None"

# === Main Loop ===
while True:
    try:
        # Update GPS
        update_gps()

        # === Sensor Readings ===
        ax, ay, az = imu.accel.x, imu.accel.y, imu.accel.z
        gy = imu.gyro.y
        bmp.measure()
        altitude = bmp.altitude
        roll = math.atan2(imu.accel.y, imu.accel.z) * (180 / math.pi)

        # Kalman filter for pitch
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
        now_us = ticks_us()
        dt = (now_us - prev_time) / 1_000_000.0
        prev_time = now_us
        pitch_angle = kalman_update(pitch_acc, gy, dt)
        pitch_angle = max(-90, min(90, pitch_angle))

        # Time
        timestamp = (ticks_ms() - start_ms) / 1000

        # Altitude derivative
        if prev_altitude is not None and prev_altitude_time is not None:
            altitude_rate = (altitude - prev_altitude) / ((ticks_ms() - prev_altitude_time) / 1000)
        else:
            altitude_rate = 0

        prev_altitude = altitude
        prev_altitude_time = ticks_ms()

        # Velocity estimation
        velocity = prev_velocity + (ay * 9.81) * dt
        prev_velocity = velocity

        # Servo control
        servo_val = map_pitch_to_servo(pitch_angle)
        servo1.duty_u16(servo_val)
        servo2.duty_u16(servo_val)
        servo_deg = 180 - pitch_angle

        # Event detection
        event = ""
        if last_event == "None" or last_event == "Landed":
            if ay * 9.81 > 0.5 and velocity > 0.1:
                event = "Liftoff"
                last_event = "Liftoff"
        elif last_event == "Liftoff":
            if velocity < -0.1:
                event = "Apogee"
                last_event = "Apogee"
        elif last_event == "Apogee":
            if abs(velocity) < 0.05 and abs(ay * 9.81) < 0.1:
                event = "Landed"
                last_event = "Landed"

        # === Log ===
        log_file.write(
            f"{timestamp:.2f},{altitude:.2f},{altitude_rate:.2f},{roll:.2f},{velocity:.2f},{servo_deg:.2f},{event},{gps_alt or 'N/A'},{gps_speed or 'N/A'},{gps_sats or 'N/A'}\n"
        )

        if ticks_ms() - last_flush_time >= 10000:
            log_file.flush()
            last_flush_time = ticks_ms()
            print(f"\n[FLUSHED] {timestamp:.2f}s | Alt: {altitude:.2f}m | Vel: {velocity:.2f} m/s | Event: {event}")

        print(f"Time: {timestamp:.2f}s | Pitch: {pitch_angle:.2f}° | Alt: {altitude:.2f}m | Vel: {velocity:.2f} m/s | Event: {event}", end="\r")

    except Exception as e:
        print("\n[ERROR]:", e)
        reconnect_imu()
        sleep_ms(500)
