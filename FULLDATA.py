from imu import MPU6050
from bmp180 import BMP180
from time import ticks_us, ticks_diff, time, sleep_ms, sleep
from machine import Pin, I2C, PWM, UART
import math
import os

# === Initialize I2C, MPU6050 ===
i2c_imu = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c_imu)

# === BMP180 auto-retry init ===
i2c_bmp = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)
bmp = None
while bmp is None:
    try:
        print("Attempting to connect to BMP180...")
        bmp = BMP180(i2c_bmp)
        bmp.oversample_sett = 2
        bmp.sea_level_pressure = 101325
        print("BMP180 connected successfully!")
    except Exception as e:
        print("BMP180 connection failed:", e)
        sleep(2)

# === Kalman Filter Setup ===
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

# === GPS UART Setup ===
uart = UART(0, baudrate=9600, tx=None, rx=13)

def convert_coord(coord, direction):
    if not coord or len(coord) < 6:
        return None
    deg_len = 2 if direction in ['N', 'S'] else 3
    degrees = int(coord[:deg_len])
    minutes = float(coord[deg_len:])
    decimal_degrees = degrees + minutes / 60.0
    if direction in ['S', 'W']:
        decimal_degrees = -decimal_degrees
    return decimal_degrees

def parse_gpgga(sentence):
    parts = sentence.split(',')
    if parts[0] != "$GPGGA":
        return None
    try:
        utc_time = parts[1]
        lat = convert_coord(parts[2], parts[3])
        lon = convert_coord(parts[4], parts[5])
        fix_quality = int(parts[6])
        num_sats = int(parts[7])
        altitude = float(parts[9])
        return (utc_time, lat, lon, fix_quality, num_sats, altitude)
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

buffer = b''

# === Create log file with header including GPS data ===
filename = get_next_log_filename()
log_file = open(filename, "w")
log_file.write("Timestamp (s), Altitude (m), Roll (deg), Pitch (deg), Velocity Z (deg/s), Alt Derivative (m/s), Servo Angle (deg), Event, GPS UTC, GPS Lat, GPS Lon, GPS Fix, GPS Sats, GPS Alt (m), GPS Speed (m/s)\n")
log_file.flush()
last_flush_time = time()

# === Flight Detection ===
liftoff_detected = False
apogee_detected = False
landed_detected = False
stable_time = 0
prev_velocity_z = 0
prev_altitude = 0
prev_alt_time = ticks_us()

# === MAIN LOOP ===
while True:
    try:
        # === Sensor Read ===
        ax, ay, az = imu.accel.x, imu.accel.y, imu.accel.z
        gy = imu.gyro.y
        gz = imu.gyro.z

        bmp.measure()
        altitude = bmp.altitude

        # === Altitude derivative dz/dt ===
        now_alt_time = ticks_us()
        dt_alt = ticks_diff(now_alt_time, prev_alt_time) / 1_000_000.0
        dz = (altitude - prev_altitude) / dt_alt if dt_alt > 0 else 0
        prev_altitude = altitude
        prev_alt_time = now_alt_time

        # === Kalman pitch ===
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
        current_time_us = ticks_us()
        dt = ticks_diff(current_time_us, prev_time_us) / 1_000_000.0
        prev_time_us = current_time_us

        pitch_angle = kalman_update(pitch_acc, gy, dt)
        pitch_angle = max(-90, min(90, pitch_angle))

        roll = math.atan2(ay, az) * (180 / math.pi)
        velocity_z = gz
        servo_deg = 180 - pitch_angle
        servo_val = map_pitch_to_servo(pitch_angle)
        servo1.duty_u16(servo_val)
        servo2.duty_u16(servo_val)

        timestamp = ticks_diff(current_time_us, boot_time_us) / 1_000_000.0

        # === Event Detection ===
        event = ""
        vertical_accel = az - 1  # in g's

        if not liftoff_detected and vertical_accel > 0.5:
            liftoff_detected = True
            event = "🚀 Liftoff"

        if liftoff_detected and not apogee_detected:
            if prev_altitude < altitude and dz < 0:  # altitude falling
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

        # === Read GPS Data from UART ===
        data = uart.read()
        if data:
            buffer += data
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                try:
                    line_str = line.decode('ascii').strip()
                except:
                    line_str = ''

                if line_str.startswith('$GPGGA'):
                    gpgga_data = parse_gpgga(line_str)
                elif line_str.startswith('$GPRMC'):
                    gprmc_speed = parse_gprmc(line_str)
                else:
                    # reset if sentence unrecognized
                    gpgga_data = None
                    gprmc_speed = None

        # === Compose GPS values or defaults ===
        gps_utc = gpgga_data[0] if gpgga_data else 'N/A'
        gps_lat = gpgga_data[1] if gpgga_data else 'N/A'
        gps_lon = gpgga_data[2] if gpgga_data else 'N/A'
        gps_fix = gpgga_data[3] if gpgga_data else 'N/A'
        gps_sats = gpgga_data[4] if gpgga_data else 'N/A'
        gps_alt = gpgga_data[5] if gpgga_data else 'N/A'
        gps_speed = gprmc_speed if 'gprmc_speed' in locals() and gprmc_speed is not None else 'N/A'

        # === Logging ===
        log_file.write(
            f"{timestamp:.2f},{altitude:.2f},{roll:.2f},{pitch_angle:.2f},{velocity_z:.2f},{dz:.2f},{servo_deg:.2f},{event},{gps_utc},{gps_lat},{gps_lon},{gps_fix},{gps_sats},{gps_alt},{gps_speed}\n"
        )

        if time() - last_flush_time >= 10:
            log_file.flush()
            last_flush_time = time()
            print(f"\n[FLUSHED] ⏱ {timestamp:.2f}s | Alt={altitude:.2f} m | dZ/dt={dz:.2f} m/s")

        print(f"⏱ {timestamp:.2f}s | Alt: {altitude:.2f} m | dZ/dt: {dz:.2f} m/s | Pitch: {pitch_angle:.2f}° | GPS: Lat {gps_lat} Lon {gps_lon} Sats {gps_sats}", end="\r")

    except Exception as e:
        print("\n[ERROR]:", e)
        reconnect_imu()
        sleep_ms(500)
