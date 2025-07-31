from machine import I2C, UART, Pin
from time import ticks_us, ticks_diff, time, sleep_ms, sleep
import math, os

# === Sensor Init ===
def connect_imu():
    try:
        from imu import MPU6050
        i2c_imu = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
        return MPU6050(i2c_imu)
    except Exception as e:
        print("[IMU] Error:", e)
        return None

def connect_bmp():
    try:
        from bmp180 import BMP180
        i2c_bmp = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)
        bmp = BMP180(i2c_bmp)
        bmp.oversample_sett = 2
        bmp.sea_level_pressure = 101325
        return bmp
    except Exception as e:
        print("[BMP180] Error:", e)
        return None

# === GPS Setup ===
uart = UART(0, baudrate=9600, tx=None, rx=13)
gps_buffer = b''

def convert_coord(coord, direction):
    try:
        deg_len = 2 if direction in ['N', 'S'] else 3
        degrees = int(coord[:deg_len])
        minutes = float(coord[deg_len:])
        decimal = degrees + minutes / 60.0
        return -decimal if direction in ['S', 'W'] else decimal
    except:
        return None

def parse_gpgga(line):
    try:
        parts = line.split(',')
        if parts[0] != "$GPGGA": return None
        lat = convert_coord(parts[2], parts[3])
        lon = convert_coord(parts[4], parts[5])
        sats = int(parts[7])
        alt = float(parts[9])
        return lat, lon, sats, alt
    except:
        return None

# === Kalman ===
Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03
pitch_angle = 0.0
bias_pitch = 0.0
P = [[0, 0], [0, 0]]

def kalman_update(new_angle, new_rate, dt):
    global pitch_angle, bias_pitch, P
    rate = new_rate - bias_pitch
    pitch_angle += dt * rate
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += Q_bias * dt
    S = P[0][0] + R_measure
    K = [P[0][0]/S, P[1][0]/S]
    y = new_angle - pitch_angle
    pitch_angle += K[0] * y
    bias_pitch += K[1] * y
    P[0][0] -= K[0] * P[0][0]
    P[0][1] -= K[0] * P[0][1]
    P[1][0] -= K[1] * P[0][0]
    P[1][1] -= K[1] * P[0][1]
    return pitch_angle

# === File Logging ===
def get_next_log_filename():
    n = 1
    while True:
        fname = f"log{n}.csv"
        if fname not in os.listdir():
            return fname
        n += 1

filename = get_next_log_filename()
log_file = open(filename, "w")
log_file.write("Time(s), Altitude(m), Roll(deg), Pitch(deg), VelZ(deg/s), dZ/dt(m/s), Lat, Lon, GPS Alt(m), Sats\n")
log_file.flush()

# === Main loop vars ===
imu = connect_imu()
bmp = connect_bmp()
boot_us = ticks_us()
last_flush = time()
prev_alt = None
prev_alt_us = ticks_us()

gpgga_data = None

# === MAIN LOOP ===
while True:
    try:
        now_us = ticks_us()
        dt = ticks_diff(now_us, boot_us) / 1_000_000.0
        boot_us = now_us

        # === Sensor Reads ===
        ax = ay = az = gy = gz = roll = pitch = vel_z = dz = "N/A"
        altitude = "N/A"

        # --- IMU ---
        if imu:
            try:
                ax, ay, az = imu.accel.x, imu.accel.y, imu.accel.z
                gy = imu.gyro.y
                gz = imu.gyro.z
                roll = math.atan2(ay, az) * (180 / math.pi)
                pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
                pitch = round(kalman_update(pitch_acc, gy, dt), 2)
                vel_z = round(gz, 2)
            except Exception as e:
                print("[IMU ERROR]:", e)
                imu = None

        # --- BMP180 ---
        if bmp:
            try:
                bmp.measure()
                altitude = bmp.altitude
                now_alt_us = ticks_us()
                if prev_alt is not None:
                    dt_alt = ticks_diff(now_alt_us, prev_alt_us) / 1_000_000.0
                    dz = (altitude - prev_alt) / dt_alt if dt_alt > 0 else "N/A"
                prev_alt = altitude
                prev_alt_us = now_alt_us
            except Exception as e:
                print("[BMP180 ERROR]:", e)
                bmp = None

        # --- GPS ---
        try:
            gps_data = uart.read()
            if gps_data:
                gps_buffer += gps_data
                while b'\n' in gps_buffer:
                    line, gps_buffer = gps_buffer.split(b'\n', 1)
                    try:
                        line_str = line.decode('ascii').strip()
                        if line_str.startswith('$GPGGA'):
                            parsed = parse_gpgga(line_str)
                            if parsed: gpgga_data = parsed
                    except:
                        continue
        except Exception as e:
            print("[GPS ERROR]:", e)

        gps_lat = gps_lon = gps_alt = gps_sats = "N/A"
        if gpgga_data:
            gps_lat, gps_lon, gps_sats, gps_alt = gpgga_data

        # === Write Line ===
        line = f"{dt:.2f},{altitude},{roll},{pitch},{vel_z},{dz},{gps_lat},{gps_lon},{gps_alt},{gps_sats}\n"
        log_file.write(line)

        # === Flush periodically ===
        if time() - last_flush >= 10:
            log_file.flush()
            last_flush = time()
            print("[FLUSH] Logged up to:", dt)

        print(f"T={dt:.2f}s Alt={altitude}m Roll={roll} Pitch={pitch} GPS=({gps_lat},{gps_lon})", end="\r")

        # === Attempt Reconnects ===
        if imu is None:
            imu = connect_imu()
        if bmp is None:
            bmp = connect_bmp()

        sleep_ms(100)

    except Exception as e:
        print("[MAIN ERROR]:", e)
        sleep_ms(500)
