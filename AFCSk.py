from imu import MPU6050
from bmp180 import BMP180
from machine import Pin, I2C, PWM, UART
from time import ticks_us, ticks_ms, sleep_ms
import math
import os

# === I2C Setup ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
i2C = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)
imu = MPU6050(i2c)
bmp = BMP180(i2C)
bmp.oversample_sett = 2

# === GPS UART Setup ===
uart = UART(0, baudrate=9600, tx=None, rx=Pin(13))
uart.write(b"$PMTK220,100*2F\r\n")  # 10Hz output rate

# === Servo Setup ===
servo1 = PWM(Pin(9))
servo2 = PWM(Pin(21))
servo1.freq(50)
servo2.freq(50)

# === Kalman Filter ===
Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03
pitch_angle = 0.0
bias_pitch = 0.0
P = [[0, 0], [0, 0]]
prev_time = ticks_us()
start_ms = ticks_ms()

def kalman_update(new_angle, new_rate, dt):
    global pitch_angle, bias_pitch, P
    rate = new_rate - bias_pitch
    pitch_angle += dt * rate
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle)
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
        print("IMU reconnection failed:", e)

def get_next_log_filename():
    n = 1
    while True:
        fname = f"log{n}.csv"
        if fname not in os.listdir():
            return fname
        n += 1

def parse_gpgga(sentence):
    try:
        parts = sentence.split(',')
        if parts[0] != "$GPGGA": return None
        alt = float(parts[9])
        sats = int(parts[7])
        return alt, sats
    except:
        return None

def parse_gprmc(sentence):
    try:
        parts = sentence.split(',')
        if parts[0] != "$GPRMC": return None
        speed_knots = float(parts[7])
        return speed_knots * 0.514444  # knots to m/s
    except:
        return None

def read_gps():
    buffer = b''
    gps_alt, gps_vel, gps_sats = None, None, None
    data = uart.read()
    if data:
        buffer += data
        lines = buffer.split(b'\n')
        for line in lines:
            try:
                line_str = line.decode('ascii').strip()
                if line_str.startswith('$GPGGA'):
                    result = parse_gpgga(line_str)
                    if result: gps_alt, gps_sats = result
                elif line_str.startswith('$GPRMC'):
                    result = parse_gprmc(line_str)
                    if result: gps_vel = result
            except:
                continue
    return gps_alt, gps_vel, gps_sats

# === Servo Neutral Position ===
servo1.duty_u16(map_pitch_to_servo(0))
servo2.duty_u16(map_pitch_to_servo(0))

# === Logging Setup ===
filename = get_next_log_filename()
log_file = open(filename, "w")
log_file.write("Time (s), Alt (m), AltRate (m/s), Roll (deg), Vel (m/s), Servo (deg), Event, GPS Alt (m), GPS Vel (m/s), GPS Sats\n")
log_file.flush()

# === Flight State Vars ===
last_flush_time = ticks_ms()
prev_altitude = None
prev_altitude_time = None
prev_velocity = 0
last_event = "None"

# === Flight Detection Thresholds ===
ACC_LIFTOFF = 0.5
VEL_APOGEE = 0.1
VEL_LANDED = 0.05
ACC_LANDED = 0.1

# === MAIN LOOP ===
while True:
    try:
        # === Sensor Reads ===
        ax, ay, az = imu.accel.x, imu.accel.y, imu.accel.z
        gy = imu.gyro.y
        bmp.measure()
        altitude = bmp.altitude
        roll = math.atan2(imu.accel.y, imu.accel.z) * (180 / math.pi)

        # === Pitch & Kalman ===
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
        dt = (ticks_us() - prev_time) / 1_000_000
        prev_time = ticks_us()
        pitch_angle = kalman_update(pitch_acc, gy, dt)
        pitch_angle = max(-90, min(90, pitch_angle))

        # === Time and Rate ===
        elapsed_ms = ticks_ms() - start_ms
        timestamp = elapsed_ms / 1000
        if prev_altitude is not None and prev_altitude_time is not None:
            altitude_rate = (altitude - prev_altitude) / ((elapsed_ms - prev_altitude_time) / 1000)
        else:
            altitude_rate = 0
        prev_altitude = altitude
        prev_altitude_time = elapsed_ms

        # === Integrate Velocity ===
        velocity = prev_velocity + (ay * 9.81) * dt
        prev_velocity = velocity

        # === Servo Control ===
        servo_val = map_pitch_to_servo(pitch_angle)
        servo1.duty_u16(servo_val)
        servo2.duty_u16(servo_val)
        servo_degrees = 180 - pitch_angle

        # === Flight Events ===
        event = ""
        if last_event in ["None", "Landed"]:
            if ay * 9.81 > ACC_LIFTOFF and velocity > 0.1:
                event = "Liftoff"
                last_event = "Liftoff"
        elif last_event == "Liftoff":
            if velocity < -VEL_APOGEE:
                event = "Apogee"
                last_event = "Apogee"
        elif last_event == "Apogee":
            if abs(velocity) < VEL_LANDED and abs(ay * 9.81) < ACC_LANDED:
                event = "Landed"
                last_event = "Landed"

        # === GPS Read ===
        gps_alt, gps_vel, gps_sats = read_gps()

        # === Log Line ===
        log_file.write(
           f"{timestamp:.2f},{altitude:.2f},{altitude_rate:.2f},{roll:.2f},{velocity:.2f},{servo_degrees:.2f},{event or ''},"
           f"{gps_alt if gps_alt is not None else 'N/A'},{gps_vel if gps_vel is not None else 'N/A'},{gps_sats if gps_sats is not None else 'N/A'}\n"
     )


        

        # === Flush Periodically ===
        if ticks_ms() - last_flush_time >= 10000:
            log_file.flush()
            last_flush_time = ticks_ms()
            print(f"\n[FLUSHED] Time={timestamp:.2f}s Alt={altitude:.2f}m Roll={roll:.2f}° Event={event or '-'}")

        print(f"Time: {timestamp:.2f}s | Alt: {altitude:.2f}m | Roll: {roll:.2f}° | Vel: {velocity:.2f} m/s | Event: {event or '-'} | GPS Alt: {gps_alt} m", end="\r")

    except Exception as e:
        print("\n[ERROR]:", e)
        reconnect_imu()
        sleep_ms(500)
