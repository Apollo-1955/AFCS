import math, os
from machine import I2C, Pin, PWM
from time import ticks_us, ticks_diff, ticks_ms, sleep_ms
from mpu6500 import MPU6500
import bmp180

# === LED Setup ===
led = Pin(25, Pin.OUT)

def blink(times, delay_ms=200):
    for _ in range(times):
        led.on()
        sleep_ms(delay_ms)
        led.off()
        sleep_ms(delay_ms)

# === I2C Init ===
i2c0 = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)  # MPU6500
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)   # BMP180

# === Sensor Initialization with Retry ===
def init_mpu():
    while True:
        try:
            mpu = MPU6500(i2c0)
            print("✅ MPU6500 connected.")
            return mpu
        except:
            print("❌ MPU6500 connection failed. Retrying...")
            blink(5, 100)
            sleep_ms(500)

def init_bmp():
    while True:
        try:
            bmp = bmp180.BMP180(i2c1)
            bmp.oversample_sett = 2
            bmp.sea_level_pressure = 101325
            print("✅ BMP180 connected.")
            return bmp
        except:
            print("❌ BMP180 connection failed. Retrying...")
            blink(1, 300)
            sleep_ms(1000)

mpu = init_mpu()
bmp = init_bmp()
led.on()  # solid ON when both OK

# === Gyro Calibration ===
def calibrate_gyro(samples=500, delay_ms=5):
    print("Calibrating gyro... Keep sensor still.")
    gx_offset = gy_offset = gz_offset = 0.0
    for _ in range(samples):
        gx, gy, gz = mpu.gyro
        gx_offset += gx; gy_offset += gy; gz_offset += gz
        sleep_ms(delay_ms)
    return gx_offset / samples, gy_offset / samples, gz_offset / samples

gx_bias, gy_bias, gz_bias = calibrate_gyro()

# === Complementary Filter Setup ===
roll = pitch = yaw = 0.0
alpha = 0.98
last = ticks_us()

# === Desired Yaw ===
desired_yaw = 0.0
yaw_updated = False

# === Servo Setup ===
servo1 = PWM(Pin(9)); servo2 = PWM(Pin(21))
servo1.freq(50); servo2.freq(50)

def angle_to_duty(angle):
    angle = max(135, min(225, angle))
    return int((angle / 180.0) * 3276) + 3277

servo1.duty_u16(angle_to_duty(180))
servo2.duty_u16(angle_to_duty(180))

# === Accelerometer Angles ===
def get_accel_angles(ax, ay, az):
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0: return 0.0, 0.0
    ax /= norm; ay /= norm; az /= norm
    roll_acc = math.atan2(ay, az)
    pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    return math.degrees(roll_acc), math.degrees(pitch_acc)

# === Flight Log Setup ===
def next_log_filename():
    n = 1
    while True:
        fname = f"flight{n}.csv"
        if fname not in os.listdir():
            return fname
        n += 1

log_filename = next_log_filename()
log_file = open(log_filename, "w")
log_file.write("time,altitude,velocity,yaw,servo1,servo2,setpoint,apogee,mpu_status,bmp_status\n")
log_file.flush()
print(f"📂 Logging to {log_filename}")

last_save = ticks_ms()
apogee_reached = False
max_alt = -1e9

# For velocity calc
prev_alt = None
prev_t = ticks_us()

boot_time = ticks_us()

# === Main Loop ===
while True:
    now = ticks_us()
    dt = ticks_diff(now, last) / 1_000_000
    last = now
    if dt <= 0 or dt > 0.1: dt = 0.01

    elapsed = ticks_diff(now, boot_time) / 1_000_000

    # Update yaw setpoint after 15s
    if elapsed >= 15 and not yaw_updated:
        desired_yaw += 50
        yaw_updated = True
        print(f"⏱ 15s — desired_yaw set to {desired_yaw}°")

    # --- Sensor Reads with fail tolerance ---
    mpu_status = "OK"
    bmp_status = "OK"

    altitude_val = "N/A"
    velocity_val = "N/A"

    # MPU
    try:
        ax, ay, az = mpu.acceleration
        gx, gy, gz = mpu.gyro
    except:
        print("⚠️ MPU read failed.")
        blink(5, 100)
        mpu_status = "DISCONNECTED"
        mpu = init_mpu()
        mpu_status = "RECONNECTED"
        ax = ay = az = gx = gy = gz = None

    # BMP
    try:
        bmp.measure()
        altitude = bmp.altitude
        altitude_val = f"{altitude:.2f}"
    except:
        print("⚠️ BMP180 read failed.")
        blink(1, 300)
        bmp_status = "DISCONNECTED"
        bmp = init_bmp()
        bmp_status = "RECONNECTED"
        altitude = None

    # LED health
    led.on() if mpu_status=="OK" and bmp_status=="OK" else led.off()

    # Gyro/Accel fusion
    if ax is not None:
        gx -= gx_bias; gy -= gy_bias; gz -= gz_bias
        gx_deg = math.degrees(gx); gy_deg = math.degrees(gy); gz_deg = math.degrees(gz)
        roll_acc, pitch_acc = get_accel_angles(ax, ay, az)
        roll_gyro = roll + gx_deg * dt
        pitch_gyro = pitch + gy_deg * dt
        yaw += gz_deg * dt
        if yaw > 180: yaw -= 360
        elif yaw < -180: yaw += 360
        roll = alpha * roll_gyro + (1 - alpha) * roll_acc
        pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc

    # Yaw control
    yaw_error = max(-90, min(90, desired_yaw - yaw))
    servo1_angle = 180 - yaw_error
    servo2_angle = 180 - yaw_error
    servo1.duty_u16(angle_to_duty(servo1_angle))
    servo2.duty_u16(angle_to_duty(servo2_angle))

    # Velocity calculation
    dt_alt = ticks_diff(now, prev_t)/1_000_000
    if altitude is not None and prev_alt is not None and dt_alt>0:
        velocity = (altitude - prev_alt)/dt_alt
        velocity_val = f"{velocity:.2f}"
    if altitude is not None:
        prev_alt = altitude
        prev_t = now

    # Apogee detection
    if altitude is not None:
        if altitude > max_alt:
            max_alt = altitude
        elif altitude < max_alt - 0.5 and not apogee_reached:
            apogee_reached = True
            print("🚀 Apogee detected!")

    apogee_flag = "APOGEE" if apogee_reached else ""

    # Log data
    log_file.write(f"{elapsed:.2f},{altitude_val},{velocity_val},{yaw:.2f},{servo1_angle:.1f},{servo2_angle:.1f},{desired_yaw:.2f},{apogee_flag},{mpu_status},{bmp_status}\n")

    # Flush every 10s
    if ticks_diff(ticks_ms(), last_save) >= 10000:
        log_file.flush()
        last_save = ticks_ms()
        print("💾 Data flushed to storage")

    sleep_ms(10)
