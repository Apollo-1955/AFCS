import math
from machine import I2C, Pin, PWM
from mpu6500 import MPU6500
from bmp180 import BMP180
from time import ticks_us, ticks_diff, ticks_ms, sleep_ms

# === I2C Init ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)

# === Onboard LED Init ===
led = Pin(25, Pin.OUT)

# === Sensor Status Flags ===
mpu_connected = False
bmp_connected = False

# === Sensor Init with Retry ===
def init_mpu():
    global mpu_connected
    while True:
        try:
            mpu = MPU6500(i2c)
            mpu_connected = True
            print("✅ MPU6500 connected.")
            return mpu
        except:
            mpu_connected = False
            print("⚠️ MPU6500 connection failed. Retrying...")
            sleep_ms(500)

def try_connect_bmp180():
    global bmp, bmp_connected
    try:
        bmp = BMP180(i2c)
        bmp.oversample_sett = 2
        bmp.sea_level_pressure = 101325
        bmp_connected = True
        print("✅ BMP180 connected.")
    except Exception as e:
        bmp = None
        bmp_connected = False
        print("⚠️ BMP180 connection failed:", e)

mpu = init_mpu()
bmp = None
try_connect_bmp180()

# === Servo Setup ===
servo1 = PWM(Pin(9))
servo2 = PWM(Pin(21))
servo1.freq(50)
servo2.freq(50)

def angle_to_duty(angle):
    angle = max(135, min(225, angle))  # ±45° around center
    return int((angle / 180.0) * 3276) + 3277

servo1.duty_u16(angle_to_duty(180))
servo2.duty_u16(angle_to_duty(180))

# === Gyro Calibration ===
def calibrate_gyro(samples=500, delay_ms=5):
    print("🔧 Calibrating gyro... Keep still.")
    gx_total = gy_total = gz_total = 0.0
    for _ in range(samples):
        gx, gy, gz = mpu.gyro
        gx_total += gx
        gy_total += gy
        gz_total += gz
        sleep_ms(delay_ms)
    print("✅ Calibration complete.")
    return gx_total / samples, gy_total / samples, gz_total / samples

gx_bias, gy_bias, gz_bias = calibrate_gyro()

# === Complementary Filter Setup ===
roll = pitch = yaw = 0.0
alpha = 0.98
last = ticks_us()

# === Yaw Target ===
desired_yaw = 0.0
yaw_updated = False  # To only apply +50° once

# === Accelerometer Angle Calculation ===
def get_accel_angles(ax, ay, az):
    GRAVITY = 9.81
    ay -= GRAVITY
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0:
        return 0.0, 0.0
    ax /= norm
    ay /= norm
    az /= norm
    roll_acc = math.atan2(ay, az)
    pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    return math.degrees(roll_acc), math.degrees(pitch_acc)

# === Timing Start ===
boot_time_us = ticks_us()

# === Main Loop ===
while True:
    now = ticks_us()
    dt = ticks_diff(now, last) / 1_000_000
    last = now
    if dt <= 0 or dt > 0.1:
        dt = 0.01

    # Sensor Read
    try:
        ax, ay, az = mpu.acceleration
        gx, gy, gz = mpu.gyro
    except:
        print("⚠️ MPU read failed. Reconnecting...")
        mpu = init_mpu()
        continue

    # BMP180 Reconnect Retry
    if not bmp_connected and ticks_ms() % 2000 < 10:
        try_connect_bmp180()

    gx -= gx_bias
    gy -= gy_bias
    gz -= gz_bias

    gx_deg = math.degrees(gx)
    gy_deg = math.degrees(gy)
    gz_deg = math.degrees(gz)

    roll_acc, pitch_acc = get_accel_angles(ax, ay, az)

    roll_gyro = roll + gx_deg * dt
    pitch_gyro = pitch + gy_deg * dt
    yaw += gz_deg * dt

    if yaw > 180:
        yaw -= 360
    elif yaw < -180:
        yaw += 360

    roll = alpha * roll_gyro + (1 - alpha) * roll_acc
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc

    # === After 10s, change desired yaw += 50°
    elapsed = ticks_diff(now, boot_time_us) / 1_000_000
    if elapsed >= 10 and not yaw_updated:
        desired_yaw += 50
        print(f"\n⏱ 10s reached — New desired yaw: {desired_yaw}°")
        yaw_updated = True

    # === Yaw Correction ===
    yaw_error = desired_yaw - yaw
    yaw_error = max(-90, min(90, yaw_error))  # Clamp

    servo1_angle = 180 - yaw_error
    servo2_angle = 180 - yaw_error

    servo1.duty_u16(angle_to_duty(servo1_angle))
    servo2.duty_u16(angle_to_duty(servo2_angle))

    # === LED Status ===
    if mpu_connected and bmp_connected:
        led.value(1)  # Solid ON
    elif mpu_connected or bmp_connected:
        led.value((ticks_ms() // 500) % 2)  # Flash
    else:
        led.value(0)  # OFF

    # === Debug Output ===
    print(f"Yaw: {yaw:.2f}° | Err: {yaw_error:.2f}° | Target: {desired_yaw:.2f}° | S1: {servo1_angle:.2f}° | S2: {servo2_angle:.2f}°", end="\r")

    sleep_ms(10)
