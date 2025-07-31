import math
from machine import I2C, Pin, PWM
from mpu6500 import MPU6500
from time import ticks_us, ticks_diff, sleep_ms

# === I2C Init ===
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)

# === Sensor Init with retry ===
def init_mpu():
    while True:
        try:
            mpu = MPU6500(i2c)
            print("MPU6500 connected.")
            return mpu
        except Exception as e:
            print("MPU6500 connection failed. Retrying...")
            sleep_ms(500)

mpu = init_mpu()

# === Gyro Calibration ===
def calibrate_gyro(samples=500, delay_ms=5):
    print("Calibrating gyro... Keep sensor still.")
    gx_offset = 0.0
    gy_offset = 0.0
    gz_offset = 0.0
    for _ in range(samples):
        gx, gy, gz = mpu.gyro
        gx_offset += gx
        gy_offset += gy
        gz_offset += gz
        sleep_ms(delay_ms)
    return gx_offset / samples, gy_offset / samples, gz_offset / samples

gx_bias, gy_bias, gz_bias = calibrate_gyro()

# === Complementary Filter Setup ===
roll = 0.0
pitch = 0.0
yaw = 0.0
alpha = 0.98
last = ticks_us()

# === Desired Yaw Angle ===
desired_yaw = 0.0  # degrees (you can change this to any heading)

# === Servo Setup ===
servo1 = PWM(Pin(9))
servo2 = PWM(Pin(21))
servo1.freq(50)
servo2.freq(50)

def angle_to_duty(angle):
    angle = max(135, min(225, angle))  # ±45° around center
    return int((angle / 180.0) * 3276) + 3277

# Center servos initially
servo1.duty_u16(angle_to_duty(180))
servo2.duty_u16(angle_to_duty(180))

# === Accelerometer Angles ===
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

# === Main Loop ===
while True:
    now = ticks_us()
    dt = ticks_diff(now, last) / 1_000_000
    last = now
    if dt <= 0 or dt > 0.1:
        dt = 0.01

    try:
        ax, ay, az = mpu.acceleration
        gx, gy, gz = mpu.gyro
    except Exception as e:
        print("Sensor read failed. Attempting reconnect...")
        mpu = init_mpu()
        continue

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

    # Wrap yaw
    if yaw > 180:
        yaw -= 360
    elif yaw < -180:
        yaw += 360

    roll = alpha * roll_gyro + (1 - alpha) * roll_acc
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc

    # === Yaw Correction ===
    yaw_error = desired_yaw - yaw
    yaw_error = max(-90, min(90, yaw_error))  # Clamp to avoid extreme actuation

    # REVERSED servo directions
    servo1_angle = 180 - yaw_error
    servo2_angle = 180 - yaw_error

    servo1.duty_u16(angle_to_duty(servo1_angle))
    servo2.duty_u16(angle_to_duty(servo2_angle))

    print(f"Yaw: {yaw:.2f}° | Error: {yaw_error:.2f}° | S1: {servo1_angle:.2f}° | S2: {servo2_angle:.2f}°")

    sleep_ms(10)
