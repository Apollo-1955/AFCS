from imu import MPU6050
from time import ticks_us
from machine import Pin, I2C, PWM
import math

# Initialize I2C and MPU6050
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)

# Kalman filter variables
Q_angle = 0.001  # Process noise variance for accelerometer
Q_bias = 0.003   # Process noise variance for gyro bias
R_measure = 0.03 # Measurement noise variance

pitch_angle = 0.0  # Initial pitch angle
bias_pitch = 0.0   # Gyro bias
P = [[0, 0], [0, 0]]  # Kalman covariance matrix

prev_time = ticks_us()

# Initialize servo on GPIO 16
servo = PWM(Pin(16))
servo.freq(50)  # Set servo frequency to 50Hz

def kalman_update(new_angle, new_rate, dt):
    global pitch_angle, bias_pitch, P

    # Predict phase
    rate = new_rate - bias_pitch
    pitch_angle += dt * rate

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += Q_bias * dt

    # Update phase
    S = P[0][0] + R_measure
    K = [P[0][0] / S, P[1][0] / S]  # Kalman gain

    y = new_angle - pitch_angle
    pitch_angle += K[0] * y
    bias_pitch += K[1] * y

    P[0][0] -= K[0] * P[0][0]
    P[0][1] -= K[0] * P[0][1]
    P[1][0] -= K[1] * P[0][0]
    P[1][1] -= K[1] * P[0][1]

    return pitch_angle

def map_pitch_to_servo(pitch):
    """ Convert pitch angle (-90° to 90°) to servo angle (90° to 270°). """
    # Maps -90° to 90° to 90° to 270°: 0 -> 180°, -90 -> 270°, 90 -> 90°
    servo_angle = 180 - pitch  # Inverts pitch for correct servo movement
    return int((servo_angle / 180.0) * 3276) + 3277  # Maps 90°-270° to duty cycle

# Move servo to 180° (midpoint) at startup
servo.duty_u16(map_pitch_to_servo(0))

while True:
    # Read sensor values
    ax, ay, az = imu.accel.x, imu.accel.y, imu.accel.z
    gy = imu.gyro.y  # Gyroscope Y-axis

    # Compute accelerometer pitch in degrees
    pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)

    # Compute delta time
    current_time = ticks_us()
    dt = (current_time - prev_time) / 1_000_000.0  # Convert us to seconds
    prev_time = current_time

    # Apply Kalman filter for precise tracking
    pitch_angle = kalman_update(pitch_acc, gy, dt)

    # Move servo to match pitch angle
    servo.duty_u16(map_pitch_to_servo(pitch_angle))

    # Print pitch angle and servo PWM
    print(f"Pitch: {pitch_angle:.2f}°  Servo Duty: {map_pitch_to_servo(pitch_angle)}", end="\r")

