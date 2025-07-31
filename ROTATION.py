from machine import Pin, PWM, I2C
from time import sleep
from imu import MPU6050
from math import atan2, degrees

# Initialize I2C for MPU6050
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)

# Initialize the two SG90 servos on GPIO 15 and 16
servo1 = PWM(Pin(15))
servo2 = PWM(Pin(16))

# Set PWM frequency to 50Hz
servo1.freq(50)
servo2.freq(50)

# Define a function to set servo angle
def set_servo_angle(servo, angle):
    # Convert angle to duty cycle
    duty = int(2040 + (angle / 180) * 820)
    servo.duty_u16(duty)

# Define a function to calculate pitch and roll from accelerometer data
def calculate_orientation(ax, ay, az):
    pitch = degrees(atan2(ay, az))
    roll = degrees(atan2(-ax, az))
    return pitch, roll

while True:
    # Get accelerometer data
    ax = imu.accel.x
    ay = imu.accel.y
    az = imu.accel.z

    # Calculate pitch and roll
    pitch, roll = calculate_orientation(ax, ay, az)

    # Map pitch and roll to servo angles
    angle1 = pitch + 90  # Adjust to servo range
    angle2 = roll + 90   # Adjust to servo range

    # Set servo angles
    set_servo_angle(servo1, angle1)
    set_servo_angle(servo2, angle2)

    # Delay for stability
    sleep(0.1)
