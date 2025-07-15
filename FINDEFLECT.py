import time
from machine import Pin, PWM

# Function to convert degrees to PWM duty (1ms to 2ms corresponding to 0-180 degrees)
def degrees_to_pwm(degrees):
    # SG90 typically needs a 1-2ms pulse at 50Hz.
    # Period = 20ms, so 1ms corresponds to 1/20 of the period, i.e., 3277.
    # 2ms corresponds to 2/20 of the period, i.e., 6553.
    pulse_width = int((degrees / 180.0) * 3276) + 3277  # maps 0-180 to 1-2ms duty cycle
    return pulse_width

# Initialize PWM on GPIO pins 16 and 15
servo_1 = PWM(Pin(9))  # GPIO 16
servo_2 = PWM(Pin(21))  # GPIO 15
servo_1.freq(50)  # 50Hz frequency for the servo
servo_2.freq(50)  # 50Hz frequency for the servo

value = 90  # Initialize starting position
increment = 10  # Increment step

while True:
    # Move servos in increments from 90 to 270 degrees
    while value <= 270:
        servo_1.duty_u16(degrees_to_pwm(value))  
        servo_2.duty_u16(degrees_to_pwm(value))  # Both servos move in the same direction

        print(f"Moving both servos to {value} degrees")  # Print the current angle
        value += increment  # Increment the angle by 10 degrees
        time.sleep(0.08)  # Wait for servos to move before next change

    # Once the value hits 270, reverse the direction
    while value >= 90:
        servo_1.duty_u16(degrees_to_pwm(value))  
        servo_2.duty_u16(degrees_to_pwm(value))  # Both servos move in the same direction

        print(f"Moving both servos to {value} degrees")  # Print the current angle
        value -= increment  # Decrement the angle by 10 degrees
        time.sleep(0.08)  # Wait for servos to move before next change
