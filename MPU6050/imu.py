#Heer Patel
#Basic Roll-Control Code for the Apex Flight Computer
#6/2/24

#Quick Note: I was about to develop a way to record the altitude, orientation, air pressure and acceleration on the Pico. - This
#data can then be viewed on computer after the flight. But due to time crunches, I didn't quite get the chance.  
from imu import MPU6050 
import time
from machine import Pin, I2C
from bmp180 import BMP180

from machine import Pin, PWM
import utime

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)
                              
bus =  I2C(scl=Pin(4), sda=Pin(5), freq=100000)  
bmp180 = BMP180(bus)
bmp180.oversample_sett = 2
bmp180.baseline = 101325

MIN1 = 1700000
MAX1 = 2200000
MID1 = 2000000

#Fin-1
pwm = PWM(Pin(16))
pwm.freq(50)

#Fin-3
pwm2 = PWM(Pin(18))
pwm2.freq(50)

#Fin-4
pwm3 = PWM(Pin(17))
pwm3.freq(50)


while True:
    
    # Temperature display from the MPU6050
    print("Temperature: ", round(imu.temperature,2), "°C")
   
    # reading values
    acceleration = imu.accel
    gyroscope = imu.gyro
    
    
    # reading values from the BMP180
    temp = bmp180.temperature
    p = bmp180.pressure
    altitude = bmp180.altitude
    print(temp, p, altitude)
        
       
    # Quick Control    
    if gyroscope.y > 45:
        pwm2.duty_ns(MIN1)
        pwm3.duty_ns(MAX1)
        

    if gyroscope.y < -45: 
        pwm2.duty_ns(MAX1)
        pwm3.duty_ns(MIN1)

        
    if gyroscope.x > -45:
        pwm.duty_ns(MAX1)
        pwm1.duty_ns(MIN1)
        
    if gyroscope.x < -45:
        pwm.duty_ns(MIN1)
        pwm1.duty_ns(MAX1)
        
    